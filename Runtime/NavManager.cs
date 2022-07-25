using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using HyperNav.Runtime.Utility;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.Pool;
using Debug = UnityEngine.Debug;

namespace HyperNav.Runtime {
    public class NavManager : MonoBehaviour {
        public static NavManager Instance { get; private set; }
        
        [SerializeField] private int _maxPathOpsPerFrame = 1000;

        private List<PendingPath> _pendingPaths = new List<PendingPath>();
        private long _currentPathID = 0;
        private bool _needsToClear;

        #region Pools

        private ObjectPool<PendingPath> _pendingPathPool = new ObjectPool<PendingPath>(
            () => new PendingPath(),
            null,
            path => {
                path.ID = -1;
                path.Frontier.Clear();
                path.NodeTable.Clear();
                path.Receiver = null;
            });

        private ObjectPool<NavPath> _pathPool = new ObjectPool<NavPath>(
            () => new NavPath(),
            null,
            path => {
                path.ID = -1;
                path.InternalWaypoints.Clear();
                path.StartHit = path.EndHit = default;
                path.HasBeenDisposed = true;
            });

        #endregion

        #region Unity Methods

        private void Awake() {
            Instance = this;
        }

        private void OnDestroy() {
            _pendingPathPool.Dispose();
            _pathPool.Dispose();
        }

        private void Update() {
            if (_pendingPaths.Count > 0) {
                int pathsToUpdate = 0;
                for (int i = 0; i < _pendingPaths.Count; i++) {
                    if (!_pendingPaths[i].Finished) pathsToUpdate++;
                }

                if (pathsToUpdate > 0) {
                    int opsPerPath = _maxPathOpsPerFrame / pathsToUpdate;
                    if (opsPerPath == 0) opsPerPath = 1;
                    
                    for (int i = 0; i < _pendingPaths.Count; i++) {
                        PendingPath path = _pendingPaths[i];
                        UpdatePath(path, opsPerPath);
                    }
                }
            }

            if (_needsToClear) {
                _needsToClear = false;
                _pendingPaths.RemoveAll(path => path.Finished);
            }
        }

        #endregion

        #region Internal Methods

        private void InitializePath(PendingPath path) {
            PathNode start = new PathNode {
                Position = path.StartHit.Position,
                FromRegion = -1,
                ToRegion = path.StartHit.Region,
                FromVolume = null,
                ToVolume = path.StartHit.Volume,
                Connection = null,
            };
            
            float heuristic = GetHeuristic(path, start);
            path.Frontier.Add(start, -heuristic);
            path.NodeTable[start] = new VisitedNodeInfo {
                CumulativeCost = 0,
                Heuristic = heuristic,
                Previous = null,
                Visited = false,
                Position = start.Position,
            };
        }

        private void UpdatePath(PendingPath path, int steps) {
            for (int i = 0; i < steps; i++) {
                if (path.Frontier.Count == 0) {
                    FailPath(path);
                    return;
                }
            
                PathNode node = path.Frontier.Remove();
                if (node.ToRegion == path.EndHit.Region && node.ToVolume == path.EndHit.Volume) {
                    CompletePath(path, node);
                    return;
                }
            
                Visit(path, node);
            }
        }

        private void FailPath(PendingPath path) {
            path.Receiver.Invoke(path.ID, null);
            _pendingPathPool.Release(path);
            _needsToClear = true;
        }

        private void CompletePath(PendingPath path, PathNode lastNode) {
            NavPath finishedPath = _pathPool.Get();

            // Initialize finishedPath properties.
            finishedPath.StartHit = path.StartHit;
            finishedPath.EndHit = path.EndHit;
            finishedPath.StartPos = path.StartPos;
            finishedPath.EndPos = path.EndPos;
            finishedPath.ID = path.ID;
            finishedPath.HasBeenDisposed = false;

            PathNode? current = lastNode;
            
            // Add all points along the path to the list, in reverse order.
            finishedPath.InternalWaypoints.Add(path.EndHit.Position);
            while (current.HasValue && current.Value.Connection != null) {
                PathNode cur = current.Value;

                Vector3 entryPoint = path.NodeTable[cur].Position;
                if (cur.Connection.HasDifferentExitPoint) {
                    finishedPath.InternalWaypoints.Add(
                        cur.Connection.GetExitPoint(
                            finishedPath.InternalWaypoints[finishedPath.InternalWaypoints.Count - 1]));
                }
                
                finishedPath.InternalWaypoints.Add(entryPoint);

                current = path.NodeTable[cur].Previous;
            }
            
            finishedPath.InternalWaypoints.Add(path.StartHit.Position);
            if (path.StartHit.IsOnEdge) {
                finishedPath.InternalWaypoints.Add(path.StartPos);
            }
            
            // Reverse the list so the points are in the correct order.
            finishedPath.InternalWaypoints.Reverse();
            
            path.Receiver(path.ID, finishedPath);
            
            _pendingPathPool.Release(path);
            _needsToClear = true;
        }
        
        private void Visit(PendingPath path, PathNode node) {
            VisitedNodeInfo info = path.NodeTable[node];
            info.Visited = true;
            path.NodeTable[node] = info;

            NavRegionData toRegion = node.ToRegionData;
            Debug.Log(node.Position);

            int connectionCount = toRegion.ConnectionCount;
            for (int i = 0; i < connectionCount; i++) {
                INavConnection connection = toRegion.GetConnection(i);
                if (connection.ConnectedVolume == node.FromVolume && connection.ConnectedRegionID == node.FromRegion) {
                    continue;
                }

                Vector3 nextPosition = connection.GetEntryPoint(node.Position);
                float linkCost = Vector3.Distance(node.Position, nextPosition);
                float cumulativeCost = info.CumulativeCost + linkCost;

                PathNode to = new PathNode {
                    FromRegion = node.ToRegion,
                    ToRegion = connection.ConnectedRegionID,
                    FromVolume = node.ToVolume,
                    ToVolume = connection.ConnectedVolume,
                    Position = nextPosition,
                    Connection = connection,
                };
                
                if (path.NodeTable.TryGetValue(to, out VisitedNodeInfo toInfo)) {
                    if (!toInfo.Visited && cumulativeCost < toInfo.CumulativeCost) {
                        toInfo.CumulativeCost = cumulativeCost;
                        path.NodeTable[to] = toInfo;
                        path.Frontier.Update(to, -(toInfo.Heuristic + cumulativeCost), true, to);
                    }
                } else {
                    float heuristic = GetHeuristic(path, to);
                    
                    path.NodeTable[to] = new VisitedNodeInfo {
                        Previous = node,
                        CumulativeCost = cumulativeCost,
                        Heuristic = heuristic,
                        Visited = false,
                        Position = to.Position,
                    };
                    path.Frontier.Add(to, -(heuristic + cumulativeCost));
                }
            }
        }

        private float GetHeuristic(PendingPath path, PathNode node) {
            float heuristic = Vector3.Distance(node.Position, path.EndHit.Position);
            return heuristic;
        }

        internal void DisposePath(NavPath path) {
            _pathPool.Release(path);
        }

        #endregion

        #region Public Methods
        
        public long FindPath(Vector3 start, Vector3 end, HyperNavPathCallback receiver, float sampleRadius = 0) {
            
            foreach (var volume in NavVolume.Volumes) {
                if (volume.SamplePosition(start, out NavHit startHit, sampleRadius) &&
                    volume.SamplePosition(end, out NavHit endHit, sampleRadius)) {
                    return FindPath(startHit, endHit, start, end, receiver);
                }
            }

            return -1;
        }

        public long FindPath(NavHit startHit, NavHit endHit, Vector3 startPos, Vector3 endPos,
                             HyperNavPathCallback receiver) {
            
            long id = _currentPathID++;

            PendingPath pendingPath = _pendingPathPool.Get();
            pendingPath.ID = id;
            pendingPath.StartHit = startHit;
            pendingPath.EndHit = endHit;
            pendingPath.StartPos = startPos;
            pendingPath.EndPos = endPos;
            pendingPath.Receiver = receiver;
            
            _pendingPaths.Add(pendingPath);
            InitializePath(pendingPath);
            return id;
        }

        public void CancelPath(long id) {
            PendingPath pendingPath = null;
            for (int i = 0; i < _pendingPaths.Count; i++) {
                PendingPath current = _pendingPaths[i];
                if (current.ID != id) continue;
                pendingPath = current;
                break;
            }

            if (pendingPath == null) return;
            
            _pendingPathPool.Release(pendingPath);
            _needsToClear = true;
        }

        #endregion

        #region Internal Data Types
        
        private struct PathNode : IEquatable<PathNode> {
            public int FromRegion;
            public int ToRegion;
            public NavVolume FromVolume;
            public NavVolume ToVolume;
            public Vector3 Position;
            public INavConnection Connection;

            public NavRegionData ToRegionData => ToVolume ? ToVolume.Data.Regions[ToRegion] : null;
            public NavRegionData FromRegionData => FromVolume ? FromVolume.Data.Regions[FromRegion] : null;
            
            public bool Equals(PathNode other) {
                return FromRegion == other.FromRegion &&
                       ToRegion == other.ToRegion &&
                       ReferenceEquals(FromVolume, other.FromVolume) &&
                       ReferenceEquals(ToVolume, other.ToVolume) &&
                       ReferenceEquals(Connection, other.Connection);
            }

            public override bool Equals(object obj) {
                return obj is PathNode other && Equals(other);
            }

            public override int GetHashCode() {
                return HashCode.Combine(FromRegion, ToRegion, FromVolume, ToVolume, Connection);
            }
        }

        private struct VisitedNodeInfo {
            public PathNode? Previous;
            public float Heuristic;
            public float CumulativeCost;
            public bool Visited;
            public Vector3 Position;
        }
        
        private class PendingPath {
            public long ID = -1;
            public HyperNavPathCallback Receiver;
            public NavHit StartHit;
            public NavHit EndHit;
            public Vector3 StartPos;
            public Vector3 EndPos;
            
            public readonly Dictionary<PathNode, VisitedNodeInfo> NodeTable = new Dictionary<PathNode, VisitedNodeInfo>();
            public readonly Heap<PathNode> Frontier = new Heap<PathNode>();

            public bool Finished => ID < 0;
        }

        #endregion
    }

    #region External Data Types
    
    public delegate void HyperNavPathCallback(long id, NavPath path);
    
    public class NavPath : IDisposable {
        public long ID { get; internal set; } = -1;
        public bool HasBeenDisposed { get; internal set; } = true;
        
        public Vector3 StartPos { get; internal set; }
        public Vector3 EndPos { get; internal set; }
        public NavHit StartHit { get; internal set; }
        public NavHit EndHit { get; internal set; }

        internal List<Vector3> InternalWaypoints { get; } = new List<Vector3>();
        public IReadOnlyList<Vector3> Waypoints => InternalWaypoints;

        public void Dispose() {
            NavManager pm = NavManager.Instance;
            if (pm != null) pm.DisposePath(this);
        }
    }

    #endregion
}