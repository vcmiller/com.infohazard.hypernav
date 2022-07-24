using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using HyperNav.Runtime.Utility;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.Pool;

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
                path.CostFunction = null;
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
            RegionInfo start = new RegionInfo {
                Region = path.StartHit.Region,
                Volume = path.StartHit.Volume,
            };
            
            float heuristic = GetHeuristic(path, start);
            path.Frontier.Add(start, -heuristic);
            path.NodeTable[start] = new VisitedNodeInfo {
                Cost = 0,
                Heuristic = heuristic,
                From = null,
                FromConnection = null,
                Visited = false,
            };
        }

        private void UpdatePath(PendingPath path, int steps) {
            for (int i = 0; i < steps; i++) {
                if (path.Frontier.Count == 0) {
                    FailPath(path);
                    return;
                }
            
                RegionInfo node = path.Frontier.Remove();
                if (node.Region == path.EndHit.Region && node.Volume == path.EndHit.Volume) {
                    CompletePath(path);
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

        private void CompletePath(PendingPath path) {
            NavPath finishedPath = _pathPool.Get();

            // Initialize finishedPath properties.
            finishedPath.StartHit = path.StartHit;
            finishedPath.EndHit = path.EndHit;
            finishedPath.StartPos = path.StartPos;
            finishedPath.EndPos = path.EndPos;
            finishedPath.ID = path.ID;
            finishedPath.HasBeenDisposed = false;
            
            RegionInfo? current = new RegionInfo {
                Region = path.EndHit.Region,
                Volume = path.EndHit.Volume,
            };
            
            // Add all points along the path to the list, in reverse order.
            finishedPath.InternalWaypoints.Add(path.EndHit.Position);
            RegionInfo? next = null;
            while (current.HasValue &&
                   path.NodeTable.TryGetValue(current.Value, out VisitedNodeInfo curInfo) &&
                   curInfo.From.HasValue) {
                RegionInfo cur = current.Value;
                RegionInfo prev = curInfo.From.Value;

                Vector3 prevPoint = path.NodeTable[prev].From?.RegionCenter ?? path.StartHit.Position;
                Vector3 nextPoint = next?.RegionCenter ?? path.EndHit.Position;
                INavConnection connection = curInfo.FromConnection;
                
                connection.GetWaypoints(prevPoint, nextPoint, finishedPath.InternalWaypoints);

                next = current;
                current = path.NodeTable[cur].From;
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
        
        private void Visit(PendingPath path, RegionInfo node) {
            VisitedNodeInfo info = path.NodeTable[node];
            info.Visited = true;
            path.NodeTable[node] = info;

            NavRegionData region = node.RegionData;

            int connectionCount = region.ConnectionCount;
            for (int i = 0; i < connectionCount; i++) {
                INavConnection connection = region.GetConnection(i);

                float linkCost = path.CostFunction?.Invoke(connection) ?? connection.Cost;
                float cumulativeCost = path.NodeTable[node].Cost + linkCost;

                RegionInfo to = new RegionInfo {
                    Region = connection.ConnectedRegionID,
                    Volume = connection.ConnectedVolume,
                };
                
                if (path.NodeTable.TryGetValue(to, out VisitedNodeInfo toInfo)) {
                    if (!toInfo.Visited && cumulativeCost < toInfo.Cost) {
                        toInfo.Cost = cumulativeCost;
                        path.NodeTable[to] = toInfo;
                        path.Frontier.Update(to, -(toInfo.Heuristic + cumulativeCost));
                    }
                } else {
                    float heuristic = GetHeuristic(path, to);
                    
                    path.NodeTable[to] = new VisitedNodeInfo {
                        From = node,
                        FromConnection = connection,
                        Cost = cumulativeCost,
                        Heuristic = heuristic,
                        Visited = false,
                    };
                    path.Frontier.Add(to, -(heuristic + cumulativeCost));
                }
            }
        }

        private float GetHeuristic(PendingPath path, RegionInfo node) {
            Vector3 regionCenter = node.Volume.transform.TransformPoint(node.RegionData.Bounds.center);
            float heuristic = Vector3.Distance(regionCenter, path.EndHit.Position);
            return heuristic;
        }

        internal void DisposePath(NavPath path) {
            _pathPool.Release(path);
        }

        #endregion

        #region Public Methods
        
        public long FindPath(Vector3 start, Vector3 end, HyperNavPathCallback receiver, float sampleRadius = 0,
                             HyperNavCostFunction costFunc = null) {
            
            foreach (var volume in NavVolume.Volumes) {
                if (volume.SamplePosition(start, out NavHit startHit, sampleRadius) &&
                    volume.SamplePosition(end, out NavHit endHit, sampleRadius)) {
                    return FindPath(startHit, endHit, start, end, receiver, costFunc);
                }
            }

            return -1;
        }

        public long FindPath(NavHit startHit, NavHit endHit, Vector3 startPos, Vector3 endPos,
                             HyperNavPathCallback receiver, HyperNavCostFunction costFunc = null) {
            
            long id = _currentPathID++;

            PendingPath pendingPath = _pendingPathPool.Get();
            pendingPath.ID = id;
            pendingPath.StartHit = startHit;
            pendingPath.EndHit = endHit;
            pendingPath.StartPos = startPos;
            pendingPath.EndPos = endPos;
            pendingPath.Receiver = receiver;
            pendingPath.CostFunction = costFunc;
            
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
        
        private struct RegionInfo : IEquatable<RegionInfo> {
            public int Region;
            public NavVolume Volume;

            public NavRegionData RegionData => Volume.Data.Regions[Region];

            public Vector3 RegionCenter => Volume.transform.TransformPoint(RegionData.Bounds.center);
            
            public bool Equals(RegionInfo other) {
                return Region == other.Region && Equals(Volume, other.Volume);
            }

            public override bool Equals(object obj) {
                return obj is RegionInfo other && Equals(other);
            }

            public override int GetHashCode() {
                return HashCode.Combine(Region, Volume);
            }
        }

        private struct VisitedNodeInfo {
            public RegionInfo? From;
            public INavConnection FromConnection;
            public float Heuristic;
            public float Cost;
            public bool Visited;
        }
        
        private class PendingPath {
            public long ID = -1;
            public HyperNavPathCallback Receiver;
            public HyperNavCostFunction CostFunction;
            public NavHit StartHit;
            public NavHit EndHit;
            public Vector3 StartPos;
            public Vector3 EndPos;
            
            public readonly Dictionary<RegionInfo, VisitedNodeInfo> NodeTable = new Dictionary<RegionInfo, VisitedNodeInfo>();
            public readonly Heap<RegionInfo> Frontier = new Heap<RegionInfo>();

            public bool Finished => ID < 0;
        }

        #endregion
    }

    #region External Data Types
    
    public delegate void HyperNavPathCallback(long id, NavPath path);

    public delegate float HyperNavCostFunction(INavConnection connection);
    
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