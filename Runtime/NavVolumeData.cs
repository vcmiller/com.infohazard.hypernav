using System;
using System.Collections.Generic;
using UnityEngine;
using HyperNav.Runtime.Utility;

namespace HyperNav.Runtime {
    [Serializable]
    public class NavRegionData {
        [SerializeField] private int _id;
        [SerializeField] private Bounds _bounds;
        [SerializeField, NonReorderable] private int[] _indices;
        [SerializeField, NonReorderable] private NavRegionConnectionData[] _connections;
        [SerializeField, NonReorderable] private NavRegionBoundPlane[] _boundPlanes;
        
        public int ID => _id;
        public IReadOnlyList<int> Indices => _indices;
        public IReadOnlyList<NavRegionBoundPlane> BoundPlanes => _boundPlanes;
        public Bounds Bounds => _bounds;
        public IReadOnlyList<NavRegionConnectionData> InternalConnections => _connections;
        public int ConnectionCount => _connections.Length;
        public INavConnection GetConnection(int index) => _connections[index];

        public static NavRegionData Create(int id, int[] indices, Bounds bounds,
                                           NavRegionConnectionData[] connections,
                                           NavRegionBoundPlane[] boundPlanes) {
            return new NavRegionData {
                _id = id,
                _bounds = bounds,
                _indices = indices,
                _connections = connections,
                _boundPlanes = boundPlanes,
            };
        }
    }

    [Serializable]
    public struct NavRegionBoundPlane {
        [SerializeField] private Vector3 _normal;
        [SerializeField] private int _intersectVertex;

        public Vector3 Normal => _normal;
        public int IntersectVertex => _intersectVertex;

        public static NavRegionBoundPlane Create(Vector3 normal, int intersectVertex) {
            return new NavRegionBoundPlane {
                _normal = normal,
                _intersectVertex = intersectVertex,
            };
        }
    }

    public interface INavConnection {
        NavVolume ConnectedVolume { get; }
        int ConnectedRegionID { get; }
        bool HasDifferentExitPoint { get; }

        Vector3 GetEntryPoint(Vector3 previous);
        Vector3 GetExitPoint(Vector3 next);
    }

    [Serializable]
    public class NavRegionConnectionData : INavConnection {
        [SerializeField] private int _connectedRegionID;
        [SerializeField] private float _cost;
        [SerializeField, NonReorderable] private int[] _vertices;
        [SerializeField, NonReorderable] private Edge[] _edges;
        [SerializeField, NonReorderable] private Triangle[] _triangles;

        public NavVolume Volume { get; internal set; }
        public NavVolume ConnectedVolume => Volume;
        public int SelfRegionID { get; internal set; }
        public int ConnectedRegionID => _connectedRegionID;
        public float Cost => _cost;

        public IReadOnlyList<int> Vertices => _vertices;
        public IReadOnlyList<Edge> Edges => _edges;
        public IReadOnlyList<Triangle> Triangles => _triangles;
        public bool HasDifferentExitPoint => false;

        public static NavRegionConnectionData Create(int connectedRegionID, float cost,
                                                    int[] vertices, Edge[] edges, Triangle[] triangles) {
            return new NavRegionConnectionData {
                _connectedRegionID = connectedRegionID,
                _cost = cost,
                _vertices = vertices,
                _edges = edges,
                _triangles = triangles,
            };
        }

        public Vector3 GetEntryPoint(Vector3 prev) => GetNearestPoint(prev);
        public Vector3 GetExitPoint(Vector3 next) => throw new InvalidOperationException();

        public Vector3 GetNearestPoint(Vector3 reference) {
            Vector3 localPos = Volume.transform.InverseTransformPoint(reference);
            
            Vector3 closestPoint = default;
            float closestDistance = float.PositiveInfinity;
            
            void TestPoint(Vector3 point) {
                float dist2 = Vector3.SqrMagnitude(point - localPos);
                if (dist2 < closestDistance) {
                    closestPoint = point;
                    closestDistance = dist2;
                }
            }

            IReadOnlyList<Vector3> verts = Volume.Data.Vertices;
            for (int i = 0; i < _triangles.Length; i++) {
                Triangle tri = _triangles[i];

                if (MathUtil.GetNearestPointOnTriangle(verts[tri.Vertex1], verts[tri.Vertex2],
                                                       verts[tri.Vertex3], localPos, out Vector3 triPoint)) {
                    TestPoint(triPoint);
                }
            }

            for (int i = 0; i < _edges.Length; i++) {
                Edge edge = _edges[i];

                if (MathUtil.GetNearestPointOnSegment(verts[edge.Vertex1], verts[edge.Vertex2], localPos,
                                                      out Vector3 edgePoint)) {
                    TestPoint(edgePoint);
                }
            }

            for (int i = 0; i < _vertices.Length; i++) {
                TestPoint(verts[_vertices[i]]);
            }

            Vector3 closest = Volume.transform.TransformPoint(closestPoint);
            Debug.DrawLine(closest, reference, Color.red, 10);
            return closest;
        }
    }
    
    public class NavVolumeData : ScriptableObject {
        [SerializeField, NonReorderable] private Vector3[] _vertices;
        [SerializeField, NonReorderable] private NavRegionData[] _regions;

        public IReadOnlyList<Vector3> Vertices => _vertices;
        public IReadOnlyList<NavRegionData> Regions => _regions;

        public static NavVolumeData Create(Vector3[] vertices, NavRegionData[] regions) {
            NavVolumeData data = CreateInstance<NavVolumeData>();
            data.Populate(vertices, regions);
            return data;
        }

        public void Populate(Vector3[] vertices, NavRegionData[] regions) {
            _vertices = vertices;
            _regions = regions;
        }

        internal void Initialize(NavVolume volume) {
            foreach (NavRegionData region in _regions) {
                foreach (NavRegionConnectionData connection in region.InternalConnections) {
                    connection.Volume = volume;
                    connection.SelfRegionID = region.ID;
                }
            }
        }
    }
}