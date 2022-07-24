using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using HyperNav.Runtime.Utility;
using UnityEngine;
using UnityEngine.AI;

namespace HyperNav.Runtime {
    [ExecuteAlways]
    public class NavVolume : MonoBehaviour {
        [SerializeField] private Bounds _bounds = new Bounds(Vector3.zero, Vector3.one);
        [SerializeField] private LayerMask _blockingLayers;
        [SerializeField] private NavVolumeData _data;
        [SerializeField] private float _maxAgentRadius = 1;
        
        [SerializeField] private float _voxelSize = 0.1f;
        [SerializeField] private int _distanceBlurRadius = 2;
        
        [SerializeField] private NavVolumeVisualizationMode _visualizationMode = NavVolumeVisualizationMode.Final;

        public Bounds Bounds {
            get => _bounds;
            set => _bounds = value;
        }

        public NavVolumeData Data => _data;

        public float VoxelSize => _voxelSize;

        public float MaxAgentRadius => _maxAgentRadius;

        public LayerMask BlockingLayers => _blockingLayers;
        
        public NavVolumeVisualizationMode VisualizationMode => _visualizationMode;
        
        public int DistanceBlurRadius => _distanceBlurRadius;

        private static readonly List<NavVolume> _volumes = new List<NavVolume>();
        public static IReadOnlyList<NavVolume> Volumes => _volumes;

        private void OnEnable() {
            if (Application.isPlaying) {
                _volumes.Add(this);
            }
            InitializeData();
        }
        
        private void OnDisable() {
#if UNITY_EDITOR
            EditorOnlyPreviewMesh = null;
#endif
            
            if (Application.isPlaying) {
                _volumes.Remove(this);
            }
        }

        public void InitializeData() {
            if (_data) _data.Initialize(this);
        }

        public bool SamplePosition(Vector3 position, out NavHit hit, float maxDistance) {
            hit = new NavHit {
                Region = -1,
            };
            
            if (Data == null) return false;

            Vector3 localPos = transform.InverseTransformPoint(position);
            
            // Check if we are inside any regions, this will be faster than the next check.
            for (int i = 0; i < Data.Regions.Count; i++) {
                NavRegionData region = Data.Regions[i];

                if (!region.Bounds.Contains(localPos)) continue;

                bool isOutside = false;
                for (int j = 0; j < region.BoundPlanes.Count; j++) {
                    NavRegionBoundPlane plane = region.BoundPlanes[j];
                    Vector3 vertex = Data.Vertices[plane.IntersectVertex];
                    Vector3 offset = localPos - vertex;
                    float dot = Vector3.Dot(offset, plane.Normal);
                    if (dot < 0) continue;

                    isOutside = true;
                    break;
                }

                if (!isOutside) {
                    hit = new NavHit {
                        Volume = this,
                        Region = i,
                        IsOnEdge = false,
                        Normal = Vector3.zero,
                        Position = position,
                    };
                    return true;
                }
            }

            if (maxDistance <= 0) return false;

            float maxDist2 = maxDistance * maxDistance;
            Bounds intersectBounds = new Bounds(localPos, Vector3.one * (maxDistance * 2));

            Vector3 closestPoint = default;
            float closestDistance = maxDist2;
            int closestRegion = -1;

            void TestPoint(Vector3 point, int region) {
                float dist2 = Vector3.SqrMagnitude(point - localPos);
                if (dist2 < closestDistance) {
                    closestPoint = point;
                    closestDistance = dist2;
                    closestRegion = region;
                }
            }
            
            // Check regions within maxDistance range.
            for (int i = 0; i < Data.Regions.Count; i++) {
                NavRegionData region = Data.Regions[i];

                if (!region.Bounds.Intersects(intersectBounds)) continue;

                int triCount = region.Indices.Count / 3;
                for (int triIndex = 0; triIndex < triCount; triIndex++) {
                    int triStart = triIndex * 3;
                    
                    int v1 = region.Indices[triStart + 0];
                    int v2 = region.Indices[triStart + 1];
                    int v3 = region.Indices[triStart + 2];

                    Vector3 v1Pos = Data.Vertices[v1];
                    Vector3 v2Pos = Data.Vertices[v2];
                    Vector3 v3Pos = Data.Vertices[v3];
                    
                    TestPoint(v1Pos, i);
                    TestPoint(v2Pos, i);
                    TestPoint(v3Pos, i);

                    if (MathUtil.GetNearestPointOnSegment(v1Pos, v2Pos, localPos, out Vector3 e1Pos)) {
                        TestPoint(e1Pos, i);
                    }
                    
                    if (MathUtil.GetNearestPointOnSegment(v2Pos, v3Pos, localPos, out Vector3 e2Pos)) {
                        TestPoint(e2Pos, i);
                    }
                    
                    if (MathUtil.GetNearestPointOnSegment(v3Pos, v1Pos, localPos, out Vector3 e3Pos)) {
                        TestPoint(e3Pos, i);
                    }

                    if (MathUtil.GetNearestPointOnTriangle(v1Pos, v2Pos, v3Pos, localPos, out Vector3 tPos)) {
                        TestPoint(tPos, i);
                    }
                }
            }

            if (closestRegion >= 0) {
                hit = new NavHit {
                    Volume = this,
                    Region = closestRegion,
                    IsOnEdge = true,
                    Normal = Vector3.zero,
                    Position = transform.TransformPoint(closestPoint),
                };
                return true;
            }

            return false;
        }

        #region Editor Code

#if UNITY_EDITOR

        private Mesh _editorOnlyPreviewMesh;
        public Mesh EditorOnlyPreviewMesh {
            get => _editorOnlyPreviewMesh;
            set {
                if (_editorOnlyPreviewMesh == value) return;
                if (_editorOnlyPreviewMesh) {
                    DestroyImmediate(_editorOnlyPreviewMesh);
                }
                _editorOnlyPreviewMesh = value;
                if (value) {
                    value.hideFlags = HideFlags.HideAndDontSave;
                }
            }
        }
        
        public Material[] EditorOnlyPreviewMaterials { get; set; }

        private void OnDestroy() {
            EditorOnlyPreviewMesh = null;
        }

#endif

        #endregion
    }
        
    public enum NavVolumeVisualizationMode {
        None,
        Voxels,
        VoxelDist,
        BasinID,
        ConvexRegions,
        CombinedRegions,
        RegionEdge,
        BasinTriangulation,
        Decimation,
        Final,
    }
}