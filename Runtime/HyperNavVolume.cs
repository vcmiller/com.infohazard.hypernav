using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace HyperNav.Runtime {
    [ExecuteAlways]
    public class HyperNavVolume : MonoBehaviour {
        [SerializeField] private Bounds _bounds = new Bounds(Vector3.zero, Vector3.one);
        [SerializeField] private LayerMask _blockingLayers;
        [SerializeField] private HyperNavData _data;
        [SerializeField] private float _maxAgentRadius = 1;
        
        [SerializeField] private float _voxelSize = 0.1f;
        [SerializeField] private int _distanceBlurRadius = 2;
        
        [SerializeField] private HyperNavVisualizationMode _visualizationMode;

        public Bounds Bounds {
            get => _bounds;
            set => _bounds = value;
        }

        public HyperNavData Data => _data;

        public float VoxelSize => _voxelSize;

        public float MaxAgentRadius => _maxAgentRadius;

        public LayerMask BlockingLayers => _blockingLayers;
        
        public HyperNavVisualizationMode VisualizationMode => _visualizationMode;
        
        public int DistanceBlurRadius => _distanceBlurRadius;
        
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

        private void OnDisable() {
            EditorOnlyPreviewMesh = null;
        }

        private void OnDestroy() {
            EditorOnlyPreviewMesh = null;
        }

#endif
    }
        
    public enum HyperNavVisualizationMode {
        None, Voxels, VoxelDist, BasinID, ConvexRegions, BasinEdge, BasinTriangulation, Decimation,
    }
}