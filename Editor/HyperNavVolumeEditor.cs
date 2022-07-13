using System;
using System.IO;
using System.Text;
using HyperNav.Runtime;
using UnityEditor;
using UnityEditor.IMGUI.Controls;
using UnityEditorInternal;
using UnityEngine;
using UnityEngine.AI;

namespace HyperNav.Editor {
    [CustomEditor(typeof(HyperNavVolume))]
    public class HyperNavVolumeEditor : UnityEditor.Editor {
        private static Color _handleColor = new Color(127f, 214f, 244f, 100f) / 255;
        private static Color _handleColorSelected = new Color(127f, 214f, 244f, 210f) / 255;
        private static Color _handleColorDisabled = new Color(127f * 0.75f, 214f * 0.75f, 244f * 0.75f, 100f) / 255;

        private BoxBoundsHandle _boundsHandle = new BoxBoundsHandle();
        [SerializeField] private bool _showVertexNumbers;
        
        private bool EditingCollider => EditMode.editMode == EditMode.SceneViewEditMode.Collider && EditMode.IsOwner(this);

        public override void OnInspectorGUI() {
            serializedObject.Update();
            
            using (new EditorGUI.DisabledScope(true)) {
                EditorGUILayout.PropertyField(serializedObject.FindProperty("_data"));
            }
            HyperNavVolume volume = (HyperNavVolume) target;
            EditMode.DoEditModeInspectorModeButton(EditMode.SceneViewEditMode.Collider, "Edit Volume",
                                                   EditorGUIUtility.IconContent("EditCollider"), GetBounds, this);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("_bounds"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("_blockingLayers"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("_maxAgentRadius"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("_visualizationMode"));
            _showVertexNumbers = EditorGUILayout.Toggle("Show Vertex Numbers", _showVertexNumbers);

            SerializedProperty algoProp = serializedObject.FindProperty("_voxelSize");
            algoProp.isExpanded = EditorGUILayout.Foldout(algoProp.isExpanded, "Algorithm Properties");
            if (algoProp.isExpanded) {
                EditorGUILayout.PropertyField(serializedObject.FindProperty("_voxelSize"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("_distanceBlurRadius"));
            }

            if (GUILayout.Button("Bake")) {
                HyperNavVolumeUtil.GetOrCreateData(serializedObject);
                HyperNavVolumeUtil.BakeData(volume);
            }

            EditorGUI.BeginDisabledGroup(volume.EditorOnlyPreviewMesh == null);
            if (GUILayout.Button("Export Preview")) {
                ExportPreviewMesh(volume.EditorOnlyPreviewMesh);
            }
            EditorGUI.EndDisabledGroup();

            serializedObject.ApplyModifiedProperties();
        }

        private Bounds GetBounds() => ((HyperNavVolume) target).Bounds;
        
        [DrawGizmo(GizmoType.Selected | GizmoType.Active | GizmoType.Pickable)]
        private static void RenderBoxGizmoSelected(HyperNavVolume volume, GizmoType gizmoType)
        {
            RenderBoxGizmo(volume, gizmoType, true);
        }

        [DrawGizmo(GizmoType.NotInSelectionHierarchy | GizmoType.Pickable)]
        private static void RenderBoxGizmoNotSelected(HyperNavVolume volume, GizmoType gizmoType)
        {
            RenderBoxGizmo(volume, gizmoType, false);
        }

        private static void ExportPreviewMesh(Mesh mesh) {
            string path = EditorUtility.SaveFilePanel("Save Mesh", Application.dataPath, "Preview.obj", "obj");
            if (string.IsNullOrEmpty(path)) return;

            StringBuilder builder = new StringBuilder();

            Vector3[] vertices = mesh.vertices;
            for (int i = 0; i < vertices.Length; i++) {
                Vector3 v = vertices[i];
                builder.Append($"v {v.x} {v.y} {v.z}").Append(Environment.NewLine);
            }

            int smCount = mesh.subMeshCount;
            for (int i = 0; i < smCount; i++) {
                builder.Append($"usemtl mat{i}").Append(Environment.NewLine);
                int[] tris = mesh.GetIndices(i);
                int triCount = tris.Length / 3;
                for (int j = 0; j < triCount; j++) {
                    int t = j * 3;
                    if (tris[t] < 0) continue;
                    builder.Append($"f {tris[t + 0] + 1} {tris[t + 1] + 1} {tris[t + 2] + 1}").Append(Environment.NewLine);
                }
            }
            
            File.WriteAllText(path, builder.ToString());
        }
        
        private static void RenderBoxGizmo(HyperNavVolume volume, GizmoType gizmoType, bool selected)
        {
            Color color = selected ? _handleColorSelected : _handleColor;
            if (!volume.enabled)
                color = _handleColorDisabled;

            Color oldColor = Gizmos.color;
            Matrix4x4 oldMatrix = Gizmos.matrix;

            // Use the unscaled matrix for the NavMeshSurface
            Matrix4x4 localToWorld = Matrix4x4.TRS(volume.transform.position, volume.transform.rotation, Vector3.one);
            Gizmos.matrix = localToWorld;

            Bounds bounds = volume.Bounds;
            
            Gizmos.color = color;
            Gizmos.DrawWireCube(bounds.center, bounds.size);

            if (selected && volume.enabled)
            {
                Color colorTrans = new Color(color.r * 0.75f, color.g * 0.75f, color.b * 0.75f, color.a * 0.15f);
                Gizmos.color = colorTrans;
                Gizmos.DrawCube(bounds.center, bounds.size);
            }
            
            Gizmos.matrix = oldMatrix;
            Gizmos.color = oldColor;
        }

        private void OnEnable() {
            Camera.onPreCull -= Camera_OnPreCull;
            Camera.onPreCull += Camera_OnPreCull;
        }

        private void OnDisable() {
            Camera.onPreCull -= Camera_OnPreCull;
        }

        private void Camera_OnPreCull(Camera camera) {
            if (!target || !camera) return;
            RenderVisualization((HyperNavVolume) target, camera);
        }

        private static void RenderVisualization(HyperNavVolume volume, Camera camera) {
            Mesh mesh = volume.EditorOnlyPreviewMesh;
            Material[] mats = volume.EditorOnlyPreviewMaterials;
            Matrix4x4 matrix = volume.transform.localToWorldMatrix;
            if (mesh != null && mats != null) {
                for (int i = 0; i < mesh.subMeshCount && i < mats.Length; i++) {
                    Graphics.DrawMesh(mesh, matrix, mats[i], 0, camera, i);
                }
            }
        }

        private void OnSceneGUI() {
            HyperNavVolume volume = (HyperNavVolume)target;
            if (EditingCollider) {
                Bounds bounds = volume.Bounds;
                Color color = volume.enabled ? _handleColor : _handleColorDisabled;
                Matrix4x4 localToWorld = Matrix4x4.TRS(volume.transform.position, volume.transform.rotation, Vector3.one);
                using (new Handles.DrawingScope(color, localToWorld))
                {
                    _boundsHandle.center = bounds.center;
                    _boundsHandle.size = bounds.size;

                    EditorGUI.BeginChangeCheck();
                    _boundsHandle.DrawHandle();
                    if (EditorGUI.EndChangeCheck())
                    {
                        Undo.RecordObject(volume, "Modified HyperNavVolume");
                        Vector3 center = _boundsHandle.center;
                        Vector3 size = _boundsHandle.size;
                        bounds.center = center;
                        bounds.size = size;
                        volume.Bounds = bounds;
                        EditorUtility.SetDirty(target);
                    }
                }
            }

            if (_showVertexNumbers) {
                Camera cam = Camera.current;
                if (volume.EditorOnlyPreviewMesh != null && cam != null) {
                    Vector3[] verts = volume.EditorOnlyPreviewMesh.vertices;

                    if (volume.VisualizationMode < HyperNavVisualizationMode.BasinTriangulation) {
                        for (int i = 0; i < volume.EditorOnlyPreviewMesh.subMeshCount; i++) {
                            int[] indices = volume.EditorOnlyPreviewMesh.GetIndices(i);
                            for (int j = 0; j < indices.Length; j++) {
                                Vector3 v = volume.transform.TransformPoint(verts[indices[j]]);

                                if (Vector3.SqrMagnitude(v - cam.transform.position) < 4) {
                                    Handles.Label(v, $"{i}: {indices[j]}");
                                }
                            }
                        }
                    } else {
                        for (int i = 0; i < verts.Length; i++) {
                            Vector3 v = volume.transform.TransformPoint(verts[i]);

                            if (Vector3.SqrMagnitude(v - cam.transform.position) < 4) {
                                Handles.Label(v, i.ToString());
                            }
                        }
                    }
                }
            }
        }
    }
}