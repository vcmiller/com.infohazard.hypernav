using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using HyperNav.Runtime;
using HyperNav.Runtime.Utility;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;
using Debug = UnityEngine.Debug;
using Random = UnityEngine.Random;

namespace HyperNav.Editor {
    public struct MultiRegionMeshInfo {
        public List<Vector3> Vertices { get; set; }
        public List<List<int>> VertexConnections { get; set; }
        public List<List<int>> VertexRegionMembership { get; set; }
        public List<List<int>> RegionTriangleLists { get; set; }
        public Dictionary<Triangle, Dictionary<int, int>> TriangleIndicesPerRegion { get; set; }

        public static MultiRegionMeshInfo CreateEmptyInfo() {
            return new MultiRegionMeshInfo {
                Vertices = new List<Vector3>(),
                VertexConnections = new List<List<int>>(),
                VertexRegionMembership = new List<List<int>>(),
                RegionTriangleLists = new List<List<int>>(),
                TriangleIndicesPerRegion = new Dictionary<Triangle, Dictionary<int, int>>(),
            };
        }
    }
    
    public static class NavVolumeUtil {
        private static Material _voxelPreviewMaterial;
        private static Material _voxelDistancePreviewMaterial;
        private static Material _basinIDPreviewMaterial;
        private static Material _voxelOutlinePreviewMaterial;
        private static Material _triangulationPreviewMaterial;
        private static Material _triangulationOutlinePreviewMaterial;

        private static readonly Vector3Int[] NeighborDirections = {
            Vector3Int.up, Vector3Int.down, Vector3Int.forward, Vector3Int.back, Vector3Int.right, Vector3Int.left,
        };
        
        public static NavVolumeData GetOrCreateData(SerializedObject serializedObject) {
            SerializedProperty dataProp = serializedObject.FindProperty("_data");
            if (dataProp.objectReferenceValue) {
                return (NavVolumeData) dataProp.objectReferenceValue;
            }
            
            NavVolumeData data = ScriptableObject.CreateInstance<NavVolumeData>();
            dataProp.objectReferenceValue = data;
            
            string scenePath = SceneManager.GetActiveScene().path;
            scenePath = scenePath.Substring(0, scenePath.LastIndexOf('.'));

            if (!AssetDatabase.IsValidFolder(scenePath)) {
                int lastSlash = scenePath.LastIndexOf('/');
                string parentFolder = scenePath.Substring(0, lastSlash);
                string folderName = scenePath.Substring(lastSlash + 1);
                AssetDatabase.CreateFolder(parentFolder, folderName);
            }

            string assetPath = scenePath + "/HyperNavVolume_" + serializedObject.targetObject.GetInstanceID() + ".asset";
            AssetDatabase.CreateAsset(data, assetPath);
            return data;
        }

        public static void BakeData(NavVolume volume) {
            Vector3Int voxelCounts = Vector3Int.RoundToInt(volume.Bounds.size / volume.VoxelSize);

            Stopwatch stopwatch = new Stopwatch();
            StringBuilder sb = new StringBuilder();

            void LogStopwatch(string label) {
                sb.Append($"{label}: {stopwatch.ElapsedMilliseconds} ms{Environment.NewLine}");
            }
            
            stopwatch.Restart();
            int[,,] voxels = new int[voxelCounts.x,voxelCounts.y,voxelCounts.z];
            CalculateBlockedVoxels(volume, voxelCounts, voxels);
            LogStopwatch("Calculate blocked voxels");

            if (volume.VisualizationMode == NavVolumeVisualizationMode.Voxels) {
                stopwatch.Restart();
                BuildVoxelPreviewMesh(volume, voxels);
                LogStopwatch("Build voxel preview mesh");
            }

            stopwatch.Restart();
            CalculateVoxelDistance(voxelCounts, voxels);
            LogStopwatch("Calculate voxel distance");

            if (volume.DistanceBlurRadius > 0) {
                stopwatch.Restart();
                BlurVoxelDistance(voxelCounts, ref voxels, volume.DistanceBlurRadius);
                LogStopwatch("Blur voxel distance");
            }
            
            stopwatch.Restart();
            int maxDist = GetMaxDist(voxelCounts, voxels);
            LogStopwatch("Get max distance");

            if (volume.VisualizationMode == NavVolumeVisualizationMode.VoxelDist) {
                stopwatch.Restart();
                BuildVoxelDistancePreviewMesh(volume, voxels, maxDist);
                LogStopwatch("Build voxel distance preview mesh");
            }
            
            stopwatch.Restart();
            int[,,] basins = CalculateBasinsBasic(voxelCounts, voxels, maxDist, out int basinCount);
            LogStopwatch("Calculate initial regions");

            if (volume.VisualizationMode == NavVolumeVisualizationMode.BasinID) {
                stopwatch.Restart();
                BuildBasinIDPreviewMesh(volume, voxels, basins, basinCount, maxDist);
                LogStopwatch("Build initial region preview mesh");
            }

            stopwatch.Restart();
            ConvexifyAllRegions(voxelCounts, basins, ref basinCount, volume);
            LogStopwatch("Calculate convex regions");

            if (volume.VisualizationMode == NavVolumeVisualizationMode.ConvexRegions) {
                stopwatch.Restart();
                BuildBasinIDPreviewMesh(volume, voxels, basins, basinCount, maxDist);
                LogStopwatch("Build convex region preview mesh");
            }
            
            stopwatch.Restart();
            CombineRegionsWherePossible(voxelCounts, basins, ref basinCount, volume);
            LogStopwatch("Combine regions");

            if (volume.VisualizationMode == NavVolumeVisualizationMode.CombinedRegions) {
                stopwatch.Restart();
                BuildBasinIDPreviewMesh(volume, voxels, basins, basinCount, maxDist);
                LogStopwatch("Build combined region preview mesh");
            }

            if (volume.VisualizationMode == NavVolumeVisualizationMode.RegionEdge) {
                stopwatch.Restart();
                int[,,] basinEdges = CalculateBasinEdges(voxelCounts, basins);
                BuildBasinIDPreviewMesh(volume, voxels, basinEdges, basinCount, maxDist);
                LogStopwatch("Build region edge preview mesh");
            }

            stopwatch.Restart();
            TriangulateBasins(voxelCounts, basins, out MultiRegionMeshInfo meshInfo, volume);
            LogStopwatch("Triangulate regions (marching cubes)");

            if (volume.VisualizationMode == NavVolumeVisualizationMode.BasinTriangulation) {
                stopwatch.Restart();
                BuildTriangulationPreviewMesh(volume, meshInfo.Vertices, meshInfo.RegionTriangleLists);
                LogStopwatch("Build triangulated preview mesh");
            }
            
            stopwatch.Restart();
            DecimateRegions(meshInfo, volume);
            foreach (var triangleList in meshInfo.RegionTriangleLists) {
                triangleList.RemoveAll(i => i < 0);
            }
            LogStopwatch("Decimate regions");

            if (volume.VisualizationMode == NavVolumeVisualizationMode.Decimation) {
                stopwatch.Restart();
                BuildTriangulationPreviewMesh(volume, meshInfo.Vertices, meshInfo.RegionTriangleLists);
                LogStopwatch("Build decimated preview mesh");
            }
            
            stopwatch.Restart();
            PopulateData(meshInfo, volume);
            LogStopwatch("Update serialized data");

            if (volume.VisualizationMode == NavVolumeVisualizationMode.Final) {
                volume.EditorOnlyPreviewMesh = null;
            }

            volume.InitializeData();
            
            Debug.Log($"Time taken to bake volume:{Environment.NewLine}{sb}");
        }

        private static void PopulateData(MultiRegionMeshInfo meshInfo, NavVolume volume) {
            List<List<int>> triLists = new List<List<int>>();
            List<Vector3> vertices = new List<Vector3>();
            Dictionary<Vector3, int> vertexIndices = new Dictionary<Vector3, int>();
            
            int regionCount = meshInfo.RegionTriangleLists.Count;

            // Go through all vertices and rebuild index lists.
            // This ensures that any vertices that are no longer used are not included in the data.
            for (int i = 0; i < regionCount; i++) {
                List<int> newTriList = new List<int>();
                triLists.Add(newTriList);
                List<int> oldTriList = meshInfo.RegionTriangleLists[i];
                for (int j = 0; j < oldTriList.Count; j++) {
                    Vector3 vertex = meshInfo.Vertices[oldTriList[j]];
                    if (!vertexIndices.TryGetValue(vertex, out int vertexIndex)) {
                        vertexIndex = vertices.Count;
                        vertices.Add(vertex);
                        vertexIndices[vertex] = vertexIndex;
                    }
                    
                    newTriList.Add(vertexIndex);
                }
            }

            NavRegionData[] regions = new NavRegionData[regionCount];

            Dictionary<int, HashSet<int>> vertexRegionMembership = new Dictionary<int, HashSet<int>>();
            Dictionary<Edge, HashSet<int>> edgeRegionMembership = new Dictionary<Edge, HashSet<int>>();
            Dictionary<Triangle, HashSet<int>> triangleRegionMembership =
                new Dictionary<Triangle, HashSet<int>>();

            void AddFeatureToRegion<T>(in T feature, int region, Dictionary<T, HashSet<int>> dict) {
                if (!dict.TryGetValue(feature, out HashSet<int> regionSet)) {
                    regionSet = new HashSet<int>();
                    dict[feature] = regionSet;
                }

                regionSet.Add(region);
            }

            for (int i = 0; i < regionCount; i++) {
                List<int> regionIndices = triLists[i];
                int triCount = regionIndices.Count / 3;
                for (int triIndex = 0; triIndex < triCount; triIndex++) {
                    int triStart = triIndex * 3;

                    int v1 = regionIndices[triStart + 0];
                    int v2 = regionIndices[triStart + 1];
                    int v3 = regionIndices[triStart + 2];

                    Edge e1 = new Edge(v1, v2);
                    Edge e2 = new Edge(v2, v3);
                    Edge e3 = new Edge(v3, v1);

                    Triangle tri = new Triangle(v1, v2, v3);

                    AddFeatureToRegion(v1, i, vertexRegionMembership);
                    AddFeatureToRegion(v2, i, vertexRegionMembership);
                    AddFeatureToRegion(v3, i, vertexRegionMembership);
                    
                    AddFeatureToRegion(e1, i, edgeRegionMembership);
                    AddFeatureToRegion(e2, i, edgeRegionMembership);
                    AddFeatureToRegion(e3, i, edgeRegionMembership);
                    
                    AddFeatureToRegion(tri, i, triangleRegionMembership);
                }
            }

            Bounds[] regionBounds = new Bounds[regionCount];
            for (int regionIndex = 0; regionIndex < regionCount; regionIndex++) {
                List<int> regionIndices = triLists[regionIndex];
                
                bool hasSetBounds = false;
                
                for (int i = 0; i < regionIndices.Count; i++) {
                    Vector3 pos = vertices[regionIndices[i]];
                    if (hasSetBounds) {
                        regionBounds[regionIndex].Encapsulate(pos);
                    } else {
                        regionBounds[regionIndex] = new Bounds(pos, Vector3.zero);
                        hasSetBounds = true;
                    }
                }
            }

            for (int regionIndex = 0; regionIndex < regionCount; regionIndex++) {
                var sharedFeatures =
                    new Dictionary<int, (List<int> sharedVertices, List<Edge> sharedEdges, List<Triangle>
                        sharedTriangles)>();
                List<NavRegionBoundPlane> boundPlanes = new List<NavRegionBoundPlane>();

                void AddVertexSharedRegions(int vertex) {
                    foreach (int otherRegion in vertexRegionMembership[vertex]) {
                        if (otherRegion == regionIndex) continue;

                        sharedFeatures.TryGetValue(otherRegion, out var features);
                        if (features.sharedVertices == null) {
                            features.sharedVertices = new List<int>();
                            sharedFeatures[otherRegion] = features;
                        }

                        if (!features.sharedVertices.Contains(vertex)) {
                            features.sharedVertices.Add(vertex);
                        }
                    }
                }

                void AddEdgeSharedRegions(Edge edge) {
                    foreach (int otherRegion in edgeRegionMembership[edge]) {
                        if (otherRegion == regionIndex) continue;

                        sharedFeatures.TryGetValue(otherRegion, out var features);
                        if (features.sharedEdges == null) {
                            features.sharedEdges = new List<Edge>();
                            sharedFeatures[otherRegion] = features;
                        }

                        if (!features.sharedEdges.Contains(edge)) {
                            features.sharedEdges.Add(edge);
                        }
                    }
                }
                
                void AddTriangleSharedRegions(Triangle triangle) {
                    foreach (int otherRegion in triangleRegionMembership[triangle]) {
                        if (otherRegion == regionIndex) continue;

                        sharedFeatures.TryGetValue(otherRegion, out var features);
                        if (features.sharedTriangles == null) {
                            features.sharedTriangles = new List<Triangle>();
                            sharedFeatures[otherRegion] = features;
                        }

                        if (!features.sharedTriangles.Contains(triangle)) {
                            features.sharedTriangles.Add(triangle);
                        }
                    }
                }

                List<int> regionIndices = triLists[regionIndex];

                Vector3 center = regionBounds[regionIndex].center;
                
                int triCount = regionIndices.Count / 3;
                for (int triIndex = 0; triIndex < triCount; triIndex++) {
                    int triStart = triIndex * 3;

                    int v1 = regionIndices[triStart + 0];
                    int v2 = regionIndices[triStart + 1];
                    int v3 = regionIndices[triStart + 2];

                    Edge e1 = new Edge(v1, v2);
                    Edge e2 = new Edge(v2, v3);
                    Edge e3 = new Edge(v3, v1);

                    Triangle tri = new Triangle(v1, v2, v3);
                    
                    AddVertexSharedRegions(v1);
                    AddVertexSharedRegions(v2);
                    AddVertexSharedRegions(v3);
                    AddEdgeSharedRegions(e1);
                    AddEdgeSharedRegions(e2);
                    AddEdgeSharedRegions(e3);
                    AddTriangleSharedRegions(tri);

                    Vector3 v2Pos = vertices[v2];
                    Vector3 normal = Vector3.Cross(vertices[v1] - v2Pos, vertices[v3] - v2Pos).normalized;
                    Vector3 toCenter = center - v2Pos;

                    if (Vector3.Dot(normal, toCenter) > 0) {
                        normal *= -1;
                    }

                    if (!boundPlanes.Any(plane => Vector3.Dot(plane.Normal, normal) > 0.99999)) {
                        boundPlanes.Add(NavRegionBoundPlane.Create(normal, v2));
                    }
                }

                NavRegionConnectionData[] connections =
                    new NavRegionConnectionData[sharedFeatures.Count];

                int index = 0;
                foreach (var pair in sharedFeatures) {
                    connections[index++] = NavRegionConnectionData.Create(
                        pair.Key,
                        Vector3.Distance(center, regionBounds[pair.Key].center),
                        pair.Value.sharedVertices?.ToArray() ?? Array.Empty<int>(),
                        pair.Value.sharedEdges?.ToArray() ?? Array.Empty<Edge>(),
                        pair.Value.sharedTriangles?.ToArray() ?? Array.Empty<Triangle>());
                }
                
                Array.Sort(connections, (data1, data2) => data1.ConnectedRegionID - data2.ConnectedRegionID);

                regions[regionIndex] =
                    NavRegionData.Create(regionIndex, triLists[regionIndex].ToArray(), regionBounds[regionIndex],
                                                    connections, boundPlanes.ToArray());
            }
            
            Undo.RecordObject(volume.Data, "Bake HyperNav Data");
            volume.Data.Populate(vertices.ToArray(), regions);
            EditorUtility.SetDirty(volume.Data);
            AssetDatabase.SaveAssets();
        }

        private static void DecimateRegions(MultiRegionMeshInfo meshInfo, NavVolume volume) {
            for (int i = 0; i < meshInfo.Vertices.Count; i++) {
                int sharpEdgeCount = 0;
                List<int> connections = meshInfo.VertexConnections[i];
                int firstSharpEdge = -1;
                int secondSharpEdge = -1;
                for (int j = 0; j < connections.Count; j++) {
                    int vertex2Index = connections[j];

                    if (IsEdgeAngled(meshInfo, i, vertex2Index, 
                                     out Vector3 o1, out Vector3 o2)) {
                        /*Debug.DrawLine(volume.transform.TransformPoint(meshInfo.Vertices[i]),
                                       volume.transform.TransformPoint(meshInfo.Vertices[vertex2Index]),
                                       Color.red, 1);*/

                        /*
                        if (i == 2067 && vertex2Index == 2068) {
                            Debug.DrawLine(volume.transform.TransformPoint(meshInfo.Vertices[i]),
                                           volume.transform.TransformPoint(o1),
                                           Color.green, 10);
                            Debug.DrawLine(volume.transform.TransformPoint(meshInfo.Vertices[i]),
                                           volume.transform.TransformPoint(o2),
                                           Color.green, 10);

                            string vertex1Cons = $"{i} cons: ";
                            foreach (int connection in connections) {
                                vertex1Cons += connection + ", ";
                            }

                            string vertex2Cons = $"{vertex2Index} cons: ";
                            foreach (int connection in meshInfo.VertexConnections[vertex2Index]) {
                                vertex2Cons += connection + ", ";
                            }
                            
                            Debug.Log(vertex1Cons);
                            Debug.Log(vertex2Cons);
                            return;
                        }*/

                        if (sharpEdgeCount == 0) {
                            firstSharpEdge = vertex2Index;
                        } else if (sharpEdgeCount == 1) {
                            secondSharpEdge = vertex2Index;
                        }
                        
                        sharpEdgeCount++;
                    }
                }
                
                if (sharpEdgeCount != 0 && sharpEdgeCount != 2) continue;

                foreach (int region in meshInfo.VertexRegionMembership[i]) {
                    if (sharpEdgeCount == 0 || !AreVerticesConnectedInRegion(meshInfo, i, firstSharpEdge, region)) {
                        // Case 1 - no sharp edges: just remove all triangles that connect to the vertex.
                        // Build an edge loop that contains all the newly loose edges.
                        // Then fill in triangles using the ear clipping algorithm.
                        int firstConnectedVertex = meshInfo.VertexConnections[i]
                                                           .First(v2 => AreVerticesConnectedInRegion(
                                                                      meshInfo, i, v2, region));
                    
                        if (!RemoveVertexTriangles(volume.transform, meshInfo, i, 
                                                   firstConnectedVertex, -1, region)) return;
                    } else {
                        // Case 2 - 2 sharp edges: same as case 1, but ensure that the two sharp edges
                        // remain as edges in the newly created triangles. We do this by running ear clipping
                        // twice, once for each side of these edges.
                        
                        if (!RemoveVertexTriangles(volume.transform, meshInfo, i, 
                                                   firstSharpEdge, secondSharpEdge, region)) return;
                        if (!RemoveVertexTriangles(volume.transform, meshInfo, i, 
                                                   secondSharpEdge, firstSharpEdge, region)) return;
                    }
                }

                foreach (int connectedVertex in meshInfo.VertexConnections[i]) {
                    meshInfo.VertexConnections[connectedVertex].Remove(i);
                }

                meshInfo.VertexConnections[i] = null;
                meshInfo.VertexRegionMembership[i] = null;
            }
        }

        private static bool AreVerticesConnectedInRegion(MultiRegionMeshInfo meshInfo, int vertex1, int vertex2, int region) {
            foreach (int vertex3 in meshInfo.VertexConnections[vertex1].Intersect(meshInfo.VertexConnections[vertex2])) {
                Triangle triangle = new Triangle(vertex1, vertex2, vertex3);
                if (meshInfo.TriangleIndicesPerRegion.TryGetValue(triangle, out Dictionary<int, int> triIndices) &&
                    triIndices.ContainsKey(region)) return true;
            }

            return false;
        }

        private static bool RemoveVertexTriangles(Transform transform, MultiRegionMeshInfo meshInfo, int vertexIndex,
                                                  int firstVertex, int lastVertex, int region) {
            
            List<int> vertexOrder = RemoveTrianglesAndGetEdgeRing(transform, meshInfo, vertexIndex, firstVertex, 
                                                                  lastVertex, region);
            if (vertexOrder == null) return false;
            if (vertexOrder.Count < 3) {
                DrawDebugVertexOrder(meshInfo, transform, vertexIndex, vertexOrder);
                Debug.LogError($"RemoveTrianglesAndGetEdgeRing returned vertex order with count < 3 when removing {vertexIndex}.");
                return false;
            }

            Vector3 v = meshInfo.Vertices[vertexIndex];
            Vector3 v1 = meshInfo.Vertices[vertexOrder[0]];
            Vector3 v2 = meshInfo.Vertices[vertexOrder[1]];

            // Get the normal of the new triangles we are creating.
            Vector3 normal = Vector3.Cross(v1 - v, v2 - v).normalized;

            // Ear clipping algorithm:
            // While there are enough points to make a triangle, choose the sharpest point and make a triangle
            // with that point and its neighbors, then remove that point from the list and repeat.
            // Avoid getting into a situation where the only triangles we can create are slivers.
            HashSet<int> flatVertices = new HashSet<int>();
            HashSet<int> concaveVertices = new HashSet<int>();
            while (vertexOrder.Count >= 3) {
                int bestVertex = -1;
                float bestDot = -1;

                // Find any vertices which, if clipped, would create a sliver triangle.
                // These are any vertices with an angle close to 180 degrees.
                flatVertices.Clear();
                concaveVertices.Clear();
                for (int j = 0; j < vertexOrder.Count; j++) {
                    GetCurrentAndNeighboringVertexIndices(vertexOrder, j, out int curVertexIndex,
                                                          out int prevVertexIndex, out int nextVertexIndex);

                    float dot = GetDotAndCrossProduct(meshInfo, curVertexIndex, prevVertexIndex, nextVertexIndex, out Vector3 cross);
                    if (dot < -0.99999) {
                        flatVertices.Add(curVertexIndex);
                        continue;
                    }

                    float crossDot = Vector3.Dot(cross, normal);
                    if (crossDot < 0) {
                        concaveVertices.Add(curVertexIndex);
                    }
                }

                // Find the angle with the sharpest point.
                // Do not consider any nearly-float angles.
                // If there are nearly-flat angles, we will choose a point adjacent to one of them.
                for (int j = 0; j < vertexOrder.Count; j++) {
                    GetCurrentAndNeighboringVertexIndices(vertexOrder, j, out int curVertexIndex,
                                                          out int prevVertexIndex, out int nextVertexIndex);

                    Vector3 vCur = meshInfo.Vertices[curVertexIndex];
                    Vector3 vNext = meshInfo.Vertices[nextVertexIndex];
                    Vector3 vPrev = meshInfo.Vertices[prevVertexIndex];
                    
                    if (flatVertices.Count > 0) {
                        if (flatVertices.Contains(curVertexIndex)) continue;
                        if (!flatVertices.Contains(prevVertexIndex) &&
                            !flatVertices.Contains(nextVertexIndex)) {
                            continue;
                        }
                    }

                    if (concaveVertices.Count > 0) {
                        if (concaveVertices.Contains(curVertexIndex)) continue;

                        bool anyInside = false;
                        foreach (int concaveVertexIndex in concaveVertices) {
                            if (concaveVertexIndex == prevVertexIndex || concaveVertexIndex == nextVertexIndex) continue;
                            Vector3 concaveVertex = meshInfo.Vertices[concaveVertexIndex];

                            if (MathUtil.IsPointInsideBound(vNext, vPrev, normal, concaveVertex) &&
                                MathUtil.IsPointInsideBound(vPrev, vCur, normal, concaveVertex) &&
                                MathUtil.IsPointInsideBound(vCur, vNext, normal, concaveVertex)) {
                                anyInside = true;
                                break;
                            }
                        }
                        
                        if (anyInside) continue;
                    }

                    float dot = GetDotAndCrossProduct(meshInfo, curVertexIndex, prevVertexIndex, nextVertexIndex, out _);
                    if (dot > bestDot) {
                        bestVertex = j;
                        bestDot = dot;
                    }
                }

                if (bestVertex == -1) {
                    Debug.LogError($"Did not find vertex to clip while removing {vertexIndex}.");
                    Debug.LogError($"Edge Ring: [{string.Join(", ", vertexOrder)}]");
                    DrawDebugVertexOrder(meshInfo, transform, vertexIndex, vertexOrder);
                    return false;
                }

                GetCurrentAndNeighboringVertexIndices(vertexOrder, bestVertex, out int newV1,
                                                      out int newV2, out int newV3);

                // Create new triangle and initialize it in the mesh data.
                Triangle newTriangle = new Triangle(newV1, newV2, newV3);
                if (!meshInfo.TriangleIndicesPerRegion.TryGetValue(
                        newTriangle, out Dictionary<int, int> triangleIndices)) {
                    triangleIndices = new Dictionary<int, int>();
                    meshInfo.TriangleIndicesPerRegion.Add(newTriangle, triangleIndices);
                }

                // Add triangle's index to cached values.
                triangleIndices[region] = meshInfo.RegionTriangleLists[region].Count;

                // Add indices so triangle is part of mesh.
                meshInfo.RegionTriangleLists[region].Add(newV1);
                meshInfo.RegionTriangleLists[region].Add(newV2);
                meshInfo.RegionTriangleLists[region].Add(newV3);

                // Add new vertex connections.
                ConnectVertices(meshInfo, newV1, newV2);
                ConnectVertices(meshInfo, newV2, newV3);
                ConnectVertices(meshInfo, newV3, newV1);

                vertexOrder.RemoveAt(bestVertex);
            }

            return true;
        }

        private static List<int> RemoveTrianglesAndGetEdgeRing(Transform transform, MultiRegionMeshInfo meshInfo,
                                                               int vertexIndex, int firstVertex, int lastVertex, int region) {
            int currentConnectedVertex = firstVertex;
            List<int> vertexOrder = new List<int>();
            while (true) {
                vertexOrder.Add(currentConnectedVertex);
                if (currentConnectedVertex == lastVertex) break;
                
                List<int> sharedVertices = meshInfo.VertexConnections[vertexIndex]
                                                   .Intersect(meshInfo.VertexConnections[currentConnectedVertex])
                                                   .ToList();

                bool foundNext = false;
                foreach (int sharedVertex in sharedVertices) {
                    if (sharedVertex != firstVertex && vertexOrder.Contains(sharedVertex)) continue;
                    Triangle triangle = new Triangle(vertexIndex, currentConnectedVertex, sharedVertex);
                    if (!meshInfo.TriangleIndicesPerRegion.TryGetValue(
                            triangle, out Dictionary<int, int> triangleRegions) ||
                        !triangleRegions.TryGetValue(region, out int triangleStartIndex)) continue;

                    triangleRegions.Remove(region);
                    meshInfo.RegionTriangleLists[region][triangleStartIndex + 0] = -1;
                    meshInfo.RegionTriangleLists[region][triangleStartIndex + 1] = -1;
                    meshInfo.RegionTriangleLists[region][triangleStartIndex + 2] = -1;
                    
                    currentConnectedVertex = sharedVertex;
                    foundNext = true;
                    break;
                }

                if (!foundNext) {
                    DrawDebugVertexOrder(meshInfo, transform, vertexIndex, vertexOrder);
                    Debug.LogError($"Error finding next edge, vertex = {vertexIndex}, region = {region}");
                    return null;
                }

                if (currentConnectedVertex == firstVertex) break;
            }

            return vertexOrder;
        }

        private static void DrawDebugVertexOrder(MultiRegionMeshInfo meshInfo, Transform transform, int vertexIndex, List<int> vertexOrder) {
            if (vertexOrder.Count <= 0) return;
            Debug.DrawLine(transform.TransformPoint(meshInfo.Vertices[vertexIndex]),
                           transform.TransformPoint(meshInfo.Vertices[vertexOrder[0]]),
                           Color.green, 10);
            for (int i = 0; i < vertexOrder.Count - 1; i++) {
                Debug.DrawLine(transform.TransformPoint(meshInfo.Vertices[vertexOrder[i]]),
                               transform.TransformPoint(meshInfo.Vertices[vertexOrder[i + 1]]),
                               Color.green, 10);
            }
        }

        private static float GetDotAndCrossProduct(MultiRegionMeshInfo meshInfo, int curVertexIndex, int prevVertexIndex,
                                           int nextVertexIndex, out Vector3 crossProduct) {
            Vector3 curVertex = meshInfo.Vertices[curVertexIndex];
            Vector3 prevVertex = meshInfo.Vertices[prevVertexIndex];
            Vector3 nextVertex = meshInfo.Vertices[nextVertexIndex];

            Vector3 v1 = Vector3.Normalize(nextVertex - curVertex);
            Vector3 v2 = Vector3.Normalize(prevVertex - curVertex);

            float dot = Vector3.Dot(v1, v2);
            crossProduct = Vector3.Cross(v1, v2);
            return dot;
        }

        private static void GetCurrentAndNeighboringVertexIndices(List<int> vertexOrder, int index, out int curVertexIndex, 
                                                                  out int prevVertexIndex, out int nextVertexIndex) {
            curVertexIndex = vertexOrder[index];
            prevVertexIndex = vertexOrder[(index + vertexOrder.Count - 1) % vertexOrder.Count];
            nextVertexIndex = vertexOrder[(index + 1) % vertexOrder.Count];
        }

        private static void ConnectVertices(MultiRegionMeshInfo meshInfo, int vertex1Index, int vertex2Index) {
            if (!meshInfo.VertexConnections[vertex1Index].Contains(vertex2Index)) {
                meshInfo.VertexConnections[vertex1Index].Add(vertex2Index);
            }
            
            if (!meshInfo.VertexConnections[vertex2Index].Contains(vertex1Index)) {
                meshInfo.VertexConnections[vertex2Index].Add(vertex1Index);
            }
        }

        private static bool IsEdgeAngled(in MultiRegionMeshInfo meshInfo, int vertex1Index, int vertex2Index,
                                         out Vector3 o1, out Vector3 o2) {
            Vector3 vertex1 = meshInfo.Vertices[vertex1Index];
            Vector3 vertex2 = meshInfo.Vertices[vertex2Index];

            List<int> connections1 = meshInfo.VertexConnections[vertex1Index];
            List<int> connections2 = meshInfo.VertexConnections[vertex2Index];

            List<int> sharedVertices = connections1.Intersect(connections2).ToList();

            for (int k = 0; k < sharedVertices.Count; k++) {
                int otherIndex1 = sharedVertices[k];
                if (!meshInfo.TriangleIndicesPerRegion.ContainsKey(new Triangle(vertex1Index, vertex2Index, otherIndex1))) {
                    continue;
                }
                
                for (int l = k + 1; l < sharedVertices.Count; l++) {
                    int otherIndex2 = sharedVertices[l];
                    if (!meshInfo.TriangleIndicesPerRegion.ContainsKey(new Triangle(vertex1Index, vertex2Index, otherIndex2))) {
                        continue;
                    }

                    if (otherIndex2 == otherIndex1 || 
                        otherIndex2 == vertex1Index || 
                        otherIndex2 == vertex2Index || 
                        otherIndex1 == vertex1Index || 
                        otherIndex1 == vertex2Index || 
                        vertex1Index == vertex2Index) {
                        
                        Debug.LogError("Duplicate vertex found.");
                    }
                    
                    Vector3 otherVertex1 = meshInfo.Vertices[otherIndex1];
                    Vector3 otherVertex2 = meshInfo.Vertices[otherIndex2];

                    Vector3 normal1 = Vector3.Cross(vertex2 - vertex1, otherVertex1 - vertex1).normalized;
                    Vector3 normal2 = Vector3.Cross(otherVertex2 - vertex1, vertex2 - vertex1).normalized;

                    float dot = Mathf.Abs(Vector3.Dot(normal1, normal2));
                    if (dot < 0.95f) {
                        o1 = otherVertex1;
                        o2 = otherVertex2;
                        return true;
                    }
                }
            }

            o1 = default;
            o2 = default;
            return false;
        }

        private static void TriangulateBasins(Vector3Int voxelCounts, int[,,] basins, out MultiRegionMeshInfo mesh, NavVolume volume) {
            mesh = MultiRegionMeshInfo.CreateEmptyInfo();
            Dictionary<Vector3, int> vertexIndices = new Dictionary<Vector3, int>();

            int regionIndex = 0;

            while (true) {
                bool regionExists = false;
                List<int> triList = null;
                for (int x = -1; x < voxelCounts.x; x++) {
                    for (int y = -1; y < voxelCounts.y; y++) {
                        for (int z = -1; z < voxelCounts.z; z++) {
                            Vector3Int current = new Vector3Int(x, y, z);
                            Vector3 currentV = current + Vector3.one * 0.5f;
                            byte caseIndex = GetMarchingCubesIndex(voxelCounts, basins, regionIndex, current);
                            if (caseIndex != 0) regionExists = true;

                            if (triList == null) {
                                triList = new List<int>();
                                mesh.RegionTriangleLists.Add(triList);
                            }

                            byte[] edgeIndices = MarchingCubesTables.TriTable[caseIndex];

                            int triCount = edgeIndices.Length / 3;

                            for (int triIndex = 0; triIndex < triCount; triIndex++) {
                                int triStart = triIndex * 3;

                                int crossPoint1 = -1;
                                int crossPoint2 = -1;
                                int nonCrossPoint = -1;
                                for (int i = 0; i < 3; i++) {
                                    int n = (i + 1) % 3;
                                    int o = (i + 2) % 3;

                                    if (MarchingCubesTables.AcrossCenterMidpoints[edgeIndices[triStart + i]] != edgeIndices[triStart + n])
                                        continue;
                                    
                                    crossPoint1 = triStart + i;
                                    crossPoint2 = triStart + n;
                                    nonCrossPoint = triStart + o;
                                    break;
                                }
                                
                                if (crossPoint1 == -1) {
                                    Vector3 vertex1 = currentV + GetMarchingCubesVertex(mesh, edgeIndices, triStart + 0);
                                    Vector3 vertex2 = currentV + GetMarchingCubesVertex(mesh, edgeIndices, triStart + 1);
                                    Vector3 vertex3 = currentV + GetMarchingCubesVertex(mesh, edgeIndices, triStart + 2);

                                    int vertex1Index = AddVertex(mesh, vertex1, volume, vertexIndices);
                                    int vertex2Index = AddVertex(mesh, vertex2, volume, vertexIndices);
                                    int vertex3Index = AddVertex(mesh, vertex3, volume, vertexIndices);
                                    
                                    AddTriangle(mesh, triList, regionIndex, vertex1Index, vertex2Index, vertex3Index);
                                } else {
                                    Vector3 vertexCross1 = currentV + GetMarchingCubesVertex(mesh, edgeIndices, crossPoint1);
                                    Vector3 vertexCross2 = currentV + GetMarchingCubesVertex(mesh, edgeIndices, crossPoint2);
                                    Vector3 vertexNonCross = currentV + GetMarchingCubesVertex(mesh, edgeIndices, nonCrossPoint);
                                    Vector3 vertexCenter = currentV + Vector3.one * 0.5f;

                                    int vertexCross1Index = AddVertex(mesh, vertexCross1, volume, vertexIndices);
                                    int vertexCross2Index = AddVertex(mesh, vertexCross2, volume, vertexIndices);
                                    int vertexNonCrossIndex = AddVertex(mesh, vertexNonCross, volume, vertexIndices);
                                    int vertexCenterIndex = AddVertex(mesh, vertexCenter, volume, vertexIndices);

                                    AddTriangle(mesh, triList, regionIndex, vertexCross1Index,
                                                vertexCenterIndex, vertexNonCrossIndex);
                                    AddTriangle(mesh, triList, regionIndex, vertexCenterIndex,
                                        vertexCross2Index, vertexNonCrossIndex);
                                }
                            }
                        }
                    }
                }

                regionIndex++;
                if (!regionExists) break;
            }
        }

        private static byte GetMarchingCubesIndex(Vector3Int voxelCounts, int[,,] basins, int basinID,
                                                  Vector3Int basePos) {
            return GetMarchingCubesIndex(voxelCounts, pos => basins[pos.x, pos.y, pos.z] == basinID, basePos);
        }

        private static byte GetMarchingCubesIndex(Vector3Int voxelCounts, Func<Vector3Int, bool> filledFunc,
                                                  Vector3Int basePos) {
            byte caseIndex = 0;

            for (int i = 0; i < MarchingCubesTables.Vertices.Length; i++) {
                Vector3Int vertex = basePos + MarchingCubesTables.Vertices[i];
                if (IsOutOfBounds(voxelCounts, vertex)) continue;
                if (filledFunc(vertex)) {
                    caseIndex |= (byte)(1 << i);
                }
            }

            return caseIndex;
        }

        private static Vector3 GetMarchingCubesVertex(MultiRegionMeshInfo mesh, byte[] edgeIndices, int edge) {
            byte edgeIndex = edgeIndices[edge];

            byte vertex1Index = MarchingCubesTables.EdgeToVertexIndices[edgeIndex, 0];
            byte vertex2Index = MarchingCubesTables.EdgeToVertexIndices[edgeIndex, 1];

            Vector3Int vertex1 = MarchingCubesTables.Vertices[vertex1Index];
            Vector3Int vertex2 = MarchingCubesTables.Vertices[vertex2Index];

            return (Vector3) (vertex1 + vertex2) / 2.0f;
        }

        private static int AddVertex(MultiRegionMeshInfo mesh, Vector3 vertex, NavVolume volume,
                                     Dictionary<Vector3, int> vertexIndices) {
            if (!vertexIndices.TryGetValue(vertex, out int vertexIndex)) {
                vertexIndex = mesh.Vertices.Count;
                mesh.Vertices.Add(volume.Bounds.min + (vertex * volume.VoxelSize));
                mesh.VertexConnections.Add(new List<int>());
                mesh.VertexRegionMembership.Add(new List<int>());
                vertexIndices.Add(vertex, vertexIndex);
            }

            return vertexIndex;
        }

        private static void AddTriangle(MultiRegionMeshInfo mesh, List<int> triList, int region,
                                        int vertex1Index, int vertex2Index, int vertex3Index) {
            int firstIndex = triList.Count;
            
            triList.Add(vertex1Index);
            triList.Add(vertex2Index);
            triList.Add(vertex3Index);

            AddVertexToRegion(mesh, vertex1Index, region);
            AddVertexToRegion(mesh, vertex2Index, region);
            AddVertexToRegion(mesh, vertex3Index, region);
            
            ConnectVertices(mesh, vertex1Index, vertex2Index);
            ConnectVertices(mesh, vertex2Index, vertex3Index);
            ConnectVertices(mesh, vertex3Index, vertex1Index);
            
            Triangle triangle = new Triangle(vertex1Index, vertex2Index, vertex3Index);
            if (!mesh.TriangleIndicesPerRegion.TryGetValue(triangle, out Dictionary<int, int> triangleMeshes)) {
                triangleMeshes = new Dictionary<int, int>();
                mesh.TriangleIndicesPerRegion.Add(triangle, triangleMeshes);
            }
            
            triangleMeshes[region] = firstIndex;
        }

        public static void AddVertexToRegion(MultiRegionMeshInfo mesh, int vertex, int region) {
            if (!mesh.VertexRegionMembership[vertex].Contains(region)) {
                mesh.VertexRegionMembership[vertex].Add(region);
            }
        }

        private static int[,,] CalculateBasinEdges(Vector3Int voxelCounts, int[,,] basins) {
            int[,,] edges = new int[voxelCounts.x, voxelCounts.y, voxelCounts.z];

            for (int x = 0; x < voxelCounts.x; x++) {
                for (int y = 0; y < voxelCounts.y; y++) {
                    for (int z = 0; z < voxelCounts.z; z++) {
                        if (IsBasinEdge(voxelCounts, basins, new Vector3Int(x, y, z))) {
                            edges[x, y, z] = basins[x, y, z];
                        } else {
                            edges[x, y, z] = -1;
                        }
                    }
                }
            }
            
            return edges;
        }

        private static bool IsBasinEdge(Vector3Int voxelCounts, int[,,] basins, Vector3Int pos) {
            int basinID = basins[pos.x, pos.y, pos.z];
            if (basinID < 0) {
                return false;
            }

            foreach (Vector3Int dir in NeighborDirections) {
                Vector3Int n = pos + dir;

                if (IsOutOfBounds(voxelCounts, n) || basins[n.x, n.y, n.z] != basinID) {
                    return true;
                }
            }

            return false;
        }

        private static void CombineRegionsWherePossible(Vector3Int voxelCounts, int[,,] basins, ref int regionCount,
                                                        NavVolume volume) {

            bool combinedAnyRegion;
            HashSet<int> goneRegions = new HashSet<int>();
            HashSet<Edge> checkedPairs = new HashSet<Edge>();
            do {
                combinedAnyRegion = false;
                
                for (int regionId = 0; regionId < regionCount; regionId++) {
                    if (goneRegions.Contains(regionId)) continue;
                    HashSet<int> adjacent = GetRegionsAdjacentToRegion(voxelCounts, basins, regionId);
                    foreach (int otherRegion in adjacent) {
                        // We only need to check each pair of regions once.
                        if (goneRegions.Contains(otherRegion) ||
                            !checkedPairs.Add(new Edge(regionId, otherRegion))) continue;
                        
                        if (CanCombineRegions(voxelCounts, basins, regionId, otherRegion)) {
                            goneRegions.Add(regionId);
                            CombineRegions(voxelCounts, basins, regionId, otherRegion);
                            combinedAnyRegion = true;
                            
                            // Region has changed so we need to recheck pairs containing it.
                            checkedPairs.RemoveWhere(pair => 
                                                         pair.Vertex1 == otherRegion || pair.Vertex2 == otherRegion);
                        }
                    }

                    if (combinedAnyRegion) break;
                }
                
            } while (combinedAnyRegion);

            int newRegionCount = regionCount - goneRegions.Count;

            // Shift all region indices down by number of lower regions that were removed.
            Dictionary<int, int> regionMap = new Dictionary<int, int>();
            Dictionary<int, int> counts = new Dictionary<int, int>();
            for (int x = 0; x < voxelCounts.x; x++) {
                for (int y = 0; y < voxelCounts.y; y++) {
                    for (int z = 0; z < voxelCounts.z; z++) {
                        int region = basins[x, y, z];
                        if (region < 0) continue;
                        if (goneRegions.Contains(region)) {
                            Debug.LogError($"Found voxel in region {region} which is supposed to be gone");
                        }
                        if (!regionMap.TryGetValue(region, out int newRegion)) {
                            newRegion = region - goneRegions.Count(r => r < region);

                            if (newRegion >= newRegionCount) {
                                Debug.LogError($"Region {newRegion} greater than region count");
                            }

                            regionMap[region] = newRegion;
                        }

                        counts.TryGetValue(newRegion, out int count);
                        count++;
                        counts[newRegion] = count;

                        basins[x, y, z] = newRegion;
                    }
                }
            }

            Debug.Log($"Reduced region count by {goneRegions.Count}, from {regionCount} to {newRegionCount}.");

            regionCount = newRegionCount;
        }

        private static void CombineRegions(Vector3Int voxelCounts, int[,,] basins, int region1, int region2) {
            for (int x = 0; x < voxelCounts.x; x++) {
                for (int y = 0; y < voxelCounts.y; y++) {
                    for (int z = 0; z < voxelCounts.z; z++) {
                        if (basins[x, y, z] == region1) {
                            basins[x, y, z] = region2;
                        }
                    }
                }
            }
        }

        private static bool CanCombineRegions(Vector3Int voxelCounts, int[,,] basins, int region1, int region2) {
            bool Predicate(Vector3Int pos) {
                int value = basins[pos.x, pos.y, pos.z];
                return value == region1 || value == region2;
            }
            
            for (int x = 0; x < voxelCounts.x - 1; x++) {
                for (int y = 0; y < voxelCounts.y - 1; y++) {
                    for (int z = 0; z < voxelCounts.z - 1; z++) {
                        Vector3Int pos = new Vector3Int(x, y, z);
                        byte cube = GetMarchingCubesIndex(voxelCounts, Predicate, pos);
                        if (MarchingCubesCavityTables.CubesWithInternalCavities[cube]) {
                            return false;
                        }
                    }
                }
            }
            
            // Look for cubes with neighbor concavities and split between them.
            for (int dir = 0; dir < MarchingCubesTables.PositiveDirections.Length; dir++) {
                Vector3Int dirVector = MarchingCubesTables.PositiveDirections[dir];
                
                for (int x = 0; x < voxelCounts.x - 1; x++) {
                    for (int y = 0; y < voxelCounts.y - 1; y++) {
                        for (int z = 0; z < voxelCounts.z - 1; z++) {
                            Vector3Int selfPos = new Vector3Int(x, y, z);
                            
                            byte selfCube = GetMarchingCubesIndex(voxelCounts, Predicate, selfPos);
                            if (selfCube == 0 || selfCube == 255) continue;

                            int[] concaveNeighbors = MarchingCubesCavityTables.CubeConcaveNeighbors[selfCube][dir];
                            if (concaveNeighbors.Length == 0) continue;
                            
                            Vector3Int neighborPos = selfPos + dirVector;
                            if (IsOutOfBounds(voxelCounts, neighborPos + Vector3Int.one)) continue;
                            
                            byte neighborCube = GetMarchingCubesIndex(voxelCounts, Predicate, neighborPos);
                            if (Array.IndexOf(concaveNeighbors, neighborCube) >= 0) {
                                return false;
                            }
                        }
                    }
                }
            }
            
            return true;
        }

        private static HashSet<int> GetRegionsAdjacentToRegion(Vector3Int voxelCounts, int[,,] basins, int region) {

            HashSet<int> adjacent = new HashSet<int>();
            for (int x = 0; x < voxelCounts.x; x++) {
                for (int y = 0; y < voxelCounts.y; y++) {
                    for (int z = 0; z < voxelCounts.z; z++) {
                        int curRegion = basins[x, y, z];
                        if (adjacent.Contains(curRegion) || curRegion == region) continue;

                        Vector3Int pos = new Vector3Int(x, y, z);
                        foreach (Vector3Int dir in NeighborDirections) {
                            Vector3Int n = pos + dir;
                            if (IsOutOfBounds(voxelCounts, n)) continue;
                            int nRegion = basins[n.x, n.y, n.z];
                            if (nRegion == region) {
                                adjacent.TryGetValue(curRegion, out int count);
                                count++;
                                adjacent.Add(curRegion);
                                break;
                            }
                        }
                    }
                }
            }

            return adjacent;
        }

        private static void ConvexifyAllRegions(Vector3Int voxelCounts, int[,,] basins, ref int regionCount, NavVolume volume) {
            for (int i = 0; i < regionCount; i++) {
                ConvexifyRegion(voxelCounts, basins, i, ref regionCount, volume);
                if (regionCount > 200) {
                    Debug.LogError("Way too many regions, probably an infinite loop, aborting");
                    break;
                }
            }
        }

        private static void ConvexifyRegion(Vector3Int voxelCounts, int[,,] basins, int basinId, ref int regionCount, NavVolume volume) {
            // Look for cubes with internal concavities and split at those points.
            for (int x = 0; x < voxelCounts.x - 1; x++) {
                for (int y = 0; y < voxelCounts.y - 1; y++) {
                    for (int z = 0; z < voxelCounts.z - 1; z++) {
                        Vector3Int pos = new Vector3Int(x, y, z);
                        byte cube = GetMarchingCubesIndex(voxelCounts, basins, basinId, pos);
                        if (MarchingCubesCavityTables.CubesWithInternalCavities[cube]) {
                            if (basinId > 100) {
                                Vector3 debugPos = volume.transform.TransformPoint(volume.Bounds.min + (Vector3) pos * volume.VoxelSize);
                                Utility.DrawDebugBounds(new Bounds(debugPos + Vector3.one, Vector3.one), Color.green, 10);
                            }
                            SplitRegionForInternalConcavity(voxelCounts, basins, basinId, ref regionCount, volume, pos);
                        }
                    }
                }
            }
            
            // Look for cubes with neighbor concavities and split between them.
            for (int dir = 0; dir < MarchingCubesTables.PositiveDirections.Length; dir++) {
                Vector3Int dirVector = MarchingCubesTables.PositiveDirections[dir];
                
                for (int x = 0; x < voxelCounts.x - 1; x++) {
                    for (int y = 0; y < voxelCounts.y - 1; y++) {
                        for (int z = 0; z < voxelCounts.z - 1; z++) {
                            Vector3Int selfPos = new Vector3Int(x, y, z);
                            
                            byte selfCube = GetMarchingCubesIndex(voxelCounts, basins, basinId, selfPos);
                            if (selfCube == 0 || selfCube == 255) continue;

                            int[] concaveNeighbors = MarchingCubesCavityTables.CubeConcaveNeighbors[selfCube][dir];
                            if (concaveNeighbors.Length == 0) continue;
                            
                            Vector3Int neighborPos = selfPos + dirVector;
                            if (IsOutOfBounds(voxelCounts, neighborPos + Vector3Int.one)) continue;
                            
                            byte neighborCube = GetMarchingCubesIndex(voxelCounts, basins, basinId, neighborPos);
                            if (Array.IndexOf(concaveNeighbors, neighborCube) >= 0) {
                                if (basinId > 100) {
                                    Vector3 debugPos = volume.transform.TransformPoint(volume.Bounds.min + (Vector3) selfPos * volume.VoxelSize);
                                    Vector3 debugPos2 = volume.transform.TransformPoint(volume.Bounds.min + (Vector3) neighborPos * volume.VoxelSize);
                                    Utility.DrawDebugBounds(new Bounds(debugPos + Vector3.one, Vector3.one), Color.green, 10);
                                    Debug.DrawLine(debugPos, debugPos2, Color.green, 10);
                                }
                                SplitRegionForNeighborConcavity(voxelCounts, basins, basinId, ref regionCount,
                                                                volume, selfPos, dir);
                            }
                        }
                    }
                }
            }
        }

        private static void SplitRegionForNeighborConcavity(Vector3Int voxelCounts, int[,,] basins, int basinId, ref int regionCount, 
                                                            NavVolume volume, Vector3Int pos, int dirIndex) {
            
            Vector3 debugPos = volume.transform.TransformPoint(volume.Bounds.min + (Vector3) pos * volume.VoxelSize);
            //Util.DrawDebugBounds(new Bounds(debugPos + Vector3.one, Vector3.one), Color.blue, 10);
            Vector3 debugPos2 = volume.transform.TransformPoint(
                volume.Bounds.min + (Vector3) (pos + MarchingCubesTables.PositiveDirections[dirIndex]) *
                volume.VoxelSize);
            //Util.DrawDebugBounds(new Bounds(debugPos2 + Vector3.one, Vector3.one), Color.red, 10);

            Vector3Int[] dirs = MarchingCubesTables.PositiveDirections;
            Vector3Int zAxis = dirs[dirIndex];
            Vector3Int xAxis = dirs[(dirIndex + 1) % dirs.Length];
            Vector3Int yAxis = dirs[(dirIndex + 2) % dirs.Length];

            int vz = Utility.Dot(pos, zAxis);

            int voxelsVx = Utility.Dot(voxelCounts, xAxis);
            int voxelsVy = Utility.Dot(voxelCounts, yAxis);
            int voxelsVz = Utility.Dot(voxelCounts, zAxis);

            int split1 = GetBrokenCubeCount(basins, basinId, vz + 1, xAxis, yAxis, zAxis,
                                            new Vector2Int(voxelsVx, voxelsVy));
            int split2 = GetBrokenCubeCount(basins, basinId, vz + 2, xAxis, yAxis, zAxis,
                                            new Vector2Int(voxelsVx, voxelsVy));

            if (split1 <= split2) {
                SplitRegion(basins, basinId, vz + 1, xAxis, yAxis, zAxis,
                            new Vector3Int(voxelsVx, voxelsVy, voxelsVz),
                            ref regionCount, volume, pos + Vector3Int.one);
            } else {
                SplitRegion(basins, basinId, vz + 2, xAxis, yAxis, zAxis,
                            new Vector3Int(voxelsVx, voxelsVy, voxelsVz),
                            ref regionCount, volume, pos + Vector3Int.one + zAxis);
            }
        }

        private static void SplitRegionForInternalConcavity(Vector3Int voxelCounts, int[,,] basins, int basinId, ref int regionCount,
                                               NavVolume volume, Vector3Int pos) {
            Vector3 debugPos = volume.transform.TransformPoint(volume.Bounds.min + (Vector3) pos * volume.VoxelSize);
            //Util.DrawDebugBounds(new Bounds(debugPos + Vector3.one, Vector3.one), Color.blue, 10);
            int cube = GetMarchingCubesIndex(voxelCounts, basins, basinId, pos);

            int xSplit = IsCubeBrokenOnAxis(cube, 0)
                ? GetBrokenCubeCount(basins, basinId, pos.x + 1,
                                     Vector3Int.forward, Vector3Int.up, Vector3Int.right,
                                     new Vector2Int(voxelCounts.z, voxelCounts.y))
                : int.MaxValue;
            
            int ySplit = IsCubeBrokenOnAxis(cube, 1)
                ? GetBrokenCubeCount(basins, basinId, pos.y + 1,
                                     Vector3Int.right, Vector3Int.forward, Vector3Int.up,
                                     new Vector2Int(voxelCounts.x, voxelCounts.z))
                : int.MaxValue;

            int zSplit = IsCubeBrokenOnAxis(cube, 2)
                ? GetBrokenCubeCount(basins, basinId, pos.z + 1,
                                     Vector3Int.right, Vector3Int.up, Vector3Int.forward,
                                     new Vector2Int(voxelCounts.x, voxelCounts.y))
                : int.MaxValue;

            if (xSplit <= ySplit && xSplit <= zSplit) {
                SplitRegion(basins, basinId, pos.x + 1,
                            Vector3Int.forward, Vector3Int.up, Vector3Int.right,
                            new Vector3Int(voxelCounts.z, voxelCounts.y, voxelCounts.x),
                            ref regionCount, volume, pos + Vector3Int.one);
            } else if (ySplit <= xSplit && ySplit <= zSplit) {
                SplitRegion(basins, basinId, pos.y + 1,
                            Vector3Int.right, Vector3Int.forward, Vector3Int.up,
                            new Vector3Int(voxelCounts.x, voxelCounts.z, voxelCounts.y),
                            ref regionCount, volume, pos + Vector3Int.one);
            } else {
                SplitRegion(basins, basinId, pos.z + 1,
                            Vector3Int.right, Vector3Int.up, Vector3Int.forward,
                            new Vector3Int(voxelCounts.x, voxelCounts.y, voxelCounts.z),
                            ref regionCount, volume, pos + Vector3Int.one);
            }
        }

        private static HashSet<Vector3Int> GetContiguousIfSplit(Vector3Int voxelCounts, int[,,] basins, int basinId,
                                                                Vector3Int start,
                                                                Vector3Int xAxis, Vector3Int yAxis, Vector3Int zAxis) {

            HashSet<Vector3Int> result = new HashSet<Vector3Int>();
            Queue<Vector3Int> queue = new Queue<Vector3Int>();

            int z = Utility.Dot(start, zAxis);
            queue.Enqueue(start);
            queue.Enqueue(start + xAxis);
            queue.Enqueue(start + yAxis);
            queue.Enqueue(start + xAxis + yAxis);
            
            while (queue.TryDequeue(out Vector3Int cur)) {
                int cz = Utility.Dot(cur, zAxis);
                if (!result.Add(cur) || IsOutOfBounds(voxelCounts, cur) || cz > z ||
                    basins[cur.x, cur.y, cur.z] != basinId) {
                    continue;
                }
                
                foreach (Vector3Int dir in NeighborDirections) {
                    Vector3Int n = cur + dir;
                    queue.Enqueue(n);
                }
            }

            return result;
        }

        private static void SplitRegion(int[,,] basins, int basinId, int startZ,
                                        Vector3Int xAxis, Vector3Int yAxis, Vector3Int zAxis,
                                        Vector3Int axisLimits, ref int regionCount,
                                        NavVolume volume, Vector3Int startPos) {

            Vector3Int voxelCounts = new Vector3Int(
                basins.GetLength(0), 
                basins.GetLength(1), 
                basins.GetLength(2));

            HashSet<Vector3Int> contiguous =
                GetContiguousIfSplit(voxelCounts, basins, basinId, startPos - Vector3Int.one, xAxis, yAxis, zAxis);
            
            Vector3 normal = volume.transform.TransformDirection(zAxis);
            Vector3 debugPos = volume.transform.TransformPoint(volume.Bounds.min + (Vector3)startPos * volume.VoxelSize);
            //Debug.DrawLine(debugPos, debugPos + normal, Color.cyan, 10);

            int newRegion = regionCount++;
            for (int vx = 0; vx < axisLimits.x; vx++) {
                for (int vy = 0; vy < axisLimits.y; vy++) {
                    for (int vz = 0; vz < axisLimits.z; vz++) {
                        Vector3Int pos = xAxis * vx + yAxis * vy + zAxis * vz;
                        bool split = vz >= startZ || !contiguous.Contains(pos);
                        if (split && basins[pos.x, pos.y, pos.z] == basinId) {
                            basins[pos.x, pos.y, pos.z] = newRegion;
                        }
                    }
                }
            }
        }

        private static int GetBrokenCubeCount(int[,,] basins, int basinId, int vz,
                                              Vector3Int xAxis, Vector3Int yAxis, Vector3Int zAxis,
                                              Vector2Int axisLimits) {

            int count = 0;
            Vector3Int zOffset = vz * zAxis;
            Vector3Int[] sideASamples = new[] {
                -zAxis - xAxis - yAxis,
                -zAxis - xAxis,
                -zAxis - yAxis,
                -zAxis
            };

            Vector3Int[] sideBSamples = new[] {
                -xAxis - yAxis,
                -xAxis,
                -yAxis,
                Vector3Int.zero,
            };

            Vector3Int voxelCounts = new Vector3Int(basins.GetLength(0), basins.GetLength(1), basins.GetLength(2));
            for (int vx = 1; vx < axisLimits.x; vx++) {
                for (int vy = 1; vy < axisLimits.y; vy++) {
                    Vector3Int pos = zOffset + vx * xAxis + vy * yAxis;

                    bool hasSideA = sideASamples.Any(v => basins[pos.x + v.x, pos.y + v.y, pos.z + v.z] == basinId);
                    bool hasSideB = sideBSamples.Any(v => basins[pos.x + v.x, pos.y + v.y, pos.z + v.z] == basinId);

                    if (hasSideA && hasSideB) {
                        byte cubeIndex = GetMarchingCubesIndex(voxelCounts, basins, basinId, pos - Vector3Int.one);
                        if (MarchingCubesCavityTables.CubesWithInternalCavities[cubeIndex]) {
                            count--;
                        } else {
                            count++;
                        }
                    } else if (hasSideA || hasSideB) {
                        count--;
                    }
                }
            }
            
            return count;
        }

        private static bool IsCubeBrokenOnAxis(int cube, int axis) {
            byte[] sideAVerts = MarchingCubesTables.VerticesOnSideAPerDirection[axis];
            byte[] sideBVerts = MarchingCubesTables.VerticesOnSideBPerDirection[axis];
            
            bool hasSideA = sideAVerts.Any(v => (cube & (1 << v)) != 0);
            bool hasSideB = sideBVerts.Any(v => (cube & (1 << v)) != 0);

            return hasSideA && hasSideB;
        }

        private static int[,,] CalculateBasins(Vector3Int voxelCounts, int[,,] voxels, int maxDist, out int basinCount) {
            int[,,] basins = new int[voxelCounts.x, voxelCounts.y, voxelCounts.z];
            for (int x = 0; x < voxelCounts.x; x++) {
                for (int y = 0; y < voxelCounts.y; y++) {
                    for (int z = 0; z < voxelCounts.z; z++) {
                        basins[x, y, z] = -1;
                    }
                }
            }

            basinCount = 0;
            for (int waterLevel = maxDist; waterLevel > 0; waterLevel--) {
                // First check for any new basins - unassigned voxels below the water level.
                for (int x = 0; x < voxelCounts.x; x++) {
                    for (int y = 0; y < voxelCounts.y; y++) {
                        for (int z = 0; z < voxelCounts.z; z++) {
                            if (voxels[x, y, z] >= waterLevel && basins[x, y, z] < 0) {
                                int newBasinId = basinCount++;
                                basins[x, y, z] = newBasinId;
                                ExpandBasin(voxelCounts, new Vector3Int(x, y, z), waterLevel, voxels, basins);
                            }
                        }
                    }
                }
                
                // Once we hit water level 1 we don't need to expand anymore.
                // Doing so would expand into blocked areas.
                if (waterLevel == 1) continue;

                // Now expand any existing basins to the next water level.
                ExpandAllBasins(voxelCounts, waterLevel - 1, voxels, basins);
            }

            return basins;
        }

        private static int[,,] CalculateBasinsBasic(Vector3Int voxelCounts, int[,,] voxels, int maxDist, out int basinCount) {
            int[,,] basins = new int[voxelCounts.x, voxelCounts.y, voxelCounts.z];
            for (int x = 0; x < voxelCounts.x; x++) {
                for (int y = 0; y < voxelCounts.y; y++) {
                    for (int z = 0; z < voxelCounts.z; z++) {
                        basins[x, y, z] = -1;
                    }
                }
            }

            basinCount = 0;
            
            for (int x = 0; x < voxelCounts.x; x++) {
                for (int y = 0; y < voxelCounts.y; y++) {
                    for (int z = 0; z < voxelCounts.z; z++) {
                        if (voxels[x, y, z] == 0) continue; // Voxel is blocked
                        if (basins[x, y, z] >= 0) continue; // Basin is already set 

                        basins[x, y, z] = basinCount++;
                        ExpandBasin(voxelCounts, new Vector3Int(x, y, z), 1, voxels, basins);
                    }
                }
            }
            
            return basins;
        }

        private static void ExpandAllBasins(Vector3Int voxelCounts, int waterLevel, int[,,] voxels, int[,,] basins) {
            Queue<Vector3Int> toExplore = new Queue<Vector3Int>();

            for (int x = 0; x < voxelCounts.x; x++) {
                for (int y = 0; y < voxelCounts.y; y++) {
                    for (int z = 0; z < voxelCounts.z; z++) {
                        if (basins[x, y, z] < 0) continue;
                        toExplore.Enqueue(new Vector3Int(x, y, z));
                    }
                }
            }
            
            ExpandBasins(voxelCounts, waterLevel, voxels, basins, toExplore);
        }

        private static void ExpandBasin(Vector3Int voxelCounts, Vector3Int startPos, int waterLevel, int[,,] voxels, int[,,] basins) {
            Queue<Vector3Int> toExplore = new Queue<Vector3Int>();
            toExplore.Enqueue(startPos);

            ExpandBasins(voxelCounts, waterLevel, voxels, basins, toExplore);
        }

        private static void ExpandBasins(Vector3Int voxelCounts, int waterLevel, int[,,] voxels, int[,,] basins, Queue<Vector3Int> toExplore) {
            while (toExplore.TryDequeue(out Vector3Int current)) {
                int basin = basins[current.x, current.y, current.z];
                for (int i = 0; i < NeighborDirections.Length; i++) {
                    Vector3Int n = current + NeighborDirections[i];

                    if (IsOutOfBounds(voxelCounts, n)) {
                        continue;
                    }

                    if (voxels[n.x, n.y, n.z] < waterLevel) continue;
                    if (basins[n.x, n.y, n.z] >= 0) continue;

                    basins[n.x, n.y, n.z] = basin;
                    toExplore.Enqueue(n);
                }
            }
        }

        private static bool IsOutOfBounds(Vector3Int voxelCounts, Vector3Int n) {
            return n.x < 0 || n.x >= voxelCounts.x ||
                   n.y < 0 || n.y >= voxelCounts.y ||
                   n.z < 0 || n.z >= voxelCounts.z;
        }

        private static void BlurVoxelDistance(Vector3Int voxelCounts, ref int[,,] voxels, int radius) {
            int[,,] newVoxels = new int[voxelCounts.x, voxelCounts.y, voxelCounts.z];
            for (int x = 0; x < voxelCounts.x; x++) {
                for (int y = 0; y < voxelCounts.y; y++) {
                    for (int z = 0; z < voxelCounts.z; z++) {
                        int total = 0;
                        int num = 0;

                        if (voxels[x, y, z] == 0) {
                            newVoxels[x, y, z] = 0;
                            continue;
                        }

                        for (int x1 = Mathf.Max(x - radius, 0); x1 <= x + radius && x1 < voxelCounts.x; x1++) {
                            for (int y1 = Mathf.Max(y - radius, 0); y1 <= y + radius && y1 < voxelCounts.y; y1++) {
                                for (int z1 = Mathf.Max(z - radius, 0); z1 <= z + radius && z1 < voxelCounts.z; z1++) {
                                    if (voxels[x1, y1, z1] == 0) continue;
                                    
                                    num++;
                                    total += voxels[x1, y1, z1];
                                }
                            }
                        }

                        newVoxels[x, y, z] = Mathf.RoundToInt(total / (float) num);
                    }
                }
            }

            voxels = newVoxels;
        }

        private static int GetMaxDist(Vector3Int voxelCounts, int[,,] voxels) {
            int maxDist = 0;
            for (int x = 0; x < voxelCounts.x; x++) {
                for (int y = 0; y < voxelCounts.y; y++) {
                    for (int z = 0; z < voxelCounts.z; z++) {
                        int dist = voxels[x, y, z];
                        if (dist > maxDist) maxDist = dist;
                    }
                }
            }

            return maxDist;
        }

        private static void CalculateVoxelDistance(Vector3Int voxelCounts, int[,,] voxels) {
            Queue<Vector3Int> toExplore = new Queue<Vector3Int>();
            for (int x = 0; x < voxelCounts.x; x++) {
                for (int y = 0; y < voxelCounts.y; y++) {
                    for (int z = 0; z < voxelCounts.z; z++) {
                        if (voxels[x, y, z] == 0) {
                            toExplore.Enqueue(new Vector3Int(x, y, z));
                        }
                    }
                }
            }

            while (toExplore.TryDequeue(out Vector3Int current)) {
                for (int i = 0; i < NeighborDirections.Length; i++) {
                    Vector3Int n = current + NeighborDirections[i];

                    if (IsOutOfBounds(voxelCounts, n)) {
                        continue;
                    }

                    ref int dist = ref voxels[n.x, n.y, n.z];
                    if (dist >= 0) continue;

                    dist = voxels[current.x, current.y, current.z] + 1;
                    toExplore.Enqueue(n);
                }
            }
        }

        private static void CalculateBlockedVoxels(NavVolume volume, Vector3Int voxelCounts, int[,,] voxels) {
            for (int x = 0; x < voxelCounts.x; x++) {
                for (int y = 0; y < voxelCounts.y; y++) {
                    for (int z = 0; z < voxelCounts.z; z++) {
                        Vector3 voxelPos = volume.Bounds.min +
                                           new Vector3(x + 0.5f, y + 0.5f, z + 0.5f) * volume.VoxelSize;
                        Vector3 worldPoint = volume.transform.TransformPoint(voxelPos);
                        Collider[] hits = Physics.OverlapSphere(worldPoint, volume.MaxAgentRadius,
                                                                volume.BlockingLayers, QueryTriggerInteraction.Ignore);
                        if (hits.Length > 0) {
                            voxels[x, y, z] = 0;
                        } else {
                            voxels[x, y, z] = -1;
                        }
                    }
                }
            }
        }

        public static void BuildTriangulationPreviewMesh(NavVolume volume, IReadOnlyList<Vector3> vertices,
                                                         IReadOnlyList<IReadOnlyList<int>> triLists) {
            Mesh mesh = new Mesh();
            
            mesh.indexFormat = IndexFormat.UInt32;
            mesh.subMeshCount = triLists.Count * 2;
            mesh.SetVertices(vertices as List<Vector3> ?? vertices.ToList());
            for (int i = 0; i < triLists.Count; i++) {
                IReadOnlyList<int> triList = triLists[i];
                mesh.SetIndices(triList as List<int> ?? triList.ToList(), MeshTopology.Triangles, i);

                List<int> lineIndices = new List<int>();
                int triCount = triList.Count / 3;
                for (int j = 0; j < triCount; j++) {
                    int triBase = j * 3;
                    for (int k = 0; k < 3; k++) {
                        int index1 = triBase + k;
                        int index2 = triBase + ((k + 1) % 3);
                        
                        lineIndices.Add(triList[index1]);
                        lineIndices.Add(triList[index2]);
                    }
                }
                mesh.SetIndices(lineIndices, MeshTopology.Lines, triLists.Count + i);
            }
            
            volume.EditorOnlyPreviewMesh = mesh;
            
            if (!_triangulationPreviewMaterial)
                _triangulationPreviewMaterial = Resources.Load<Material>("HyperNav/TriangulationPreviewMaterial");
            
            if (!_triangulationOutlinePreviewMaterial)
                _triangulationOutlinePreviewMaterial = Resources.Load<Material>("HyperNav/TriangulationOutlinePreviewMaterial");

            volume.EditorOnlyPreviewMaterials =
                Enumerable.Repeat(_triangulationPreviewMaterial, triLists.Count)
                          .Concat(Enumerable.Repeat(_triangulationOutlinePreviewMaterial, triLists.Count))
                          .ToArray();
        }

        private static void BuildBasinIDPreviewMesh(NavVolume volume, int[,,] voxels, int[,,] basins, int basinCount, float maxDist) {
            Mesh mesh = new Mesh();

            int sizeX = basins.GetLength(0);
            int sizeY = basins.GetLength(1);
            int sizeZ = basins.GetLength(2);

            List<Vector3> positions = new List<Vector3>();
            List<Color> colors = new List<Color>();
            List<Vector2> uvs = new List<Vector2>();
            
            List<int>[] quadIndices = new List<int>[basinCount];
            
            Vector3 boxSize = volume.VoxelSize * Vector3.one * 0.2f;

            Dictionary<int, Color> basinColors = new Dictionary<int, Color>();

            for (int x = 0; x < sizeX; x++) {
                for (int y = 0; y < sizeY; y++) {
                    for (int z = 0; z < sizeZ; z++) {
                        int basin = basins[x, y, z];
                        if (basin < 0) continue;
                        Vector3 voxelPos = volume.Bounds.min +
                                           new Vector3(x + 0.5f, y + 0.5f, z + 0.5f) * volume.VoxelSize;

                        int firstIndex = positions.Count;
                        AddBoxPositions(positions, voxelPos, boxSize);

                        List<int> indices = quadIndices[basin];
                        if (indices == null) {
                            indices = new List<int>();
                            quadIndices[basin] = indices;
                        }
                        
                        AddBoxIndices(indices, firstIndex);

                        if (!basinColors.TryGetValue(basin, out Color color)) {
                            color = new Color(Random.value, Random.value, Random.value);
                            basinColors[basin] = color;
                        }

                        AddRepeatingData(colors, color, 8);

                        float dist = voxels[x, y, z] / maxDist;
                        AddRepeatingData(uvs, new Vector2(dist, 0), 8);
                    }
                }
            }

            mesh.indexFormat = IndexFormat.UInt32;
            mesh.subMeshCount = basinCount;
            mesh.SetVertices(positions);
            mesh.SetColors(colors);
            mesh.SetUVs(0, uvs);

            for (int i = 0; i < basinCount; i++) {
                mesh.SetIndices(quadIndices[i], MeshTopology.Quads, i);
            }

            volume.EditorOnlyPreviewMesh = mesh;
            
            if (!_basinIDPreviewMaterial)
                _basinIDPreviewMaterial = Resources.Load<Material>("HyperNav/BasinIDPreviewMaterial");

            volume.EditorOnlyPreviewMaterials = Enumerable.Repeat(_basinIDPreviewMaterial, basinCount).ToArray();
        }

        private static void BuildVoxelDistancePreviewMesh(NavVolume volume, int[,,] voxels, float maxDist) {
            Mesh mesh = new Mesh();

            int sizeX = voxels.GetLength(0);
            int sizeY = voxels.GetLength(1);
            int sizeZ = voxels.GetLength(2);

            List<Vector3> positions = new List<Vector3>();
            List<int> quadIndices = new List<int>();
            List<Color> colors = new List<Color>();
            Vector3 boxSize = volume.VoxelSize * Vector3.one * 0.2f;

            for (int x = 0; x < sizeX; x++) {
                for (int y = 0; y < sizeY; y++) {
                    for (int z = 0; z < sizeZ; z++) {
                        Vector3 voxelPos = volume.Bounds.min +
                                           new Vector3(x + 0.5f, y + 0.5f, z + 0.5f) * volume.VoxelSize;

                        int firstIndex = positions.Count;
                        AddBoxPositions(positions, voxelPos, boxSize);
                        AddBoxIndices(quadIndices, firstIndex);

                        float colorVal = voxels[x, y, z] / maxDist;
                        Color color = Color.Lerp(Color.red, Color.blue, colorVal);
                        AddRepeatingData(colors, color, 8);
                    }
                }
            }

            mesh.indexFormat = IndexFormat.UInt32;
            mesh.SetVertices(positions);
            mesh.SetIndices(quadIndices, MeshTopology.Quads, 0);
            mesh.SetColors(colors);

            volume.EditorOnlyPreviewMesh = mesh;
            
            if (!_voxelDistancePreviewMaterial)
                _voxelDistancePreviewMaterial = Resources.Load<Material>("HyperNav/VoxelDistancePreviewMaterial");

            volume.EditorOnlyPreviewMaterials = new[] { _voxelDistancePreviewMaterial };
        }

        private static void BuildVoxelPreviewMesh(NavVolume volume, int[,,] voxels) {
            Mesh mesh = new Mesh();

            int sizeX = voxels.GetLength(0);
            int sizeY = voxels.GetLength(1);
            int sizeZ = voxels.GetLength(2);

            List<Vector3> positions = new List<Vector3>();
            List<int> quadIndices = new List<int>();
            List<int> lineIndices = new List<int>();
            Vector3 boxSize = volume.VoxelSize * Vector3.one;

            for (int x = 0; x < sizeX; x++) {
                for (int y = 0; y < sizeY; y++) {
                    for (int z = 0; z < sizeZ; z++) {
                        if (voxels[x, y, z] != 0) continue;
                        Vector3 voxelPos = volume.Bounds.min +
                                           new Vector3(x + 0.5f, y + 0.5f, z + 0.5f) * volume.VoxelSize;

                        int firstIndex = positions.Count;
                        AddBoxPositions(positions, voxelPos, boxSize);
                        AddBoxIndices(quadIndices, firstIndex);
                        AddBoxLineIndices(lineIndices, firstIndex);
                    }
                }
            }

            mesh.indexFormat = IndexFormat.UInt32;
            mesh.subMeshCount = 2;
            mesh.SetVertices(positions);
            mesh.SetIndices(quadIndices, MeshTopology.Quads, 0);
            mesh.SetIndices(lineIndices, MeshTopology.Lines, 1);

            volume.EditorOnlyPreviewMesh = mesh;
            
            if (!_voxelPreviewMaterial)
                _voxelPreviewMaterial = Resources.Load<Material>("HyperNav/VoxelPreviewMaterial");
            if (!_voxelOutlinePreviewMaterial)
                _voxelOutlinePreviewMaterial = Resources.Load<Material>("HyperNav/VoxelOutlinePreviewMaterial");

            volume.EditorOnlyPreviewMaterials = new[] { _voxelPreviewMaterial, _voxelOutlinePreviewMaterial };
        }

        private static void AddRepeatingData<T>(List<T> values, T value, int count) {
            for (int i = 0; i < count; i++) {
                values.Add(value);
            }
        }

        private static void AddBoxPositions(List<Vector3> positions, Vector3 center, Vector3 size) {
            Vector3 ext = size * 0.5f;

            positions.Add(center + new Vector3(-ext.x, -ext.y, -ext.z));
            positions.Add(center + new Vector3(-ext.x, -ext.y, +ext.z));
            positions.Add(center + new Vector3(-ext.x, +ext.y, -ext.z));
            positions.Add(center + new Vector3(-ext.x, +ext.y, +ext.z));
            positions.Add(center + new Vector3(+ext.x, -ext.y, -ext.z));
            positions.Add(center + new Vector3(+ext.x, -ext.y, +ext.z));
            positions.Add(center + new Vector3(+ext.x, +ext.y, -ext.z));
            positions.Add(center + new Vector3(+ext.x, +ext.y, +ext.z));
        }

        private static void AddBoxLineIndices(List<int> indices, int firstVertex) {
            int nnn = firstVertex + 0;
            int nnp = firstVertex + 1;
            int npn = firstVertex + 2;
            int npp = firstVertex + 3;
            int pnn = firstVertex + 4;
            int pnp = firstVertex + 5;
            int ppn = firstVertex + 6;
            int ppp = firstVertex + 7;
            
            AddLineIndices(indices, nnn, pnn, pnp, nnp); // bottom
            AddLineIndices(indices, npp, ppp, ppn, npn); // top
            AddLineIndices(indices, nnn, npn); // back left
            AddLineIndices(indices, pnn, ppn); // back right
            AddLineIndices(indices, nnp, npp); // front left
            AddLineIndices(indices, pnp, ppp); // front right
        }

        private static void AddBoxIndices(List<int> indices, int firstVertex) {
            int nnn = firstVertex + 0;
            int nnp = firstVertex + 1;
            int npn = firstVertex + 2;
            int npp = firstVertex + 3;
            int pnn = firstVertex + 4;
            int pnp = firstVertex + 5;
            int ppn = firstVertex + 6;
            int ppp = firstVertex + 7;
            
            AddQuadIndices(indices, nnn, pnn, pnp, nnp); // bottom
            AddQuadIndices(indices, npp, ppp, ppn, npn); // top
            AddQuadIndices(indices, npn, ppn, pnn, nnn); // back
            AddQuadIndices(indices, nnp, pnp, ppp, npp); // front
            AddQuadIndices(indices, npp, npn, nnn, nnp); // left
            AddQuadIndices(indices, ppn, ppp, pnp, pnn); // right
        }

        private static void AddLineIndices(List<int> indices, params int[] toAdd) {
            for (int i = 0; i < toAdd.Length - 1; i++) {
                indices.Add(toAdd[i]);
                indices.Add(toAdd[i + 1]);
            }
        }

        private static void AddQuadIndices(List<int> indices, int v1, int v2, int v3, int v4) {
            indices.Add(v1);
            indices.Add(v2);
            indices.Add(v3);
            indices.Add(v4);
        }
    }
}