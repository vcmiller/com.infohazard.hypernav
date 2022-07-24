using System;
using UnityEngine;

namespace HyperNav.Runtime.Utility {
    [Serializable]
    public struct Triangle : IEquatable<Triangle> {
        public int Vertex1 => _minVertex;
        public int Vertex2 => _midVertex;
        public int Vertex3 => _maxVertex;

        [SerializeField] private int _minVertex;
        [SerializeField] private int _midVertex;
        [SerializeField] private int _maxVertex;
        
        public Triangle(int vertex1, int vertex2, int vertex3) {
            if (vertex1 == vertex2 || vertex1 == vertex3 || vertex2 == vertex3) {
                Debug.LogError($"Triangle vertices must not be the same index: {vertex1}, {vertex2}, {vertex3}.");
            }

            if (vertex1 > vertex2) {
                // vertex 1 > vertex2
                if (vertex3 > vertex1) {
                    // vertex3 > vertex1 > vertex2
                    _maxVertex = vertex3;
                    _minVertex = vertex2;
                    _midVertex = vertex1;
                } else if (vertex2 > vertex3) {
                    // vertex1 > vertex2 > vertex3
                    _maxVertex = vertex1;
                    _minVertex = vertex3;
                    _midVertex = vertex2;
                } else {
                    // vertex1 > vertex3 > vertex2
                    _maxVertex = vertex1;
                    _minVertex = vertex2;
                    _midVertex = vertex3;
                }
            } else {
                // vertex2 > vertex1
                if (vertex3 > vertex2) {
                    // vertex3 > vertex2 > vertex1
                    _maxVertex = vertex3;
                    _minVertex = vertex1;
                    _midVertex = vertex2;
                } else if (vertex3 > vertex1) {
                    // vertex2 > vertex3 > vertex1
                    _maxVertex = vertex2;
                    _minVertex = vertex1;
                    _midVertex = vertex3;
                } else {
                    // vertex2 > vertex1 > vertex3
                    _maxVertex = vertex2;
                    _minVertex = vertex3;
                    _midVertex = vertex1;
                }
            }
        }

        public override bool Equals(object obj) {
            if (!(obj is Triangle triangle)) return false;
            return Equals(triangle);
        }

        public bool Equals(Triangle other) {
            return _minVertex == other._minVertex && _midVertex == other._midVertex && _maxVertex == other._maxVertex;
        }

        public override int GetHashCode() {
            return _minVertex ^ _midVertex ^ _maxVertex;
        }
    }
}