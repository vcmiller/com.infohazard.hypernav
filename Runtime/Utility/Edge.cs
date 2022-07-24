using System;
using UnityEngine;

namespace HyperNav.Runtime.Utility {
    [Serializable]
    public struct Edge : IEquatable<Edge> {
        public int Vertex1 => _minVertex;
        public int Vertex2 => _maxVertex;

        [SerializeField] private int _minVertex;
        [SerializeField] private int _maxVertex;

        public Edge(int vertex1, int vertex2) {
            if (vertex1 == vertex2) {
                Debug.LogError($"Edge vertices must not be the same index: {vertex1}, {vertex2}.");
            }

            if (vertex1 > vertex2) {
                _minVertex = vertex2;
                _maxVertex = vertex1;
            } else {
                _minVertex = vertex1;
                _maxVertex = vertex2;
            }
        }

        public override bool Equals(object obj) {
            if (!(obj is Edge edge)) return false;
            return Equals(edge);
        }

        public bool Equals(Edge other) {
            return _minVertex == other._minVertex && _maxVertex == other._maxVertex;
        }

        public override int GetHashCode() {
            return _minVertex ^ _maxVertex;
        }
    }
}