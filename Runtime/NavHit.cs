using UnityEngine;

namespace HyperNav.Runtime {
    public struct NavHit {
        public NavVolume Volume { get; set; }
        public int Region { get; set; }
        public bool IsOnEdge { get; set; }
        public Vector3 Position { get; set; }
        public Vector3 Normal { get; set; }
    }
}