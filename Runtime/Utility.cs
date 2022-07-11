using UnityEngine;

namespace HyperNav.Runtime {
    public class Utility {
        public static int Dot(Vector3Int v1, Vector3Int v2) {
            return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
        }
        
        public static void DrawDebugBounds(Bounds bounds, Color color, float duration = 0.0f, bool depthTest = true) {
            Debug.DrawLine(new Vector3(bounds.min.x, bounds.min.y, bounds.min.z), new Vector3(bounds.max.x, bounds.min.y, bounds.min.z), color, duration, depthTest);
            Debug.DrawLine(new Vector3(bounds.min.x, bounds.max.y, bounds.min.z), new Vector3(bounds.max.x, bounds.max.y, bounds.min.z), color, duration, depthTest);
            Debug.DrawLine(new Vector3(bounds.min.x, bounds.max.y, bounds.max.z), new Vector3(bounds.max.x, bounds.max.y, bounds.max.z), color, duration, depthTest);
            Debug.DrawLine(new Vector3(bounds.min.x, bounds.min.y, bounds.max.z), new Vector3(bounds.max.x, bounds.min.y, bounds.max.z), color, duration, depthTest);
            Debug.DrawLine(new Vector3(bounds.min.x, bounds.min.y, bounds.min.z), new Vector3(bounds.min.x, bounds.max.y, bounds.min.z), color, duration, depthTest);
            Debug.DrawLine(new Vector3(bounds.max.x, bounds.min.y, bounds.min.z), new Vector3(bounds.max.x, bounds.max.y, bounds.min.z), color, duration, depthTest);
            Debug.DrawLine(new Vector3(bounds.max.x, bounds.min.y, bounds.max.z), new Vector3(bounds.max.x, bounds.max.y, bounds.max.z), color, duration, depthTest);
            Debug.DrawLine(new Vector3(bounds.min.x, bounds.min.y, bounds.max.z), new Vector3(bounds.min.x, bounds.max.y, bounds.max.z), color, duration, depthTest);
            Debug.DrawLine(new Vector3(bounds.min.x, bounds.min.y, bounds.min.z), new Vector3(bounds.min.x, bounds.min.y, bounds.max.z), color, duration, depthTest);
            Debug.DrawLine(new Vector3(bounds.max.x, bounds.min.y, bounds.min.z), new Vector3(bounds.max.x, bounds.min.y, bounds.max.z), color, duration, depthTest);
            Debug.DrawLine(new Vector3(bounds.max.x, bounds.max.y, bounds.min.z), new Vector3(bounds.max.x, bounds.max.y, bounds.max.z), color, duration, depthTest);
            Debug.DrawLine(new Vector3(bounds.min.x, bounds.max.y, bounds.min.z), new Vector3(bounds.min.x, bounds.max.y, bounds.max.z), color, duration, depthTest);
        }
    }
}