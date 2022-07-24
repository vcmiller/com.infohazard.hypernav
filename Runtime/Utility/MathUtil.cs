using UnityEngine;

namespace HyperNav.Runtime.Utility {
    public static class MathUtil {
        public static bool GetNearestPointOnSegment(Vector3 v1, Vector3 v2, Vector3 point, out Vector3 pointOnSegment) {
            pointOnSegment = default;
            
            Vector3 v1ToV2 = v2 - v1;

            if (Vector3.Dot(v1ToV2, point - v1) < 0) return false;
            if (Vector3.Dot(-v1ToV2, point - v2) < 0) return false;

            Vector3 proj = Vector3.Project(point - v1, v1ToV2);
            pointOnSegment = v1 + proj;
            return true;
        }

        public static bool GetNearestPointOnTriangle(Vector3 v1, Vector3 v2, Vector3 v3, Vector3 point,
                                                     out Vector3 pointOnTriangle) {
            pointOnTriangle = default;
            
            Vector3 normal = Vector3.Cross(v3 - v2, v1 - v2);

            if (!IsPointInsideBound(v1, v2, normal, point) ||
                !IsPointInsideBound(v2, v3, normal, point) ||
                !IsPointInsideBound(v3, v1, normal, point)) {
                return false;
            }

            Vector3 proj = Vector3.ProjectOnPlane(point - v1, normal);
            pointOnTriangle = v1 + proj;
            return true;
        }

        public static bool IsPointInsideBound(Vector3 v1, Vector3 v2, Vector3 normal, Vector3 point) {
            Vector3 edge = v2 - v1;
            Vector3 cross = Vector3.Cross(normal, edge).normalized;
            Vector3 pointOffset = (point - v1).normalized;

            float dot = Vector3.Dot(pointOffset, cross);
            return dot > -.00001f;
        }
    }
}