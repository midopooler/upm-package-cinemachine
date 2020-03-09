#if !UNITY_2019_3_OR_NEWER
#define CINEMACHINE_PHYSICS
#define CINEMACHINE_PHYSICS_2D
#endif

using UnityEngine;
using System.Collections.Generic;
using Cinemachine.Utility;
using System;

namespace Cinemachine
{
#if CINEMACHINE_PHYSICS || CINEMACHINE_PHYSICS_2D
    /// <summary>
    /// An add-on module for Cinemachine Virtual Camera that post-processes
    /// the final position of the virtual camera. It will confine the virtual
    /// camera's position to the volume specified in the Bounding Volume field.
    /// </summary>
    [DocumentationSorting(DocumentationSortingAttribute.Level.UserRef)]
    [AddComponentMenu("")] // Hide in menu
    [SaveDuringPlay]
#if UNITY_2018_3_OR_NEWER
    [ExecuteAlways]
#else
    [ExecuteInEditMode]
#endif
    public class CinemachineConfiner : CinemachineExtension
    {
#if CINEMACHINE_PHYSICS && CINEMACHINE_PHYSICS_2D
        /// <summary>The confiner can operate using a 2D bounding shape or a 3D bounding volume</summary>
        public enum Mode
        {
            /// <summary>Use a 2D bounding shape, suitable for an orthographic camera</summary>
            Confine2D,
            /// <summary>Use a 3D bounding shape, suitable for perspective cameras</summary>
            Confine3D
        };
        /// <summary>The confiner can operate using a 2D bounding shape or a 3D bounding volume</summary>
        [Tooltip("The confiner can operate using a 2D bounding shape or a 3D bounding volume")]
        public Mode m_ConfineMode;
#endif

#if CINEMACHINE_PHYSICS
        /// <summary>The volume within which the camera is to be contained.</summary>
        [Tooltip("The volume within which the camera is to be contained")]
        public Collider m_BoundingVolume;
#endif

#if CINEMACHINE_PHYSICS_2D

        /// <summary>The 2D shape within which the camera is to be contained.</summary>
        [Tooltip("The 2D shape within which the camera is to be contained")]
        public Collider2D m_BoundingShape2D;
        private Collider2D m_BoundingShape2DCache;
#endif
        /// <summary>If camera is orthographic, screen edges will be confined to the volume.</summary>
        [Tooltip("If camera is orthographic, screen edges will be confined to the volume.  "
            + "If not checked, then only the camera center will be confined")]
        public bool m_ConfineScreenEdges = true;
        
        [Tooltip("If camera is orthographic, screen edges will be confined to the volume.  "
                 + "If not checked, then only the camera center will be confined")]
        public bool m_ResizeCameraToFitConfiner = true;

        public float m_DefaultCameraOrthoSize = 3;
        

        /// <summary>How gradually to return the camera to the bounding volume if it goes beyond the borders</summary>
        [Tooltip("How gradually to return the camera to the bounding volume if it goes beyond the borders.  "
            + "Higher numbers are more gradual.")]
        [Range(0, 10)]
        public float m_Damping = 0;

        /// <summary>See whether the virtual camera has been moved by the confiner</summary>
        /// <param name="vcam">The virtual camera in question.  This might be different from the
        /// virtual camera that owns the confiner, in the event that the camera has children</param>
        /// <returns>True if the virtual camera has been repositioned</returns>
        public bool CameraWasDisplaced(CinemachineVirtualCameraBase vcam)
        {
            return GetExtraState<VcamExtraState>(vcam).confinerDisplacement > 0;
        }

        private void OnValidate()
        {
            m_Damping = Mathf.Max(0, m_Damping);
        }

        class VcamExtraState
        {
            public Vector3 m_previousDisplacement;
            public float confinerDisplacement;
        };

        /// <summary>Check if the bounding volume is defined</summary>
        public bool IsValid
        {
            get
            {
#if CINEMACHINE_PHYSICS && !CINEMACHINE_PHYSICS_2D
                return m_BoundingVolume != null;
#elif CINEMACHINE_PHYSICS_2D && !CINEMACHINE_PHYSICS
                return m_BoundingShape2D != null;
#else
                return (m_ConfineMode == Mode.Confine3D && m_BoundingVolume != null)
                    || (m_ConfineMode == Mode.Confine2D && m_BoundingShape2D != null);
#endif
            }
        }

        /// <summary>Callback to to the camera confining</summary>
        protected override void PostPipelineStageCallback(
            CinemachineVirtualCameraBase vcam,
            CinemachineCore.Stage stage, ref CameraState state, float deltaTime)
        {
            if (IsValid)
            {
                // Move the body before the Aim is calculated
                if (stage == CinemachineCore.Stage.Body)
                {
                    Vector3 displacement;

                    if (m_ResizeCameraToFitConfiner)
                    {
                        ResizeCameraToFitConfiner(vcam.Follow.position, deltaTime, ref state);
                        
                    }
                    
                    if (m_ConfineScreenEdges && state.Lens.Orthographic)
                        displacement = ConfineScreenEdges(vcam, ref state);
                    else
                        displacement = ConfinePoint(state.CorrectedPosition);

                    VcamExtraState extra = GetExtraState<VcamExtraState>(vcam);
                    if (m_Damping > 0 && deltaTime >= 0 && VirtualCamera.PreviousStateIsValid)
                    {
                        Vector3 delta = displacement - extra.m_previousDisplacement;
                        delta = Damper.Damp(delta, m_Damping, deltaTime);
                        displacement = extra.m_previousDisplacement + delta;
                    }
                    extra.m_previousDisplacement = displacement;
                    state.PositionCorrection += displacement;
                    extra.confinerDisplacement = displacement.magnitude;
                }
            }
        }

        private List<List<Vector2>> m_pathCache;
        private int m_pathTotalPointCount;

        /// <summary>Call this if the bounding shape's points change at runtime</summary>
        public void InvalidatePathCache()
        {
            m_pathCache = null;
            m_BoundingShape2DCache = null;
        }

        bool ValidatePathCache()
        {
#if CINEMACHINE_PHYSICS_2D
            if (m_BoundingShape2DCache != m_BoundingShape2D)
            {
                InvalidatePathCache();
                m_BoundingShape2DCache = m_BoundingShape2D;
            }
            
            Type colliderType = m_BoundingShape2D == null ? null:  m_BoundingShape2D.GetType();
            if (colliderType == typeof(PolygonCollider2D))
            {
                PolygonCollider2D poly = m_BoundingShape2D as PolygonCollider2D;
                if (m_pathCache == null || m_pathCache.Count != poly.pathCount || m_pathTotalPointCount != poly.GetTotalPointCount())
                {
                    m_pathCache = new List<List<Vector2>>();
                    for (int i = 0; i < poly.pathCount; ++i)
                    {
                        Vector2[] path = poly.GetPath(i);
                        List<Vector2> dst = new List<Vector2>();
                        for (int j = 0; j < path.Length; ++j)
                            dst.Add(path[j]);
                        m_pathCache.Add(dst);
                    }
                    m_pathTotalPointCount = poly.GetTotalPointCount();
                }
                return true;
            }
            else if (colliderType == typeof(CompositeCollider2D))
            {
                CompositeCollider2D poly = m_BoundingShape2D as CompositeCollider2D;
                if (m_pathCache == null || m_pathCache.Count != poly.pathCount || m_pathTotalPointCount != poly.pointCount)
                {
                    m_pathCache = new List<List<Vector2>>();
                    Vector2[] path = new Vector2[poly.pointCount];
                    for (int i = 0; i < poly.pathCount; ++i)
                    {
                        int numPoints = poly.GetPath(i, path);
                        List<Vector2> dst = new List<Vector2>();
                        for (int j = 0; j < numPoints; ++j)
                            dst.Add(path[j]);
                        m_pathCache.Add(dst);
                    }
                    m_pathTotalPointCount = poly.pointCount;
                }
                return true;
            }
#endif
            InvalidatePathCache();
            return false;
        }

        private Vector3 ConfinePoint(Vector3 camPos)
        {
#if CINEMACHINE_PHYSICS
            // 3D version
    #if CINEMACHINE_PHYSICS_2D
            if (m_ConfineMode == Mode.Confine3D)
    #endif
                return m_BoundingVolume.ClosestPoint(camPos) - camPos;
#endif

#if CINEMACHINE_PHYSICS_2D
            // 2D version
            Vector2 p = camPos;
            Vector2 closest = p;
            if (m_BoundingShape2D.OverlapPoint(camPos))
                return Vector3.zero;
            // Find the nearest point on the shape's boundary
            if (!ValidatePathCache())
                return Vector3.zero;

            float bestDistance = float.MaxValue;
            for (int i = 0; i < m_pathCache.Count; ++i)
            {
                int numPoints = m_pathCache[i].Count;
                if (numPoints > 0)
                {
                    Vector2 v0 = m_BoundingShape2D.transform.TransformPoint(m_pathCache[i][numPoints-1]);
                    for (int j = 0; j < numPoints; ++j)
                    {
                        Vector2 v = m_BoundingShape2D.transform.TransformPoint(m_pathCache[i][j]);
                        Vector2 c = Vector2.Lerp(v0, v, p.ClosestPointOnSegment(v0, v));
                        float d = Vector2.SqrMagnitude(p - c);
                        if (d < bestDistance)
                        {
                            bestDistance = d;
                            closest = c;
                        }
                        v0 = v;
                    }
                }
            }
            return closest - p;
#endif
        }

        private float prevShrink = 1;
        private void ResizeCameraToFitConfiner(in Vector3 follow, in float deltaTime, ref CameraState state, float tolerance = 1e-5f)
        {
            float heightFromCenter = state.Lens.OrthographicSize;
            float widthFromCenter = heightFromCenter * state.Lens.Aspect;

            if (Math.Abs(heightFromCenter) < tolerance || Math.Abs(widthFromCenter) < tolerance)
            {
                return;
            }
            
            float maxBoxWidth = 2 * widthFromCenter;
            float maxBoxHeight = 2 * heightFromCenter;
            float maxBoxDiagonal = Mathf.Sqrt(maxBoxWidth * maxBoxWidth + maxBoxHeight * maxBoxHeight);
            float maxBoxWidthAroundFollow, maxBoxHeightAroundFollow, maxBoxDiagonalAroundFollow;
            {
                int layer = LayerMask.GetMask("CinemachineCamCollider");
                Vector2 follow2D = new Vector2(follow.x, follow.y);
                {
                    RaycastHit2D hitLeft = Physics2D.Raycast(follow2D, Vector2.left, maxBoxWidth, layer);
                    RaycastHit2D hitRight = Physics2D.Raycast(follow2D, Vector2.right, maxBoxWidth, layer);
                    maxBoxWidthAroundFollow = Mathf.Clamp(
                        (hitLeft.collider == null ? maxBoxWidth : hitLeft.distance) +
                        (hitRight.collider == null ? maxBoxWidth : hitRight.distance),
                        0.01f, maxBoxWidth);
                }
                {
                    RaycastHit2D hitTop = Physics2D.Raycast(follow2D, Vector2.up, maxBoxHeight, layer);
                    RaycastHit2D hitBottom = Physics2D.Raycast(follow2D, Vector2.down, maxBoxHeight, layer);
                    maxBoxHeightAroundFollow = Mathf.Clamp(
                        (hitTop.collider == null ? maxBoxHeight : hitTop.distance) +
                        (hitBottom.collider == null ? maxBoxHeight : hitBottom.distance),
                        0.01f, maxBoxHeight);
                }
                float maxBoxDiagonalAroundFollow1;
                {
                    Vector2 topRight = maxBoxWidth * Vector2.right + maxBoxHeight * Vector2.up;
                    Vector2 bottomLeft = maxBoxWidth * Vector2.left + maxBoxHeight * Vector2.down;
                    RaycastHit2D hitTopRight =
                        Physics2D.Raycast(follow2D, topRight.normalized, maxBoxDiagonal, layer);
                    RaycastHit2D hitBottomLeft =
                        Physics2D.Raycast(follow2D, bottomLeft.normalized, maxBoxDiagonal, layer);
                    maxBoxDiagonalAroundFollow1 = Mathf.Clamp(
                        (hitTopRight.collider == null ? maxBoxDiagonal : hitTopRight.distance) +
                        (hitBottomLeft.collider == null ? maxBoxDiagonal : hitBottomLeft.distance),
                        0.01f, maxBoxDiagonal);
                }
                float maxBoxDiagonalAroundFollow2;
                {
                    Vector2 topLeft = maxBoxWidth * Vector2.left + maxBoxHeight * Vector2.up;
                    Vector2 bottomRight = maxBoxWidth * Vector2.right + maxBoxHeight * Vector2.down;
                    RaycastHit2D hitTopLeft =
                        Physics2D.Raycast(follow2D, topLeft.normalized, maxBoxDiagonal, layer);
                    RaycastHit2D hitBottomRight =
                        Physics2D.Raycast(follow2D, bottomRight.normalized, maxBoxDiagonal, layer);
                    maxBoxDiagonalAroundFollow2 = Mathf.Clamp(
                        (hitTopLeft.collider == null ? maxBoxDiagonal : hitTopLeft.distance) +
                        (hitBottomRight.collider == null ? maxBoxDiagonal : hitBottomRight.distance),
                        0.01f, maxBoxDiagonal);
                }
                maxBoxDiagonalAroundFollow = Mathf.Min(maxBoxDiagonalAroundFollow1, maxBoxDiagonalAroundFollow2);
            }
            float shrink = Mathf.Min(maxBoxDiagonalAroundFollow / maxBoxDiagonal, Mathf.Min(
                                     maxBoxWidthAroundFollow    / maxBoxWidth,
                                     maxBoxHeightAroundFollow   / maxBoxHeight));

            shrink = Mathf.Lerp(prevShrink, shrink, Math.Abs(m_Damping) < tolerance ? 1 : deltaTime * 1.0f / m_Damping);
            prevShrink = shrink;
            
            var lens = state.Lens;
            lens.OrthographicSize = m_DefaultCameraOrthoSize * shrink;
            state.Lens = lens;
        }

        // Camera must be orthographic
        private Vector3 ConfineScreenEdges(CinemachineVirtualCameraBase vcam, ref CameraState state)
        {
            Quaternion rot = Quaternion.Inverse(state.CorrectedOrientation);
            float heightFromCenter = state.Lens.OrthographicSize;
            float widthFromCenter = heightFromCenter * state.Lens.Aspect;
            Vector3 w = (rot * Vector3.right) * widthFromCenter;
            Vector3 h = (rot * Vector3.up) * heightFromCenter;
            
            Vector3 camPos = state.CorrectedPosition;
            Vector3 displacement = Vector3.zero;
            Vector3 lastD = Vector3.zero;
            const int kMaxIter = 12;
            for (int i = 0; i < kMaxIter; ++i)
            {
                Vector3 d = ConfinePoint((camPos - h) - w);
                if (d.AlmostZero())
                    d = ConfinePoint((camPos + h) + w);
                if (d.AlmostZero())
                    d = ConfinePoint((camPos - h) + w);
                if (d.AlmostZero())
                    d = ConfinePoint((camPos + h) - w);
                if (d.AlmostZero())
                    break;
                if ((d + lastD).AlmostZero())
                {
                    displacement += d * 0.5f;  // confiner too small: center it
                    break;
                }
                displacement += d;
                camPos += d;
                lastD = d;
            }
            return displacement;
        }
    }
#endif
}
