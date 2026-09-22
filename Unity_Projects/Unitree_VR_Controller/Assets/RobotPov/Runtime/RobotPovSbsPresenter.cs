using System;
using UnityEngine;
using UnityEngine.Rendering;

namespace RobotPov
{
    /// <summary>
    /// Presents one decoded texture as mono, side-by-side, or top-bottom stereo.
    /// CurvedImmersive wraps a large head-locked surface around the camera so the
    /// video fills the headset view instead of appearing as a floating card.
    /// </summary>
    [DisallowMultipleComponent]
    [AddComponentMenu("Robot POV/SBS Video Presenter")]
    public sealed class RobotPovSbsPresenter : MonoBehaviour
    {
        private static readonly int MainTextureId = Shader.PropertyToID("_MainTex");
        private static readonly int EyeSwapId = Shader.PropertyToID("_EyeSwap");
        private static readonly int FlipVerticalId = Shader.PropertyToID("_FlipY");
        private static readonly int LayoutId = Shader.PropertyToID("_Layout");
        private static readonly int SignalId = Shader.PropertyToID("_Signal");
        private static readonly int DepthTestId = Shader.PropertyToID("_ZTest");

        [SerializeField]
        private RobotPovWebRtcReceiver receiver;

        [SerializeField]
        private RobotPovRuntimeConfig config = new RobotPovRuntimeConfig();

        [SerializeField]
        private Camera targetCamera;

        [SerializeField]
        private Transform worldAnchor;

        private GameObject visualRoot;
        private GameObject screenObject;
        private MeshFilter screenMeshFilter;
        private Mesh screenMesh;
        private MeshRenderer screenRenderer;
        private Material screenMaterial;
        private TextMesh noSignalText;
        private TextMesh diagnosticsText;
        private bool subscribed;
        private bool placementApplied;
        private bool previousSignalAvailable;
        private string previousReason = string.Empty;
        private float nextDiagnosticsUpdate;
        private RobotPovDisplaySurface geometrySurface;
        private bool geometryConfigured;

        public void Configure(
            RobotPovRuntimeConfig runtimeConfig,
            RobotPovWebRtcReceiver videoReceiver,
            Camera cameraOverride,
            Transform optionalWorldAnchor)
        {
            if (runtimeConfig == null)
            {
                throw new ArgumentNullException(nameof(runtimeConfig));
            }

            config = runtimeConfig;
            receiver = videoReceiver;
            targetCamera = cameraOverride;
            worldAnchor = optionalWorldAnchor;
            config.ValidateAndClamp();
            placementApplied = false;

            if (isActiveAndEnabled)
            {
                Unsubscribe();
                EnsureVisuals();
                Subscribe();
                ApplyPlacement();
                RefreshVisualState(true);
            }
        }

        private void OnEnable()
        {
            if (config == null)
            {
                config = new RobotPovRuntimeConfig();
            }

            config.ValidateAndClamp();
            EnsureVisuals();
            visualRoot.SetActive(true);
            Subscribe();
            ApplyPlacement();
            RefreshVisualState(true);
        }

        private void OnDisable()
        {
            Unsubscribe();
            if (visualRoot != null)
            {
                visualRoot.SetActive(false);
            }
        }

        private void OnDestroy()
        {
            Unsubscribe();
            if (screenMaterial != null)
            {
                Destroy(screenMaterial);
                screenMaterial = null;
            }

            if (visualRoot != null)
            {
                Destroy(visualRoot);
                visualRoot = null;
            }

            if (screenMesh != null)
            {
                Destroy(screenMesh);
                screenMesh = null;
            }
        }

        private void Update()
        {
            if (!placementApplied
                || (config.ScreenMode == RobotPovScreenMode.HeadLocked
                    && targetCamera == null))
            {
                ApplyPlacement();
            }

            RefreshVisualState(false);
        }

        private void EnsureVisuals()
        {
            if (visualRoot != null)
            {
                RebuildGeometryIfNeeded();
                ApplyMaterialConfiguration();
                return;
            }

            visualRoot = new GameObject("Robot POV Display");
            visualRoot.transform.SetParent(transform, false);

            screenObject = new GameObject("Robot POV Immersive Video Surface");
            screenObject.transform.SetParent(visualRoot.transform, false);
            screenMeshFilter = screenObject.AddComponent<MeshFilter>();
            screenRenderer = screenObject.AddComponent<MeshRenderer>();
            screenRenderer.shadowCastingMode = ShadowCastingMode.Off;
            screenRenderer.receiveShadows = false;
            screenRenderer.lightProbeUsage = LightProbeUsage.Off;
            screenRenderer.reflectionProbeUsage = ReflectionProbeUsage.Off;
            screenRenderer.sortingOrder = 10;
            RebuildGeometryIfNeeded();

            Shader shader = Resources.Load<Shader>("RobotPovSbs");
            if (shader == null)
            {
                shader = Shader.Find("RobotPov/StereoSideBySide");
            }

            if (shader != null)
            {
                screenMaterial = new Material(shader)
                {
                    name = "Robot POV SBS Runtime Material"
                };
                screenRenderer.sharedMaterial = screenMaterial;
            }
            else
            {
                screenRenderer.enabled = false;
                Debug.LogError(
                    "[Robot POV] Resources/RobotPovSbs.shader is missing; video display is disabled.");
            }

            noSignalText = CreateText(
                "NO SIGNAL",
                TextAnchor.MiddleCenter,
                TextAlignment.Center,
                new Color(1f, 0.35f, 0.28f, 1f),
                64,
                0.025f,
                new Vector3(0f, 0f, -0.025f));
            noSignalText.name = "No Signal Overlay";

            diagnosticsText = CreateText(
                "ROBOT POV",
                TextAnchor.LowerLeft,
                TextAlignment.Left,
                new Color(0.82f, 0.92f, 1f, 0.95f),
                48,
                0.012f,
                new Vector3(
                    -config.ScreenWidth * 0.5f + 0.06f,
                    -config.ScreenHeight * 0.5f + 0.06f,
                    -0.03f));
            diagnosticsText.name = "Video Diagnostics Overlay";

            ApplyMaterialConfiguration();
        }

        private void RebuildGeometryIfNeeded()
        {
            if (screenMeshFilter == null || screenObject == null)
            {
                return;
            }

            if (geometryConfigured && geometrySurface == config.DisplaySurface)
            {
                return;
            }

            if (screenMesh != null)
            {
                Destroy(screenMesh);
            }

            screenMesh = config.DisplaySurface == RobotPovDisplaySurface.CurvedImmersive
                ? BuildCurvedMesh()
                : BuildFlatMesh();
            screenMesh.name = config.DisplaySurface == RobotPovDisplaySurface.CurvedImmersive
                ? "Robot POV Curved Immersive Mesh"
                : "Robot POV Flat Quad Mesh";
            screenMeshFilter.sharedMesh = screenMesh;
            geometrySurface = config.DisplaySurface;
            geometryConfigured = true;
            UpdateOverlayPlacement();
        }

        private Mesh BuildFlatMesh()
        {
            float halfWidth = config.ScreenWidth * 0.5f;
            float halfHeight = config.ScreenHeight * 0.5f;
            Mesh mesh = new Mesh();
            mesh.vertices = new[]
            {
                new Vector3(-halfWidth, -halfHeight, 0f),
                new Vector3(halfWidth, -halfHeight, 0f),
                new Vector3(halfWidth, halfHeight, 0f),
                new Vector3(-halfWidth, halfHeight, 0f),
            };
            mesh.uv = new[]
            {
                new Vector2(0f, 0f), new Vector2(1f, 0f),
                new Vector2(1f, 1f), new Vector2(0f, 1f),
            };
            mesh.triangles = new[] { 0, 2, 1, 0, 3, 2 };
            mesh.RecalculateBounds();
            return mesh;
        }

        private Mesh BuildCurvedMesh()
        {
            const int horizontalSegments = 48;
            const int verticalSegments = 20;
            float radius = config.ScreenDistance;
            float halfHorizontal = config.ImmersiveHorizontalFovDegrees * Mathf.Deg2Rad * 0.5f;
            float halfVertical = config.ImmersiveVerticalFovDegrees * Mathf.Deg2Rad * 0.5f;
            int vertexCount = (horizontalSegments + 1) * (verticalSegments + 1);
            Vector3[] vertices = new Vector3[vertexCount];
            Vector2[] uv = new Vector2[vertexCount];
            int vertex = 0;

            for (int y = 0; y <= verticalSegments; y++)
            {
                float v = (float)y / verticalSegments;
                float elevation = Mathf.Lerp(-halfVertical, halfVertical, v);
                float localY = Mathf.Tan(elevation) * radius;
                for (int x = 0; x <= horizontalSegments; x++)
                {
                    float u = (float)x / horizontalSegments;
                    float azimuth = Mathf.Lerp(-halfHorizontal, halfHorizontal, u);
                    vertices[vertex] = new Vector3(
                        Mathf.Sin(azimuth) * radius,
                        localY,
                        Mathf.Cos(azimuth) * radius);
                    uv[vertex] = new Vector2(u, v);
                    vertex++;
                }
            }

            int[] triangles = new int[horizontalSegments * verticalSegments * 6];
            int triangle = 0;
            for (int y = 0; y < verticalSegments; y++)
            {
                for (int x = 0; x < horizontalSegments; x++)
                {
                    int row = horizontalSegments + 1;
                    int a = y * row + x;
                    int b = a + 1;
                    int c = a + row;
                    int d = c + 1;
                    triangles[triangle++] = a;
                    triangles[triangle++] = c;
                    triangles[triangle++] = b;
                    triangles[triangle++] = b;
                    triangles[triangle++] = c;
                    triangles[triangle++] = d;
                }
            }

            Mesh mesh = new Mesh();
            mesh.indexFormat = vertexCount > 65535
                ? IndexFormat.UInt32
                : IndexFormat.UInt16;
            mesh.vertices = vertices;
            mesh.uv = uv;
            mesh.triangles = triangles;
            mesh.RecalculateBounds();
            return mesh;
        }

        private void UpdateOverlayPlacement()
        {
            if (noSignalText == null || diagnosticsText == null)
            {
                return;
            }

            if (config.DisplaySurface == RobotPovDisplaySurface.CurvedImmersive)
            {
                float radius = config.ScreenDistance * 0.98f;
                float horizontal = Mathf.Tan(config.ImmersiveHorizontalFovDegrees * Mathf.Deg2Rad * 0.5f);
                float vertical = Mathf.Tan(config.ImmersiveVerticalFovDegrees * Mathf.Deg2Rad * 0.5f);
                noSignalText.transform.localPosition = new Vector3(0f, 0f, radius);
                diagnosticsText.transform.localPosition = new Vector3(
                    -config.ScreenDistance * horizontal + 0.08f,
                    -config.ScreenDistance * vertical + 0.08f,
                    radius);
            }
            else
            {
                noSignalText.transform.localPosition = new Vector3(0f, 0f, -0.025f);
                diagnosticsText.transform.localPosition = new Vector3(
                    -config.ScreenWidth * 0.5f + 0.06f,
                    -config.ScreenHeight * 0.5f + 0.06f,
                    -0.03f);
            }
        }

        private TextMesh CreateText(
            string value,
            TextAnchor anchor,
            TextAlignment alignment,
            Color color,
            int fontSize,
            float characterSize,
            Vector3 localPosition)
        {
            GameObject textObject = new GameObject(value);
            textObject.transform.SetParent(visualRoot.transform, false);
            textObject.transform.localPosition = localPosition;
            textObject.transform.localRotation = Quaternion.identity;

            TextMesh text = textObject.AddComponent<TextMesh>();
            text.text = value;
            text.anchor = anchor;
            text.alignment = alignment;
            text.color = color;
            text.fontSize = fontSize;
            text.characterSize = characterSize;
            text.richText = false;

            MeshRenderer textRenderer = textObject.GetComponent<MeshRenderer>();
            textRenderer.shadowCastingMode = ShadowCastingMode.Off;
            textRenderer.receiveShadows = false;
            textRenderer.sortingOrder = 20;
            return text;
        }

        private void ApplyPlacement()
        {
            if (visualRoot == null)
            {
                return;
            }

            if (config.ScreenMode == RobotPovScreenMode.HeadLocked)
            {
                if (targetCamera == null)
                {
                    targetCamera = Camera.main;
                }

                if (targetCamera == null)
                {
                    placementApplied = false;
                    return;
                }

                visualRoot.transform.SetParent(targetCamera.transform, false);
                visualRoot.transform.localPosition = config.DisplaySurface == RobotPovDisplaySurface.CurvedImmersive
                    ? Vector3.zero
                    : new Vector3(0f, 0f, config.ScreenDistance);
                visualRoot.transform.localRotation = Quaternion.identity;
                visualRoot.transform.localScale = Vector3.one;
            }
            else
            {
                visualRoot.transform.SetParent(worldAnchor, false);
                if (worldAnchor == null)
                {
                    visualRoot.transform.position = config.WorldSpacePosition;
                    visualRoot.transform.rotation = Quaternion.Euler(config.WorldSpaceEulerAngles);
                }
                else
                {
                    visualRoot.transform.localPosition = config.WorldSpacePosition;
                    visualRoot.transform.localRotation = Quaternion.Euler(config.WorldSpaceEulerAngles);
                }

                visualRoot.transform.localScale = Vector3.one;
            }

            placementApplied = true;
            RebuildGeometryIfNeeded();
            UpdateOverlayPlacement();
            ApplyMaterialConfiguration();
        }

        private void ApplyMaterialConfiguration()
        {
            if (screenMaterial == null)
            {
                return;
            }

            screenMaterial.SetFloat(EyeSwapId, config.SwapEyes ? 1f : 0f);
            screenMaterial.SetFloat(FlipVerticalId, config.FlipVertical ? 1f : 0f);
            float layout = config.VideoLayout == RobotPovVideoLayout.Mono
                ? 0f
                : (config.VideoLayout == RobotPovVideoLayout.StereoTopBottom ? 2f : 1f);
            screenMaterial.SetFloat(LayoutId, layout);
            screenMaterial.SetFloat(
                DepthTestId,
                config.ScreenMode == RobotPovScreenMode.HeadLocked
                    ? (float)CompareFunction.Always
                    : (float)CompareFunction.LessEqual);
            screenMaterial.renderQueue = config.ScreenMode == RobotPovScreenMode.HeadLocked
                ? (int)RenderQueue.Overlay
                : (int)RenderQueue.Geometry;
        }

        private void Subscribe()
        {
            if (subscribed || receiver == null)
            {
                return;
            }

            receiver.TextureChanged += HandleTextureChanged;
            receiver.StateChanged += HandleStateChanged;
            subscribed = true;
        }

        private void Unsubscribe()
        {
            if (!subscribed || receiver == null)
            {
                subscribed = false;
                return;
            }

            receiver.TextureChanged -= HandleTextureChanged;
            receiver.StateChanged -= HandleStateChanged;
            subscribed = false;
        }

        private void HandleTextureChanged(Texture texture)
        {
            if (screenMaterial != null)
            {
                screenMaterial.SetTexture(MainTextureId, texture == null ? Texture2D.blackTexture : texture);
            }

            RefreshVisualState(true);
        }

        private void HandleStateChanged(RobotPovConnectionState _, string __)
        {
            RefreshVisualState(true);
        }

        private void RefreshVisualState(bool force)
        {
            if (visualRoot == null)
            {
                return;
            }

            bool signalAvailable = receiver != null && receiver.SignalAvailable;
            string reason = receiver == null
                ? "VIDEO RECEIVER NOT CONFIGURED"
                : receiver.NoSignalReason;

            if (force
                || signalAvailable != previousSignalAvailable
                || !string.Equals(reason, previousReason, StringComparison.Ordinal))
            {
                previousSignalAvailable = signalAvailable;
                previousReason = reason;

                if (screenMaterial != null)
                {
                    screenMaterial.SetTexture(
                        MainTextureId,
                        receiver != null && receiver.CurrentTexture != null
                            ? receiver.CurrentTexture
                            : Texture2D.blackTexture);
                    screenMaterial.SetFloat(SignalId, signalAvailable ? 1f : 0f);
                }

                if (noSignalText != null)
                {
                    noSignalText.gameObject.SetActive(!signalAvailable);
                    noSignalText.text = "NO SIGNAL\n" + reason;
                }
            }

            if (force || Time.unscaledTime >= nextDiagnosticsUpdate)
            {
                nextDiagnosticsUpdate = Time.unscaledTime + 0.5f;
                UpdateDiagnostics(signalAvailable);
            }
        }

        private void UpdateDiagnostics(bool signalAvailable)
        {
            if (diagnosticsText == null)
            {
                return;
            }

            diagnosticsText.gameObject.SetActive(config.DiagnosticsOverlay);
            if (!config.DiagnosticsOverlay)
            {
                return;
            }

            if (receiver == null)
            {
                diagnosticsText.text = "ROBOT POV  |  RECEIVER MISSING";
                return;
            }

            diagnosticsText.text = string.Format(
                "ROBOT POV  |  {0}  |  {1:0.#} FPS  |  RTT {2:0} ms  |  RECONNECT {3}  |  {4}",
                receiver.SourceName.ToUpperInvariant(),
                receiver.RenderedFps,
                receiver.RoundTripTimeMs,
                receiver.ReconnectCount,
                signalAvailable ? "LIVE" : "NO SIGNAL");
        }
    }
}
