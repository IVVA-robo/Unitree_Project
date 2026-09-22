using System;
using UnityEngine;

namespace RobotPov
{
    public enum RobotPovScreenMode
    {
        HeadLocked,
        WorldSpace
    }

    public enum RobotPovVideoLayout
    {
        StereoSideBySide,
        Mono,
        StereoTopBottom
    }

    public enum RobotPovDisplaySurface
    {
        FlatQuad,
        CurvedImmersive
    }

    /// <summary>
    /// Serializable, asset-free configuration for the video-only Robot POV runtime.
    /// Keeping this as a plain serializable object lets a dedicated scene carry its
    /// complete offline/LAN configuration without requiring a ScriptableObject.
    /// </summary>
    [Serializable]
    public sealed class RobotPovRuntimeConfig
    {
        // Current TP-Link LAN address used by the offline R1 setup. Batch builds
        // and the Pico scene can still override this with ROBOT_POV_SERVER_URL.
        public const string DefaultBaseUrl = "http://192.168.8.9:8080";
        public const string DefaultProfile = "low-latency";
        public const string DefaultClientId = "unity-pico-r1";
        public const string DefaultPreferredCodec = "h264";

        [Header("LAN server")]
        [SerializeField, Tooltip("Robot POV HTTP server. No cloud signaling is used.")]
        private string serverBaseUrl = DefaultBaseUrl;

        [SerializeField, Tooltip("high, balanced, low-latency, or bad-wifi")]
        private string profile = DefaultProfile;

        [SerializeField]
        private string clientId = DefaultClientId;

        [SerializeField, Tooltip("H.264 is preferred for Pico hardware decoding; VP8 remains a negotiated fallback.")]
        private string preferredCodec = DefaultPreferredCodec;

        [SerializeField]
        private bool connectOnEnable = true;

        [Header("Timeouts and recovery")]
        [SerializeField, Min(1f)]
        private float iceGatheringTimeoutSeconds = 5f;

        [SerializeField, Min(1f)]
        private float requestTimeoutSeconds = 10f;

        [SerializeField, Min(1f)]
        private float firstFrameTimeoutSeconds = 10f;

        [SerializeField, Min(1f), Tooltip("Reconnect if decoded frames stop while the source reports healthy.")]
        private float mediaStaleTimeoutSeconds = 4f;

        [SerializeField, Min(0.1f)]
        private float reconnectInitialDelaySeconds = 0.5f;

        [SerializeField, Min(0.1f)]
        private float reconnectMaximumDelaySeconds = 8f;

        [SerializeField, Min(0.25f)]
        private float statusPollIntervalSeconds = 1f;

        [SerializeField, Min(0.5f)]
        private float metricsIntervalSeconds = 2f;

        [Header("Display")]
        [SerializeField]
        private RobotPovVideoLayout videoLayout = RobotPovVideoLayout.StereoSideBySide;

        [SerializeField, Tooltip("CurvedImmersive fills the headset field of view without a floating card.")]
        private RobotPovDisplaySurface displaySurface = RobotPovDisplaySurface.CurvedImmersive;

        [SerializeField]
        private bool swapEyes;

        [SerializeField, Tooltip("Additional local display override; server-side camera flipping is independent.")]
        private bool flipVertical;

        [SerializeField]
        private RobotPovScreenMode screenMode = RobotPovScreenMode.HeadLocked;

        [SerializeField, Min(0.25f)]
        private float screenDistance = 2f;

        [SerializeField, Min(0.1f)]
        private float screenWidth = 3f;

        [SerializeField, Min(0.1f), Tooltip("The supplied SBS profiles contain two 4:3 eyes.")]
        private float screenHeight = 2.25f;

        [SerializeField, Range(90f, 170f)]
        private float immersiveHorizontalFovDegrees = 140f;

        [SerializeField, Range(60f, 140f)]
        private float immersiveVerticalFovDegrees = 100f;

        [SerializeField, Tooltip("Keep false for an unobstructed first-person view; diagnostics remain available in logs.")]
        private bool diagnosticsOverlay;

        [SerializeField, Tooltip("Used relative to the optional world anchor, or in global coordinates when no anchor is set.")]
        private Vector3 worldSpacePosition = new Vector3(0f, 1.6f, 2.5f);

        [SerializeField]
        private Vector3 worldSpaceEulerAngles;

        public string BaseUrl => NormalizeBaseUrl(serverBaseUrl);
        public string OfferUrl => BaseUrl + "/offer";
        public string StatusUrl => BaseUrl + "/api/status";
        public string MetricsUrl => BaseUrl + "/api/client-metrics";
        public string Profile => NormalizeProfile(profile);
        public string ClientId => string.IsNullOrWhiteSpace(clientId) ? DefaultClientId : clientId.Trim();
        public string PreferredCodec => NormalizePreferredCodec(preferredCodec);
        public bool ConnectOnEnable => connectOnEnable;
        public float IceGatheringTimeoutSeconds => iceGatheringTimeoutSeconds;
        public float RequestTimeoutSeconds => requestTimeoutSeconds;
        public float FirstFrameTimeoutSeconds => firstFrameTimeoutSeconds;
        public float MediaStaleTimeoutSeconds => mediaStaleTimeoutSeconds;
        public float ReconnectInitialDelaySeconds => reconnectInitialDelaySeconds;
        public float ReconnectMaximumDelaySeconds => reconnectMaximumDelaySeconds;
        public float StatusPollIntervalSeconds => statusPollIntervalSeconds;
        public float MetricsIntervalSeconds => metricsIntervalSeconds;
        public RobotPovVideoLayout VideoLayout => videoLayout;
        public RobotPovDisplaySurface DisplaySurface => displaySurface;
        public bool SwapEyes => swapEyes;
        public bool FlipVertical => flipVertical;
        public RobotPovScreenMode ScreenMode => screenMode;
        public float ScreenDistance => screenDistance;
        public float ScreenWidth => screenWidth;
        public float ScreenHeight => screenHeight;
        public float ImmersiveHorizontalFovDegrees => immersiveHorizontalFovDegrees;
        public float ImmersiveVerticalFovDegrees => immersiveVerticalFovDegrees;
        public bool DiagnosticsOverlay => diagnosticsOverlay;
        public Vector3 WorldSpacePosition => worldSpacePosition;
        public Vector3 WorldSpaceEulerAngles => worldSpaceEulerAngles;

        /// <summary>
        /// Allows a scene builder or a local setup UI to override only the LAN endpoint.
        /// It deliberately does not expose any robot-control destination.
        /// </summary>
        public void SetEndpoint(string baseUrl, string requestedProfile)
        {
            serverBaseUrl = NormalizeBaseUrl(baseUrl);
            profile = NormalizeProfile(requestedProfile);
        }

        public void SetAutoConnect(bool value)
        {
            connectOnEnable = value;
        }

        public void SetScreenMode(RobotPovScreenMode value)
        {
            screenMode = value;
        }

        /// <summary>
        /// Select how one decoded texture is presented to the two XR eyes.
        /// A physical R1 front-camera JPEG is mono and must be shown in full to
        /// both eyes; it must not be interpreted as side-by-side stereo.
        /// </summary>
        public void SetVideoLayout(RobotPovVideoLayout value)
        {
            videoLayout = value;
        }

        public void SetDisplaySurface(RobotPovDisplaySurface value)
        {
            displaySurface = value;
        }

        public void SetDiagnosticsOverlay(bool value)
        {
            diagnosticsOverlay = value;
        }

        /// <summary>Configure the head-locked/world-space video surface.</summary>
        public void SetDisplayGeometry(float distance, float width, float height)
        {
            screenDistance = distance;
            screenWidth = width;
            screenHeight = height;
            ValidateAndClamp();
        }

        public void ValidateAndClamp()
        {
            serverBaseUrl = NormalizeBaseUrl(serverBaseUrl);
            profile = NormalizeProfile(profile);
            clientId = ClientId;
            preferredCodec = NormalizePreferredCodec(preferredCodec);
            iceGatheringTimeoutSeconds = Mathf.Max(1f, iceGatheringTimeoutSeconds);
            requestTimeoutSeconds = Mathf.Max(1f, requestTimeoutSeconds);
            firstFrameTimeoutSeconds = Mathf.Max(1f, firstFrameTimeoutSeconds);
            mediaStaleTimeoutSeconds = Mathf.Max(1f, mediaStaleTimeoutSeconds);
            reconnectInitialDelaySeconds = Mathf.Max(0.1f, reconnectInitialDelaySeconds);
            reconnectMaximumDelaySeconds = Mathf.Max(
                reconnectInitialDelaySeconds,
                reconnectMaximumDelaySeconds);
            statusPollIntervalSeconds = Mathf.Max(0.25f, statusPollIntervalSeconds);
            metricsIntervalSeconds = Mathf.Max(0.5f, metricsIntervalSeconds);
            screenDistance = Mathf.Max(0.25f, screenDistance);
            screenWidth = Mathf.Max(0.1f, screenWidth);
            screenHeight = Mathf.Max(0.1f, screenHeight);
            immersiveHorizontalFovDegrees = Mathf.Clamp(immersiveHorizontalFovDegrees, 90f, 170f);
            immersiveVerticalFovDegrees = Mathf.Clamp(immersiveVerticalFovDegrees, 60f, 140f);
        }

        private static string NormalizeBaseUrl(string value)
        {
            string candidate = string.IsNullOrWhiteSpace(value) ? DefaultBaseUrl : value.Trim();
            candidate = candidate.TrimEnd('/');

            if (Uri.TryCreate(candidate, UriKind.Absolute, out Uri uri)
                && (uri.Scheme == Uri.UriSchemeHttp || uri.Scheme == Uri.UriSchemeHttps))
            {
                return candidate;
            }

            return DefaultBaseUrl;
        }

        private static string NormalizeProfile(string value)
        {
            string candidate = string.IsNullOrWhiteSpace(value)
                ? DefaultProfile
                : value.Trim().ToLowerInvariant();

            switch (candidate)
            {
                case "high":
                case "balanced":
                case "low-latency":
                case "bad-wifi":
                    return candidate;
                default:
                    return DefaultProfile;
            }
        }

        private static string NormalizePreferredCodec(string value)
        {
            string candidate = string.IsNullOrWhiteSpace(value)
                ? DefaultPreferredCodec
                : value.Trim().ToLowerInvariant();
            return candidate == "vp8" || candidate == "h264"
                ? candidate
                : DefaultPreferredCodec;
        }
    }
}
