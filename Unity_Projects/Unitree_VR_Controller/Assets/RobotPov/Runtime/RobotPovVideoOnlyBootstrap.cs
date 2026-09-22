using UnityEngine;

namespace RobotPov
{
    /// <summary>
    /// Explicit scene bootstrap for Robot POV video. Add this component only to
    /// a dedicated video-only scene; it performs no global or automatic startup.
    /// </summary>
    [DisallowMultipleComponent]
    [AddComponentMenu("Robot POV/Video Only Bootstrap")]
    public sealed class RobotPovVideoOnlyBootstrap : MonoBehaviour
    {
        [Header("Quick setup")]
        [SerializeField]
        private string serverBaseUrl = RobotPovRuntimeConfig.DefaultBaseUrl;

        [SerializeField]
        private string profile = RobotPovRuntimeConfig.DefaultProfile;

        [SerializeField]
        private bool autoStart = true;

        [SerializeField]
        private bool headLocked = true;

        [SerializeField]
        private RobotPovVideoLayout videoLayout =
            RobotPovVideoLayout.StereoSideBySide;

        [SerializeField]
        private RobotPovDisplaySurface displaySurface =
            RobotPovDisplaySurface.CurvedImmersive;

        [SerializeField]
        private bool diagnosticsOverlay;

        [SerializeField, Min(0.25f)]
        private float screenDistance = 2f;

        [SerializeField, Min(0.1f)]
        private float screenWidth = 3f;

        [SerializeField, Min(0.1f)]
        private float screenHeight = 2.25f;

        [Header("Scene references (optional)")]
        [SerializeField]
        private Camera targetCamera;

        [SerializeField]
        private Transform worldAnchor;

        [Header("Advanced video settings")]
        [SerializeField]
        private RobotPovRuntimeConfig runtimeConfig = new RobotPovRuntimeConfig();

        private GameObject runtimeRoot;
        private RobotPovWebRtcReceiver receiver;
        private RobotPovSbsPresenter presenter;

        public RobotPovWebRtcReceiver Receiver => receiver;

        private void Awake()
        {
            EnsureRuntime();
        }

        private void OnEnable()
        {
            EnsureRuntime();
            runtimeRoot.SetActive(true);
        }

        private void OnDisable()
        {
            if (runtimeRoot != null)
            {
                runtimeRoot.SetActive(false);
            }
        }

        private void OnValidate()
        {
            ApplyQuickSetup();

            if (Application.isPlaying && runtimeRoot != null)
            {
                ConfigureRuntimeComponents();
            }
        }

        public void RestartVideo()
        {
            EnsureRuntime();
            receiver.RestartNow();
        }

        public void SetLanEndpoint(string baseUrl, string requestedProfile)
        {
            serverBaseUrl = baseUrl;
            profile = requestedProfile;
            ApplyQuickSetup();

            if (runtimeRoot != null)
            {
                ConfigureRuntimeComponents();
            }
        }

        private void EnsureRuntime()
        {
            if (runtimeRoot != null)
            {
                return;
            }

            ApplyQuickSetup();

            runtimeRoot = new GameObject("Robot POV Video Runtime");
            runtimeRoot.transform.SetParent(transform, false);
            runtimeRoot.SetActive(false);

            receiver = runtimeRoot.AddComponent<RobotPovWebRtcReceiver>();
            presenter = runtimeRoot.AddComponent<RobotPovSbsPresenter>();
            ConfigureRuntimeComponents();
        }

        private void ApplyQuickSetup()
        {
            if (runtimeConfig == null)
            {
                runtimeConfig = new RobotPovRuntimeConfig();
            }

            runtimeConfig.SetEndpoint(serverBaseUrl, profile);
            runtimeConfig.SetAutoConnect(autoStart);
            runtimeConfig.SetScreenMode(
                headLocked ? RobotPovScreenMode.HeadLocked : RobotPovScreenMode.WorldSpace);
            runtimeConfig.SetVideoLayout(videoLayout);
            runtimeConfig.SetDisplaySurface(displaySurface);
            runtimeConfig.SetDiagnosticsOverlay(diagnosticsOverlay);
            runtimeConfig.SetDisplayGeometry(
                screenDistance,
                screenWidth,
                screenHeight);
            runtimeConfig.ValidateAndClamp();
            serverBaseUrl = runtimeConfig.BaseUrl;
            profile = runtimeConfig.Profile;
        }

        private void ConfigureRuntimeComponents()
        {
            if (receiver == null || presenter == null)
            {
                return;
            }

            receiver.Configure(runtimeConfig);
            presenter.Configure(runtimeConfig, receiver, targetCamera, worldAnchor);
        }
    }
}
