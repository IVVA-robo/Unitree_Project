using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Security.Cryptography;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;
using UnityEngine.SpatialTracking;

namespace RobotPov.Editor
{
    /// <summary>
    /// Creates and builds a temporary, video-only Pico scene without touching
    /// the teleoperation scene or its EditorBuildSettings entry.
    /// </summary>
    public static class RobotPovVideoOnlyBuild
    {
        private const string BootstrapTypeName =
            "RobotPov.RobotPovVideoOnlyBootstrap";
        private const string GeneratedFolder =
            "Assets/Editor/RobotPovGenerated";
        private const string GeneratedScenePath =
            GeneratedFolder + "/RobotPovViewer.generated.unity";
        private const string DefaultApkPath =
            "Builds/RobotPovVideoOnly/RobotPovVideoOnly.apk";
        private const string VideoOnlyApplicationId =
            "com.ionosrobots.unitree.robotpov";
        private const string VideoOnlyProductName = "Unitree Robot POV";
        private const string WebRtcArchiveRelativePath =
            "Editor/OfflinePackages~/com.unity.webrtc-3.0.0-pre.8.tgz";
        private const string WebRtcArchiveSha256 =
            "9a36d45121ff6f5cef3e4c77e9f1fb263f88f7567f19ecd9250e8beef9f28d54";

        private static readonly HashSet<string> ForbiddenComponentNames =
            new HashSet<string>(StringComparer.Ordinal)
            {
                "VRUdpSender",
                "UDP_Controller",
            };

        [MenuItem("Robot POV/Build video-only Android APK")]
        public static void BuildAndroid()
        {
            BuildAndroidInternal();
        }

        /// <summary>Entry point for Unity's -executeMethod argument.</summary>
        public static void BuildAndroidBatch()
        {
            BuildAndroidInternal();
        }

        [MenuItem("Robot POV/Validate video-only build")]
        public static void ValidateVideoOnlyBuild()
        {
            Type bootstrapType = ResolveRequiredType(BootstrapTypeName);
            AndroidBuildSettingsSnapshot settings =
                new AndroidBuildSettingsSnapshot();
            try
            {
                ConfigureAndroidSettingsForBuild();
                ValidateAndroidSettings(true);
                ValidateOfflineWebRtcPackage();
                ValidateBootstrapContract(bootstrapType);
            }
            finally
            {
                settings.Restore();
            }

            Debug.Log("Robot POV video-only build validation passed.");
        }

        /// <summary>Batch-mode validation without creating a scene or APK.</summary>
        public static void ValidateVideoOnlyBuildBatch()
        {
            ValidateVideoOnlyBuild();
        }

        private static void BuildAndroidInternal()
        {
            Type bootstrapType = ResolveRequiredType(BootstrapTypeName);
            AndroidBuildSettingsSnapshot settings =
                new AndroidBuildSettingsSnapshot();
            try
            {
                ConfigureAndroidSettingsForBuild();
                ValidateAndroidSettings(true);
                ValidateOfflineWebRtcPackage();
                ValidateBootstrapContract(bootstrapType);

                if (EditorUserBuildSettings.activeBuildTarget
                        != BuildTarget.Android
                    && !EditorUserBuildSettings.SwitchActiveBuildTarget(
                        BuildTargetGroup.Android,
                        BuildTarget.Android))
                {
                    throw new InvalidOperationException(
                        "Unable to switch the project to the Android build target.");
                }

                BuildWithTemporaryScene(bootstrapType);
            }
            finally
            {
                settings.Restore();
            }
        }

        private static void BuildWithTemporaryScene(Type bootstrapType)
        {
            Scene previousActiveScene = SceneManager.GetActiveScene();
            bool replaceUntitledBatchScene =
                Application.isBatchMode
                && previousActiveScene.IsValid()
                && string.IsNullOrEmpty(previousActiveScene.path);

            if (!Application.isBatchMode
                && previousActiveScene.IsValid()
                && string.IsNullOrEmpty(previousActiveScene.path))
            {
                throw new InvalidOperationException(
                    "Save the currently open scene before building Robot POV. "
                    + "The build tool will not discard an untitled Editor scene.");
            }

            bool createdFolder = EnsureGeneratedFolder();
            Scene generatedScene = default;
            try
            {
                AssetDatabase.DeleteAsset(GeneratedScenePath);
                generatedScene = CreateVideoOnlyScene(
                    bootstrapType,
                    replaceUntitledBatchScene
                        ? NewSceneMode.Single
                        : NewSceneMode.Additive);
                if (!EditorSceneManager.SaveScene(
                        generatedScene,
                        GeneratedScenePath,
                        false))
                {
                    throw new InvalidOperationException(
                        "Failed to save the temporary Robot POV scene.");
                }

                AssertVideoOnlyScene(generatedScene, bootstrapType);
                BuildTemporarySceneOnly(GeneratedScenePath);
            }
            finally
            {
                if (generatedScene.IsValid() && generatedScene.isLoaded)
                {
                    EditorSceneManager.CloseScene(generatedScene, true);
                }

                if (previousActiveScene.IsValid() && previousActiveScene.isLoaded)
                {
                    SceneManager.SetActiveScene(previousActiveScene);
                }

                AssetDatabase.DeleteAsset(GeneratedScenePath);
                if (createdFolder && AssetDatabase.IsValidFolder(GeneratedFolder))
                {
                    AssetDatabase.DeleteAsset(GeneratedFolder);
                }

                AssetDatabase.Refresh();
            }
        }

        private static Scene CreateVideoOnlyScene(
            Type bootstrapType,
            NewSceneMode creationMode)
        {
            Scene scene = EditorSceneManager.NewScene(
                NewSceneSetup.EmptyScene,
                creationMode);
            SceneManager.SetActiveScene(scene);

            GameObject cameraObject = new GameObject("XR Main Camera");
            cameraObject.tag = "MainCamera";

            Camera camera = cameraObject.AddComponent<Camera>();
            camera.clearFlags = CameraClearFlags.SolidColor;
            camera.backgroundColor = Color.black;
            camera.nearClipPlane = 0.01f;
            camera.farClipPlane = 10.0f;
            camera.allowHDR = false;
            camera.allowMSAA = false;
            camera.stereoTargetEye = StereoTargetEyeMask.Both;
            cameraObject.AddComponent<AudioListener>();

            TrackedPoseDriver poseDriver =
                cameraObject.AddComponent<TrackedPoseDriver>();
            if (!poseDriver.SetPoseSource(
                    TrackedPoseDriver.DeviceType.GenericXRDevice,
                    TrackedPoseDriver.TrackedPose.Center))
            {
                throw new InvalidOperationException(
                    "Unable to configure the XR center-eye pose source.");
            }

            poseDriver.trackingType =
                TrackedPoseDriver.TrackingType.RotationAndPosition;
            poseDriver.updateType =
                TrackedPoseDriver.UpdateType.UpdateAndBeforeRender;
            poseDriver.UseRelativeTransform = false;

            GameObject bootstrapObject =
                new GameObject("Robot POV Video-Only Bootstrap");
            Component bootstrap = bootstrapObject.AddComponent(bootstrapType);
            ConfigureBootstrap(bootstrap);

            return scene;
        }

        private static void ConfigureBootstrap(Component bootstrap)
        {
            SerializedObject serialized = new SerializedObject(bootstrap);
            SerializedProperty serverUrl = RequireProperty(
                serialized,
                "serverBaseUrl");
            SerializedProperty profile = RequireProperty(serialized, "profile");
            RequireProperty(serialized, "autoStart").boolValue = true;
            RequireProperty(serialized, "headLocked").boolValue = true;

            string configuredUrl = Environment.GetEnvironmentVariable(
                "ROBOT_POV_SERVER_URL");
            if (!string.IsNullOrWhiteSpace(configuredUrl))
            {
                serverUrl.stringValue = configuredUrl.Trim();
            }

            string configuredProfile = Environment.GetEnvironmentVariable(
                "ROBOT_POV_PROFILE");
            if (!string.IsNullOrWhiteSpace(configuredProfile))
            {
                profile.stringValue = configuredProfile.Trim();
            }

            ValidateLanUrl(serverUrl.stringValue);
            ValidateProfile(profile.stringValue);
            serialized.ApplyModifiedPropertiesWithoutUndo();
        }

        private static SerializedProperty RequireProperty(
            SerializedObject serialized,
            string name)
        {
            SerializedProperty property = serialized.FindProperty(name);
            if (property == null)
            {
                throw new InvalidOperationException(
                    $"{BootstrapTypeName} is missing serialized field '{name}'.");
            }

            return property;
        }

        private static void AssertVideoOnlyScene(Scene scene, Type bootstrapType)
        {
            int cameraCount = 0;
            int bootstrapCount = 0;
            foreach (GameObject root in scene.GetRootGameObjects())
            {
                foreach (Component component in
                         root.GetComponentsInChildren<Component>(true))
                {
                    if (component == null)
                    {
                        throw new InvalidOperationException(
                            "Video-only scene contains a missing script.");
                    }

                    Type type = component.GetType();
                    if (ForbiddenComponentNames.Contains(type.Name)
                        || ForbiddenComponentNames.Contains(type.FullName))
                    {
                        throw new InvalidOperationException(
                            $"Forbidden control component found: {type.FullName}");
                    }

                    if (component is Camera)
                    {
                        cameraCount++;
                    }

                    if (type == bootstrapType)
                    {
                        bootstrapCount++;
                    }
                }
            }

            if (cameraCount != 1 || bootstrapCount != 1)
            {
                throw new InvalidOperationException(
                    "Video-only scene must contain exactly one Camera and "
                    + "one RobotPovVideoOnlyBootstrap.");
            }
        }

        private static void BuildTemporarySceneOnly(string scenePath)
        {
            string configuredOutput = Environment.GetEnvironmentVariable(
                "ROBOT_POV_APK_PATH");
            string outputPath = string.IsNullOrWhiteSpace(configuredOutput)
                ? DefaultApkPath
                : configuredOutput.Trim();
            outputPath = Path.GetFullPath(outputPath);
            Directory.CreateDirectory(Path.GetDirectoryName(outputPath));

            BuildPlayerOptions options = new BuildPlayerOptions
            {
                scenes = new[] { scenePath },
                locationPathName = outputPath,
                target = BuildTarget.Android,
                targetGroup = BuildTargetGroup.Android,
                options = BuildOptions.CleanBuildCache,
                extraScriptingDefines = new[] { "ROBOT_POV_VIDEO_ONLY" },
            };

            BuildReport report = BuildPipeline.BuildPlayer(options);
            if (report.summary.result != BuildResult.Succeeded)
            {
                throw new InvalidOperationException(
                    "Robot POV APK build failed: " + report.summary.result);
            }

            if (report.summary.totalErrors != 0 || !File.Exists(outputPath))
            {
                throw new InvalidOperationException(
                    "Build reported success but the APK is missing or has errors.");
            }

            Debug.Log(
                $"Robot POV video-only APK: {outputPath} "
                + $"({report.summary.totalSize} bytes)");
        }

        private static void ConfigureAndroidSettingsForBuild()
        {
            PlayerSettings.SetScriptingBackend(
                BuildTargetGroup.Android,
                ScriptingImplementation.IL2CPP);
            PlayerSettings.Android.targetArchitectures =
                AndroidArchitecture.ARM64;
            PlayerSettings.Android.forceInternetPermission = true;
            PlayerSettings.Android.optimizedFramePacing = false;
            PlayerSettings.SetUseDefaultGraphicsAPIs(BuildTarget.Android, false);
            PlayerSettings.SetGraphicsAPIs(
                BuildTarget.Android,
                new[] { GraphicsDeviceType.OpenGLES3 });
            PlayerSettings.insecureHttpOption = InsecureHttpOption.AlwaysAllowed;
            PlayerSettings.SetApplicationIdentifier(
                BuildTargetGroup.Android,
                VideoOnlyApplicationId);
            PlayerSettings.productName = VideoOnlyProductName;
            EditorUserBuildSettings.buildAppBundle = false;
            AssetDatabase.SaveAssets();
        }

        private static void ValidateAndroidSettings(bool requireInsecureHttp)
        {
            if (PlayerSettings.GetScriptingBackend(BuildTargetGroup.Android)
                != ScriptingImplementation.IL2CPP)
            {
                throw new InvalidOperationException("Android must use IL2CPP.");
            }

            if (PlayerSettings.Android.targetArchitectures
                != AndroidArchitecture.ARM64)
            {
                throw new InvalidOperationException("Android must be ARM64-only.");
            }

            GraphicsDeviceType[] graphicsApis =
                PlayerSettings.GetGraphicsAPIs(BuildTarget.Android);
            if (PlayerSettings.GetUseDefaultGraphicsAPIs(BuildTarget.Android)
                || graphicsApis.Length != 1
                || graphicsApis[0] != GraphicsDeviceType.OpenGLES3)
            {
                throw new InvalidOperationException(
                    "Android must use fixed OpenGLES3 for deterministic Pico/WebRTC "
                    + "compatibility.");
            }

            if (!PlayerSettings.Android.forceInternetPermission
                || PlayerSettings.Android.optimizedFramePacing)
            {
                throw new InvalidOperationException(
                    "INTERNET must be enabled and optimized frame pacing disabled.");
            }

            if (requireInsecureHttp
                && PlayerSettings.insecureHttpOption
                    != InsecureHttpOption.AlwaysAllowed)
            {
                throw new InvalidOperationException(
                    "Robot POV build must allow HTTP for the private exhibition LAN.");
            }

            if (requireInsecureHttp
                && (PlayerSettings.GetApplicationIdentifier(
                        BuildTargetGroup.Android)
                        != VideoOnlyApplicationId
                    || PlayerSettings.productName != VideoOnlyProductName
                    || EditorUserBuildSettings.buildAppBundle))
            {
                throw new InvalidOperationException(
                    "Robot POV build must use its separate application identity "
                    + "and APK output.");
            }

            ValidateAndroidManifest();
        }

        private static void ValidateAndroidManifest()
        {
            string manifestPath = Path.Combine(
                Application.dataPath,
                "Plugins/Android/AndroidManifest.xml");
            if (!File.Exists(manifestPath))
            {
                throw new FileNotFoundException(
                    "Custom AndroidManifest.xml is required.",
                    manifestPath);
            }

            string xml = File.ReadAllText(manifestPath);
            if (!xml.Contains("android.permission.INTERNET")
                || !xml.Contains("android:usesCleartextTraffic=\"true\"")
                || !xml.Contains("com.unity3d.player.UnityPlayerActivity")
                || !xml.Contains("android.intent.action.MAIN")
                || !xml.Contains("android.intent.category.LAUNCHER"))
            {
                throw new InvalidOperationException(
                    "Android manifest must request INTERNET, permit local HTTP, "
                    + "and retain Unity's launcher activity.");
            }
        }

        private static void ValidateOfflineWebRtcPackage()
        {
            string archivePath = Path.Combine(
                Application.dataPath,
                WebRtcArchiveRelativePath);
            if (!File.Exists(archivePath))
            {
                throw new FileNotFoundException(
                    "Vendored WebRTC package is missing.",
                    archivePath);
            }

            using (SHA256 sha256 = SHA256.Create())
            using (FileStream stream = File.OpenRead(archivePath))
            {
                string actual = string.Concat(
                    sha256.ComputeHash(stream).Select(
                        value => value.ToString("x2")));
                if (!string.Equals(
                        actual,
                        WebRtcArchiveSha256,
                        StringComparison.Ordinal))
                {
                    throw new InvalidOperationException(
                        "Vendored com.unity.webrtc archive checksum mismatch.");
                }
            }

            UnityEditor.PackageManager.PackageInfo package =
                UnityEditor.PackageManager.PackageInfo.GetAllRegisteredPackages()
                    .FirstOrDefault(item => item.name == "com.unity.webrtc");
            if (package == null || package.version != "3.0.0-pre.8")
            {
                throw new InvalidOperationException(
                    "com.unity.webrtc 3.0.0-pre.8 is not resolved.");
            }
        }

        private static void ValidateBootstrapContract(Type bootstrapType)
        {
            if (!typeof(MonoBehaviour).IsAssignableFrom(bootstrapType)
                || bootstrapType.IsAbstract)
            {
                throw new InvalidOperationException(
                    BootstrapTypeName + " must be a concrete MonoBehaviour.");
            }

            if (ResolveType("Unity.WebRTC.RTCPeerConnection") == null)
            {
                throw new InvalidOperationException(
                    "Unity.WebRTC runtime types are unavailable.");
            }
        }

        private static Type ResolveRequiredType(string fullName)
        {
            Type type = ResolveType(fullName);
            if (type == null)
            {
                throw new InvalidOperationException(
                    $"Required video-only type is missing: {fullName}");
            }

            return type;
        }

        private static Type ResolveType(string fullName)
        {
            foreach (System.Reflection.Assembly assembly in
                     AppDomain.CurrentDomain.GetAssemblies())
            {
                Type type = assembly.GetType(fullName, false);
                if (type != null)
                {
                    return type;
                }
            }

            return null;
        }

        private static void ValidateProfile(string profile)
        {
            string[] allowed =
            {
                "high",
                "balanced",
                "low-latency",
                "bad-wifi",
            };
            if (!allowed.Contains(profile))
            {
                throw new InvalidOperationException(
                    $"Unsupported ROBOT_POV_PROFILE: {profile}");
            }
        }

        private static void ValidateLanUrl(string value)
        {
            if (!Uri.TryCreate(value, UriKind.Absolute, out Uri uri)
                || (uri.Scheme != Uri.UriSchemeHttp
                    && uri.Scheme != Uri.UriSchemeHttps)
                || !IsLocalHost(uri.Host))
            {
                throw new InvalidOperationException(
                    "ROBOT_POV_SERVER_URL must be an HTTP(S) loopback, private-IP, "
                    + "or .local LAN URL.");
            }
        }

        private static bool IsLocalHost(string host)
        {
            if (string.Equals(host, "localhost", StringComparison.OrdinalIgnoreCase)
                || host.EndsWith(".local", StringComparison.OrdinalIgnoreCase))
            {
                return true;
            }

            if (!IPAddress.TryParse(host, out IPAddress address))
            {
                return false;
            }

            if (IPAddress.IsLoopback(address))
            {
                return true;
            }

            byte[] bytes = address.GetAddressBytes();
            if (bytes.Length == 4)
            {
                return bytes[0] == 10
                    || (bytes[0] == 172 && bytes[1] >= 16 && bytes[1] <= 31)
                    || (bytes[0] == 192 && bytes[1] == 168);
            }

            return bytes.Length == 16
                && ((bytes[0] & 0xfe) == 0xfc
                    || (bytes[0] == 0xfe && (bytes[1] & 0xc0) == 0x80));
        }

        private static bool EnsureGeneratedFolder()
        {
            if (AssetDatabase.IsValidFolder(GeneratedFolder))
            {
                return false;
            }

            string guid = AssetDatabase.CreateFolder(
                "Assets/Editor",
                "RobotPovGenerated");
            if (string.IsNullOrEmpty(guid))
            {
                throw new InvalidOperationException(
                    "Unable to create the temporary scene folder.");
            }

            return true;
        }

        private sealed class AndroidBuildSettingsSnapshot
        {
            private readonly string projectSettingsAssetPath;
            private readonly byte[] projectSettingsAssetBytes;
            private readonly ScriptingImplementation scriptingBackend;
            private readonly AndroidArchitecture architectures;
            private readonly bool forceInternetPermission;
            private readonly bool optimizedFramePacing;
            private readonly bool useDefaultGraphicsApis;
            private readonly GraphicsDeviceType[] graphicsApis;
            private readonly InsecureHttpOption insecureHttpOption;
            private readonly string applicationIdentifier;
            private readonly string productName;
            private readonly bool buildAppBundle;

            public AndroidBuildSettingsSnapshot()
            {
                projectSettingsAssetPath = Path.Combine(
                    Directory.GetParent(Application.dataPath).FullName,
                    "ProjectSettings/ProjectSettings.asset");
                projectSettingsAssetBytes =
                    File.ReadAllBytes(projectSettingsAssetPath);
                scriptingBackend = PlayerSettings.GetScriptingBackend(
                    BuildTargetGroup.Android);
                architectures = PlayerSettings.Android.targetArchitectures;
                forceInternetPermission =
                    PlayerSettings.Android.forceInternetPermission;
                optimizedFramePacing =
                    PlayerSettings.Android.optimizedFramePacing;
                useDefaultGraphicsApis =
                    PlayerSettings.GetUseDefaultGraphicsAPIs(BuildTarget.Android);
                graphicsApis = PlayerSettings.GetGraphicsAPIs(BuildTarget.Android);
                insecureHttpOption = PlayerSettings.insecureHttpOption;
                applicationIdentifier =
                    PlayerSettings.GetApplicationIdentifier(
                        BuildTargetGroup.Android);
                productName = PlayerSettings.productName;
                buildAppBundle = EditorUserBuildSettings.buildAppBundle;
            }

            public void Restore()
            {
                PlayerSettings.SetScriptingBackend(
                    BuildTargetGroup.Android,
                    scriptingBackend);
                PlayerSettings.Android.targetArchitectures = architectures;
                PlayerSettings.Android.forceInternetPermission =
                    forceInternetPermission;
                PlayerSettings.Android.optimizedFramePacing =
                    optimizedFramePacing;
                PlayerSettings.SetUseDefaultGraphicsAPIs(
                    BuildTarget.Android,
                    useDefaultGraphicsApis);
                PlayerSettings.SetGraphicsAPIs(BuildTarget.Android, graphicsApis);
                PlayerSettings.insecureHttpOption = insecureHttpOption;
                PlayerSettings.SetApplicationIdentifier(
                    BuildTargetGroup.Android,
                    applicationIdentifier);
                PlayerSettings.productName = productName;
                EditorUserBuildSettings.buildAppBundle = buildAppBundle;
                AssetDatabase.SaveAssets();

                // SetApplicationIdentifier turns Unity's implicit default ID into
                // an explicit per-platform override even when restoring the same
                // text. Restore the serialized settings bytes as well so a
                // validation/build cannot leave that hidden project mutation.
                if (!File.ReadAllBytes(projectSettingsAssetPath)
                        .SequenceEqual(projectSettingsAssetBytes))
                {
                    File.WriteAllBytes(
                        projectSettingsAssetPath,
                        projectSettingsAssetBytes);
                    AssetDatabase.Refresh(ImportAssetOptions.ForceSynchronousImport);
                }
            }
        }
    }
}
