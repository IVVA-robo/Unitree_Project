#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
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
    /// Builds a combined Pico demo from a temporary copy of the existing teleop
    /// scene. The source scene and EditorBuildSettings are never modified.
    ///
    /// This combines the receive-only Robot POV display with the existing UDP
    /// intent sender. It does not add a robot transport; the ROS side must remain
    /// bridge-only/dry-run while this demo is evaluated.
    /// </summary>
    public static class RobotPovCombinedDemoBuild
    {
        private const string SourceScenePath = "Assets/Scenes/SampleScene.unity";
        private const string GeneratedFolder =
            "Assets/Editor/RobotPovCombinedGenerated";
        private const string GeneratedScenePath =
            GeneratedFolder + "/RobotPovCombined.generated.unity";
        private const string DefaultApkPath =
            "Builds/CombinedDemo/UnitreeR1TelepresenceDemo.apk";

        private const string HostEnvironmentVariable = "UNITREE_LAPTOP_HOST";
        private const string ProfileEnvironmentVariable = "ROBOT_POV_PROFILE";
        private const string OutputEnvironmentVariable =
            "UNITREE_COMBINED_APK_PATH";

        private const string DefaultLaptopHost = "192.168.8.9";
        // Pico decodes MJPEG on Unity's main thread. Start conservatively so
        // video cannot starve the 72 Hz UDP sender during the first combined test.
        private const string DefaultProfile = "bad-wifi";
        private const int VideoPort = 8080;
        private const int UdpPort = 9090;

        private const float ScreenDistance = 2.0f;
        private const float ScreenWidth = 3.0f;
        private const float ScreenHeight = 1.6875f;
        private const float FloatTolerance = 0.0001f;

        private const string CombinedApplicationId =
            "com.ionosrobots.unitree.telepresencedemo";
        private const string CombinedProductName =
            "Unitree R1 Telepresence Demo";

        [MenuItem("Robot POV/Validate combined telepresence build")]
        public static void ValidateCombinedBuild()
        {
            BuildConfiguration configuration = ReadConfiguration();
            AndroidBuildSettingsSnapshot settings =
                new AndroidBuildSettingsSnapshot();
            try
            {
                ConfigureAndroidSettings();
                ValidateAndroidSettings();
                WithPreparedScene(
                    configuration,
                    delegate(Scene scene)
                    {
                        AssertCombinedScene(scene, configuration);
                    });
            }
            finally
            {
                settings.Restore();
            }

            Debug.Log(
                "Combined telepresence validation passed: "
                + configuration.VideoBaseUrl
                + ", UDP "
                + configuration.LaptopHost
                + ":"
                + UdpPort
                + ", mono 16:9, profile="
                + configuration.Profile);
        }

        /// <summary>Entry point for Unity's -executeMethod argument.</summary>
        public static void ValidateCombinedBuildBatch()
        {
            ValidateCombinedBuild();
        }

        [MenuItem("Robot POV/Build combined telepresence Android APK")]
        public static void BuildAndroid()
        {
            BuildAndroidInternal();
        }

        /// <summary>Entry point for Unity's -executeMethod argument.</summary>
        public static void BuildAndroidBatch()
        {
            BuildAndroidInternal();
        }

        private static void BuildAndroidInternal()
        {
            BuildConfiguration configuration = ReadConfiguration();
            AndroidBuildSettingsSnapshot settings =
                new AndroidBuildSettingsSnapshot();
            try
            {
                ConfigureAndroidSettings();
                ValidateAndroidSettings();

                if (EditorUserBuildSettings.activeBuildTarget
                        != BuildTarget.Android
                    && !EditorUserBuildSettings.SwitchActiveBuildTarget(
                        BuildTargetGroup.Android,
                        BuildTarget.Android))
                {
                    throw new InvalidOperationException(
                        "Unable to switch the project to the Android build target.");
                }

                WithPreparedScene(
                    configuration,
                    delegate(Scene scene)
                    {
                        AssertCombinedScene(scene, configuration);
                        BuildPreparedScene(configuration);
                    });
            }
            finally
            {
                settings.Restore();
            }
        }

        private static void WithPreparedScene(
            BuildConfiguration configuration,
            Action<Scene> action)
        {
            RequireSourceScene();

            Scene previousActiveScene = SceneManager.GetActiveScene();
            bool createdFolder = EnsureGeneratedFolder();
            Scene generatedScene = default;
            try
            {
                AssetDatabase.DeleteAsset(GeneratedScenePath);
                if (!AssetDatabase.CopyAsset(
                        SourceScenePath,
                        GeneratedScenePath))
                {
                    throw new InvalidOperationException(
                        "Unable to copy the teleop scene for the combined build.");
                }

                AssetDatabase.ImportAsset(
                    GeneratedScenePath,
                    ImportAssetOptions.ForceSynchronousImport);
                generatedScene = EditorSceneManager.OpenScene(
                    GeneratedScenePath,
                    OpenSceneMode.Additive);
                SceneManager.SetActiveScene(generatedScene);

                PrepareCombinedScene(generatedScene, configuration);
                if (!EditorSceneManager.SaveScene(
                        generatedScene,
                        GeneratedScenePath,
                        false))
                {
                    throw new InvalidOperationException(
                        "Unable to save the temporary combined scene.");
                }

                action(generatedScene);
            }
            finally
            {
                if (generatedScene.IsValid() && generatedScene.isLoaded)
                {
                    EditorSceneManager.CloseScene(generatedScene, true);
                }

                if (previousActiveScene.IsValid()
                    && previousActiveScene.isLoaded)
                {
                    SceneManager.SetActiveScene(previousActiveScene);
                }

                AssetDatabase.DeleteAsset(GeneratedScenePath);
                if (createdFolder
                    && AssetDatabase.IsValidFolder(GeneratedFolder))
                {
                    AssetDatabase.DeleteAsset(GeneratedFolder);
                }

                AssetDatabase.Refresh();
            }
        }

        private static void PrepareCombinedScene(
            Scene scene,
            BuildConfiguration configuration)
        {
            AssertNoMissingScripts(scene);

            List<Camera> cameras = ComponentsInScene<Camera>(scene);
            List<VRUdpSender> senders = ComponentsInScene<VRUdpSender>(scene);
            List<UDP_Controller> legacySenders =
                ComponentsInScene<UDP_Controller>(scene);
            List<RobotPovVideoOnlyBootstrap> existingBootstraps =
                ComponentsInScene<RobotPovVideoOnlyBootstrap>(scene);

            if (cameras.Count != 1
                || senders.Count != 1
                || legacySenders.Count != 0
                || existingBootstraps.Count != 0)
            {
                throw new InvalidOperationException(
                    "Source teleop scene must contain exactly one Camera, one "
                    + "VRUdpSender, no UDP_Controller, and no pre-existing "
                    + "RobotPovVideoOnlyBootstrap.");
            }

            ConfigureUdpSender(senders[0], configuration);
            ConfigureXrCamera(cameras[0]);

            GameObject videoObject = new GameObject(
                "Robot POV Combined Video");
            RobotPovVideoOnlyBootstrap bootstrap =
                videoObject.AddComponent<RobotPovVideoOnlyBootstrap>();
            ConfigureVideoBootstrap(
                bootstrap,
                cameras[0],
                configuration);
        }

        private static void ConfigureXrCamera(Camera camera)
        {
            TrackedPoseDriver poseDriver =
                camera.GetComponent<TrackedPoseDriver>();
            if (poseDriver == null)
            {
                poseDriver = camera.gameObject.AddComponent<TrackedPoseDriver>();
            }

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
        }

        private static void ConfigureUdpSender(
            VRUdpSender sender,
            BuildConfiguration configuration)
        {
            SerializedObject serialized = new SerializedObject(sender);
            RequireProperty(serialized, "host").stringValue =
                configuration.LaptopHost;
            RequireProperty(serialized, "port").intValue = UdpPort;
            RequireProperty(serialized, "autoDiscoverHost").boolValue = true;
            RequireProperty(serialized, "discoveryPort").intValue = 9091;
            RequireProperty(
                serialized,
                "requireLeftGripAsDeadman").boolValue = true;
            serialized.ApplyModifiedPropertiesWithoutUndo();
        }

        private static void ConfigureVideoBootstrap(
            RobotPovVideoOnlyBootstrap bootstrap,
            Camera targetCamera,
            BuildConfiguration configuration)
        {
            SerializedObject serialized = new SerializedObject(bootstrap);
            RequireProperty(serialized, "serverBaseUrl").stringValue =
                configuration.VideoBaseUrl;
            RequireProperty(serialized, "profile").stringValue =
                configuration.Profile;
            RequireProperty(serialized, "autoStart").boolValue = true;
            RequireProperty(serialized, "headLocked").boolValue = true;
            SetEnum(RequireProperty(serialized, "displaySurface"), "FlatQuad");
            SetEnum(RequireProperty(serialized, "videoLayout"), "Mono");
            RequireProperty(serialized, "screenDistance").floatValue =
                ScreenDistance;
            RequireProperty(serialized, "screenWidth").floatValue =
                ScreenWidth;
            RequireProperty(serialized, "screenHeight").floatValue =
                ScreenHeight;
            RequireProperty(serialized, "targetCamera").objectReferenceValue =
                targetCamera;
            RequireProperty(serialized, "worldAnchor").objectReferenceValue =
                null;

            SerializedProperty runtimeConfig = RequireProperty(
                serialized,
                "runtimeConfig");
            RequireRelative(runtimeConfig, "serverBaseUrl").stringValue =
                configuration.VideoBaseUrl;
            RequireRelative(runtimeConfig, "profile").stringValue =
                configuration.Profile;
            RequireRelative(runtimeConfig, "connectOnEnable").boolValue = true;
            SetEnum(RequireRelative(runtimeConfig, "displaySurface"), "FlatQuad");
            SetEnum(RequireRelative(runtimeConfig, "videoLayout"), "Mono");
            RequireRelative(runtimeConfig, "swapEyes").boolValue = false;
            RequireRelative(runtimeConfig, "flipVertical").boolValue = false;
            SetEnum(
                RequireRelative(runtimeConfig, "screenMode"),
                "HeadLocked");
            RequireRelative(runtimeConfig, "screenDistance").floatValue =
                ScreenDistance;
            RequireRelative(runtimeConfig, "screenWidth").floatValue =
                ScreenWidth;
            RequireRelative(runtimeConfig, "screenHeight").floatValue =
                ScreenHeight;
            serialized.ApplyModifiedPropertiesWithoutUndo();
        }

        private static void AssertCombinedScene(
            Scene scene,
            BuildConfiguration configuration)
        {
            AssertNoMissingScripts(scene);

            List<Camera> cameras = ComponentsInScene<Camera>(scene);
            List<VRUdpSender> senders = ComponentsInScene<VRUdpSender>(scene);
            List<RobotPovVideoOnlyBootstrap> bootstraps =
                ComponentsInScene<RobotPovVideoOnlyBootstrap>(scene);
            List<TrackedPoseDriver> poseDrivers =
                ComponentsInScene<TrackedPoseDriver>(scene);
            List<UDP_Controller> legacySenders =
                ComponentsInScene<UDP_Controller>(scene);

            if (cameras.Count != 1
                || senders.Count != 1
                || bootstraps.Count != 1
                || poseDrivers.Count != 1
                || legacySenders.Count != 0)
            {
                throw new InvalidOperationException(
                    "Combined scene invariant failed: expected exactly 1 Camera, "
                    + "1 TrackedPoseDriver, 1 VRUdpSender, 1 "
                    + "RobotPovVideoOnlyBootstrap, and 0 UDP_Controller "
                    + "components.");
            }

            if (poseDrivers[0].gameObject != cameras[0].gameObject)
            {
                throw new InvalidOperationException(
                    "TrackedPoseDriver must be attached to the single XR camera.");
            }

            VRUdpSender sender = senders[0];
            RobotPovVideoOnlyBootstrap bootstrap = bootstraps[0];
            if (sender.gameObject == bootstrap.gameObject
                || bootstrap.transform.IsChildOf(sender.transform)
                || sender.transform.IsChildOf(bootstrap.transform))
            {
                throw new InvalidOperationException(
                    "Video and UDP components must have independent scene lifecycles.");
            }

            SerializedObject senderSerialized = new SerializedObject(sender);
            if (RequireProperty(senderSerialized, "host").stringValue
                    != configuration.LaptopHost
                || RequireProperty(senderSerialized, "port").intValue
                    != UdpPort
                || !RequireProperty(
                        senderSerialized,
                        "autoDiscoverHost").boolValue
                || RequireProperty(
                        senderSerialized,
                        "discoveryPort").intValue
                    != 9091
                || !RequireProperty(
                        senderSerialized,
                        "requireLeftGripAsDeadman").boolValue)
            {
                throw new InvalidOperationException(
                    "VRUdpSender is not configured for the common laptop host, "
                    + "UDP port 9090, and mandatory grip deadman.");
            }

            SerializedObject videoSerialized = new SerializedObject(bootstrap);
            if (RequireProperty(videoSerialized, "serverBaseUrl").stringValue
                    != configuration.VideoBaseUrl
                || RequireProperty(videoSerialized, "profile").stringValue
                    != configuration.Profile
                || !RequireProperty(videoSerialized, "autoStart").boolValue
                || !RequireProperty(videoSerialized, "headLocked").boolValue
                || RequireProperty(videoSerialized, "displaySurface").enumValueIndex
                    != EnumIndex(RequireProperty(videoSerialized, "displaySurface"), "FlatQuad")
                || RequireProperty(
                        videoSerialized,
                        "videoLayout").enumValueIndex
                    != EnumIndex(
                        RequireProperty(videoSerialized, "videoLayout"),
                        "Mono")
                || Math.Abs(
                        RequireProperty(
                            videoSerialized,
                            "screenWidth").floatValue
                        - ScreenWidth) > FloatTolerance
                || Math.Abs(
                        RequireProperty(
                            videoSerialized,
                            "screenHeight").floatValue
                        - ScreenHeight) > FloatTolerance
                || Math.Abs(
                        RequireProperty(
                            videoSerialized,
                            "screenDistance").floatValue
                        - ScreenDistance) > FloatTolerance
                || RequireProperty(
                        videoSerialized,
                        "targetCamera").objectReferenceValue
                    != cameras[0])
            {
                throw new InvalidOperationException(
                    "Robot POV bootstrap endpoint, profile, camera, or startup "
                    + "configuration is invalid.");
            }

            SerializedProperty runtimeConfig = RequireProperty(
                videoSerialized,
                "runtimeConfig");
            SerializedProperty layout = RequireRelative(
                runtimeConfig,
                "videoLayout");
            SerializedProperty screenMode = RequireRelative(
                runtimeConfig,
                "screenMode");
            float width = RequireRelative(
                runtimeConfig,
                "screenWidth").floatValue;
            float height = RequireRelative(
                runtimeConfig,
                "screenHeight").floatValue;
            float distance = RequireRelative(
                runtimeConfig,
                "screenDistance").floatValue;

            if (layout.enumValueIndex != EnumIndex(layout, "Mono")
                || RequireRelative(runtimeConfig, "displaySurface").enumValueIndex
                    != EnumIndex(RequireRelative(runtimeConfig, "displaySurface"), "FlatQuad")
                || screenMode.enumValueIndex
                    != EnumIndex(screenMode, "HeadLocked")
                || Math.Abs(distance - ScreenDistance) > FloatTolerance
                || Math.Abs(width - ScreenWidth) > FloatTolerance
                || Math.Abs(height - ScreenHeight) > FloatTolerance
                || Math.Abs((width / height) - (16.0f / 9.0f))
                    > FloatTolerance)
            {
                throw new InvalidOperationException(
                    "Robot POV display must be head-locked mono 16:9 at "
                    + "3.0 x 1.6875 metres.");
            }

            Uri videoUri = new Uri(configuration.VideoBaseUrl);
            if (!string.Equals(
                    videoUri.Host,
                    configuration.LaptopHost,
                    StringComparison.OrdinalIgnoreCase)
                || videoUri.Scheme != Uri.UriSchemeHttp
                || videoUri.Port != VideoPort)
            {
                throw new InvalidOperationException(
                    "Video and UDP must use the same private-LAN laptop host; "
                    + "video must use HTTP port 8080.");
            }
        }

        private static void BuildPreparedScene(
            BuildConfiguration configuration)
        {
            string configuredOutput = Environment.GetEnvironmentVariable(
                OutputEnvironmentVariable);
            string outputPath = string.IsNullOrWhiteSpace(configuredOutput)
                ? DefaultApkPath
                : configuredOutput.Trim();
            outputPath = Path.GetFullPath(outputPath);

            string outputDirectory = Path.GetDirectoryName(outputPath);
            if (string.IsNullOrWhiteSpace(outputDirectory))
            {
                throw new InvalidOperationException(
                    "Combined APK output directory is invalid.");
            }

            Directory.CreateDirectory(outputDirectory);
            BuildPlayerOptions options = new BuildPlayerOptions
            {
                scenes = new[] { GeneratedScenePath },
                locationPathName = outputPath,
                target = BuildTarget.Android,
                targetGroup = BuildTargetGroup.Android,
                options = BuildOptions.CleanBuildCache,
                extraScriptingDefines = new[]
                {
                    "ROBOT_POV_COMBINED_DEMO",
                },
            };

            BuildReport report = BuildPipeline.BuildPlayer(options);
            if (report.summary.result != BuildResult.Succeeded
                || report.summary.totalErrors != 0
                || !File.Exists(outputPath))
            {
                throw new InvalidOperationException(
                    "Combined Android build failed: "
                    + report.summary.result
                    + ", errors="
                    + report.summary.totalErrors);
            }

            Debug.Log(
                "Combined telepresence APK: "
                + outputPath
                + " ("
                + report.summary.totalSize
                + " bytes, host="
                + configuration.LaptopHost
                + ", profile="
                + configuration.Profile
                + ")");
        }

        private static BuildConfiguration ReadConfiguration()
        {
            string host = Environment.GetEnvironmentVariable(
                HostEnvironmentVariable);
            host = string.IsNullOrWhiteSpace(host)
                ? DefaultLaptopHost
                : host.Trim();
            ValidatePrivateLanHost(host);

            string profile = Environment.GetEnvironmentVariable(
                ProfileEnvironmentVariable);
            profile = string.IsNullOrWhiteSpace(profile)
                ? DefaultProfile
                : profile.Trim().ToLowerInvariant();
            ValidateProfile(profile);

            return new BuildConfiguration(
                host,
                "http://" + host + ":" + VideoPort,
                profile);
        }

        private static void ValidatePrivateLanHost(string host)
        {
            if (host.Contains(":")
                || host.Contains("/")
                || host.Contains("\\")
                || host.Contains(" "))
            {
                throw new InvalidOperationException(
                    HostEnvironmentVariable
                    + " must contain only a host name or IPv4 address, without "
                    + "a scheme, path, or port.");
            }

            if (host.EndsWith(".local", StringComparison.OrdinalIgnoreCase))
            {
                return;
            }

            if (!IPAddress.TryParse(host, out IPAddress address))
            {
                throw new InvalidOperationException(
                    HostEnvironmentVariable
                    + " must be a private IPv4 address or a .local host name.");
            }

            byte[] bytes = address.GetAddressBytes();
            bool isPrivateIpv4 = bytes.Length == 4
                && (bytes[0] == 10
                    || (bytes[0] == 100 && bytes[1] >= 64 && bytes[1] <= 127)
                    || (bytes[0] == 172
                        && bytes[1] >= 16
                        && bytes[1] <= 31)
                    || (bytes[0] == 192 && bytes[1] == 168));
            if (!isPrivateIpv4 || IPAddress.IsLoopback(address))
            {
                throw new InvalidOperationException(
                    HostEnvironmentVariable
                    + " must identify the laptop on the private LAN; loopback "
                    + "and public addresses are forbidden.");
            }
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
                    "Unsupported "
                    + ProfileEnvironmentVariable
                    + ": "
                    + profile);
            }
        }

        private static void ConfigureAndroidSettings()
        {
            PlayerSettings.SetScriptingBackend(
                BuildTargetGroup.Android,
                ScriptingImplementation.IL2CPP);
            PlayerSettings.Android.targetArchitectures =
                AndroidArchitecture.ARM64;
            PlayerSettings.Android.forceInternetPermission = true;
            PlayerSettings.Android.optimizedFramePacing = false;
            PlayerSettings.SetUseDefaultGraphicsAPIs(
                BuildTarget.Android,
                false);
            PlayerSettings.SetGraphicsAPIs(
                BuildTarget.Android,
                new[] { GraphicsDeviceType.OpenGLES3 });
            PlayerSettings.insecureHttpOption =
                InsecureHttpOption.AlwaysAllowed;
            PlayerSettings.SetApplicationIdentifier(
                BuildTargetGroup.Android,
                CombinedApplicationId);
            PlayerSettings.productName = CombinedProductName;
            EditorUserBuildSettings.buildAppBundle = false;
            AssetDatabase.SaveAssets();
        }

        private static void ValidateAndroidSettings()
        {
            if (PlayerSettings.GetScriptingBackend(BuildTargetGroup.Android)
                    != ScriptingImplementation.IL2CPP
                || PlayerSettings.Android.targetArchitectures
                    != AndroidArchitecture.ARM64)
            {
                throw new InvalidOperationException(
                    "Combined Android build must use IL2CPP and ARM64 only.");
            }

            GraphicsDeviceType[] graphicsApis =
                PlayerSettings.GetGraphicsAPIs(BuildTarget.Android);
            if (PlayerSettings.GetUseDefaultGraphicsAPIs(BuildTarget.Android)
                || graphicsApis.Length != 1
                || graphicsApis[0] != GraphicsDeviceType.OpenGLES3)
            {
                throw new InvalidOperationException(
                    "Combined Android build must use fixed OpenGLES3.");
            }

            if (!PlayerSettings.Android.forceInternetPermission
                || PlayerSettings.Android.optimizedFramePacing
                || PlayerSettings.insecureHttpOption
                    != InsecureHttpOption.AlwaysAllowed
                || PlayerSettings.GetApplicationIdentifier(
                    BuildTargetGroup.Android) != CombinedApplicationId
                || PlayerSettings.productName != CombinedProductName
                || EditorUserBuildSettings.buildAppBundle)
            {
                throw new InvalidOperationException(
                    "Combined Android identity/network settings are invalid.");
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
                || !xml.Contains("com.unity3d.player.UnityPlayerActivity"))
            {
                throw new InvalidOperationException(
                    "Android manifest must permit private-LAN HTTP and retain "
                    + "Unity's launcher activity.");
            }
        }

        private static void RequireSourceScene()
        {
            if (AssetDatabase.LoadAssetAtPath<SceneAsset>(SourceScenePath)
                == null)
            {
                throw new FileNotFoundException(
                    "The canonical teleop scene is missing.",
                    SourceScenePath);
            }
        }

        private static void AssertNoMissingScripts(Scene scene)
        {
            foreach (GameObject root in scene.GetRootGameObjects())
            {
                foreach (Component component in
                         root.GetComponentsInChildren<Component>(true))
                {
                    if (component == null)
                    {
                        throw new InvalidOperationException(
                            "Combined scene contains a missing script.");
                    }
                }
            }
        }

        private static List<T> ComponentsInScene<T>(Scene scene)
            where T : Component
        {
            List<T> result = new List<T>();
            foreach (GameObject root in scene.GetRootGameObjects())
            {
                result.AddRange(root.GetComponentsInChildren<T>(true));
            }

            return result;
        }

        private static SerializedProperty RequireProperty(
            SerializedObject serialized,
            string name)
        {
            SerializedProperty property = serialized.FindProperty(name);
            if (property == null)
            {
                throw new InvalidOperationException(
                    serialized.targetObject.GetType().FullName
                    + " is missing serialized property '"
                    + name
                    + "'.");
            }

            return property;
        }

        private static SerializedProperty RequireRelative(
            SerializedProperty parent,
            string name)
        {
            SerializedProperty property = parent.FindPropertyRelative(name);
            if (property == null)
            {
                throw new InvalidOperationException(
                    "RobotPovRuntimeConfig is missing serialized property '"
                    + name
                    + "'.");
            }

            return property;
        }

        private static void SetEnum(
            SerializedProperty property,
            string value)
        {
            property.enumValueIndex = EnumIndex(property, value);
        }

        private static int EnumIndex(
            SerializedProperty property,
            string value)
        {
            int index = Array.IndexOf(property.enumNames, value);
            if (index < 0)
            {
                throw new InvalidOperationException(
                    "Serialized enum '"
                    + property.propertyPath
                    + "' has no value '"
                    + value
                    + "'.");
            }

            return index;
        }

        private static bool EnsureGeneratedFolder()
        {
            if (AssetDatabase.IsValidFolder(GeneratedFolder))
            {
                return false;
            }

            string guid = AssetDatabase.CreateFolder(
                "Assets/Editor",
                "RobotPovCombinedGenerated");
            if (string.IsNullOrEmpty(guid))
            {
                throw new InvalidOperationException(
                    "Unable to create the temporary combined-scene folder.");
            }

            return true;
        }

        private sealed class BuildConfiguration
        {
            public BuildConfiguration(
                string laptopHost,
                string videoBaseUrl,
                string profile)
            {
                LaptopHost = laptopHost;
                VideoBaseUrl = videoBaseUrl;
                Profile = profile;
            }

            public string LaptopHost { get; private set; }
            public string VideoBaseUrl { get; private set; }
            public string Profile { get; private set; }
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
                    PlayerSettings.GetUseDefaultGraphicsAPIs(
                        BuildTarget.Android);
                graphicsApis = PlayerSettings.GetGraphicsAPIs(
                    BuildTarget.Android);
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
                PlayerSettings.SetGraphicsAPIs(
                    BuildTarget.Android,
                    graphicsApis);
                PlayerSettings.insecureHttpOption = insecureHttpOption;
                PlayerSettings.SetApplicationIdentifier(
                    BuildTargetGroup.Android,
                    applicationIdentifier);
                PlayerSettings.productName = productName;
                EditorUserBuildSettings.buildAppBundle = buildAppBundle;
                AssetDatabase.SaveAssets();

                // SetApplicationIdentifier may materialize an implicit default
                // as a serialized Android override. Restore the exact file bytes
                // so validation/build leaves no hidden ProjectSettings mutation.
                if (!File.ReadAllBytes(projectSettingsAssetPath)
                        .SequenceEqual(projectSettingsAssetBytes))
                {
                    File.WriteAllBytes(
                        projectSettingsAssetPath,
                        projectSettingsAssetBytes);
                    AssetDatabase.Refresh(
                        ImportAssetOptions.ForceSynchronousImport);
                }
            }
        }
    }
}
#endif
