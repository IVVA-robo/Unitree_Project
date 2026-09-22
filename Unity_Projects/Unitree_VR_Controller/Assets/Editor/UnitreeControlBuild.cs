#if UNITY_EDITOR
using System;
using System.IO;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

/// <summary>
/// Headless Android builder for the regular VR teleoperation scene.
/// The output path is supplied through UNITREE_CONTROL_APK_PATH so a
/// network-specific demo APK can be built without changing project settings.
/// </summary>
public static class UnitreeControlBuild
{
    private const string ScenePath = "Assets/Scenes/SampleScene.unity";

    public static void BuildAndroidBatch()
    {
        string output = Environment.GetEnvironmentVariable(
            "UNITREE_CONTROL_APK_PATH");
        if (string.IsNullOrWhiteSpace(output))
        {
            output = "Builds/Unitree_VR_Controller.apk";
        }

        output = Path.GetFullPath(output);
        string directory = Path.GetDirectoryName(output);
        if (!string.IsNullOrWhiteSpace(directory))
        {
            Directory.CreateDirectory(directory);
        }

        if (EditorUserBuildSettings.activeBuildTarget != BuildTarget.Android
            && !EditorUserBuildSettings.SwitchActiveBuildTarget(
                BuildTargetGroup.Android, BuildTarget.Android))
        {
            throw new InvalidOperationException(
                "Could not switch Unity to the Android build target.");
        }

        BuildPlayerOptions options = new BuildPlayerOptions
        {
            scenes = new[] { ScenePath },
            locationPathName = output,
            target = BuildTarget.Android,
            targetGroup = BuildTargetGroup.Android,
            options = BuildOptions.None,
        };

        BuildReport report = BuildPipeline.BuildPlayer(options);
        if (report.summary.result != BuildResult.Succeeded)
        {
            throw new InvalidOperationException(
                $"Android control build failed: {report.summary.result}");
        }

        Debug.Log(
            $"Android control APK built: {output} "
            + $"({report.summary.totalSize} bytes)");
    }
}
#endif
