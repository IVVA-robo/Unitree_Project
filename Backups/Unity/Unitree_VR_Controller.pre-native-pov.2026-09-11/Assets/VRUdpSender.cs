using System;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using UnityEngine.XR;

// Attach this component to one persistent GameObject in the Unity scene.
// It uses OpenXR/CommonUsages, so no Pico-specific API is required.
public sealed class VRUdpSender : MonoBehaviour
{
    [Header("ROS host")]
    [SerializeField] private string host = "192.168.8.131";
    [SerializeField] private int port = 9090;

    [Header("Streaming")]
    [Range(10.0f, 120.0f)]
    [SerializeField] private float sendRateHz = 72.0f;

    // Hold the left grip to permit motion; index triggers remain free for fingers.
    // Change this mapping if the project uses a dedicated physical deadman button.
    [SerializeField] private bool requireLeftGripAsDeadman = true;

    private UdpClient udp;
    private InputDevice leftDevice;
    private InputDevice rightDevice;
    private InputDevice headDevice;
    private float nextSendTime;
    private uint sequence;

    [Serializable]
    private sealed class PoseWire
    {
        public float[] p;
        public float[] q;
    }

    [Serializable]
    private sealed class SticksWire
    {
        public float[] left;
        public float[] right;
    }

    [Serializable]
    private sealed class PacketWire
    {
        public int v;
        public uint seq;
        public long client_time_ms;
        public PoseWire left;
        public PoseWire right;
        public PoseWire head;
        public SticksWire sticks;
        public float[] triggers;
        public bool deadman;
    }

    private void OnEnable()
    {
        udp = new UdpClient();
        udp.Connect(host, port);
        AcquireDevices();
    }

    private void OnDisable()
    {
        // Send an explicit zero/deadman=false snapshot before closing when possible.
        if (udp != null)
        {
            try { SendSnapshot(false); }
            catch (Exception) { /* The ROS watchdog remains the final fallback. */ }
            udp.Close();
            udp = null;
        }
    }

    private void Update()
    {
        if (Time.unscaledTime < nextSendTime)
            return;

        nextSendTime = Time.unscaledTime + 1.0f / sendRateHz;
        if (!leftDevice.isValid || !rightDevice.isValid || !headDevice.isValid)
            AcquireDevices();

        bool deadman = !requireLeftGripAsDeadman;
        if (requireLeftGripAsDeadman)
            leftDevice.TryGetFeatureValue(CommonUsages.gripButton, out deadman);

        SendSnapshot(deadman);
    }

    private void AcquireDevices()
    {
        leftDevice = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
        rightDevice = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
        headDevice = InputDevices.GetDeviceAtXRNode(XRNode.Head);
    }

    private void SendSnapshot(bool deadman)
    {
        if (udp == null)
            return;

        Vector2 leftStick = ReadAxis(leftDevice);
        Vector2 rightStick = ReadAxis(rightDevice);
        float leftTrigger = ReadTrigger(leftDevice);
        float rightTrigger = ReadTrigger(rightDevice);

        PacketWire packet = new PacketWire
        {
            v = 1,
            seq = sequence++,
            client_time_ms = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(),
            left = ReadPose(leftDevice),
            right = ReadPose(rightDevice),
            head = ReadPose(headDevice),
            sticks = new SticksWire
            {
                left = new[] { leftStick.x, leftStick.y },
                right = new[] { rightStick.x, rightStick.y }
            },
            triggers = new[] { leftTrigger, rightTrigger },
            deadman = deadman && leftDevice.isValid && rightDevice.isValid && headDevice.isValid
        };

        // JsonUtility emits JSON numbers with invariant decimal separators.
        byte[] bytes = Encoding.UTF8.GetBytes(JsonUtility.ToJson(packet));
        if (bytes.Length > 1200)
        {
            Debug.LogError($"VR UDP packet unexpectedly large: {bytes.Length} bytes");
            return;
        }
        udp.Send(bytes, bytes.Length);
    }

    private static Vector2 ReadAxis(InputDevice device)
    {
        return device.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 axis)
            ? Vector2.ClampMagnitude(axis, 1.0f)
            : Vector2.zero;
    }

    private static float ReadTrigger(InputDevice device)
    {
        return device.TryGetFeatureValue(CommonUsages.trigger, out float value)
            ? Mathf.Clamp01(value)
            : 0.0f;
    }

    private static PoseWire ReadPose(InputDevice device)
    {
        if (!device.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 p))
            p = Vector3.zero;
        if (!device.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion q))
            q = Quaternion.identity;

        // Unity LH (right, up, forward) -> ROS REP-103 (forward, left, up).
        return new PoseWire
        {
            p = new[] { p.z, -p.x, p.y },
            q = new[] { -q.z, q.x, -q.y, q.w }
        };
    }
}
