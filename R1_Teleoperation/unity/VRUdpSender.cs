using System;
using System.Collections.Generic;
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using UnityEngine.XR;

// Attach this component to one persistent GameObject in the Unity scene.
// It uses OpenXR/CommonUsages, so no Pico-specific API is required.
public sealed class VRUdpSender : MonoBehaviour
{
    // Robot POV shares this result so video and control reconnect to the same
    // laptop without a separate address setting inside the headset.
    public static event Action<string> EndpointDiscovered;
    public static string LastDiscoveredHost { get; private set; }

    [Header("ROS host")]
    [SerializeField] private string host = "192.168.8.9";
    [SerializeField] private int port = 9090;

    [Header("LAN discovery")]
    // The configured host is retained as a safe fallback for networks that
    // block broadcasts.  In normal operation the bridge endpoint is resolved
    // automatically, so changing the laptop's Wi-Fi address does not require
    // rebuilding the APK.
    [SerializeField] private bool autoDiscoverHost = true;
    [SerializeField] private int discoveryPort = 9091;
    [SerializeField] private float discoveryIntervalSeconds = 1.0f;
    [SerializeField] private float discoveryFallbackDelaySeconds = 5.0f;

    [Header("Streaming")]
    [Range(10.0f, 120.0f)]
    [SerializeField] private float sendRateHz = 72.0f;

    // Hold the left grip to permit motion; index triggers remain free for fingers.
    // Change this mapping if the project uses a dedicated physical deadman button.
    [SerializeField] private bool requireLeftGripAsDeadman = true;

    private UdpClient udp;
    private R1UsbPoseTransport usbTransport;
    private InputDevice leftDevice;
    private InputDevice rightDevice;
    private InputDevice headDevice;
    private bool analogDeadman;
    private bool deadmanReleaseObserved;
    private bool lastReportedLeftX;
    private bool lastReportedRightB;
    private float nextInputDiagnosticTime;
    private float nextSendTime;
    private float nextReconnectTime;
    private uint sequence;
    private UdpClient discoveryUdp;
    private string discoveryNonce;
    private bool discoveryResolved;
    private float nextDiscoveryProbeTime;
    private float discoveryFallbackTime;

    private const float ReconnectDelaySeconds = 1.0f;
    private const string DiscoveryProbePrefix = "R1_TELEOP_DISCOVER v1 ";
    // Pico's legacy XR provider exposes the squeeze axis under this name on
    // firmware versions where CommonUsages.grip is absent.
    private static readonly InputFeatureUsage<float> PicoGrip1DAxisUsage =
        new InputFeatureUsage<float>("Grip1DAxis");
    // Most Pico/OpenXR runtimes expose the stick as Primary2DAxis. Some
    // firmware/plugin combinations publish the same physical control under an
    // alias. Probe those aliases before treating a valid controller as zero.
    private static readonly InputFeatureUsage<Vector2>[] StickAxisUsages =
    {
        CommonUsages.primary2DAxis,
        new InputFeatureUsage<Vector2>("Joystick"),
        new InputFeatureUsage<Vector2>("Thumbstick")
    };

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
    private sealed class TrackingWire
    {
        public bool left;
        public bool right;
        public bool head;
    }

    [Serializable]
    private sealed class ButtonsWire
    {
        public bool left_x;
        public bool right_b;
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
        public TrackingWire tracking;
        public ButtonsWire buttons;
        public bool deadman;
    }

    private void OnEnable()
    {
        ResetDeadmanInterlock();
        AcquireDevices();
        if (R1UsbPoseTransport.Requested())
        {
            usbTransport = new R1UsbPoseTransport();
            NotifyEndpointDiscovered("127.0.0.1");
            Debug.Log("R1 USB mode selected; video and poses use ADB reverse");
            return;
        }
        if (autoDiscoverHost)
            StartDiscovery();
        else
            TryOpenEndpoint();
    }

    private void OnDisable()
    {
        // Send an explicit zero/deadman=false snapshot before closing when possible.
        SendDeadmanFalseBestEffort();
        usbTransport?.Dispose();
        usbTransport = null;
        CloseDiscovery();
        CloseEndpoint();
        ResetDeadmanInterlock();
    }

    private void Update()
    {
        if (usbTransport != null)
        {
            usbTransport.Poll();
            if (usbTransport.Reconnected) ResetDeadmanInterlock();
            if (!usbTransport.HasRequest) return;
        }
        PollDiscovery();

        if (Time.unscaledTime < nextSendTime)
            return;

        nextSendTime = Time.unscaledTime + 1.0f / sendRateHz;
        if (udp == null && usbTransport == null)
        {
            if (autoDiscoverHost && !discoveryResolved)
            {
                if (Time.unscaledTime < discoveryFallbackTime)
                    return;

                // Broadcast may be disabled by a hotspot/AP.  Continue with
                // the build-time host after a bounded wait, while preserving
                // the same deadman interlock and reconnect behavior.
                discoveryResolved = true;
                CloseDiscovery();
                NotifyEndpointDiscovered(host);
                Debug.LogWarning(
                    $"VR UDP discovery timed out; using fallback endpoint {host}:{port}");
            }

            if (Time.unscaledTime >= nextReconnectTime)
                TryOpenEndpoint();
            return;
        }

        if (!leftDevice.isValid || !rightDevice.isValid || !headDevice.isValid)
        {
            // Replacing/reconnecting an XR device is another control-path
            // reconnect and therefore also requires a fresh grip release.
            ResetDeadmanInterlock();
            AcquireDevices();
        }

        bool deadman = !requireLeftGripAsDeadman;
        if (requireLeftGripAsDeadman)
        {
            // Pico firmware/Input System variants do not all expose the
            // binary gripButton feature. Keep the binary mapping when it is
            // available, and fall back to the analog grip value otherwise.
            bool gripButton = false;
            float gripAmount = 0.0f;
            float picoGripAmount = 0.0f;
            bool hasGripButton = leftDevice.TryGetFeatureValue(
                CommonUsages.gripButton, out gripButton);
            bool hasGripAxis = leftDevice.TryGetFeatureValue(
                CommonUsages.grip, out gripAmount);
            bool hasPicoGripAxis = leftDevice.TryGetFeatureValue(
                PicoGrip1DAxisUsage, out picoGripAmount);
            if (hasPicoGripAxis && (!hasGripAxis || picoGripAmount > gripAmount))
            {
                hasGripAxis = true;
                gripAmount = picoGripAmount;
            }
            const float gripOnThreshold = 0.65f;
            const float gripOffThreshold = 0.45f;

            // Use hysteresis for analog-only Pico mappings so a hand held on
            // the grip cannot chatter the safety gate around one threshold.
            if (hasGripAxis)
            {
                if (analogDeadman)
                {
                    if (gripAmount <= gripOffThreshold)
                        analogDeadman = false;
                }
                else if (gripAmount >= gripOnThreshold)
                {
                    analogDeadman = true;
                }
            }
            else
            {
                analogDeadman = false;
            }

            bool deadmanInputAvailable = hasGripButton || hasGripAxis;
            bool requestedDeadman = (hasGripButton && gripButton) || analogDeadman;
            bool deadmanInputReleased = (!hasGripButton || !gripButton)
                && (!hasGripAxis || gripAmount <= gripOffThreshold);

            if (Time.unscaledTime >= nextInputDiagnosticTime)
            {
                nextInputDiagnosticTime = Time.unscaledTime + 1.0f;
                bool hasLeftStick = TryReadAxis(
                    leftDevice, out Vector2 leftStick, out string leftStickSource);
                bool hasRightStick = TryReadAxis(
                    rightDevice, out Vector2 rightStick, out string rightStickSource);
                Debug.Log(
                    $"VR deadman input: left={leftDevice.name} valid={leftDevice.isValid} "
                    + $"gripButton={hasGripButton}:{gripButton} "
                    + $"grip={hasGripAxis}:{gripAmount:F3} "
                    + $"picoGrip={hasPicoGripAxis}:{picoGripAmount:F3} "
                    + $"releaseObserved={deadmanReleaseObserved} "
                    + $"requested={requestedDeadman} "
                    + $"leftStick={hasLeftStick}:{leftStickSource}:"
                    + $"({leftStick.x:F3},{leftStick.y:F3}) "
                    + $"rightStick={hasRightStick}:{rightStickSource}:"
                    + $"({rightStick.x:F3},{rightStick.y:F3})");
            }

            // A grip that was already held when the app/transport started must
            // never arm motion.  Require one observable release before a later
            // press can pass through.  Missing/disconnected XR input is not a
            // release, otherwise a controller reconnect could bypass the gate.
            if (!deadmanReleaseObserved)
            {
                if (leftDevice.isValid
                    && deadmanInputAvailable
                    && deadmanInputReleased)
                    deadmanReleaseObserved = true;

                deadman = false;
            }
            else
            {
                deadman = deadmanInputAvailable && requestedDeadman;
            }
        }

        try
        {
            SendSnapshot(deadman);
        }
        catch (SocketException exception)
        {
            HandleTransportFailure(exception);
        }
        catch (ObjectDisposedException exception)
        {
            HandleTransportFailure(exception);
        }
    }

    /// <summary>
    /// Safely updates the UDP destination.  Editor build scripts may call this
    /// while preparing a scene; no socket is opened outside Play Mode.
    /// At runtime the old endpoint receives deadman=false before it is closed,
    /// and the new endpoint starts behind the release-to-rearm interlock.
    /// </summary>
    public void ConfigureEndpoint(string endpointHost, int endpointPort)
    {
        if (string.IsNullOrWhiteSpace(endpointHost))
            throw new ArgumentException("UDP endpoint host must not be empty", nameof(endpointHost));
        if (endpointPort < 1 || endpointPort > 65535)
            throw new ArgumentOutOfRangeException(
                nameof(endpointPort), endpointPort, "UDP endpoint port must be between 1 and 65535");

        bool runtimeActive = Application.isPlaying && isActiveAndEnabled;
        if (runtimeActive)
        {
            SendDeadmanFalseBestEffort();
            CloseEndpoint();
        }

        host = endpointHost.Trim();
        port = endpointPort;
        discoveryResolved = true;
        NotifyEndpointDiscovered(host);
        CloseDiscovery();
        ResetDeadmanInterlock();

        if (runtimeActive)
            TryOpenEndpoint();
    }

    private void TryOpenEndpoint()
    {
        if (udp != null)
            return;

        UdpClient candidate = null;
        try
        {
            candidate = new UdpClient();
            candidate.Connect(host, port);
            udp = candidate;
            candidate = null;
            nextReconnectTime = 0.0f;
            ResetDeadmanInterlock();

            // Establish an explicitly safe state at the new destination before
            // the regular pose stream is allowed to evaluate the grip input.
            SendSnapshot(false);
        }
        catch (Exception exception) when (
            exception is SocketException
            || exception is ObjectDisposedException
            || exception is ArgumentException)
        {
            candidate?.Close();
            CloseEndpoint();
            nextReconnectTime = Time.unscaledTime + ReconnectDelaySeconds;
            Debug.LogWarning(
                $"VR UDP endpoint {host}:{port} is unavailable; retrying: {exception.Message}");
        }
    }

    private void HandleTransportFailure(Exception exception)
    {
        Debug.LogWarning(
            $"VR UDP transport to {host}:{port} failed; deadman was disarmed: {exception.Message}");
        CloseEndpoint();
        ResetDeadmanInterlock();
        nextReconnectTime = Time.unscaledTime + ReconnectDelaySeconds;
        if (autoDiscoverHost)
            StartDiscovery();
    }

    private void StartDiscovery()
    {
        CloseDiscovery();
        discoveryResolved = false;
        discoveryNonce = Guid.NewGuid().ToString("N");
        nextDiscoveryProbeTime = 0.0f;
        discoveryFallbackTime = Time.unscaledTime
            + Mathf.Max(1.0f, discoveryFallbackDelaySeconds);

        try
        {
            discoveryUdp = new UdpClient(0);
            discoveryUdp.EnableBroadcast = true;
            discoveryUdp.Client.Blocking = false;
        }
        catch (Exception exception) when (
            exception is SocketException || exception is ObjectDisposedException)
        {
            CloseDiscovery();
            discoveryResolved = true;
            NotifyEndpointDiscovered(host);
            Debug.LogWarning(
                $"VR UDP discovery is unavailable; using fallback endpoint {host}:{port}: "
                + exception.Message);
        }
    }

    private void PollDiscovery()
    {
        if (!autoDiscoverHost || discoveryResolved || discoveryUdp == null)
            return;

        if (Time.unscaledTime >= nextDiscoveryProbeTime)
        {
            SendDiscoveryProbe();
            nextDiscoveryProbeTime = Time.unscaledTime
                + Mathf.Max(0.25f, discoveryIntervalSeconds);
        }

        for (int attempt = 0; attempt < 16; ++attempt)
        {
            try
            {
                if (discoveryUdp.Available <= 0)
                    return;

                IPEndPoint source = new IPEndPoint(IPAddress.Any, 0);
                byte[] bytes = discoveryUdp.Receive(ref source);
                if (TryAcceptDiscoveryResponse(bytes, source))
                    return;
            }
            catch (SocketException exception)
            {
                // A non-blocking socket can report EWOULDBLOCK between the
                // Available check and Receive call.
                if (exception.SocketErrorCode == SocketError.WouldBlock
                    || exception.SocketErrorCode == SocketError.IOPending)
                    return;
                Debug.LogWarning(
                    $"VR UDP discovery receive failed: {exception.Message}");
                return;
            }
            catch (ObjectDisposedException)
            {
                return;
            }
        }
    }

    private void SendDiscoveryProbe()
    {
        if (discoveryUdp == null || string.IsNullOrEmpty(discoveryNonce))
            return;

        byte[] bytes = Encoding.ASCII.GetBytes(
            DiscoveryProbePrefix + discoveryNonce);
        try
        {
            foreach (IPAddress broadcast in GetDiscoveryBroadcastAddresses())
            {
                discoveryUdp.Send(
                    bytes,
                    bytes.Length,
                    new IPEndPoint(broadcast, discoveryPort));
            }
        }
        catch (Exception exception) when (
            exception is SocketException || exception is ObjectDisposedException)
        {
            Debug.LogWarning(
                $"VR UDP discovery probe failed: {exception.Message}");
        }
    }

    private static List<IPAddress> GetDiscoveryBroadcastAddresses()
    {
        var addresses = new List<IPAddress> { IPAddress.Broadcast };
        try
        {
            foreach (NetworkInterface networkInterface
                in NetworkInterface.GetAllNetworkInterfaces())
            {
                if (networkInterface.OperationalStatus
                    != OperationalStatus.Up)
                    continue;

                IPInterfaceProperties properties =
                    networkInterface.GetIPProperties();
                foreach (UnicastIPAddressInformation unicast
                    in properties.UnicastAddresses)
                {
                    IPAddress address = unicast.Address;
                    IPAddress mask = unicast.IPv4Mask;
                    if (address.AddressFamily != AddressFamily.InterNetwork
                        || mask == null
                        || IPAddress.IsLoopback(address))
                        continue;

                    byte[] addressBytes = address.GetAddressBytes();
                    byte[] maskBytes = mask.GetAddressBytes();
                    byte[] broadcastBytes = new byte[4];
                    for (int index = 0; index < 4; ++index)
                    {
                        broadcastBytes[index] = (byte)(
                            addressBytes[index]
                            | (byte)~maskBytes[index]);
                    }

                    var broadcast = new IPAddress(broadcastBytes);
                    if (!addresses.Contains(broadcast))
                        addresses.Add(broadcast);
                }
            }
        }
        catch (Exception exception) when (
            exception is NetworkInformationException
            || exception is SocketException
            || exception is PlatformNotSupportedException)
        {
            // The global broadcast remains a valid fallback on platforms that
            // do not expose interface metadata to managed code.
        }

        return addresses;
    }

    private bool TryAcceptDiscoveryResponse(
        byte[] bytes,
        IPEndPoint source)
    {
        if (source == null || !IsPrivateLanAddress(source.Address))
            return false;

        string response = Encoding.ASCII.GetString(bytes).Trim();
        string[] parts = response.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
        if (parts.Length != 4
            || parts[0] != "R1_TELEOP_ENDPOINT"
            || parts[1] != "v1"
            || parts[3] != discoveryNonce
            || !int.TryParse(parts[2], out int discoveredPort)
            || discoveredPort < 1
            || discoveredPort > 65535)
            return false;

        string discoveredHost = source.Address.ToString();
        discoveryResolved = true;
        CloseDiscovery();
        Debug.Log(
            $"VR UDP endpoint discovered at {discoveredHost}:{discoveredPort}");
        ConfigureEndpoint(discoveredHost, discoveredPort);
        return true;
    }

    private static bool IsPrivateLanAddress(IPAddress address)
    {
        if (address == null || address.AddressFamily != AddressFamily.InterNetwork)
            return false;

        byte[] octets = address.GetAddressBytes();
        return octets[0] == 10
            || (octets[0] == 172 && octets[1] >= 16 && octets[1] <= 31)
            || (octets[0] == 192 && octets[1] == 168);
    }

    private static void NotifyEndpointDiscovered(string endpointHost)
    {
        if (string.IsNullOrWhiteSpace(endpointHost))
            return;
        LastDiscoveredHost = endpointHost.Trim();
        EndpointDiscovered?.Invoke(LastDiscoveredHost);
    }

    private void CloseDiscovery()
    {
        UdpClient endpoint = discoveryUdp;
        discoveryUdp = null;
        endpoint?.Close();
    }

    private void SendDeadmanFalseBestEffort()
    {
        if (udp == null)
            return;

        try
        {
            SendSnapshot(false);
        }
        catch (Exception exception) when (
            exception is SocketException || exception is ObjectDisposedException)
        {
            // The receiver watchdog remains the fallback when the transport is gone.
        }
    }

    private void CloseEndpoint()
    {
        UdpClient endpoint = udp;
        udp = null;
        endpoint?.Close();
    }

    private void ResetDeadmanInterlock()
    {
        analogDeadman = false;
        deadmanReleaseObserved = false;
    }

    private void AcquireDevices()
    {
        leftDevice = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
        rightDevice = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
        headDevice = InputDevices.GetDeviceAtXRNode(XRNode.Head);
    }

    private void SendSnapshot(bool deadman)
    {
        if (udp == null && usbTransport == null)
            return;

        Vector2 leftStick = ReadAxis(leftDevice);
        Vector2 rightStick = ReadAxis(rightDevice);
        float leftTrigger = ReadTrigger(leftDevice);
        float rightTrigger = ReadTrigger(rightDevice);
        bool leftTracked = IsPoseTracked(leftDevice);
        bool rightTracked = IsPoseTracked(rightDevice);
        bool headTracked = IsPoseTracked(headDevice);
        // OpenXR/CommonUsages maps the left primary face button to X and the
        // right secondary face button to B on Pico/Meta-style controllers.
        bool leftX = ReadButton(leftDevice, CommonUsages.primaryButton);
        bool rightB = ReadButton(rightDevice, CommonUsages.secondaryButton);
        if (leftX != lastReportedLeftX || rightB != lastReportedRightB)
        {
            lastReportedLeftX = leftX;
            lastReportedRightB = rightB;
            Debug.Log(
                $"VR action buttons: left_x={leftX} right_b={rightB}");
        }

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
            tracking = new TrackingWire
            {
                left = leftTracked,
                right = rightTracked,
                head = headTracked
            },
            buttons = new ButtonsWire
            {
                left_x = leftX,
                right_b = rightB
            },
            deadman = deadman && leftTracked && rightTracked && headTracked
        };

        // JsonUtility emits JSON numbers with invariant decimal separators.
        byte[] bytes = Encoding.UTF8.GetBytes(JsonUtility.ToJson(packet));
        if (bytes.Length > 1200)
        {
            Debug.LogError($"VR UDP packet unexpectedly large: {bytes.Length} bytes");
            return;
        }
        if (usbTransport != null)
            usbTransport.Send(bytes);
        else
            udp.Send(bytes, bytes.Length);
    }

    private static Vector2 ReadAxis(InputDevice device)
    {
        return TryReadAxis(device, out Vector2 axis, out string ignoredSource)
            ? axis : Vector2.zero;
    }

    private static bool ReadButton(
        InputDevice device,
        InputFeatureUsage<bool> usage)
    {
        return device.isValid
            && device.TryGetFeatureValue(usage, out bool pressed)
            && pressed;
    }

    private static bool TryReadAxis(
        InputDevice device,
        out Vector2 axis,
        out string source)
    {
        axis = Vector2.zero;
        source = "missing";
        if (!device.isValid)
            return false;

        bool found = false;
        foreach (InputFeatureUsage<Vector2> usage in StickAxisUsages)
        {
            if (!device.TryGetFeatureValue(usage, out Vector2 candidate))
                continue;
            if (float.IsNaN(candidate.x) || float.IsNaN(candidate.y)
                || float.IsInfinity(candidate.x) || float.IsInfinity(candidate.y))
                continue;
            candidate = Vector2.ClampMagnitude(candidate, 1.0f);
            if (!found || candidate.sqrMagnitude > axis.sqrMagnitude)
            {
                axis = candidate;
                source = usage.name;
                found = true;
            }
        }

#if ENABLE_INPUT_SYSTEM
        if (TryReadInputSystemAxis(
                device.characteristics,
                out Vector2 inputSystemAxis,
                out string inputSystemSource)
            && (!found || inputSystemAxis.sqrMagnitude > axis.sqrMagnitude))
        {
            axis = inputSystemAxis;
            source = inputSystemSource;
            found = true;
        }
#endif
        return found;
    }

#if ENABLE_INPUT_SYSTEM
    private static bool TryReadInputSystemAxis(
        InputDeviceCharacteristics characteristics,
        out Vector2 axis,
        out string source)
    {
        axis = Vector2.zero;
        source = "InputSystem:missing";
        UnityEngine.InputSystem.Utilities.InternedString expectedHand;
        if ((characteristics & InputDeviceCharacteristics.Left) != 0)
            expectedHand = UnityEngine.InputSystem.CommonUsages.LeftHand;
        else if ((characteristics & InputDeviceCharacteristics.Right) != 0)
            expectedHand = UnityEngine.InputSystem.CommonUsages.RightHand;
        else
            return false;

        string[] controlPaths =
        {
            "thumbstick", "{Primary2DAxis}", "primary2DAxis", "joystick"
        };
        bool found = false;
        foreach (UnityEngine.InputSystem.InputDevice inputDevice in
                 UnityEngine.InputSystem.InputSystem.devices)
        {
            bool handMatches = false;
            foreach (UnityEngine.InputSystem.Utilities.InternedString usage in
                     inputDevice.usages)
            {
                if (usage == expectedHand)
                {
                    handMatches = true;
                    break;
                }
            }
            if (!handMatches)
                continue;

            foreach (string controlPath in controlPaths)
            {
                UnityEngine.InputSystem.Controls.Vector2Control control =
                    inputDevice.TryGetChildControl<
                        UnityEngine.InputSystem.Controls.Vector2Control>(
                            controlPath);
                if (control == null)
                    continue;
                Vector2 candidate = control.ReadValue();
                if (float.IsNaN(candidate.x) || float.IsNaN(candidate.y)
                    || float.IsInfinity(candidate.x)
                    || float.IsInfinity(candidate.y))
                    continue;
                candidate = Vector2.ClampMagnitude(candidate, 1.0f);
                if (!found || candidate.sqrMagnitude > axis.sqrMagnitude)
                {
                    axis = candidate;
                    source = "InputSystem:" + inputDevice.displayName
                        + "/" + control.name;
                    found = true;
                }
            }
        }
        return found;
    }
#endif

    private static float ReadTrigger(InputDevice device)
    {
        return device.TryGetFeatureValue(CommonUsages.trigger, out float value)
            ? Mathf.Clamp01(value)
            : 0.0f;
    }

    private static bool IsPoseTracked(InputDevice device)
    {
        if (!device.isValid)
            return false;

        bool isTracked;
        bool reportsTracking = device.TryGetFeatureValue(
            CommonUsages.isTracked, out isTracked);
        bool reportsPoseFlags = device.TryGetFeatureValue(
            CommonUsages.trackingState, out InputTrackingState poseFlags);
        bool hasPosition = device.TryGetFeatureValue(
            CommonUsages.devicePosition, out Vector3 ignoredPosition);
        bool hasRotation = device.TryGetFeatureValue(
            CommonUsages.deviceRotation, out Quaternion ignoredRotation);
        InputTrackingState required =
            InputTrackingState.Position | InputTrackingState.Rotation;
        bool completePose = !reportsPoseFlags || (poseFlags & required) == required;
        // OpenXR runtimes can briefly lower isTracked while they continue to
        // provide a valid predicted pose from controller IMU + camera history.
        // Games consume the per-component trackingState flags in that window;
        // do the same here. The bridge still applies its bounded hold/zero
        // policy when either position or rotation really becomes unavailable.
        bool runtimeTracked = reportsPoseFlags ? completePose :
            (!reportsTracking || isTracked);
        return runtimeTracked && hasPosition && hasRotation;
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
