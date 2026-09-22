using System;
using System.Collections;
using System.IO;
using System.Net;
using System.Text;
using System.Threading;
using Unity.WebRTC;
using UnityEngine;
using UnityEngine.Networking;

namespace RobotPov
{
    public enum RobotPovConnectionState
    {
        Idle,
        Connecting,
        Connected,
        NoSignal,
        Reconnecting,
        Stopped,
        Error
    }

    /// <summary>
    /// LAN-only, receive-only WebRTC client for the Robot POV server.
    /// This component owns video signaling only and has no robot-control path.
    /// </summary>
    [DisallowMultipleComponent]
    [AddComponentMenu("Robot POV/WebRTC Video Receiver")]
    public sealed class RobotPovWebRtcReceiver : MonoBehaviour
    {
        [Serializable]
        private sealed class OfferPayload
        {
            public string sdp;
            public string type;
            public string profile;
            public string preferredCodec;
            public string clientTag;
            public string layout;
        }

        [Serializable]
        private sealed class OfferResponse
        {
            public string sdp;
            public string type;
            public string profile;
        }

        [Serializable]
        private sealed class SourceStatusPayload
        {
            public string source;
            public bool connected;
            public bool stale;
            public bool has_frame;
            public float received_age_s;
            public string error;
        }

        [Serializable]
        private sealed class ServerStatusPayload
        {
            public bool ok;
            public string profile;
            public SourceStatusPayload source;
        }

        [Serializable]
        private sealed class ClientMetricsPayload
        {
            public string clientId;
            public string transport;
            public string profile;
            public float renderedFps;
            public float latencyMs;
            public float rttMs;
            public float jitterMs;
            public long packetsLost;
            public long framesDropped;
            public float frameAgeMs;
            public int reconnects;
            public bool noSignal;
        }

        /// <summary>
        /// All mutable MJPEG reader state is scoped to one connection attempt.
        /// A reader that is slow to leave a blocking Stream.Read can therefore
        /// never publish frames or failures into a replacement attempt.
        /// </summary>
        private sealed class MjpegReaderSession
        {
            public MjpegReaderSession(
                int generation,
                int readerId,
                string url,
                int requestTimeoutMilliseconds)
            {
                Generation = generation;
                ReaderId = readerId;
                Url = url;
                RequestTimeoutMilliseconds = requestTimeoutMilliseconds;
            }

            public readonly int Generation;
            public readonly int ReaderId;
            public readonly string Url;
            public readonly int RequestTimeoutMilliseconds;
            public readonly CancellationTokenSource Cancellation =
                new CancellationTokenSource();

            public volatile string FailureReason = string.Empty;
            public volatile HttpWebRequest Request;
            public Thread Thread;

            // Interlocked.Exchange turns this into a one-frame latest-value
            // mailbox. The decoder can never accumulate a main-thread backlog.
            public byte[] PendingFrame;
        }

        [SerializeField]
        private RobotPovRuntimeConfig config = new RobotPovRuntimeConfig();

        private RTCPeerConnection peer;
        private VideoStreamTrack receivedVideoTrack;
        private OnVideoReceived videoReceivedHandler;
        private Coroutine connectionCoroutine;
        private Coroutine webRtcUpdateCoroutine;
        private Coroutine statusCoroutine;
        private Coroutine metricsCoroutine;

        // Pico 4 Ultra currently aborts inside the bundled libwebrtc when a
        // transceiver is created.  Keep a managed HTTP/MJPEG path available so
        // the video-only viewer remains usable on Android without touching the
        // robot-control transport.
        private volatile MjpegReaderSession mjpegReader;
        private int nextMjpegReaderId;
        private bool useMjpegFallback;
        private Texture2D mjpegTexture;

        private int generation;
        private int reconnectCount;
        private bool peerFailed;
        private string peerFailureReason = string.Empty;
        private RTCPeerConnectionState peerConnectionState = RTCPeerConnectionState.New;
        private RTCIceConnectionState iceConnectionState = RTCIceConnectionState.New;

        private Texture currentTexture;
        private float firstTextureRealtime = -1f;
        private float lastDecodedFrameRealtime = -1f;
        private float lastStatsSampleRealtime = -1f;
        private uint lastFramesDecoded;
        private bool hasDecodedFrameStats;
        private float renderedFps;
        private float rttMs;
        private float jitterMs;
        private long packetsLost;
        private long framesDropped;

        private bool statusEverReceived;
        private bool serverReachable;
        private float lastStatusSuccessRealtime = -1f;
        private bool sourceConnected;
        private bool sourceStale = true;
        private bool sourceHasFrame;
        private float sourceReceivedAgeSeconds = -1f;
        private string sourceName = "unknown";
        private string sourceError = string.Empty;

        private RobotPovConnectionState state = RobotPovConnectionState.Idle;
        private string stateDetail = "Video receiver is idle";

        public event Action<Texture> TextureChanged;
        public event Action<RobotPovConnectionState, string> StateChanged;

        public Texture CurrentTexture => currentTexture;
        public RobotPovConnectionState State => state;
        public string StateDetail => stateDetail;
        public int ReconnectCount => reconnectCount;
        public string SourceName => sourceName;
        public float RenderedFps => renderedFps;
        public float RoundTripTimeMs => rttMs;
        public float JitterMs => jitterMs;
        public long PacketsLost => packetsLost;
        public long FramesDropped => framesDropped;
        public float FrameAgeMs => ComputeFrameAgeMs();
        public bool SignalAvailable => ComputeSignalAvailable();
        public string NoSignalReason => ComputeNoSignalReason();

        public void Configure(RobotPovRuntimeConfig runtimeConfig)
        {
            if (runtimeConfig == null)
            {
                throw new ArgumentNullException(nameof(runtimeConfig));
            }

            config = runtimeConfig;
            config.ValidateAndClamp();

            if (isActiveAndEnabled)
            {
                RestartNow();
            }
        }

        public void RestartNow()
        {
            if (!isActiveAndEnabled)
            {
                return;
            }

            StopRuntime("Restart requested");
            StartRuntime();
        }

        private void OnEnable()
        {
            StartRuntime();
        }

        private void OnDisable()
        {
            StopRuntime("Video receiver stopped");
        }

        private void OnValidate()
        {
            if (config == null)
            {
                config = new RobotPovRuntimeConfig();
            }

            config.ValidateAndClamp();
        }

        private void StartRuntime()
        {
            if (connectionCoroutine != null || webRtcUpdateCoroutine != null)
            {
                return;
            }

            if (config == null)
            {
                config = new RobotPovRuntimeConfig();
            }

            config.ValidateAndClamp();
            generation++;
            int activeGeneration = generation;
            useMjpegFallback = Application.platform == RuntimePlatform.Android;

            ResetMediaStatistics();
            ClearTexture();

            if (!config.ConnectOnEnable)
            {
                SetState(RobotPovConnectionState.Idle, "Automatic video connection is disabled");
                return;
            }

            statusCoroutine = StartCoroutine(StatusPollingLoop(activeGeneration));
            metricsCoroutine = StartCoroutine(MetricsLoop(activeGeneration));
            if (useMjpegFallback)
            {
                SetState(
                    RobotPovConnectionState.Connecting,
                    "Connecting to Robot POV over LAN MJPEG (Pico-safe fallback)");
                connectionCoroutine = StartCoroutine(MjpegConnectionLoop(activeGeneration));
            }
            else
            {
                // com.unity.webrtc 3.0.0-pre.8 initializes its native Context itself.
                // The public per-frame update coroutine is still required for decoded textures.
                webRtcUpdateCoroutine = StartCoroutine(WebRTC.Update());
                connectionCoroutine = StartCoroutine(ConnectionLoop(activeGeneration));
            }
        }

        private void StopRuntime(string reason)
        {
            generation++;
            StopAllCoroutines();
            connectionCoroutine = null;
            webRtcUpdateCoroutine = null;
            statusCoroutine = null;
            metricsCoroutine = null;
            StopMjpegReader();
            DisposePeer();
            ClearTexture();
            SetState(RobotPovConnectionState.Stopped, reason);
        }

        private void Update()
        {
            if (useMjpegFallback)
            {
                PumpMjpegFrames();
            }
        }

        private IEnumerator MjpegConnectionLoop(int activeGeneration)
        {
            int failureStreak = 0;
            bool firstAttempt = true;

            while (IsCurrent(activeGeneration))
            {
                SetState(
                    firstAttempt ? RobotPovConnectionState.Connecting : RobotPovConnectionState.Reconnecting,
                    firstAttempt
                        ? "Connecting to Robot POV over LAN MJPEG"
                        : "Reconnecting to Robot POV over LAN MJPEG");

                MjpegReaderSession reader = StartMjpegReader(activeGeneration);

                float frameDeadline = Time.realtimeSinceStartup + config.FirstFrameTimeoutSeconds;
                while (IsCurrent(activeGeneration)
                       && IsMjpegReaderCurrent(reader)
                       && currentTexture == null
                       && string.IsNullOrEmpty(reader.FailureReason)
                       && Time.realtimeSinceStartup < frameDeadline)
                {
                    PumpMjpegFrames(reader);
                    yield return null;
                }

                if (!IsCurrent(activeGeneration))
                {
                    break;
                }

                if (currentTexture != null
                    && IsMjpegReaderCurrent(reader)
                    && string.IsNullOrEmpty(reader.FailureReason))
                {
                    failureStreak = 0;
                    RefreshPublishedState();

                    while (IsCurrent(activeGeneration)
                           && IsMjpegReaderCurrent(reader)
                           && string.IsNullOrEmpty(reader.FailureReason))
                    {
                        PumpMjpegFrames(reader);
                        if (lastDecodedFrameRealtime > 0f
                            && Time.realtimeSinceStartup - lastDecodedFrameRealtime
                            > config.MediaStaleTimeoutSeconds)
                        {
                            reader.FailureReason = "MJPEG video stream stalled";
                            break;
                        }

                        RefreshPublishedState();
                        yield return null;
                    }
                }

                string failure = string.IsNullOrEmpty(reader.FailureReason)
                    ? "MJPEG connection closed"
                    : reader.FailureReason;
                StopMjpegReader(reader);
                ClearTexture();

                if (!IsCurrent(activeGeneration))
                {
                    break;
                }

                failureStreak++;
                reconnectCount++;
                firstAttempt = false;
                float delay = Mathf.Min(
                    config.ReconnectMaximumDelaySeconds,
                    config.ReconnectInitialDelaySeconds * Mathf.Pow(2f, failureStreak - 1));
                SetState(
                    RobotPovConnectionState.Reconnecting,
                    string.Format("{0}; retry in {1:0.0}s", failure, delay));

                float deadline = Time.realtimeSinceStartup + delay;
                while (IsCurrent(activeGeneration) && Time.realtimeSinceStartup < deadline)
                {
                    yield return null;
                }
            }

            connectionCoroutine = null;
        }

        private MjpegReaderSession StartMjpegReader(int activeGeneration)
        {
            StopMjpegReader();
            string layout = RequestedLayout();
            string url = config.BaseUrl
                + "/stream.mjpg?profile="
                + Uri.EscapeDataString(config.Profile)
                + "&layout="
                + Uri.EscapeDataString(layout);
            int timeoutMilliseconds = Mathf.CeilToInt(config.RequestTimeoutSeconds * 1000f);
            MjpegReaderSession reader = new MjpegReaderSession(
                activeGeneration,
                ++nextMjpegReaderId,
                url,
                timeoutMilliseconds);
            mjpegReader = reader;
            reader.Thread = new Thread(() => MjpegReaderMain(reader))
            {
                IsBackground = true,
                Name = "RobotPov-MJPEG-" + reader.ReaderId
            };
            reader.Thread.Start();
            return reader;
        }

        private string RequestedLayout()
        {
            switch (config.VideoLayout)
            {
                case RobotPovVideoLayout.Mono:
                    return "mono";
                case RobotPovVideoLayout.StereoTopBottom:
                    return "top-bottom";
                default:
                    return "stereo";
            }
        }

        private void StopMjpegReader()
        {
            StopMjpegReader(mjpegReader);
        }

        private void StopMjpegReader(MjpegReaderSession reader)
        {
            if (reader == null)
            {
                return;
            }

            reader.Cancellation.Cancel();
            if (ReferenceEquals(mjpegReader, reader))
            {
                mjpegReader = null;
            }

            // Abort is required in addition to the token because the legacy
            // HttpWebRequest stream can otherwise remain blocked in Read.
            HttpWebRequest request = reader.Request;
            if (request != null)
            {
                try
                {
                    request.Abort();
                }
                catch (Exception)
                {
                    // Cancellation is best-effort; generation checks below
                    // still isolate a reader that exits after the join timeout.
                }
            }

            Thread thread = reader.Thread;
            if (thread != null && thread.IsAlive && !ReferenceEquals(thread, Thread.CurrentThread))
            {
                thread.Join(500);
            }

            Interlocked.Exchange(ref reader.PendingFrame, null);
        }

        private void MjpegReaderMain(MjpegReaderSession reader)
        {
            try
            {
                if (reader.Cancellation.IsCancellationRequested)
                {
                    return;
                }

                HttpWebRequest request = WebRequest.CreateHttp(reader.Url);
                request.Timeout = reader.RequestTimeoutMilliseconds;
                request.ReadWriteTimeout = request.Timeout;
                request.KeepAlive = false;
                reader.Request = request;
                if (reader.Cancellation.IsCancellationRequested)
                {
                    request.Abort();
                    return;
                }

                using (HttpWebResponse response = (HttpWebResponse)request.GetResponse())
                using (Stream stream = response.GetResponseStream())
                {
                    if (response.StatusCode != HttpStatusCode.OK || stream == null)
                    {
                        SetMjpegFailure(
                            reader,
                            "MJPEG request returned " + response.StatusCode);
                        return;
                    }

                    byte[] buffer = new byte[32 * 1024];
                    using (MemoryStream frame = new MemoryStream(96 * 1024))
                    {
                        bool inJpeg = false;
                        int previous = -1;
                        while (!reader.Cancellation.IsCancellationRequested)
                        {
                            int count = stream.Read(buffer, 0, buffer.Length);
                            if (count <= 0)
                            {
                                break;
                            }

                            for (int index = 0; index < count; index++)
                            {
                                int value = buffer[index];
                                if (!inJpeg)
                                {
                                    if (previous == 0xFF && value == 0xD8)
                                    {
                                        frame.SetLength(0);
                                        frame.WriteByte(0xFF);
                                        frame.WriteByte(0xD8);
                                        inJpeg = true;
                                    }
                                }
                                else
                                {
                                    frame.WriteByte((byte)value);
                                    if (previous == 0xFF && value == 0xD9)
                                    {
                                        EnqueueMjpegFrame(reader, frame.ToArray());
                                        frame.SetLength(0);
                                        inJpeg = false;
                                    }
                                }

                                previous = value;
                            }
                        }
                    }
                }
            }
            catch (Exception exception)
            {
                if (!reader.Cancellation.IsCancellationRequested)
                {
                    SetMjpegFailure(reader, "MJPEG request failed: " + exception.Message);
                }
            }
            finally
            {
                reader.Request = null;
            }
        }

        private void SetMjpegFailure(MjpegReaderSession reader, string reason)
        {
            if (IsMjpegReaderCurrent(reader) && string.IsNullOrEmpty(reader.FailureReason))
            {
                reader.FailureReason = reason;
            }
        }

        private bool IsMjpegReaderCurrent(MjpegReaderSession reader)
        {
            return reader != null
                && !reader.Cancellation.IsCancellationRequested
                && reader.Generation == Volatile.Read(ref generation)
                && ReferenceEquals(reader, mjpegReader);
        }

        private void EnqueueMjpegFrame(MjpegReaderSession reader, byte[] frame)
        {
            if (frame == null || frame.Length < 4 || !IsMjpegReaderCurrent(reader))
            {
                return;
            }

            // A reader owns its mailbox, so even a blocked reader that exits
            // late cannot overwrite the current reader's newest frame.
            Interlocked.Exchange(ref reader.PendingFrame, frame);
        }

        private void PumpMjpegFrames()
        {
            PumpMjpegFrames(mjpegReader);
        }

        private void PumpMjpegFrames(MjpegReaderSession reader)
        {
            if (!IsMjpegReaderCurrent(reader))
            {
                return;
            }

            byte[] latest = Interlocked.Exchange(ref reader.PendingFrame, null);
            if (latest == null || !IsMjpegReaderCurrent(reader))
            {
                return;
            }

            if (mjpegTexture == null)
            {
                mjpegTexture = new Texture2D(2, 2, TextureFormat.RGBA32, false);
                mjpegTexture.name = "Robot POV MJPEG Texture";
            }

            if (!mjpegTexture.LoadImage(latest, false))
            {
                return;
            }

            currentTexture = mjpegTexture;
            float now = Time.realtimeSinceStartup;
            if (firstTextureRealtime < 0f)
            {
                firstTextureRealtime = now;
            }

            if (lastDecodedFrameRealtime > 0f)
            {
                float elapsed = Mathf.Max(0.001f, now - lastDecodedFrameRealtime);
                renderedFps = Mathf.Lerp(renderedFps, 1f / elapsed, 0.25f);
            }

            lastDecodedFrameRealtime = now;
            TextureChanged?.Invoke(currentTexture);
        }

        private IEnumerator ConnectionLoop(int activeGeneration)
        {
            int failureStreak = 0;
            bool firstAttempt = true;

            while (IsCurrent(activeGeneration))
            {
                SetState(
                    firstAttempt ? RobotPovConnectionState.Connecting : RobotPovConnectionState.Reconnecting,
                    firstAttempt ? "Connecting to Robot POV" : "Reconnecting to Robot POV");

                string failure = null;
                yield return ConnectOnce(activeGeneration, value => failure = value);

                if (!IsCurrent(activeGeneration))
                {
                    break;
                }

                if (string.IsNullOrEmpty(failure))
                {
                    failureStreak = 0;
                    RefreshPublishedState();
                    yield return MonitorConnectedPeer(activeGeneration, value => failure = value);
                }

                DisposePeer();
                ClearTexture();

                if (!IsCurrent(activeGeneration))
                {
                    break;
                }

                failureStreak++;
                reconnectCount++;
                firstAttempt = false;
                float delay = Mathf.Min(
                    config.ReconnectMaximumDelaySeconds,
                    config.ReconnectInitialDelaySeconds * Mathf.Pow(2f, failureStreak - 1));

                SetState(
                    RobotPovConnectionState.Reconnecting,
                    string.Format("{0}; retry in {1:0.0}s", SafeFailure(failure), delay));

                float deadline = Time.realtimeSinceStartup + delay;
                while (IsCurrent(activeGeneration) && Time.realtimeSinceStartup < deadline)
                {
                    yield return null;
                }
            }

            connectionCoroutine = null;
        }

        private IEnumerator ConnectOnce(int activeGeneration, Action<string> completed)
        {
            RTCPeerConnection attemptPeer = null;
            string setupError = null;

            try
            {
                RTCConfiguration rtcConfiguration = new RTCConfiguration
                {
                    // Offline-first LAN mode: deliberately no STUN or TURN server.
                    iceServers = Array.Empty<RTCIceServer>()
                };

                attemptPeer = new RTCPeerConnection(ref rtcConfiguration);
                peer = attemptPeer;
                peerFailed = false;
                peerFailureReason = string.Empty;
                peerConnectionState = RTCPeerConnectionState.New;
                iceConnectionState = RTCIceConnectionState.New;

                attemptPeer.OnConnectionStateChange = value =>
                    HandlePeerConnectionState(attemptPeer, activeGeneration, value);
                attemptPeer.OnIceConnectionChange = value =>
                    HandleIceConnectionState(attemptPeer, activeGeneration, value);
                attemptPeer.OnTrack = trackEvent =>
                    HandleRemoteTrack(attemptPeer, activeGeneration, trackEvent);

                // Keep the documented no-init form for desktop WebRTC.  Pico
                // uses the managed MJPEG path above because this bundled
                // libwebrtc build aborts for either AddTransceiver overload.
                attemptPeer.AddTransceiver(TrackKind.Video);
            }
            catch (Exception exception)
            {
                setupError = "WebRTC setup failed: " + exception.Message;
            }

            if (!string.IsNullOrEmpty(setupError))
            {
                completed(setupError);
                yield break;
            }

            RTCSessionDescriptionAsyncOperation offerOperation = null;
            string operationError = null;
            try
            {
                offerOperation = attemptPeer.CreateOffer();
            }
            catch (Exception exception)
            {
                operationError = "Could not create SDP offer: " + exception.Message;
            }

            if (offerOperation == null)
            {
                completed(operationError);
                yield break;
            }

            yield return offerOperation;

            if (!IsPeerCurrent(attemptPeer, activeGeneration))
            {
                completed("Connection attempt cancelled");
                yield break;
            }

            if (offerOperation.IsError)
            {
                completed("SDP offer failed: " + offerOperation.Error.message);
                yield break;
            }

            RTCSessionDescription localOffer = offerOperation.Desc;
            RTCSetSessionDescriptionAsyncOperation localOperation = null;
            try
            {
                localOperation = attemptPeer.SetLocalDescription(ref localOffer);
            }
            catch (Exception exception)
            {
                operationError = "Could not set local SDP: " + exception.Message;
            }

            if (localOperation == null)
            {
                completed(operationError);
                yield break;
            }

            yield return localOperation;

            if (!IsPeerCurrent(attemptPeer, activeGeneration))
            {
                completed("Connection attempt cancelled");
                yield break;
            }

            if (localOperation.IsError)
            {
                completed("Setting local SDP failed: " + localOperation.Error.message);
                yield break;
            }

            // Non-trickle signaling: never POST a partial SDP. A timeout aborts
            // this attempt and lets the reconnect loop create a fresh peer.
            float gatheringDeadline = Time.realtimeSinceStartup + config.IceGatheringTimeoutSeconds;
            RTCIceGatheringState gatheringState = RTCIceGatheringState.New;
            while (IsPeerCurrent(attemptPeer, activeGeneration)
                   && Time.realtimeSinceStartup < gatheringDeadline)
            {
                if (!TryReadGatheringState(attemptPeer, out gatheringState))
                {
                    completed("ICE gathering state became unavailable");
                    yield break;
                }

                if (gatheringState == RTCIceGatheringState.Complete)
                {
                    break;
                }

                yield return null;
            }

            if (!IsPeerCurrent(attemptPeer, activeGeneration))
            {
                completed("Connection attempt cancelled");
                yield break;
            }

            if (gatheringState != RTCIceGatheringState.Complete)
            {
                completed("ICE gathering did not complete before timeout");
                yield break;
            }

            RTCSessionDescription finalLocalDescription = default(RTCSessionDescription);
            operationError = null;
            try
            {
                finalLocalDescription = attemptPeer.LocalDescription;
            }
            catch (Exception exception)
            {
                operationError = "Final local SDP is unavailable: " + exception.Message;
            }

            if (!string.IsNullOrEmpty(operationError))
            {
                completed(operationError);
                yield break;
            }

            OfferPayload offerPayload = new OfferPayload
            {
                sdp = finalLocalDescription.sdp,
                type = finalLocalDescription.type.ToString().ToLowerInvariant(),
                profile = config.Profile,
                preferredCodec = config.PreferredCodec,
                clientTag = config.ClientId,
                layout = RequestedLayout()
            };

            string answerJson = null;
            string requestError = null;
            using (UnityWebRequest request = CreateJsonPost(config.OfferUrl, offerPayload))
            {
                yield return request.SendWebRequest();
                if (request.result == UnityWebRequest.Result.Success)
                {
                    answerJson = request.downloadHandler.text;
                }
                else
                {
                    requestError = FormatRequestError(request);
                }
            }

            if (!IsPeerCurrent(attemptPeer, activeGeneration))
            {
                completed("Connection attempt cancelled");
                yield break;
            }

            if (!string.IsNullOrEmpty(requestError))
            {
                completed("Offer request failed: " + requestError);
                yield break;
            }

            OfferResponse answer = null;
            try
            {
                answer = JsonUtility.FromJson<OfferResponse>(answerJson);
            }
            catch (Exception exception)
            {
                operationError = "Invalid SDP answer JSON: " + exception.Message;
            }

            if (!string.IsNullOrEmpty(operationError))
            {
                completed(operationError);
                yield break;
            }

            if (answer == null || string.IsNullOrWhiteSpace(answer.sdp))
            {
                completed("Robot POV server returned an empty SDP answer");
                yield break;
            }

            if (!string.IsNullOrEmpty(answer.type)
                && !string.Equals(answer.type, "answer", StringComparison.OrdinalIgnoreCase))
            {
                completed("Robot POV server returned unexpected SDP type: " + answer.type);
                yield break;
            }

            RTCSessionDescription remoteAnswer = new RTCSessionDescription
            {
                type = RTCSdpType.Answer,
                sdp = answer.sdp
            };

            RTCSetSessionDescriptionAsyncOperation remoteOperation = null;
            operationError = null;
            try
            {
                remoteOperation = attemptPeer.SetRemoteDescription(ref remoteAnswer);
            }
            catch (Exception exception)
            {
                operationError = "Could not set remote SDP: " + exception.Message;
            }

            if (remoteOperation == null)
            {
                completed(operationError);
                yield break;
            }

            yield return remoteOperation;

            if (!IsPeerCurrent(attemptPeer, activeGeneration))
            {
                completed("Connection attempt cancelled");
                yield break;
            }

            if (remoteOperation.IsError)
            {
                completed("Setting remote SDP failed: " + remoteOperation.Error.message);
                yield break;
            }

            float frameDeadline = Time.realtimeSinceStartup + config.FirstFrameTimeoutSeconds;
            while (IsPeerCurrent(attemptPeer, activeGeneration)
                   && currentTexture == null
                   && !peerFailed
                   && Time.realtimeSinceStartup < frameDeadline)
            {
                yield return null;
            }

            if (!IsPeerCurrent(attemptPeer, activeGeneration))
            {
                completed("Connection attempt cancelled");
                yield break;
            }

            if (peerFailed)
            {
                completed(SafeFailure(peerFailureReason));
                yield break;
            }

            if (currentTexture == null)
            {
                completed("WebRTC did not receive its first video frame");
                yield break;
            }

            completed(null);
        }

        private IEnumerator MonitorConnectedPeer(int activeGeneration, Action<string> completed)
        {
            while (IsPeerCurrent(peer, activeGeneration))
            {
                if (peerFailed)
                {
                    completed(SafeFailure(peerFailureReason));
                    yield break;
                }

                if (hasDecodedFrameStats
                    && sourceConnected
                    && sourceHasFrame
                    && !sourceStale
                    && lastDecodedFrameRealtime > 0f
                    && Time.realtimeSinceStartup - lastDecodedFrameRealtime
                    > config.MediaStaleTimeoutSeconds)
                {
                    completed("Decoded video stream stalled");
                    yield break;
                }

                RefreshPublishedState();
                yield return null;
            }

            completed("WebRTC peer was closed");
        }

        private IEnumerator StatusPollingLoop(int activeGeneration)
        {
            WaitForSecondsRealtime interval = new WaitForSecondsRealtime(
                config.StatusPollIntervalSeconds);

            while (IsCurrent(activeGeneration))
            {
                string body = null;
                bool requestSucceeded = false;

                using (UnityWebRequest request = UnityWebRequest.Get(config.StatusUrl))
                {
                    request.timeout = Mathf.Max(1, Mathf.CeilToInt(config.RequestTimeoutSeconds));
                    request.SetRequestHeader("Cache-Control", "no-store");
                    yield return request.SendWebRequest();
                    requestSucceeded = request.result == UnityWebRequest.Result.Success;
                    if (requestSucceeded)
                    {
                        body = request.downloadHandler.text;
                    }
                }

                if (!IsCurrent(activeGeneration))
                {
                    yield break;
                }

                if (requestSucceeded)
                {
                    ApplyServerStatus(body);
                }
                else
                {
                    float grace = Mathf.Max(2f, config.StatusPollIntervalSeconds * 2.5f);
                    serverReachable = statusEverReceived
                                      && Time.realtimeSinceStartup - lastStatusSuccessRealtime <= grace;
                    RefreshPublishedState();
                }

                yield return interval;
            }
        }

        private IEnumerator MetricsLoop(int activeGeneration)
        {
            WaitForSecondsRealtime interval = new WaitForSecondsRealtime(
                config.MetricsIntervalSeconds);

            while (IsCurrent(activeGeneration))
            {
                yield return interval;

                RTCPeerConnection statsPeer = peer;
                if (!useMjpegFallback
                    && statsPeer != null
                    && IsPeerCurrent(statsPeer, activeGeneration))
                {
                    yield return CollectPeerStatistics(statsPeer, activeGeneration);
                }

                if (!IsCurrent(activeGeneration))
                {
                    yield break;
                }

                ClientMetricsPayload payload = new ClientMetricsPayload
                {
                    clientId = config.ClientId,
                    transport = useMjpegFallback ? "mjpeg" : "webrtc",
                    profile = config.Profile,
                    renderedFps = Mathf.Max(0f, renderedFps),
                    latencyMs = Mathf.Max(0f, rttMs * 0.5f + jitterMs),
                    rttMs = Mathf.Max(0f, rttMs),
                    jitterMs = Mathf.Max(0f, jitterMs),
                    packetsLost = Math.Max(0L, packetsLost),
                    framesDropped = Math.Max(0L, framesDropped),
                    frameAgeMs = Mathf.Max(0f, ComputeFrameAgeMs()),
                    reconnects = reconnectCount,
                    noSignal = !ComputeSignalAvailable()
                };

                using (UnityWebRequest request = CreateJsonPost(config.MetricsUrl, payload))
                {
                    yield return request.SendWebRequest();
                    // Metrics are best-effort and must never disturb video recovery.
                }
            }
        }

        private IEnumerator CollectPeerStatistics(
            RTCPeerConnection statsPeer,
            int activeGeneration)
        {
            RTCStatsReportAsyncOperation operation = null;
            try
            {
                operation = statsPeer.GetStats();
            }
            catch (Exception)
            {
                operation = null;
            }

            if (operation == null)
            {
                yield break;
            }

            yield return operation;

            if (!IsPeerCurrent(statsPeer, activeGeneration)
                || operation.IsError
                || operation.Value == null)
            {
                yield break;
            }

            RTCStatsReport report = operation.Value;
            try
            {
                ParsePeerStatistics(report);
            }
            catch (Exception exception)
            {
                Debug.LogWarning("[Robot POV] Could not parse optional WebRTC stats: " + exception.Message);
            }
            finally
            {
                report.Dispose();
            }
        }

        private void ParsePeerStatistics(RTCStatsReport report)
        {
            RTCInboundRTPStreamStats videoInbound = null;
            RTCIceCandidatePairStats selectedPair = null;

            foreach (RTCStats item in report.Stats.Values)
            {
                RTCInboundRTPStreamStats inbound = item as RTCInboundRTPStreamStats;
                if (inbound != null && string.Equals(inbound.kind, "video", StringComparison.OrdinalIgnoreCase))
                {
                    videoInbound = inbound;
                }

                RTCIceCandidatePairStats candidatePair = item as RTCIceCandidatePairStats;
                if (candidatePair != null
                    && candidatePair.nominated
                    && string.Equals(candidatePair.state, "succeeded", StringComparison.OrdinalIgnoreCase))
                {
                    selectedPair = candidatePair;
                }
            }

            float now = Time.realtimeSinceStartup;
            if (videoInbound != null)
            {
                uint decoded = videoInbound.framesDecoded;
                if (!hasDecodedFrameStats || decoded > lastFramesDecoded)
                {
                    if (hasDecodedFrameStats && lastStatsSampleRealtime > 0f)
                    {
                        float elapsed = Mathf.Max(0.001f, now - lastStatsSampleRealtime);
                        renderedFps = (decoded - lastFramesDecoded) / elapsed;
                    }

                    lastDecodedFrameRealtime = now;
                }

                if (videoInbound.framesPerSecond > 0d)
                {
                    renderedFps = (float)videoInbound.framesPerSecond;
                }

                lastFramesDecoded = decoded;
                lastStatsSampleRealtime = now;
                hasDecodedFrameStats = true;
                jitterMs = Mathf.Max(0f, (float)(videoInbound.jitter * 1000d));
                packetsLost = Math.Max(0, videoInbound.packetsLost);
                framesDropped = videoInbound.framesDropped;
            }

            if (selectedPair != null && selectedPair.currentRoundTripTime > 0d)
            {
                rttMs = Mathf.Max(0f, (float)(selectedPair.currentRoundTripTime * 1000d));
            }

            RefreshPublishedState();
        }

        private void ApplyServerStatus(string json)
        {
            ServerStatusPayload payload = null;
            try
            {
                payload = JsonUtility.FromJson<ServerStatusPayload>(json);
            }
            catch (Exception exception)
            {
                Debug.LogWarning("[Robot POV] Invalid /api/status response: " + exception.Message);
            }

            if (payload == null || !payload.ok || payload.source == null)
            {
                serverReachable = false;
                RefreshPublishedState();
                return;
            }

            statusEverReceived = true;
            serverReachable = true;
            lastStatusSuccessRealtime = Time.realtimeSinceStartup;
            sourceName = string.IsNullOrWhiteSpace(payload.source.source)
                ? "unknown"
                : payload.source.source;
            sourceConnected = payload.source.connected;
            sourceStale = payload.source.stale;
            sourceHasFrame = payload.source.has_frame;
            sourceReceivedAgeSeconds = payload.source.received_age_s;
            sourceError = payload.source.error ?? string.Empty;
            RefreshPublishedState();
        }

        private void HandleRemoteTrack(
            RTCPeerConnection owner,
            int activeGeneration,
            RTCTrackEvent trackEvent)
        {
            if (!IsPeerCurrent(owner, activeGeneration))
            {
                return;
            }

            VideoStreamTrack videoTrack = trackEvent.Track as VideoStreamTrack;
            if (videoTrack == null)
            {
                return;
            }

            if (receivedVideoTrack != null && videoReceivedHandler != null)
            {
                receivedVideoTrack.OnVideoReceived -= videoReceivedHandler;
            }

            receivedVideoTrack = videoTrack;
            videoReceivedHandler = texture =>
                HandleVideoTexture(owner, videoTrack, activeGeneration, texture);
            receivedVideoTrack.OnVideoReceived += videoReceivedHandler;
        }

        private void HandleVideoTexture(
            RTCPeerConnection owner,
            VideoStreamTrack track,
            int activeGeneration,
            Texture texture)
        {
            if (!IsPeerCurrent(owner, activeGeneration)
                || track != receivedVideoTrack
                || texture == null)
            {
                return;
            }

            currentTexture = texture;
            firstTextureRealtime = Time.realtimeSinceStartup;
            lastDecodedFrameRealtime = firstTextureRealtime;
            TextureChanged?.Invoke(currentTexture);
            RefreshPublishedState();
        }

        private void HandlePeerConnectionState(
            RTCPeerConnection owner,
            int activeGeneration,
            RTCPeerConnectionState value)
        {
            if (!IsPeerCurrent(owner, activeGeneration))
            {
                return;
            }

            peerConnectionState = value;
            if (value == RTCPeerConnectionState.Disconnected
                || value == RTCPeerConnectionState.Failed
                || value == RTCPeerConnectionState.Closed)
            {
                peerFailed = true;
                peerFailureReason = "WebRTC connection state: " + value;
            }

            RefreshPublishedState();
        }

        private void HandleIceConnectionState(
            RTCPeerConnection owner,
            int activeGeneration,
            RTCIceConnectionState value)
        {
            if (!IsPeerCurrent(owner, activeGeneration))
            {
                return;
            }

            iceConnectionState = value;
            if (value == RTCIceConnectionState.Disconnected
                || value == RTCIceConnectionState.Failed
                || value == RTCIceConnectionState.Closed)
            {
                peerFailed = true;
                peerFailureReason = "ICE connection state: " + value;
            }

            RefreshPublishedState();
        }

        private void RefreshPublishedState()
        {
            if (state == RobotPovConnectionState.Stopped || state == RobotPovConnectionState.Idle)
            {
                return;
            }

            if (peerFailed)
            {
                return;
            }

            if (currentTexture == null)
            {
                return;
            }

            if (ComputeSignalAvailable())
            {
                SetState(RobotPovConnectionState.Connected, "Robot POV video live");
            }
            else
            {
                SetState(RobotPovConnectionState.NoSignal, ComputeNoSignalReason());
            }
        }

        private bool ComputeSignalAvailable()
        {
            if ((!useMjpegFallback && peer == null) || peerFailed || currentTexture == null)
            {
                return false;
            }

            if (useMjpegFallback)
            {
                return statusEverReceived
                       && serverReachable
                       && sourceConnected
                       && !sourceStale
                       && sourceHasFrame
                       && lastDecodedFrameRealtime > 0f
                       && Time.realtimeSinceStartup - lastDecodedFrameRealtime
                       <= config.MediaStaleTimeoutSeconds;
            }

            bool peerConnected = peerConnectionState == RTCPeerConnectionState.Connected
                                 || iceConnectionState == RTCIceConnectionState.Connected
                                 || iceConnectionState == RTCIceConnectionState.Completed;
            if (!peerConnected)
            {
                return false;
            }

            if (!statusEverReceived || !serverReachable)
            {
                return false;
            }

            if (!sourceConnected || sourceStale || !sourceHasFrame)
            {
                return false;
            }

            return !hasDecodedFrameStats
                   || lastDecodedFrameRealtime <= 0f
                   || Time.realtimeSinceStartup - lastDecodedFrameRealtime
                   <= config.MediaStaleTimeoutSeconds;
        }

        private string ComputeNoSignalReason()
        {
            if (state == RobotPovConnectionState.Reconnecting)
            {
                return stateDetail;
            }

            if (!statusEverReceived)
            {
                return "WAITING FOR ROBOT POV SERVER";
            }

            if (!serverReachable)
            {
                return "ROBOT POV SERVER UNREACHABLE";
            }

            if (!sourceConnected)
            {
                return string.IsNullOrWhiteSpace(sourceError)
                    ? "CAMERA DISCONNECTED"
                    : "CAMERA ERROR: " + sourceError;
            }

            if (sourceStale)
            {
                return sourceReceivedAgeSeconds >= 0f
                    ? string.Format("CAMERA FRAME STALE ({0:0.0}s)", sourceReceivedAgeSeconds)
                    : "CAMERA FRAME STALE";
            }

            if (!sourceHasFrame)
            {
                return "WAITING FOR CAMERA FRAME";
            }

            if (currentTexture == null)
            {
                return useMjpegFallback ? "WAITING FOR MJPEG VIDEO" : "WAITING FOR WEBRTC VIDEO";
            }

            if (hasDecodedFrameStats
                && lastDecodedFrameRealtime > 0f
                && Time.realtimeSinceStartup - lastDecodedFrameRealtime
                > config.MediaStaleTimeoutSeconds)
            {
                return "VIDEO STREAM STALLED";
            }

            if (peerFailed)
            {
                return SafeFailure(peerFailureReason).ToUpperInvariant();
            }

            return "NO SIGNAL";
        }

        private float ComputeFrameAgeMs()
        {
            float reference = lastDecodedFrameRealtime > 0f
                ? lastDecodedFrameRealtime
                : firstTextureRealtime;
            return reference > 0f
                ? Mathf.Max(0f, (Time.realtimeSinceStartup - reference) * 1000f)
                : 0f;
        }

        private void DisposePeer()
        {
            RTCPeerConnection oldPeer = peer;
            peer = null;

            if (receivedVideoTrack != null && videoReceivedHandler != null)
            {
                receivedVideoTrack.OnVideoReceived -= videoReceivedHandler;
            }

            receivedVideoTrack = null;
            videoReceivedHandler = null;

            if (oldPeer != null)
            {
                oldPeer.OnConnectionStateChange = null;
                oldPeer.OnIceConnectionChange = null;
                oldPeer.OnTrack = null;
                try
                {
                    oldPeer.Dispose();
                }
                catch (Exception exception)
                {
                    Debug.LogWarning("[Robot POV] WebRTC cleanup warning: " + exception.Message);
                }
            }

            peerFailed = false;
            peerFailureReason = string.Empty;
            peerConnectionState = RTCPeerConnectionState.New;
            iceConnectionState = RTCIceConnectionState.New;
            ResetMediaStatistics();
        }

        private void ResetMediaStatistics()
        {
            firstTextureRealtime = -1f;
            lastDecodedFrameRealtime = -1f;
            lastStatsSampleRealtime = -1f;
            lastFramesDecoded = 0;
            hasDecodedFrameStats = false;
            renderedFps = 0f;
            rttMs = 0f;
            jitterMs = 0f;
            packetsLost = 0;
            framesDropped = 0;
        }

        private void ClearTexture()
        {
            if (currentTexture == null)
            {
                return;
            }

            currentTexture = null;
            TextureChanged?.Invoke(null);
        }

        private void SetState(RobotPovConnectionState newState, string detail)
        {
            string safeDetail = string.IsNullOrWhiteSpace(detail) ? newState.ToString() : detail;
            if (state == newState && string.Equals(stateDetail, safeDetail, StringComparison.Ordinal))
            {
                return;
            }

            state = newState;
            stateDetail = safeDetail;
            StateChanged?.Invoke(state, stateDetail);

            if (newState == RobotPovConnectionState.Error)
            {
                Debug.LogError("[Robot POV] " + stateDetail);
            }
            else if (newState == RobotPovConnectionState.Reconnecting
                     || newState == RobotPovConnectionState.NoSignal)
            {
                Debug.LogWarning("[Robot POV] " + stateDetail);
            }
            else
            {
                Debug.Log("[Robot POV] " + stateDetail);
            }
        }

        private bool IsCurrent(int activeGeneration)
        {
            return isActiveAndEnabled && activeGeneration == generation;
        }

        private bool IsPeerCurrent(RTCPeerConnection candidate, int activeGeneration)
        {
            return IsCurrent(activeGeneration) && candidate != null && candidate == peer;
        }

        private static bool TryReadGatheringState(
            RTCPeerConnection candidate,
            out RTCIceGatheringState value)
        {
            try
            {
                value = candidate.GatheringState;
                return true;
            }
            catch (Exception)
            {
                value = RTCIceGatheringState.New;
                return false;
            }
        }

        private UnityWebRequest CreateJsonPost<T>(string url, T payload)
        {
            byte[] body = Encoding.UTF8.GetBytes(JsonUtility.ToJson(payload));
            UnityWebRequest request = new UnityWebRequest(url, UnityWebRequest.kHttpVerbPOST)
            {
                uploadHandler = new UploadHandlerRaw(body),
                downloadHandler = new DownloadHandlerBuffer(),
                timeout = Mathf.Max(1, Mathf.CeilToInt(config.RequestTimeoutSeconds))
            };
            request.SetRequestHeader("Content-Type", "application/json");
            request.SetRequestHeader("Accept", "application/json");
            request.SetRequestHeader("Cache-Control", "no-store");
            return request;
        }

        private static string FormatRequestError(UnityWebRequest request)
        {
            string response = request.downloadHandler == null
                ? string.Empty
                : request.downloadHandler.text;
            if (response != null && response.Length > 180)
            {
                response = response.Substring(0, 180);
            }

            return string.IsNullOrWhiteSpace(response)
                ? string.Format("HTTP {0}: {1}", request.responseCode, request.error)
                : string.Format("HTTP {0}: {1}", request.responseCode, response);
        }

        private static string SafeFailure(string value)
        {
            return string.IsNullOrWhiteSpace(value) ? "WebRTC connection lost" : value;
        }
    }
}
