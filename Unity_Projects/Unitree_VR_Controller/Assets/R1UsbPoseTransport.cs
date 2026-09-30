using System;
using System.IO;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

// USB-only, pull-based transport through adb reverse tcp:19092 tcp:19092.
// One laptop challenge requests ONE current pose. No pose queue or Wi-Fi fallback.
public sealed class R1UsbPoseTransport : IDisposable
{
    private TcpClient client;
    private IAsyncResult connecting;
    private NetworkStream stream;
    private readonly StringBuilder request = new StringBuilder();
    private string token;
    private float retryAt;
    private float lastProgress;
    public bool HasRequest => token != null;
    public bool Reconnected { get; private set; }

    public static bool Requested()
    {
#if UNITY_ANDROID && !UNITY_EDITOR
        using (var player = new AndroidJavaClass("com.unity3d.player.UnityPlayer"))
        using (var activity = player.GetStatic<AndroidJavaObject>("currentActivity"))
        using (var intent = activity.Call<AndroidJavaObject>("getIntent"))
            return intent.Call<bool>("getBooleanExtra", "r1_usb", false);
#else
        return false;
#endif
    }

    public void Poll()
    {
        Reconnected = false;
        float now = Time.realtimeSinceStartup;
        try
        {
            if (client == null)
            {
                if (now < retryAt) return;
                client = new TcpClient { NoDelay = true };
                connecting = client.BeginConnect("127.0.0.1", 19092, null, null);
                lastProgress = now;
            }
            if (connecting != null)
            {
                if (!connecting.IsCompleted)
                {
                    if (now - lastProgress > 0.5f) Dispose();
                    return;
                }
                client.EndConnect(connecting);
                connecting = null;
                stream = client.GetStream();
                stream.ReadTimeout = 50;
                stream.WriteTimeout = 50;
                lastProgress = now;
                Reconnected = true;
                Debug.Log("R1 USB pose transport connected (ADB, no Wi-Fi fallback)");
            }
            if (now - lastProgress > 0.5f)
            {
                Dispose();
                return;
            }
            for (int count = 0; count < 64 && stream.DataAvailable && token == null; count++)
            {
                int value = stream.ReadByte();
                if (value < 0) throw new IOException("USB tunnel closed");
                if (value == '\n')
                {
                    if (request.Length != 32)
                        throw new IOException("Invalid USB challenge");
                    token = request.ToString();
                    request.Clear();
                    lastProgress = now;
                }
                else if ((value >= '0' && value <= '9') || (value >= 'a' && value <= 'f'))
                {
                    request.Append((char)value);
                    if (request.Length > 32) throw new IOException("USB challenge too long");
                }
                else throw new IOException("Invalid USB challenge character");
            }
        }
        catch (Exception exception) when (exception is SocketException
            || exception is IOException || exception is ObjectDisposedException)
        {
            Debug.LogWarning("R1 USB waiting for cable/tunnel: " + exception.Message);
            Dispose();
        }
    }

    public void Send(byte[] pose)
    {
        if (token == null || stream == null) return;
        try
        {
            byte[] prefix = Encoding.ASCII.GetBytes(token + " ");
            byte[] frame = new byte[prefix.Length + pose.Length + 1];
            Buffer.BlockCopy(prefix, 0, frame, 0, prefix.Length);
            Buffer.BlockCopy(pose, 0, frame, prefix.Length, pose.Length);
            frame[frame.Length - 1] = (byte)'\n';
            stream.Write(frame, 0, frame.Length);
            token = null;
            lastProgress = Time.realtimeSinceStartup;
        }
        catch (Exception exception) when (exception is SocketException
            || exception is IOException || exception is ObjectDisposedException)
        {
            Debug.LogWarning("R1 USB pose send failed: " + exception.Message);
            Dispose();
        }
    }

    public void Dispose()
    {
        stream?.Dispose();
        client?.Close();
        stream = null;
        client = null;
        connecting = null;
        token = null;
        request.Clear();
        retryAt = Time.realtimeSinceStartup + 0.5f;
    }
}
