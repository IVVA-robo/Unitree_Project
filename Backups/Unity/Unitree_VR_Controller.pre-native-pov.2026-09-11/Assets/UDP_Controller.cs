using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using UnityEngine.XR;

public class UDP_Controller : MonoBehaviour
{
    [Header("Network Settings")]
    public string hostIP = "192.168.8.131";
    public int port = 9090;
    private UdpClient udpClient;

    void Start()
    {
        udpClient = new UdpClient();
        Debug.Log($"UDP Controller Started. Targeting {hostIP}:{port}");
    }

    void Update()
    {
        // 1. Получаем повороты контроллеров
        Quaternion rightRot = GetNodeRotation(XRNode.RightHand);
        Quaternion leftRot = GetNodeRotation(XRNode.LeftHand);
        
        // 2. Получаем данные со стиков
        Vector2 leftStick = GetNodeAxis(XRNode.LeftHand);
        Vector2 rightStick = GetNodeAxis(XRNode.RightHand);

        // 3. Формируем строки с разделителем ";" (меняем запятые на точки на всякий случай)
        string rightStr = $"{rightRot.x};{rightRot.y};{rightRot.z};{rightRot.w}".Replace(",", ".");
        string leftStr = $"{leftRot.x};{leftRot.y};{leftRot.z};{leftRot.w}".Replace(",", ".");
        string leftStickStr = $"{leftStick.x};{leftStick.y}".Replace(",", ".");
        string rightStickStr = $"{rightStick.x};{rightStick.y}".Replace(",", ".");

        // 4. Склеиваем всё вместе через "|"
        string finalMessage = $"{rightStr}|{leftStr}|{leftStickStr}|{rightStickStr}";

        // 5. Отправляем по UDP
        byte[] data = Encoding.UTF8.GetBytes(finalMessage);
        udpClient.Send(data, data.Length, hostIP, port);
    }

    // Вспомогательная функция для поворота
    Quaternion GetNodeRotation(XRNode node)
    {
        InputDevice device = InputDevices.GetDeviceAtXRNode(node);
        if (device.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rot))
            return rot;
        return Quaternion.identity;
    }

    // Вспомогательная функция для стика
    Vector2 GetNodeAxis(XRNode node)
    {
        InputDevice device = InputDevices.GetDeviceAtXRNode(node);
        if (device.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 axis))
            return axis;
        return Vector2.zero;
    }

    void OnApplicationQuit()
    {
        if (udpClient != null)
            udpClient.Close();
    }
}
