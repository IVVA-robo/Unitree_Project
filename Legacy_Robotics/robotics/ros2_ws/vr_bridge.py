import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import socket
import json

class VRBridge(Node):
    def __init__(self):
        super().__init__('vr_bridge')
        # 1. Подключаемся к моторам робота
        self.publisher_ = self.create_publisher(Float64MultiArray, '/arms_controller/commands', 10)
        
        # 2. Настраиваем UDP-радиоприемник
        self.udp_ip = "0.0.0.0"  # Слушаем Wi-Fi со всех адресов
        self.udp_port = 5005     # Канал (порт) для связи с Pico 4 Pro
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.setblocking(False) # Не зависать, если данных пока нет
        
        self.get_logger().info(f"VR-Мост АКТИВЕН! Жду данные от Pico 4 Pro на порту {self.udp_port}...")
        
        # 3. Читаем эфир 50 раз в секунду (0.02 сек)
        self.timer = self.create_timer(0.02, self.receive_vr_data)

    def receive_vr_data(self):
        try:
            # Пытаемся поймать пакет данных по Wi-Fi
            data, addr = self.sock.recvfrom(1024)
            
            # Расшифровываем текстовое сообщение (ожидаем формат JSON)
            vr_message = json.loads(data.decode('utf-8'))
            
            # Берем углы для левой и правой руки (по умолчанию нули, если данных нет)
            left_arm = vr_message.get("left_arm", [0.0, 0.0, 0.0, 0.0, 0.0])
            right_arm = vr_message.get("right_arm", [0.0, 0.0, 0.0, 0.0, 0.0])
            
            # Склеиваем 10 цифр вместе и отправляем моторам
            msg = Float64MultiArray()
            msg.data = [float(a) for a in (left_arm + right_arm)]
            self.publisher_.publish(msg)
            
            # === ВОТ ЭТА СТРОЧКА ДЛЯ ОТЛАДКИ ===
            self.get_logger().info(f"Поймал сигнал из VR! Передаю в моторы: {msg.data}")
            
        except BlockingIOError:
            # Если эфир пуст — ничего не делаем
            pass
        except json.JSONDecodeError:
            self.get_logger().warning("Получены кривые данные, это не JSON!")
        except Exception as e:
            self.get_logger().error(f"Ошибка связи: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VRBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("VR-Мост отключен.")
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()