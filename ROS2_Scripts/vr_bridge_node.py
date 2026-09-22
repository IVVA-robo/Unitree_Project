import rclpy
from rclpy.node import Node
import socket
import threading
from geometry_msgs.msg import Quaternion, Twist

class VRBridgeNode(Node):
    def __init__(self):
        super().__init__('vr_bridge_node')
        
        # ROS 2 Паблишеры
        self.right_pub = self.create_publisher(Quaternion, '/vr/right_hand', 10)
        self.left_pub = self.create_publisher(Quaternion, '/vr/left_hand', 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Настройка UDP
        self.udp_port = 9090
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", self.udp_port))
        
        self.get_logger().info(f"VR Bridge запущен! Слушаю UDP порт {self.udp_port}...")
        
        # Запуск слушателя в отдельном потоке
        self.listen_thread = threading.Thread(target=self.udp_listen_loop, daemon=True)
        self.listen_thread.start()

    def udp_listen_loop(self):
        while rclpy.ok():
            try:
                data, _ = self.sock.recvfrom(1024)
                raw_msg = data.decode('utf-8').strip()
                self.parse_and_publish(raw_msg)
            except Exception as e:
                self.get_logger().error(f"UDP Ошибка: {e}")

    def parse_and_publish(self, raw_msg):
        try:
            # Фикс локализации: меняем запятые на точки
            msg = raw_msg.replace(',', '.')
            parts = msg.split('|')
            
            if len(parts) == 4:
                # 1. Правая рука (Кватернион)
                rx, ry, rz, rw = map(float, parts[0].split(';'))
                right_quat = Quaternion(x=rx, y=ry, z=rz, w=rw)
                self.right_pub.publish(right_quat)
                
                # 2. Левая рука (Кватернион)
                lx, ly, lz, lw = map(float, parts[1].split(';'))
                left_quat = Quaternion(x=lx, y=ly, z=lz, w=lw)
                self.left_pub.publish(left_quat)
                
                # 3. Локомоция (Стики -> Twist)
                ls_x, ls_y = map(float, parts[2].split(';')) # Левый стик: Вперед/Вбок
                rs_x, rs_y = map(float, parts[3].split(';')) # Правый стик: Поворот
                
                twist = Twist()
                twist.linear.x = ls_y   # Вперед/назад
                twist.linear.y = ls_x   # Влево/вправо (стрейф)
                twist.angular.z = rs_x  # Поворот вокруг оси
                self.vel_pub.publish(twist)
                
        except Exception as e:
            self.get_logger().warn(f"Ошибка парсинга строки: {raw_msg} -> {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VRBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
