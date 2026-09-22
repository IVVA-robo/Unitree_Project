import socket
import json
import time
import math

udp_ip = "127.0.0.1"
udp_port = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print("Эмулятор VR запущен! Имитирую плавные движения рук...")
t = 0.0

try:
    while True:
        # Генерируем плавный угол от -1.0 до 1.0 радиана
        angle = math.sin(t)
        
        # Двигаем плечи и локти обеих рук
        vr_data = {
            "left_arm": [angle, 0.0, 0.0, abs(angle), 0.0],
            "right_arm": [-angle, 0.0, 0.0, abs(angle), 0.0]
        }
        
        # Отправляем пакет в наш VR-мост
        sock.sendto(json.dumps(vr_data).encode('utf-8'), (udp_ip, udp_port))
        
        t += 0.05
        time.sleep(0.02) # 50 FPS
except KeyboardInterrupt:
    print("\nЭмулятор остановлен.")