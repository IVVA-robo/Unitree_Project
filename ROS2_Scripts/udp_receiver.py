import socket

# Настройки сети (должны совпадать с Unity)
UDP_IP = "0.0.0.0"  # Слушаем все входящие подключения на этом ПК
UDP_PORT = 9090

# Создаем и настраиваем сокет
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"✅ UDP сервер запущен!\nОжидание данных от шлема Pico на порту {UDP_PORT}...\n(Для выхода нажми Ctrl+C)")

try:
    while True:
        # Получаем пакет данных
        data, addr = sock.recvfrom(1024)
        message = data.decode('utf-8')
        
        # Разбиваем строку на 4 числа (X, Y, Z, W)
        values = message.split(',')
        if len(values) == 4:
            x, y, z, w = map(float, values)
            # Выводим красиво отформатированные данные в консоль
            print(f"VR Поворот -> X: {x: .4f} | Y: {y: .4f} | Z: {z: .4f} | W: {w: .4f}")
        else:
            print(f"Неизвестный формат от {addr}: {message}")

except KeyboardInterrupt:
    print("\nОстановка сервера...")
finally:
    sock.close()
