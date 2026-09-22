import time
import sys
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

def state_handler(msg: LowState_):
    print("\n=== Получены данные от Unitree R1 ===")
    print(msg)
    print("======================================\n")
    sys.exit(0)

if __name__ == '__main__':
    # 1. Настройка сети на наш USB-Ethernet
    ChannelFactoryInitialize(0, "enxb4b024be59fe")
    
    # 2. Создание подписчика на LowState
    sub = ChannelSubscriber("rt/lf/lowstate", LowState_)
    sub.Init(state_handler, 10)
    
    print("Слушаем робота... Ожидание данных...")
    
    # 3. Бесконечный цикл ожидания
    while True:
        time.sleep(1)
