# Архитектура телеуправления Pico 4 Ultra ↔ Unitree R1

## Решение для транспорта

На первом этапе следует сохранить UDP между Unity и ПК. Это уже работающий путь,
он имеет низкую задержку, не требует ROS 2/DDS-библиотек в Android-приложении и
легко профилируется. Вместо локализованной CSV-строки используется один JSON-пакет
на кадр с версией, sequence number, временем, позами и органами управления.

ROS# обычно означает WebSocket/rosbridge и добавляет ещё один сервер, сериализацию
и менее предсказуемую задержку. Он полезен для UI и некритичной телеметрии, но не
даёт выигрыша этому real-time-ish каналу. micro-ROS предназначен прежде всего для
микроконтроллеров/RTOS; Android-шлем не является его естественной целью. Если позже
понадобится нативная ROS-интеграция Unity, разумнее оценить ROS-TCP-Connector или
свой бинарный UDP-протокол (CBOR/MessagePack/Protobuf), но только после измерений.

JSON выбран как отладочный v1. При 72–90 Гц такой пакет остаётся намного меньше
обычного MTU 1500 байт и не фрагментируется. UDP-пакет содержит полный snapshot,
поэтому потеря одного кадра не повреждает состояние следующего.

## Слои

```text
Pico / Unity / OpenXR
  └─ UDP JSON v1, 72–90 Hz, Wi-Fi LAN
       └─ vr_teleop_bridge (валидация, frame conversion, dead zone, watchdog)
            ├─ /vr/{left,right}_controller/pose  geometry_msgs/PoseStamped
            ├─ /vr/head/pose                     geometry_msgs/PoseStamped
            ├─ /vr/joy                           sensor_msgs/Joy
            ├─ /vr/cmd_vel                       geometry_msgs/TwistStamped
            └─ /vr/teleop/active                 std_msgs/Bool
                 └─ r1_kinematics_control
                      ├─ EMA + URDF DLS IK → ros2_control JointTrajectory
                      ├─ trigger EMA → left/right hand controllers
                      └─ deadman/watchdog gate → Gazebo /r1/sim/cmd_vel
                            └─ r1_sim_whole_body_planner (Gazebo only)
                                 ├─ alternating leg proxy
                                 ├─ arm counter-swing
                                 └─ bounded waist compensation

Robot cameras → ROS image/GStreamer/WebRTC → hardware-decoded stereo texture in Unity
```

В physical live launch IK запускается в `dry_run=true` как shaping/safety слой:
его `/r1_kinematics_control/debug/arm_trajectory` получает `r1_live_writer` при
`enable_arms=true`. Writer проверяет полный набор десяти arm joints, свежесть
feedback/deadman/kill, ограничивает slew и только затем передаёт объединённый
13-joint кадр в официальный `ArmSdk`. Ноги не проходят через этот путь: их
`TwistStamped` после deadzone/ramp приходит в R1 `LocoClient.SetVelocity`;
остановка использует `StopMove`.

Мост намеренно не публикует глобальный `/cmd_vel`: этот топик слишком легко
случайно подключить к настоящему роботу. `/vr/cmd_vel` — только намерение оператора.
R1-адаптер должен отдельно проверять `/vr/teleop/active`, свежесть команды,
ограничения скорости и состояние робота, а при timeout вызывать штатный StopMove.

Локально установленный `unitree_ros2` содержит high-level locomotion clients для
G1/H2 (`/api/sport/request`, API 7105), но R1-специфичный контракт в найденной
версии отсутствует. Нельзя считать G1/H2 API совместимым с R1 без документации и
проверки прошивки. Пакет `my_robot_control`, публикующий `unitree_go/LowCmd`, также
не следует использовать для R1: сообщение семейства `unitree_go` и управление
отдельными моторами без подтверждённой карты суставов опасны.

## ROS 2 сообщения и системы координат

| Данные | Сообщение | Причина |
|---|---|---|
| Кисти и HMD | `PoseStamped` | позиция + ориентация + timestamp + `frame_id` |
| Сырые стики/кнопки | `sensor_msgs/Joy` | стандартный интерфейс teleop |
| Скорость тела | `TwistStamped` | линейная/угловая скорость и время получения |
| Разрешение движения | `std_msgs/Bool` | простое явное состояние deadman/watchdog |
| Камеры | `sensor_msgs/Image`/`CompressedImage` внутри ROS; WebRTC/GStreamer до Unity | ROS для обработки, аппаратный видеокодек для низкой задержки |

Один `Quaternion` для руки недостаточен: он задаёт только ориентацию. Для
ретаргетинга руки нужна хотя бы полная поза контроллера относительно XR origin,
а лучше также позы головы/плеч и калибровка пропорций человека к роботу.

Unity использует `x=right, y=up, z=forward`, ROS REP-103 для тела —
`x=forward, y=left, z=up`. Unity-скрипт преобразует:

```text
position_ros   = ( z, -x,  y)
quaternion_ros = (-qz, qx, -qy, qw)
```

Нода публикует позы в `vr_tracking`. Позже calibration-нода должна создать TF
`robot_base → vr_tracking_calibrated`; не следует выдавать XR origin за `base_link`.
Timestamp сообщений — время приёма ROS-нодой, потому что часы Android и ПК пока не
синхронизированы. `client_time_ms` сохраняется в wire protocol для будущей метрики.

Оси `Joy`: `[left_x, left_y, right_x, right_y, left_trigger, right_trigger]`;
кнопки: `[deadman, left_trigger_pressed, right_trigger_pressed]`.
В типичной раскладке Unity `+left_x` означает вправо, поэтому ROS `linear.y` имеет
обратный знак. `+right_x` означает поворот вправо, поэтому ROS `angular.z` также
имеет обратный знак. Все знаки и пределы задаются ROS-параметрами.

## JSON wire protocol v1

```json
{
  "v": 1,
  "seq": 42,
  "client_time_ms": 1789000000000,
  "left":  {"p": [0.35, 0.25, 1.20], "q": [0.0, 0.0, 0.0, 1.0]},
  "right": {"p": [0.35,-0.25, 1.20], "q": [0.0, 0.0, 0.0, 1.0]},
  "head":  {"p": [0.0, 0.0, 1.65], "q": [0.0, 0.0, 0.0, 1.0]},
  "sticks": {"left": [0.0, 0.5], "right": [0.0, 0.0]},
  "triggers": [0.0, 0.0],
  "deadman": true
}
```

Все числа JSON используют точку независимо от локали. Максимальный размер
датаграммы по умолчанию 4096 байт. Нода отклоняет NaN/Inf, неверную структуру,
неправдоподобную позицию, вырожденный кватернион, дубликаты и пакеты не по порядку.
Кватернионы нормализуются. После `packet_timeout_sec` (по умолчанию 0.25 с) нода
снимает active и публикует нулевую скорость.

`deadman` должен удерживаться физической кнопкой. Это программное разрешение, а не
аварийный останов. Для испытаний рядом с роботом нужен отдельный штатный/физический
E-stop; его нельзя строить на том же Wi-Fi и UDP-канале.

## Пошаговый план

1. **Мост без робота.** Собрать пакет, запустить Unity sender или test sender,
   проверить частоту, оси, знаки, watchdog и потери пакетов. Записывать rosbag.
2. **TF и калибровка.** Зафиксировать XR origin, реализовать recenter, измерить
   neutral pose и преобразование к `base_link`. Добавить калибровку плеч/масштаба.
3. **Симуляция рук.** По `PoseStamped` решать IK обеих рук (MoveIt 2 или
   собственный differential IK), ограничивать workspace, joint position/velocity,
   фильтровать jitter и обнаруживать self-collision. Сначала RViz/симулятор.
   Локальный `r1_sim_whole_body_planner` теперь синхронизирует эту позу с
   визуальным шагом ног и малой компенсацией туловища, но не является balance loop.
4. **Адаптер локомоции.** После получения официального R1 SDK написать отдельную
   ноду `/vr/cmd_vel → R1 high-level locomotion`. В ней обязательны собственный
   watchdog, enable-state, saturation/rate limiter и явный `StopMove`.
5. **Hardware-in-loop.** Робот на страховочной раме, минимальные скорости,
   помощник у E-stop. Сначала руки и ноги раздельно, затем совместно.
6. **Видео.** Определить camera topics/RTSP на R1. Для прототипа использовать
   GStreamer H.264/H.265; для интерактивной двусторонней сессии — WebRTC. Декодировать
   аппаратно на Pico. Не пересылать raw ROS Image через JSON/UDP. Измерить glass-to-
   glass latency; целиться в <80–100 мс и предусмотреть потерю/заморозку изображения.
7. **Эксплуатационная безопасность.** Network isolation, authentication/encryption
   (VPN/DTLS/QUIC), health telemetry, state machine, logging, тесты packet loss и
   recovery. Только после этого повышать speed limits.

## Проверка первого этапа

```bash
# Терминал 1
source /opt/ros/humble/setup.bash
source /home/unitree/Unitree_Project/R1_Teleoperation/ros2_ws/install/setup.bash
ros2 launch vr_teleop_bridge vr_bridge.launch.py

# Терминал 2: deadman удержан, движение вперёд
ros2 run vr_teleop_bridge vr_test_sender \
  --host 192.168.8.131 --deadman --left-y 0.5

# Терминал 3
ros2 topic echo /vr/cmd_vel
ros2 topic echo /vr/teleop/active
```

Ожидается `linear.x ≈ 0.14 m/s` при настройках по умолчанию (dead zone
перенормируется). При штатном завершении sender сразу отправляет `active: false` и
нулевую команду; при обрыве процесса или сети watchdog делает это через ~0.25 с.
Для первой проверки на ПК можно передать `--host 127.0.0.1`.

Для явной проверки watchdog используйте `--simulate-drop`: sender завершится без
финального пакета deadman=false, и мост обязан остановить intent по timeout.
