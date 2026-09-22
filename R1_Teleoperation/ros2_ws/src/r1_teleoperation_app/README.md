# r1_teleoperation_app

Единая точка запуска безопасной демонстрации телеуправления R1. Один launch
поднимает уже существующие компоненты проекта:

* UDP-мост Pico (`vr_teleop_bridge`);
* Gazebo, headset-relative IK и визуализацию ног (`r1_telepresence_sim`);
* video-only POV web server (`r1_robot_pov`).

По умолчанию включены `ROS_LOCALHOST_ONLY=1`, `ROS_DOMAIN_ID=81`, видео из
Gazebo ROS camera topics, `headset_relative_enabled=true` и
`hardware_enabled=false`. Hardware adapter, Unitree DDS и моторные команды в
этот launch намеренно не входят.

После сборки из корня проекта:

```bash
source /opt/ros/humble/setup.bash
source /home/unitree/Unitree_Project/Legacy_Robotics/robotics/ros2_ws/install/setup.bash
source ros2_ws/install/setup.bash
ros2 launch r1_teleoperation_app r1_teleoperation.launch.py
```

Видео открывается на `http://127.0.0.1:8080/` (или на IP ноутбука в LAN).
Для запуска без видео используйте `start_video:=false`. Для live ROS-камеры
нужно явно указать проверенный профиль и источник, например
`video_source:=ros video_env_file:=/absolute/path/to/profile.env`.

Из корня проекта есть wrapper с общей обработкой Ctrl-C:

```bash
./scripts/r1-teleoperation
```

Он проверяет, что UDP/HTTP/Gazebo-порты свободны, и отказывается запускать
второй экземпляр поверх уже работающего сервиса. При необходимости порты и
режимы задаются переменными `R1_TELEOP_UDP_PORT`, `R1_TELEOP_VIDEO_PORT`,
`R1_TELEOP_GAZEBO_PORT`, `R1_TELEOP_START_VIDEO` и
`R1_TELEOP_START_BRIDGE`, а также `R1_TELEOP_START_SIMULATION`.

Состояние уже запущенного приложения можно получить безопасной read-only
командой:

```bash
./scripts/r1-teleoperation-status
```

## Единый hardware-adjacent dry-run

Для проверки входа от очков, поворота головы и стиков **без реального
управления R1** используйте отдельный launch:

```bash
source /opt/ros/humble/setup.bash
source /home/unitree/Unitree_Project/R1_Teleoperation/install/setup.bash
ros2 launch r1_teleoperation_app r1_teleop_dry_run.launch.py
```

Он принудительно устанавливает `ROS_LOCALHOST_ONLY=1`,
`ROBOT_DRY_RUN=1`, `ROBOT_ENABLE_ACTUATION=0` и все подтверждения live-режима
в `0`. В launch нет Unitree `ArmSdk`, `LocoClient`, `rt/arm_sdk`, `rt/lowcmd`,
sport-команд и реального `/cmd_vel`. Голова и locomotion публикуют только
отладочные данные в `/r1_hardware_adapter/debug/head/*` и
`/r1/locomotion_dry_run/debug/*`.

При старте программный kill switch зафиксирован в `true`. Для диагностического
теста после проверки статуса можно снять его только в dry-run:

```bash
ros2 service call /r1/safety/set_kill std_srvs/srv/SetBool "{data: false}"
```

Затем, пока поток pose свежий, вызовите нейтральную калибровку головы и
удерживайте обычный Deadman в VR:

```bash
ros2 service call /r1/head/calibrate_neutral std_srvs/srv/Trigger "{}"
```

Профиль стиков по умолчанию — `slow-safe`. Для проверок отладочной логики без
физического R1 можно выбрать `locomotion_mode:=normal` или
`locomotion_mode:=exhibition`; это меняет только debug topic.

Опциональный arm pipeline остаётся выключенным, чтобы запуск работал без
Ethernet и с роботом на зарядке. Только когда нужен **read-only** LowState и
IK-debug подключённого робота, запускайте:

```bash
ros2 launch r1_teleoperation_app r1_teleop_dry_run.launch.py \
  start_arm_pipeline:=true arm_network_interface:=enxb4b024be59fe
```

Эта опция по-прежнему не создаёт writer и не посылает команды моторам.

## Physical live launch и DDS ABI

`r1_teleop_live.launch.py` содержит процессы, слинкованные с Unitree
CycloneDDS, даже когда writer настроен на `transport:=mock`. Поэтому launch до
старта любого дочернего процесса принудительно выбирает ROS
`rmw_fastrtps_cpp`, ставит каталог согласованной vendor-пары
`libddsc.so.0`/`libddscxx.so.0` первым в `LD_LIBRARY_PATH` и проверяет наличие
всех трёх библиотек. Отсутствующий Fast DDS, неподдерживаемая архитектура,
относительный путь SDK или неполная vendor-пара дают `[BLOCKED]`; ни один
Unitree-процесс при этом не запускается.

Прямой запуск без аргументов не создаёт и read-only соединение с роботом:
`start_readonly_reader` по умолчанию равен `false`, include
`r1_sdk_transport` пропускается, а его `sdk_enabled` связан с тем же явным
opt-in вместо безусловного `true`. Штатный `scripts/r1-live-session` начинает
с `START_READER=true` и передаёт его явно только после обязательного SDK
preflight. Если wrapper уже проверил существующий reader и свежий конечный
26-суставный sample, он явно передаёт `false` и переиспользует этот reader.

Обычный physical-сеанс всё равно следует запускать только через проектный
live-wrapper: launch-защита закрывает ABI-ошибку, но не заменяет Ethernet,
LowState и commissioning preflight. Для нестандартного расположения SDK до
запуска wrapper задайте абсолютный `R1_UNITREE_SDK_ROOT`; по умолчанию
используется проверенный SDK из `Legacy_Robotics`.
