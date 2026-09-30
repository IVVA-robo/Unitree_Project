# r1_kinematics_control

Безопасный промежуточный слой между `vr_teleop_bridge` и контроллерами Gazebo.

## Что делает узел

- трансформирует VR `PoseStamped` в base frame каждой руки через TF2;
- считает `hand_local = inverse(head_pose) * hand_world`, поэтому ходьба
  по комнате не сдвигает руки аватара;
- строит yaw-стабилизированный body proxy с шеей, грудью, поясом и
  зеркально-симметричными плечами;
- сглаживает позицию time-based EMA, ориентацию — shortest-path quaternion SLERP;
- извлекает serial chains из URDF и решает IK adaptive damped least squares;
- ограничивает workspace, joint limits, IK step и выходную joint velocity;
- удерживает левую и правую кисть по разные стороны центральной плоскости
  туловища (`body_proxy.min_hand_lateral_m`), чтобы перекрещенные контроллеры
  не превращались в crossed-arm IK pose;
- публикует обе руки одним `trajectory_msgs/JointTrajectory`;
- плавно интерполирует пальцы между YAML-позами open/closed;
- пропускает `TwistStamped` в namespaced Gazebo intent только при свежем deadman;
- немедленно публикует нулевой `Twist` при deadman=false и при timeout.

## Headset-relative калибровка

`headset_relative_enabled: true` — режим по умолчанию. Старые
`left_pose_neutral_vr`/`right_pose_neutral_vr` используются только при
`headset_relative_enabled: false`.

Для первой калибровки:

1. Отпустить deadman, стать прямо и смотреть вперёд.
2. Держать контроллеры в одинаковой нейтральной позе.
3. Вызвать:

   ```bash
   ros2 service call /vr/calibrate_body std_srvs/srv/Trigger '{}'
   ```

Калибровка сохраняется в
`~/Unitree_Project/Configs/Calibration/r1_body_calibration.json`. Она
запоминает высоту HMD, рост, ширину плеч, reach рук, offsets
шеи/груди/пояса и две точно зеркальные нейтральные точки рук.
Точные рост, ширину плеч и reach лучше вписать в `body_proxy` в YAML.

После загрузки сохранённой калибровки узел защищает каждое новое включение
Deadman от устаревшей нейтрали. На первом синхронном VR-сэмпле после каждого
re-arm обе кисти сравниваются с сохранёнными точками; по умолчанию допустима
ошибка `0.15 m` (`body_proxy.neutral_pose_max_error_m`). При превышении
публикация целей рук блокируется до следующей калибровки, а в журнале
появляется `body_calibration_neutral_mismatch`. Это предотвращает отправку
старого смещения в IK и live writer.

HMD и два контроллера считаются одним синхронным кадром при разбросе меток
времени не более `body_proxy.sync_tolerance_sec`. Для Pico по умолчанию
используется `0.08` с; это меньше `pose_timeout_sec=0.25` с, поэтому потеря
tracking по-прежнему останавливает live-сеанс через watchdog.

Безопасная процедура повторной калибровки:

1. Отпустить Deadman и удерживать HMD прямо, а оба контроллера — в нужной
   нейтральной позе.
2. Выполнить сервис калибровки:

   ```bash
   ros2 service call /vr/calibrate_body std_srvs/srv/Trigger '{}'
   ```

3. Проверить ответ сервиса и только после этого повторно запускать prepare или
   arm-only live-тест. Если поза снова изменится, повторить калибровку; обходить
   эту проверку или увеличивать timeout watchdog нельзя.

Чтобы изменить порог для стендовой настройки, задайте параметр
`body_proxy.neutral_pose_max_error_m` в `config/r1_control.yaml`. Значение
проверяется при старте и ограничено диапазоном `(0, 0.50]` метров.

## Dry-run и диагностика

Обычный launch стартует с `dry_run:=true`: IK рассчитывается, но в
controller topics ничего не отправляется. `telepresence_sim.launch.py` явно
отключает dry-run только для локально изолированного Gazebo и передаёт
`simulation_mode:=true`; вне этого launch не-dry-run запуск отвергается.

Промежуточные позы и команды:

```text
/r1_kinematics_control/debug/head_world
/r1_kinematics_control/debug/{left,right}_hand_world
/r1_kinematics_control/debug/{left,right}_hand_local
/r1_kinematics_control/debug/{left,right}_hand_body
/r1_kinematics_control/debug/{left,right}_target
/r1_kinematics_control/debug/arm_trajectory
```

`*_hand_local` — точный результат `inverse(head_pose) * hand_world`. В логе
печатаются world/local координаты и ошибки симметрии.

## Зависимости

Дополнительный `pip install ikpy` не нужен. Solver использует только NumPy:

```bash
sudo apt install ros-humble-ros2-control ros-humble-gazebo-ros2-control \
  ros-humble-joint-trajectory-controller python3-numpy
```

## Настройка URDF и суставов

1. Посмотреть доступные links/joints и контроллеры:

   ```bash
   check_urdf /absolute/path/to/r1.urdf
   ros2 control list_hardware_interfaces
   ros2 control list_controllers
   ```

2. Отредактировать `config/r1_control.yaml`: `base_link`, `tip_link`, finger joint
   arrays и controller topics. `joint_names: ["AUTO"]` автоматически берёт все
   подвижные joints между base и tip. Явный список включает строгую проверку против
   URDF, поэтому ошибка в имени не дойдёт до контроллера.

3. Имена и порядок `arm_command_topic` должны совпадать с joints соответствующего
   `JointTrajectoryController`. Шаблон есть в `controllers.example.yaml`.

URDF можно передать параметром `robot_description`, файлом `urdf_path` либо
transient-local топиком `/robot_description` типа `std_msgs/String`.

## TF-калибровка

Позы моста имеют `frame_id=vr_tracking`. Перед IK требуется TF из tracking frame в
base frame руки. Для проверки в пустом мире допустим временный identity transform:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
  --frame-id torso_link --child-frame-id vr_tracking
```

Это только стендовая заглушка, не реальная антропометрическая калибровка.

## Сборка и запуск

```bash
cd /home/unitree/Unitree_Project/R1_Teleoperation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select vr_teleop_bridge r1_kinematics_control
source install/setup.bash

ros2 launch r1_kinematics_control r1_kinematics_control.launch.py \
  urdf_path:=/absolute/path/to/r1.urdf dry_run:=true
```

Нода не запускает Gazebo и controller spawners: их имена зависят от фактической
модели R1. До deadman=true она остаётся в safe state.

`/r1/sim/cmd_vel` здесь является только симуляционным выходом существующего locomotion
controller/plugin. Обычный `ros2_control` не синтезирует походку двуногого робота
из `Twist`. Если модель R1 не предоставляет gait/base controller с таким входом,
нужен её штатный walking controller либо отдельный MPC/whole-body control слой;
одна ретрансляция не должна напрямую формировать траектории суставов ног.

Этот DLS solver специально лёгкий и удобен для Gazebo-прототипа, но не проверяет
self-collision и столкновения рук с окружением. Перед переносом на физический R1
рекомендуется заменить слой IK на MoveIt 2/MoveIt Servo либо поставить MoveIt
collision validation между этим solver и ros2_control. Deadman и watchdog должны
остаться независимыми от MoveIt.

Проверка headset-relative и симметрии:

```bash
python3 -m pytest -q test/test_body_proxy.py
```

Тест подтверждает, что общее перемещение и yaw-поворот HMD не
меняют команды, а зеркальные позы дают симметричные robot targets.

## `/vr/joy` для пальцев

Поддерживаются аналоговые trigger axes и дискретные buttons. Аналоговый режим
предпочтителен. Обновлённый мост публикует axes `[LX, LY, RX, RY, LT, RT]` и buttons
`[deadman, LT_pressed, RT_pressed]`. Если axis index равен `-1`, используется button
index; бинарное нажатие всё равно превращается EMA-фильтром в плавное сгибание.
