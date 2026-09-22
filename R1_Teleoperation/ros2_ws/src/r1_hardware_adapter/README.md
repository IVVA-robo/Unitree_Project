# r1_hardware_adapter

Безопасная граница между ROS 2 и будущим физическим адаптером Unitree R1.
Пакет также содержит read-only монитор телеметрии R1.

Пакет пока намеренно работает только в `dry_run=true` и отказывается
запускаться при `hardware_enabled=true` или `dry_run=false`. Он принимает
траектории рук и безопасную скорость, проверяет Deadman, свежесть пакетов,
конечность чисел, ограничивает скорость и шаг суставов, а затем публикует
только диагностические сообщения под `/r1_hardware_adapter/debug/*`.

Ни один топик Unitree DDS (`rt/lowcmd`) и ни один locomotion RPC не создаётся.
Это позволяет тестировать safety boundary рядом с подключённым роботом, не
посылая ему команд.

## Запуск

```bash
source /opt/ros/humble/setup.bash
source /home/unitree/Unitree_Project/R1_Teleoperation/install/setup.bash
ros2 launch r1_hardware_adapter r1_hardware_adapter.launch.py
```

Проверка:

```bash
ros2 topic echo /r1_hardware_adapter/debug/status
ros2 topic echo /r1_hardware_adapter/debug/cmd_vel
```

Дальнейшая реализация физического транспорта требует отдельного ревью: точной
версии R1 SDK, списка моторных индексов, выбора `rt/lowcmd`/`sport`, проверки
FSM, E-stop и испытания на подвесе. До этого флаг `hardware_enabled` менять
нельзя.

## Read-only телеметрия физического R1

При подключённом Ethernet запускайте отдельным терминалом:

```bash
source /opt/ros/humble/setup.bash
source /home/unitree/Unitree_Project/R1_Teleoperation/install/setup.bash
ros2 launch r1_hardware_adapter r1_state_monitor.launch.py
```

Монитор подписывается на SDK-канал `rt/lf/lowstate` через интерфейс, указанный
в `config/r1_state_monitor.yaml` (по умолчанию `enxb4b024be59fe`). Он публикует
`/r1/hardware/joint_states` и `/r1/hardware/status`, но не создаёт ни одного
командного DDS-канала. `JointState` содержит позицию `q`, скорость `dq` и
оценку момента `tau_est` для всех 26 настроенных суставов; сообщение не
публикуется, если хотя бы один из этих массивов неполон или содержит
NaN/Infinity.

Строка статуса сохраняет прежние поля и дополнительно содержит максимальные
по модулю `dq` и `tau_est`, диапазон двух значений температуры каждого
настроенного мотора и компактный список `motorstate` в формате
`IDL_индекс:значение` (`none`, если все значения нулевые). Эти поля помогают
диагностике, но остаются сырыми данными SDK: `tau_est` является оценкой, два
температурных канала не декодируются, а `motorstate` выводится как непрозрачный
числовой код. Они не заменяют физический E-stop: готовность к движению
намеренно не объявляется.

## Полный аппаратный dry-run

После сборки и source основного и legacy workspace можно поднять всю цепочку
от живой телеметрии до двух независимых диагностических выходов:

```bash
ros2 launch r1_hardware_adapter r1_hardware_dry_run.launch.py
```

Launch берёт настоящий R1 URDF из пакета `unitree_r1_description`, подаёт
`/r1/hardware/joint_states` в IK и связывает только debug-выход IK с
dry-run safety gate. Параметры `dry_run=true` и `hardware_enabled=false`
заданы внутри launch без пользовательского переключателя. В этом режиме нет
publisher для `rt/arm_sdk`/`rt/lowcmd` и нет клиента сервиса `sport`. ROS 2
топики дочерних узлов дополнительно ограничены этим ноутбуком через
`ROS_LOCALHOST_ONLY=1`; raw SDK-подписчик читает LowState через явно заданный
Ethernet-интерфейс.

Проверять расчёт можно по:

```bash
ros2 topic echo /r1_kinematics_control/debug/arm_trajectory
ros2 topic echo /r1_hardware_adapter/debug/arm_trajectory
ros2 topic echo /r1_hardware_adapter/debug/status
```

VR-мост намеренно не входит в этот launch: его запускают отдельно только при
готовых очках. Если уже запущен одиночный `r1_state_monitor`, сначала штатно
остановите его, чтобы не создавать второй узел с тем же именем.
