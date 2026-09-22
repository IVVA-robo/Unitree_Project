# r1_sdk_transport

Отдельный C++-процесс для границы между ROS 2 и Unitree SDK2. Сейчас это
только read-only транспорт: он может читать `rt/lf/lowstate`, публиковать
проверенный `sensor_msgs/JointState` и прогонять цели рук через safety-gate в
debug-топик. В основном исходнике `r1_sdk_transport.cpp` намеренно нет
`ArmSdk`, `LocoClient`, `rt/arm_sdk`, `rt/lowcmd` и `sport`.

Отдельный executable `r1_arm_sdk_traffic_observer` является ограниченным по
времени read-only наблюдателем. Он создаёт только
typed receive channel `CreateRecvChannel<LowCmd_>` для `rt/arm_sdk`; основной
LowState transport по-прежнему не включает `LowCmd` вообще. Возвращённый
`ChannelPtr` сохраняется только для read-only доступа к реальному typed DDS
reader; publisher/RPC не создаются. Наблюдатель нужен потому, что
встроенный arm action service R1 может держать постоянный, но неактивный DDS
writer endpoint. До инициализации локального transport gate запускается с
`--expected-writers 1`; на границе ownership probe после инициализации нашего
известного ArmSdk writer — с `--expected-writers 2`. Topic/type metadata и
полный набор publication handles должны быть неизменны, число matched writers
должно точно совпадать с фазой, а последние `2 s` окна не должно быть DDS match
churn. До полного набора match разрешён только discovery grace. Сами endpoints
не считаются трафиком: любой реально полученный LowCmd sample всё равно
блокирует probe. Одновременно второй
read-only subscriber проверяет `rt/arm/action/state`: за шестисекундное окно
обязателен хотя бы один корректный JSON `{"holding":false,"id":0,"name":""}`.
Отсутствие состояния, malformed JSON, `holding=true` или не-idle `id/name`
закрывают gate. Последний action-state sample должен быть моложе `3.5 s`;
это допускает наблюдаемую частоту firmware около одного сообщения в `3 s`,
но не принимает старый retained sample в начале шестисекундного окна.

Это соответствует официальному ограничению R1: встроенный arm action service
сам использует `rt/arm_sdk`, его нельзя применять одновременно с внешним
ArmSdk writer (ошибка занятости `7400`, а перекрывающиеся потоки могут вызвать
неверное движение):
<https://support.unitree.com/home/en/R1_developer/arm_control_routine>.

```bash
make r1-arm-sdk-traffic-check
```

Коды завершения: `0` — ожидаемый для фазы набор из одного или двух typed writers
стабилен не менее `2 s`, action state idle и не получено ни одного sample;
`10` — свежий активный поток;
`11` — samples были, но к концу окна поток уже не свежий/недостаточно
непрерывный; `12/13/14` — action-state отсутствует, malformed или активен;
`15` — typed writer отсутствует, число endpoint неожиданное, handle сменился
или был match churn; `20/21/64` — observer недоступен, runtime/ABI или usage
error.
Любой код кроме `0` блокирует commissioning writer. В отчёте выводятся частота,
диапазон `mode_pr`, вычисленный по формуле R1 `weight=mode_pr/100`, а также
последнее значение и диапазон `q` головы в слотах `29` (pitch) и `30` (yaw).
Vendor CycloneDDS `0.10.2` не реализует ISO C++
`matched_publications()`, поэтому observer берёт C entity именно из созданного
typed C++ reader и вызывает read-only C API
`dds_get_subscription_matched_status`, `dds_get_matched_publications` и
`dds_get_matched_publication_data`. Это не создаёт дополнительных DDS entities
и позволяет доказать topic/type, точное число writers, постоянство handle и
отсутствие churn. Код `0` остаётся только одним из обязательных условий
физического probe; остальные interlock-проверки всё равно обязательны.

## Безопасный запуск без робота

```bash
cd /home/unitree/Unitree_Project/R1_Teleoperation
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch r1_sdk_transport r1_sdk_transport.launch.py sdk_enabled:=false
```

При таком запуске SDK не инициализируется, а узел только показывает в статусе,
что LowState отключён:

```bash
ros2 topic echo /r1/sdk_transport/status
```

Launch-файл принудительно выбирает `rmw_fastrtps_cpp` и ставит каталог
CycloneDDS из скомпилированного Unitree SDK первым в `LD_LIBRARY_PATH`.
Исполняемый файл не доверяет одному окружению: до `rclcpp::init` он сверяет
канонические пути загруженных `libddsc.so.0` и `libddscxx.so.0` и фактически
выбранный RMW, а после инициализации ROS повторяет проверку RMW. Любое
расхождение завершает процесс до `ChannelFactory::Init`.

Для прямого ручного запуска без launch-файла окружение нужно задать явно:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1
R1_VENDOR_DDS_DIR=/home/unitree/Unitree_Project/Legacy_Robotics/robotics/unitree_sdk/unitree_sdk2/thirdparty/lib/x86_64
export LD_LIBRARY_PATH="${R1_VENDOR_DDS_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
ros2 run r1_sdk_transport r1_sdk_transport --ros-args -p sdk_enabled:=false
```

Офлайн-проверка только ABI guard, без запуска ROS, SDK и сети:

```bash
ros2 run r1_sdk_transport r1_sdk_transport --abi-check-only
```

## Read-only проверка подключённого R1

Перед этой командой робот должен быть включён, а Ethernet-интерфейс уже
подключён. Команда только создаёт DDS reader и не может включить моторы:

```bash
ros2 launch r1_sdk_transport r1_sdk_transport.launch.py \
  sdk_enabled:=true network_interface:=enxb4b024be59fe
```

Проверка:

```bash
ros2 topic echo /r1/sdk/joint_states
ros2 topic echo /r1/sdk_transport/motors_healthy
ros2 topic echo /r1/sdk_transport/status
```

`/r1/sdk_transport/motors_healthy` публикуется непрерывно и равен `true`
только пока LowState свежий, все 26 настроенных `q/dq` конечны и поле
`motorstate` равно нулю в каждом физическом IDL slot. При любом ненулевом
коде Boolean становится `false`, а status содержит
`motorstate_nonzero=N motorstate=slot:code,...`. При отсутствии/устаревании
LowState публикуется `false` и `motorstate=unavailable`; прошлое значение
`true` поэтому нельзя использовать как долговременное разрешение.

Требования safety-gate:

* свежий `/vr/teleop/active == true`;
* свежий и конечный LowState для всех 10 суставов рук;
* имя каждого сустава должно быть в конфигурации без дубликатов;
* положение ограничивается `arm_joint_min_rad`/`arm_joint_max_rad`;
* шаг ограничивается одновременно `max_joint_step_rad` и
  `max_joint_velocity_rad_s * dt`;
* таймер watchdog сбрасывает arm-gate при устаревшем deadman или цели.

Первый принятый target получает seed из измеренных `q` LowState. Это устраняет
скачок при захвате управления. Результат публикуется только в
`/r1/sdk_transport/debug/arm_trajectory`.

## Важное ограничение

Параметры `commissioning_interlock`, `arm_writer_enabled` и
`locomotion_writer_enabled` намеренно приводят к аварийному завершению, если
их включить. Для реального writer-процесса необходим отдельный review,
физический E-stop и подтверждение оператора; этот пакет не является таким
writer-ом. Локомоция не реализована и не может быть включена изменением YAML.

Физический LowState содержит 26 суставов (IDL slots `0..13`, `15..19`,
`22..26`, `29..30`), тогда как текущий simulation URDF описывает 24. В YAML
явно сохранены `head_pitch_joint` и `head_yaw_joint`; до утверждения URDF-map
они только публикуются в feedback и не попадают в arm-команды.
