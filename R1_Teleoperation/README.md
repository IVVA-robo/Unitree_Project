# Pico 4 Ultra → ROS 2 → Unitree R1 teleoperation

Безопасный прототип телеуправления Unitree R1. Unity передаёт состояние XR по UDP,
ROS 2-мост валидирует команды, а отдельный пакет вычисляет IK и управляет только
контроллерами симулятора. Исходный код изолированного `r1_live_writer` уже
реализован. Первый staged `StandUp` подтвердил стабильный `FSM=4`. Обычный
head-tracking затем был безопасно заблокирован до `ArmSdk`, когда LowState
показал `head_pitch=0.0073 rad`, `head_yaw=1.4299 rad` (численно `81.9°` в
координате сустава). В отдельной поздней, явно разрешённой recenter-сессии
свежий seed был около `yaw=1.4606`, `pitch=0.0084`; локальный writer пометил
ArmSdk как claimed и поставил кадры в асинхронную очередь, но это не доказывает
приём firmware, а физическая обратная связь не последовала даже за малым
target. Writer сработал по `head_recenter_following_timeout`, удержал
текущую feedback-позу, плавно освободил ownership до `weight=0` и защёлкнул
kill. После этого пользователь сообщил о красном индикаторе на груди. Все
live-процессы остановлены. Повторять recenter или обычный head tracking нельзя
до расшифровки индикатора и отдельного минимального ownership-handshake.
После освобождения рабочей зоны головы и следующего штатного включения робот
сам, без локального writer и без VR-команд, повторяемо повернул голову вбок.
Read-only LowState для этой видимой загрузочной позы дал почти неподвижный
`head_yaw=2.0063 rad` (почти официальный положительный предел `2.0071 rad`) и
`head_pitch=-0.0037 rad`. Официальная инструкция Unitree для запуска R1 на
подвесе описывает этот левый поворот как переход головы в калибровочную позу.
Это подтверждает загрузочное поведение firmware, но
не определяет безопасный физический центр: отправлять `q=0` по этому наблюдению
нельзя. При каждом включении карабины, провода и другие препятствия должны быть
полностью убраны из рабочей зоны головы.
Запуск Unity/APK/ROS остаётся полностью пассивным: будущая центровка будет
отдельным подтверждаемым действием с Deadman и обратной связью, а не
автоматическим движением при открытии приложения.
Штатный режим проекта остаётся
`dry-run` / simulation / mock
без команд моторам. Live-wrapper'ы fail-closed и не стартуют без полного набора
явных acknowledgements; их нельзя использовать, пока робот на зарядке.
По официальной инструкции R1 заряжается только выключенная и вынутая из робота
батарея штатным зарядным устройством; зарядка установленной батареи не входит в
поддерживаемый workflow проекта.

Канонический путь проекта на этом ноутбуке:
`/home/unitree/Unitree_Project/R1_Teleoperation`. Общая карта файлов находится в
`/home/unitree/Unitree_Project/README.md`.

Подробная архитектура, контракт топиков и порядок интеграции описаны в
[`docs/architecture.md`](docs/architecture.md).
Порядок физического commissioning и E-stop находится в
[`docs/r1_hardware_commissioning.md`](docs/r1_hardware_commissioning.md).
Инструкция по безопасной диагностике удалённого рабочего стола R1 находится в
[`docs/nomachine.md`](docs/nomachine.md).

## Состав

- `unity/VRUdpSender.cs` — Unity/OpenXR-отправитель для Pico 4 Ultra;
- `ros2_ws/src/vr_teleop_bridge` — пакет ROS 2 Humble;
- `ros2_ws/src/r1_kinematics_control` — EMA, URDF IK, пальцы и deadman-gate Gazebo;
- `ros2_ws/src/r1_hardware_adapter` — dry-run safety boundary для будущего
  физического Unitree DDS-адаптера (hardware output отключён);
- `ros2_ws/src/r1_sdk_transport` — отдельный C++/SDK read-only транспорт:
  LowState, seed/limits/watchdog и debug arm-gate без DDS/API writer;
- `ros2_ws/src/r1_live_writer` — изолированный fail-closed writer для R1:
  по умолчанию mock, с отключёнными командами головы, locomotion и `prepare`;
  SDK-путь не является разрешением на физический запуск;
- `ros2_ws/src/r1_teleop_safety` — fail-closed software kill switch,
  read-only preflight и dry-run план перевода робота в штатный режим;
- `ros2_ws/src/r1_telepresence_sim` — Gazebo-сценарий, безопасный адаптер `/r1/sim/cmd_vel`,
  simulation-only визуализатор ног через `leg_trajectory_controller` и desktop
  stereo viewer;
- `ros2_ws/src/r1_robot_pov` — отдельный offline-first video-only WebRTC/MJPEG
  viewer для браузера и Pico;
- `ros2_ws/src/r1_teleoperation_app` — единая simulation-only точка запуска всех
  перечисленных компонентов;
- `vr_test_sender` — устанавливаемый генератор UDP-пакетов без шлема.

## Robot POV без робота

```bash
make robot-pov-deps
make robot-pov-cache-deps
make robot-pov
```

Viewer открывается по напечатанному LAN URL; облако и интернет не требуются.
Выставочный режим, QR, профили плохого Wi-Fi и preflight описаны в
[`docs/robot_pov.md`](docs/robot_pov.md). Этот процесс строго video-only и не
запускает управление роботом.

Быстрые команды immersive POV:

```bash
make pov-demo             # локальный движущийся mock
make pov-vr               # физический R1, video-only
make pov-stereo-demo      # mock с true stereo SBS
make pov-low-latency      # R1: 30 FPS / минимальная задержка
make pov-high-quality     # R1: повышенное разрешение и bitrate
make pov-exhibition       # ROS/выставочный .env
make pov-preflight        # read-only проверка камеры и сервера
```

В браузерном viewer кнопка `VR` включает immersive WebXR, `На весь экран`
убирает элементы браузера, а `Оверлей` скрывает диагностические карточки.
Поддерживаются `Mono`, `Stereo SBS` и `Stereo top-bottom`; один источник R1
автоматически дублируется на оба глаза как mono fallback.

Для подключённого физического R1 используется отдельный безопасный профиль:

```bash
./scripts/r1-camera-preflight
./scripts/robot-pov r1
```

Он читает только фронтальную камеру через подтверждённый SDK-сервис
`videohub`; teleop, IK, locomotion и actuator-клиенты не импортируются и не
запускаются. На проверенном R1 получен поток `1280x720` примерно `14 FPS`,
который viewer публикует как mono WebRTC/MJPEG. Проверка работающего сервера:

```bash
./scripts/robot-pov r1-preflight --live
```

## Русская панель оператора

Для запуска проверок и режимов без ручного ввода команд используйте desktop-панель:

```bash
make operator-panel
```

Ярлык **Unitree R1 Панель оператора** также находится на рабочем столе. Вкладка
«Сеть и подключения» запускает read-only preflight, «Видео глазами робота»
управляет Robot POV, а вкладка «Руки / ноги / teleop» содержит dry-run и
защищённые live-кнопки. Вывод всех процессов виден в «Логах». STOP и KILL
доступны сверху постоянно.

Параметры сохраняются в
[`config/operator_panel.json`](config/operator_panel.json). По умолчанию включён
`dry_run`, а live-запросы выключены. Подробности находятся в
[`docs/operator_panel.md`](docs/operator_panel.md).

## Быстрый запуск

```bash
cd /home/unitree/Unitree_Project/R1_Teleoperation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select vr_teleop_bridge r1_kinematics_control
source install/setup.bash
ros2 launch vr_teleop_bridge vr_bridge.launch.py
```

Во втором терминале можно проверить мост без робота и без Pico:

```bash
ros2 run vr_teleop_bridge vr_test_sender --host 192.168.8.131 --deadman
```

Затем проверить ROS 2:

```bash
ros2 topic echo /vr/teleop/active
ros2 topic echo /vr/cmd_vel
ros2 topic hz /vr/right_controller/pose
```

Полный simulation-only сценарий (физический R1 не подключается):

```bash
cd /home/unitree/Unitree_Project/R1_Teleoperation
source /opt/ros/humble/setup.bash
source /home/unitree/Unitree_Project/Legacy_Robotics/robotics/ros2_ws/install/setup.bash
colcon build --symlink-install \
  --packages-select vr_teleop_bridge r1_kinematics_control r1_telepresence_sim
source install/setup.bash
ros2 launch r1_telepresence_sim telepresence_sim.launch.py \
  display:=false hardware_enabled:=false model_z:=0.25
```

Для симуляции калибровка сохраняется отдельно в
`~/.ros/r1_telepresence_sim_body_calibration.json`; файл физического R1 при этом
не изменяется.

То же можно запустить короткой безопасной командой из корня проекта:

```bash
./scripts/r1-sim
```

### Единое приложение телеуправления

После сборки доступен один запуск, который поднимает VR-мост, Gazebo с IK и
визуализацией ног, а также video-only сервер:

```bash
cd /home/unitree/Unitree_Project/R1_Teleoperation
make r1-teleoperation-build
./scripts/r1-teleoperation
```

Откройте видео по `http://192.168.8.9:8080/` (или адресу ноутбука в вашей
сети). По умолчанию сервер получает стереокадры из Gazebo ROS topics,
`ROS_LOCALHOST_ONLY=1`, headset-relative режим и `hardware_enabled=false`. Для headless-запуска можно
указать `R1_TELEOP_DISPLAY=false`; для остановки всех компонентов нажмите
`Ctrl-C` в окне запуска. Приложение Pico при этом должно оставаться открытым.

### Автоопределение адреса ноутбука в Pico

`VRUdpSender` в объединённом APK автоматически ищет ROS UDP bridge в локальной
сети. Pico отправляет широковещательный probe `R1_TELEOP_DISCOVER v1` на порт
`9091`, bridge отвечает своим LAN-адресом и портом команд `9090`, после чего
поток XR идёт непосредственно на найденный адрес. Интернет и DNS для этого не
нужны. Если точка доступа блокирует broadcast, APK через 5 секунд использует
сохранённый при сборке адрес как fallback и продолжает переподключение.

Порт discovery задаётся в
`ros2_ws/src/vr_teleop_bridge/config/vr_bridge.yaml`. Обычная сборка сама берёт
текущий IPv4 интерфейса `wlp4s0` только как резервный адрес для видео и
fallback-команд:

```bash
UNITREE_LAPTOP_HOST=192.168.8.9 ./scripts/build-unity-telepresence build
```

После установки APK смена адреса ноутбука в той же локальной сети не требует
пересборки, если broadcast между Pico и ноутбуком разрешён.

Для полного запуска без интернета используйте
[`docs/r1_offline_lan.md`](docs/r1_offline_lan.md) и
`./scripts/r1-offline-session r1`. Этот режим поднимает bridge и video-only
источник R1, ждёт появления Ethernet/робота и публикует локальное имя ноутбука
через Avahi; командный hardware
путь при этом не запускается.

Параметры видео и Gazebo меняются без правки кода переменными
`R1_TELEOP_VIDEO_SOURCE`, `R1_TELEOP_VIDEO_ENV_FILE`, `R1_TELEOP_VIDEO_PORT`,
`R1_TELEOP_UDP_PORT`, `R1_TELEOP_GAZEBO_PORT` и `R1_TELEOP_DOMAIN_ID`. Wrapper
проверяет занятые порты перед запуском и не стартует поверх уже работающего
моста, POV-сервера или Gazebo. Live-источник камеры и любой hardware-путь не
включаются автоматически и требуют отдельной проверки.

Read-only диагностика всех компонентов:

```bash
./scripts/r1-teleoperation-status
```

Для автоматической проверки запуска без ручного управления используйте:

```bash
./scripts/r1-sim-smoke
```

Smoke-тест изолирует DDS через `ROS_LOCALHOST_ONLY=1`, проверяет URDF, узел IK,
ключевые топики и нулевую скорость при отпущенном deadman, затем останавливает
весь launch-процесс. Лог сохраняется в `logs/sim/`.

Скрипт принудительно использует `ROS_LOCALHOST_ONLY=1` и
`hardware_enabled=false`. Параметры можно изменить переменными
`R1_SIM_DOMAIN_ID`, `R1_SIM_GAZEBO_PORT`, `R1_SIM_DISPLAY`, `R1_SIM_MODEL_Z` и
`R1_SIM_CALIBRATION_FILE`. Значение `R1_SIM_MODEL_Z=0.25` поднимает
фиксированную модель над полом, чтобы движение ног было заметнее.
Wrapper автоматически подключает установленный legacy-workspace с пакетом
`unitree_r1_description` (`../Legacy_Robotics/robotics/ros2_ws/install/setup.bash`).
Если описание находится в другом месте, укажите его через
`R1_LEGACY_WS_SETUP=/absolute/path/to/setup.bash`.

Оба simulation launch-файла по умолчанию устанавливают
`ROS_LOCALHOST_ONLY=1`. Это не мешает UDP-пакетам Pico приходить по Wi-Fi, но
не даёт симуляционным ROS 2-командам обнаруживаться на Ethernet физического
робота. Отключать изоляцию (`ros_localhost_only:=0`) можно только в отдельной,
намеренно изолированной ROS-сети без реального R1.

Инструкция IK/Gazebo находится в
[`ros2_ws/src/r1_kinematics_control/README.md`](ros2_ws/src/r1_kinematics_control/README.md).
Описание камер и simulation-only ограничений — в
[`ros2_ws/src/r1_telepresence_sim/README.md`](ros2_ws/src/r1_telepresence_sim/README.md).
Результаты read-only проверки физического R1 и найденный SDK-контракт записаны
в [`docs/r1_hardware_integration.md`](docs/r1_hardware_integration.md).
До завершения симуляционных тестов не подключайте `/vr/cmd_vel` или траектории к
locomotion/low-level API реального робота.

Каркас физического адаптера можно проверить рядом с симуляцией, но он всегда
остаётся dry-run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch r1_hardware_adapter r1_hardware_adapter.launch.py
```

Он не создаёт `rt/lowcmd` и не вызывает `sport`; смена
`hardware_enabled`/`dry_run` на небезопасные значения намеренно завершается
ошибкой.

Для чтения состояния подключённого R1 без команд:

```bash
ros2 launch r1_hardware_adapter r1_state_monitor.launch.py
```

Эквивалентный отдельный C++-транспорт (тоже только чтение) можно собрать и
запустить так:

```bash
make r1-sdk-build
make r1-sdk-readonly
```

То же можно выполнить напрямую: `./scripts/r1-sdk-readonly` (интерфейс
меняется через `R1_SDK_NETWORK_INTERFACE`).

По умолчанию launch оставляет `sdk_enabled=false`, поэтому он безопасен даже
без робота. Для подключённого Ethernet-интерфейса `r1-sdk-readonly` включает
только reader `rt/lf/lowstate` и публикует `/r1/sdk/joint_states`; командные
каналы `rt/arm_sdk`, `rt/lowcmd`, `sport` отсутствуют. Попытка включить
`hardware_enabled`, commissioning interlock или любой writer-флаг завершается
ошибкой. Это сохраняет read-only границу даже при наличии отдельного пакета
`r1_live_writer`: он по умолчанию запускается только с `transport=mock` и не
использует этот reader для физических команд.

Перед физическим commissioning всегда сначала запускайте read-only preflight:

```bash
make r1-sdk-preflight
```

Скрипт проверяет изолированный Ethernet-маршрут, full-duplex и отсутствие
новых RX-ошибок за контрольную секунду, ping, свежую конечную
телеметрию со всеми 26 суставами, свежий `motors_healthy=true` и независимое
status-подтверждение `motorstate_nonzero=0`, fail-closed safety-параметры и отсутствие
известных writer-процессов/топиков. Он может временно запустить только
read-only subscriber и не отправляет команды приводам, locomotion, sport или
trajectory.

Когда нужен полный расчёт на живой телеметрии, но без управления физическим
роботом, используется аппаратный dry-run:

```bash
source /home/unitree/Unitree_Project/Legacy_Robotics/robotics/ros2_ws/install/setup.bash
source install/setup.bash
ros2 launch r1_hardware_adapter r1_hardware_dry_run.launch.py
```

Он жёстко оставляет `dry_run=true`, использует R1 URDF и перенаправляет IK и
локомоцию только через два диагностических слоя. Командных Unitree DDS/API в
этом launch нет, а ROS 2 debug-трафик остаётся на ноутбуке благодаря
`ROS_LOCALHOST_ONLY=1`.

## Head tracking и locomotion: единый dry-run

Новый независимый от Gazebo запуск проверяет путь «очки → голова / стики →
диагностика» без физического робота. Он принудительно включает
`ROBOT_DRY_RUN=1`, локальный DDS и не создаёт `ArmSdk`, `LocoClient`,
`rt/arm_sdk`, `rt/lowcmd`, `sport` или реальный `/cmd_vel`.

```bash
cd /home/unitree/Unitree_Project/R1_Teleoperation
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
cd ..
make teleop-dry-run
```

По умолчанию запускаются safety supervisor, VR UDP-мост, контроллер головы и
контроллер locomotion. Read-only LowState/IK для подключённого R1 не входят в
этот запуск; они включаются только явным
`R1_DRY_RUN_START_ARM_PIPELINE=true` и по-прежнему не дают команд приводам.
Если уже запущен старый VR-мост на `9090`, остановите его либо используйте
отдельный UDP-порт, например:

```bash
R1_DRY_RUN_DOMAIN_ID=91 R1_DRY_RUN_UDP_PORT=19090 make teleop-dry-run
```

Для dry-run допускается пустой source IP: мост заблокирует первый корректный
UDP-источник на время процесса. Для физического commissioning должен быть
зафиксирован IP Pico, например
`R1_DRY_RUN_VR_ALLOWED_SOURCE_IP=192.168.8.120`; без
подтверждённого источника live transport не должен запускаться.

Перед диагностическим движением нужно явным образом снять программный kill
switch. Это разрешает только debug-топики, а не робота:

```bash
ros2 service call /r1/safety/set_kill std_srvs/srv/SetBool '{data: false}'
```

Когда приходит свежая HMD-поза, посмотрите прямо и сохраните нейтраль головы:

```bash
ros2 service call /r1/head/calibrate_neutral std_srvs/srv/Trigger '{}'
```

Параметры головы — scale, inversion, deadzone, EMA, speed limit и угловые
лимиты — находятся в
[`r1_head_dry_run.yaml`](ros2_ws/src/r1_hardware_adapter/config/r1_head_dry_run.yaml).
По умолчанию R1 использует только yaw и pitch: roll отключён, так как для него
нет подтверждённого физического сустава. Диагностика публикуется в
`/r1_hardware_adapter/debug/head/{vr_euler,relative_euler,command_euler,joint_trajectory,status}`.

Для ног доступны профили `slow-safe`, `normal`, `exhibition`; первый включён по
умолчанию. Они меняют только значения
`/r1/locomotion_dry_run/debug/*`: input остаётся `/vr/cmd_vel`, Deadman —
`/vr/teleop/active`. Скорости clamp-ятся, плавно разгоняются/тормозятся, а
отсутствие команды более `0.25 s`, устаревший Deadman, NaN или kill возвращают
debug target к нулю. Быстрые отдельные запуски:

```bash
make head-dry-run
make locomotion-dry-run
make teleop-dry-run-smoke
```

`make teleop-dry-run-smoke` сам выбирает свободные localhost-only ROS domain и
UDP-порт, отправляет mock HMD/stick packets, проверяет startup kill, калибровку,
Deadman и packet watchdog, затем останавливает только созданные им процессы.
Он не требует робота, очки или Ethernet.

Аварийно вернуть оба debug-контура в ноль можно в любой момент:

```bash
ros2 service call /r1/safety/emergency_stop std_srvs/srv/Trigger '{}'
```

Проверить состояние без публикации команд:

```bash
ros2 topic echo /r1_hardware_adapter/debug/head/status
ros2 topic echo /r1/locomotion_dry_run/debug/status
make robot-preflight
make head-live-dry-arm
```

`make robot-preflight` выполняет только пассивную проверку Ethernet, VR-потока,
Deadman, kill и переменных окружения. `make head-live-dry-arm` запускает полный
путь HMD → ограниченная команда головы → `r1_live_writer`, но принудительно
оставляет `transport=mock`, `send_commands=false` и не запускает robot reader.
Локальная задержка ROS между мостом и preflight показывается отдельно от
end-to-end latency: Unity timestamp пока не передаётся в ROS-контракте,
поэтому E2E latency честно помечается как недоступная.

Физический ROS-граф принудительно работает через `rmw_fastrtps_cpp`, а
Unitree SDK — через согласованную vendor-пару CycloneDDS. Это обязательно:
смешивание ROS `libddsc.so.0` и vendor `libddscxx.so.0` вызывает аварийное
завершение внутри `ChannelFactory`. Перед каждым physical preflight запускается
bounded-проверка инициализации/завершения SDK в отдельном network namespace,
где существует только loopback. Её можно безопасно проверить отдельно:

```bash
make r1-sdk-abi-probe
```

Probe создаёт ROS node, `ChannelFactory`, `LocoClient` и `ArmSdk` только на
`lo`; он не видит Ethernet робота и не вызывает `StandUp`, RPC движения или
публикацию команды. Неверный RMW или путь любой DDS-библиотеки блокируется до
инициализации Unitree transport.

`make robot-prepare` не является диагностической командой: он доступен только
когда отдельный live-wrapper уже запущен, повторно проверяет Ethernet,
LowState, VR, Deadman и параметры writer, затем снимает software kill, сбрасывает
локальную защёлку и вызывает официальный `/r1/live_writer/prepare` (`StandUp`;
в locomotion-сессии затем `Start` для перехода в FSM `811`). Без этого второго
шага R1 возвращает `SetVelocity` error `127`, даже если `StandUp` успешен.
В head/arms-only сессии prepare останавливается на стабильном `FSM=4` и не
переводит ноги в sport mode. После подтверждённых состояний writer всё равно требует отпустить Deadman,
при включённой locomotion увидеть нулевую обработанную команду и снова зажать
Deadman; head-only режим пропускает проверку скорости. Старый ненулевой стик не
может автоматически начать движение после подъёма.

`make robot-live-arm-check` и
`make teleop-live-preflight` — read-only проверки acknowledgement/link/LowState:
они не создают writer и не посылают команду. Для них всё равно нужны все
live-подтверждения, поэтому их не следует выполнять, пока робот на зарядке.

`make r1-live-writer-live`, `make head-live-test`, `make arms-live`,
`make legs-live`, `make locomotion-live-test` и `make teleop-live` передают управление в
fail-closed wrapper. Без полного набора interlock-переменных, валидного fixed
VR IP, Ethernet link, успешного read-only preflight и свободного UDP-порта он
завершается с `BLOCKED` **до** запуска writer. При наличии всех подтверждений
это уже потенциально физический сеанс; его нельзя запускать как обычный тест.
В head-only сессии до `make robot-prepare` нужно при отпущенном Deadman смотреть
прямо и вызвать `/r1/head/calibrate_neutral`; prepare-скрипт блокируется, если
статус головы не содержит `calibrated=True`.

В live-графе `make arms-live` запускает существующий headset-relative IK с R1 URDF
и подаёт его `/r1_kinematics_control/debug/arm_trajectory` в writer. Writer
принимает все десять arm joints, сливает их со свежим 13-joint feedback seed и
отправляет через официальный `ArmSdk`; добавлены независимые arm freshness,
deadman/watchdog и второй slew limit (`max_arm_joint_delta_rad=0.25`,
`max_arm_joint_rate_rad_s=1.0`). `make legs-live` оставляет ноги только на
официальном high-level `LocoClient` (`SetVelocity`/`StopMove`), а правый стик Y
по-прежнему игнорируется в `vr_teleop_bridge`.

`make teleop-live` объединяет эти arm+legs потоки. Физическая голова запускается
отдельно через `make head-live-test` только после разрешения её commissioning
проблемы; она не является скрытой частью arm+legs режима.

Для адаптера TP-Link UE200 проект по умолчанию блокирует live при отчёте
`cdc_ether`/half-duplex. Если это именно стабильный TP-Link CDC-лиnk без новых
RX-ошибок, оператор может явно разрешить узкий обход для текущего сеанса:

```bash
export R1_SDK_ALLOW_HALF_DUPLEX=1
make robot-readonly-preflight
```

Это не отключает проверки ping, свежей 26-joint телеметрии, `motorstate=0`,
`motors_healthy=true`, kill/deadman и watchdog; при любом новом RX-ошибочном
кадре preflight завершается ошибкой. Без этой переменной half-duplex остаётся
жёстким блокером. Проверка/переход драйвера также доступны через
`sudo ./scripts/r1-ue200-driver-test --check` и `--apply`, но только при
остановленных writer-процессах и физически отсоединённом RJ45.

Безопасная последовательность доступна уже сейчас и не посылает команд R1:

```bash
make r1-live-writer-build
make r1-live-writer-check
make r1-sdk-abi-probe
make r1-live-writer-mock
make r1-live-writer-mock-smoke
make r1-head-ownership-probe-mock-smoke
make robot-readonly-preflight
make robot-preflight
```

Сборку, тесты, ABI-probe, полный MockTransport smoke, VR dry-run и Gazebo
smoke без обращения к Ethernet робота можно выполнить одной командой:

```bash
make r1-offline-commissioning-check
```

Она целиком перезапускает себя в проверенном network namespace, где существует
только loopback-интерфейс, принудительно закрывает actuation-переменные и
блокируется, если уже запущен SDK/robot-facing процесс (включая vendor examples
и sport client). Перед тестами она также принудительно переконфигурирует оба
SDK-linked пакета с одним `R1_UNITREE_SDK_ROOT` и проверяет CMake cache,
установленный launch, RUNPATH и runtime ABI guard. Успешный результат остаётся только
программной проверкой и не разрешает физическое движение.

`make r1-live-writer-mock` принудительно выбирает mock-транспорт, закрывает
все флаги actuation и публикует лишь диагностические сообщения.
`make r1-live-writer-mock-smoke` изолирует ROS domain и loopback UDP, проверяет
prepare/Deadman/kill/watchdog пути только через `MockTransport`.
`make r1-head-ownership-probe-mock-smoke` в отдельном loopback-only namespace
проверяет безопасный `weight=0` discovery, плавный захват ownership, единственный
yaw-шаг не более `0.005 rad`, подтверждение направления, возврат точно в seed,
release до `weight=0`, disarm/kill и аварийный останов при измеренной скорости
головы выше probe-specific предела `0.04 rad/s`. Старое имя
`make r1-head-recenter-mock-smoke` сохранено
только как офлайн-совместимый alias этого же probe-теста; физический full
recenter через старые команды безусловно заблокирован.
Команды
`make robot-stop` и `make robot-kill` — не часть офлайн-последовательности: они
сначала пытаются вызвать stop/kill у **уже существующего** writer (это может
отправить физические `StopMove`/hold/release), затем защёлкивают центральный
программный kill. Они не запускают writer и не заменяют физический E-stop.

Перед первым физическим commissioning нужны: робот **снят с зарядки**,
свободная зона, физический E-stop под рукой, spotter и отдельное явное
подтверждение. SDK-путь требует одновременно все следующие interlock-переменные:

```bash
export ROBOT_DRY_RUN=0
export ROBOT_ENABLE_ACTUATION=1
export ROBOT_CONFIRM_OFF_CHARGER=1
export ROBOT_CONFIRM_CLEAR_AREA=1
export ROBOT_CONFIRM_ESTOP_READY=1
export ROBOT_CONFIRM_COMMISSIONING=1
export ROBOT_COMMISSIONING_TOKEN='<unique token, at least 16 characters>'
export ROBOT_VR_SOURCE_IP='<fixed Pico IPv4 address>'
```

Эти флаги сами по себе не включают управление и не заменяют физические меры
безопасности. Помимо них writer проверяет launch-параметры, совпадение токена и
фиксированного IP, профиль `slow-safe`, свежие `LowState`, команду, Deadman,
свежий `motors_healthy=true`, clear kill и успешный `prepare`. Live domain по умолчанию единый:
`R1_LIVE_ROS_DOMAIN_ID=88` (меняется только явным значением этой переменной).

Отдельный ownership micro-probe головы дополнительно требует
`ROBOT_CONFIRM_HEAD_OWNERSHIP_PROBE=1`; это подтверждение не входит в обычную
live-сессию и должно задаваться только для отдельно разрешённого теста. Старый
`ROBOT_CONFIRM_HEAD_RECENTER` не принимается. Physical full recenter
заблокирован, а успешный probe только возвращается к исходному seed и не
центрирует голову.

Настройки могут сделать dry-run более консервативным, но не снять встроенные
потолки: yaw/pitch/roll target не выше `0.80 rad`, скорость головы не выше
`2.0 rad/s`, а profiles locomotion не превышают значений `exhibition`.
`r1_live_writer` имеет отдельные source-backed пределы первого запуска;
физическая проверка этих пределов на R1 остаётся открытой задачей commissioning.

## Headset-relative режим и калибровка

Новый режим от первого лица включён по умолчанию
(`headset_relative_enabled:=true`). Обычный launch также безопасно запускается с
`dry_run:=true`: вычисления и debug-топики работают, но команды контроллерам не
публикуются.

После запуска отпустите deadman, встаньте прямо, смотрите вперёд и держите обе
руки в одинаковой нейтральной позе. Затем выполните:

```bash
ros2 service call /vr/calibrate_body std_srvs/srv/Trigger '{}'
```

Параметры сохраняются в
`~/Unitree_Project/Configs/Calibration/r1_body_calibration.json`. Проверить
зеркальную симметрию можно по полю `target_symmetry_xyz` в логе узла и debug-топикам
`/r1_kinematics_control/debug/{left,right}_target`. Нулевые значения
`target_symmetry_xyz` означают зеркально одинаковые декартовы цели.
