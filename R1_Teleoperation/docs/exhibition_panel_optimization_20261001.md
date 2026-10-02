# Оптимизация выставочной панели — 1 октября 2026

## Итог

Главные кнопки панели переведены на короткие асинхронные пути. Исправные
Ethernet, USB-VR, ROS 2, writer и Robot POV переиспользуются; повторный запуск
одинаковых процессов блокируется. Панель получает подтверждение режима прямо
из exhibition-manager и больше не ждёт очередной пятисекундный status poll.

После программной проверки проведена поэтапная физическая приёмка: подключение,
Robot POV, холодная СТОЙКА, холодный и тёплый RUN, два LOCK, восстановление USB
и краткая потеря Ethernet. Каждая физическая команда выполнялась только после
актуального подтверждения оператора. KILL, deadman, watchdog, envelope,
writer-count, traffic gates, подтверждение STOP и обязательные проверки
StandUp/FSM сохранены. IP-адреса, порты, `ROS_DOMAIN_ID=88`, Ethernet-схема и
USB-схема VR не менялись.

Бэкап до изменений находится вне рабочего дерева:

```text
/home/unitree/r1-panel-optimization-20261001-100023/active-code-config-before.tar.gz
SHA256 00c55bf517d7a8c6a407050b6721d3d69025426602a4ede8925417d87c63704f

/home/unitree/r1-panel-controller-status-before-20261001-111200/source-before.tar.gz
SHA256 5867b5174ecdc3babddc89065965fc0b1e349b20223bf3e1026785f118f1485d

/home/unitree/r1-panel-ethernet-recovery-before-20261001-113000/source-before.tar.gz
SHA256 df62395247d0f68313e4cd6bda7532c2a3df98e4fb64611f0953884b75ab2613

/home/unitree/r1-static-fastpath-before-20261001-DDTWTE/source-before.tar.gz
SHA256 ea52f28e2637b2e0a75a8fc55fe5fbcb51f03e8068643dc150aebc6c7d87acb1

/home/unitree/r1-static-prepare-client-before-20261001-YctgpI/source-before-client.tar.gz
SHA256 b0297d18d52f3d4b6ca8c76e4c561e55585cfa5da47395bbee51d2482b78e5ff

/home/unitree/r1-static-offline-reuse-before-20261001-kNzucI/source-before-offline-reuse.tar.gz
SHA256 3112af2324ec024cf45d02f32cfcb1b12b98ec57097cc5ddc2ca0c5417c47055

/home/unitree/r1-pov-respawn-before-20261001-DLXXci/source-before-pov-respawn.tar.gz
SHA256 9ef0f6efe35f98e6050fe9aef564705123d4f38639ff3a82562a0145a41b8ffe

/home/unitree/r1-pov-manager-grace-before-20261001-2YpNUj/source-before-manager-grace.tar.gz
SHA256 87dc88f19f7c03848745faf25cea4f9b83546ffec047c19387a0e3d158b8bc80

/home/unitree/r1-cold-warmup-before-20261001-1tQ6Yu/
history.bundle SHA256 68d1f078716e4c5a88195c4e48642a6a177b1dbca7d78b772c857c3831b37b5b

/home/unitree/r1-control-prepare-before-20261001-rugasB/
history.bundle SHA256 97101c2cdea68d56d041abc909c9e282f459a9c6ec7a8d8e317bc6575832d0af
working-tree.patch SHA256 318dbf853d5f65520cd0cda16cca39cfb04905a2fef5721e6d6bd94ae2c7d757

/home/unitree/r1-sdk-refresh-race-before-20261001-dmD5Ek/
scripts/r1-sdk-warmup SHA256 87d1d012e5634dbba81a3882c40875d936109c8fafb30e11358449226365a7bf
tests/test_sdk_preflight_attestation.py SHA256 4a16ceb085b5d60914feb2d07d1c4eee43082c33eee8d51ed0a2a3f6bebb21f2
tests/test_sdk_warmup_handoff.py SHA256 f8aabd5a6faddede678e63445466bbc6e6c5c027c5e0c7028d696d2f0fd31f97

/home/unitree/r1-lateral-axis-before-20261001-UAArIG/

/home/unitree/r1-panel-finalization-before-20261001-km4LVl/
working-tree.patch SHA256 cabaad5617511dccf5e862372e9d2dcba80ad4927c559ee64b79d180fd932ac6

/home/unitree/r1-cold-boot-handoff-before-20261002-W5jj6P/
working-tree.patch SHA256 d277c1ab99923493f3b70d77fbd16d864e9cadfd1ed3ac6bcc40b0aadbaec60e
```

Последние каталоги сохраняют состояние до исправления cold warmup, полный
dirty-tree patch до объединения control prepare observation, состояние до
устранения периодического окна без SDK-кэша, файлы до исправления боковой оси и
финальный dirty-tree patch перед публикацией. Они не являются образом PC2 или
полного диска робота.

## 1. Изменённые файлы

Код:

- `operator_panel/app.py` — быстрые обработчики, дедупликация, тайм-ауты,
  click-to-result timing, немедленные manager-события, удаление кнопок
  калибровки рук; после подтверждённого `OFFLINE → OK` асинхронно заменяет
  только read-only SDK warmup после его штатной очистки, уступая приоритет
  любой физической команде; при холодном нажатии больше не отменяет уже
  выполняющийся свежий SDK preflight, а ограниченно ждёт его результат и затем
  подтверждённую очистку reader;
- `operator_panel/commands.py` — сохранён отдельный commissioning-путь
  калибровки без кнопки на панели;
- `exhibition/orchestrator.py` — структурированные подтверждения состояния,
  выборочный reconnect, разделение startup/явной калибровки и одноразовая
  session-bound аттестация static readiness; reconnect ждёт process-local POV
  respawn перед полным service fallback;
- `exhibition/readiness.py`, `exhibition/readiness_attestation.py` — единый
  typed graph/parameter snapshot и приватная HMAC-подписанная передача его
  результата в static prepare без сохранения commissioning token;
- `scripts/r1-exhibition-wait-ready`, `scripts/r1-robot-prepare` — устранение
  повторных DDS discovery, parameter dump и VR/debug observer в СТОЙКЕ при
  сохранённых KILL/writer/traffic/FSM проверках; полный control graph теперь
  также использует один пассивный observer вместо шести последовательных ROS
  CLI readers, а нестандартные commissioning-конфигурации сохраняют прежний
  fallback;
- `ros2_ws/src/r1_teleop_safety/r1_teleop_safety/preflight.py` — тем же одним
  read-only DDS participant одновременно проверяет VR, active/deadman, debug
  controllers, команды головы/рук/ног и пять стабильных физических samples
  головы; сервисы и SDK writer не создаёт;
- `exhibition/prepare_client.py`, `scripts/r1-robot-prepare-client` — один
  bounded ROS participant для финальных service call/status вместо отдельных
  CLI-процессов; последовательность release/0,3 s/reset/prepare не изменена;
- `scripts/r1-live-session`, `scripts/r1-exhibition-offline-handoff` — static
  не отклоняется из-за UDP `9090`, который сам не использует; совместимый
  systemd POV проверяется и переиспользуется, а reconnect восстанавливает
  только его;
- `scripts/r1-panel-status` — быстрый статус без ROS-запросов и без повторного
  ping одного адреса; ADB-статус контроллеров в СТОЙКЕ и корректное отображение
  `KILL` после потери Ethernet;
- `scripts/r1-exhibition-session-gate` и `scripts/r1-exhibition-ros-client` —
  read-only health-проверка существующего сеанса без service call, снятия KILL
  или возобновления движения;
- `scripts/r1-sdk-preflight` — корректный поиск `ros2` после загрузки ROS
  environment при запуске панели после перезагрузки ноутбука;
- `scripts/r1-sdk-warmup` — перекрывающийся read-only refresh: предыдущая
  исправная аттестация остаётся доступной до атомарной записи новой, но
  немедленно удаляется при потере Ethernet или неуспешном preflight;
- `ros2_ws/src/r1_robot_pov/launch/robot_pov.launch.py` — автоматический
  перезапуск только завершившегося video-only процесса с задержкой 2 секунды;
  исправные VR bridge и родительский launch сохраняются.
- `ros2_ws/src/r1_live_writer/src/transport.cpp` — знак боковой скорости
  преобразуется только на физической границе ROS → Unitree pilot: стандартный
  ROS `+Y` остаётся движением влево, а аппаратный `lx` получает требуемую для
  этого R1 инверсию.

Документация:

- `README.md`;
- `docs/operator_panel.md`;
- `docs/exhibition_mode.md`;
- `docs/exhibition_quick_start_ru.md`;
- `docs/БЫСТРЫЙ_ЗАПУСК.md`;
- этот отчёт.

Тесты:

- `tests/test_operator_panel.py`;
- `tests/test_operator_panel_service.py`;
- `tests/test_exhibition_orchestrator.py`;
- `tests/test_sdk_preflight_attestation.py`;
- `tests/test_sdk_warmup_handoff.py`;
- `tests/test_fast_static_prepare.py`;
- `tests/test_zero_torque.py`;
- `ros2_ws/src/r1_teleop_safety/test/test_r1_teleop_safety_preflight.py`;
- `ros2_ws/src/r1_sdk_transport/test/test_traffic_gate_entrypoints.py`;
- `ros2_ws/src/r1_robot_pov/test/test_launch_recovery.py`.
- `ros2_ws/src/r1_live_writer/test/test_safety.cpp` и
  `test_r1_live_writer_package_contract.py` — контракт обоих боковых
  направлений и физической инверсии.

## 2. Найденные и исправленные задержки

### НАЙТИ И ПОДКЛЮЧИТЬ

- Удалён автоматический полный `Проверить всё` при каждом нажатии.
- Сначала используется настроенный рабочий control endpoint `.161`.
- Один и тот же адрес больше не пингуется дважды.
- При ответе `.161` быстрый путь не ждёт недоступный PC2 `.164`.
- Быстрый snapshot не загружает ROS environment и не выполняет ни одного
  `ros2`-запроса.
- Уже подтверждённое соединение сразу показывает «ПОДКЛЮЧЕНО».
- После реального `OFFLINE → OK/READY` прежний read-only warmup завершается
  асинхронно и только после подтверждённой очистки запускается один свежий
  worker. Первый `OK`, активный manager/writer, Zero Torque, закрытие панели и
  ожидающий RUN/LOCK/СТОЙКА не вызывают фоновый restart.

### LOCK / RUN / СТОЙКА

- LOCK и повторный RUN используют существующий control graph.
- Повторный LOCK при уже готовом static-сеансе теперь корректный no-op, а не
  ошибочный вызов RUN helper.
- Повторная СТОЙКА при готовом static-сеансе ничего не перезапускает.
- Результат приходит строкой `EXHIBITION_STATE` непосредственно от manager.
- При переключении static/control STOP текущего владельца остаётся
  обязательным; новый режим запускается только после подтверждённой очистки.
- Static readiness теперь передаёт prepare одноразовый результат уже
  выполненной typed discovery и проверки параметров. Ранее второй discovery,
  полный parameter dump и общий двухсекундный VR/debug observer занимали
  основную часть `prepare` (`14,779 s` в логе 11:38); сам StandUp и пять
  стабильных FSM 4 тогда заняли только `0,592 s`.
- Аттестация живёт 10 секунд, имеет права `0600`, связана HMAC с token,
  boot/session/domain и удаляется при первом чтении. Token не пишется в файл и
  не попадает в argv/лог.
- После первого физического замера оставшиеся отдельные `ros2 service call` и
  `ros2 topic echo` объединены в один клиент. Он запускается только после
  успешного traffic gate и повторно требует все live acknowledgements;
  legacy-путь control/commissioning не менялся.
- После аппаратного подтверждения единого клиента найден ещё один независимый
  участок: остановка исправного offline-сеанса занимала `2,311 s`, после чего
  manager немедленно запускал второй экземпляр того же Robot POV. СТОЙКА
  теперь условно сохраняет точный совместимый systemd-сеанс и его `:8080`, а
  собственный POV использует только как fallback. RUN по-прежнему выполняет
  полный port handoff.
- Убрано ожидание завершения фонового SDK warmup в UI-потоке: cleanup и
  продолжение выполняются асинхронно.
- Устранено периодическое окно без SDK-кэша. Раньше каждые 180 секунд warmup
  сначала удалял исправную аттестацию, а затем примерно 30–35 секунд создавал
  новую; нажатие в этот интервал повторяло весь cold preflight. Теперь refresh
  начинается через 120 секунд и перекрывается с последним исправным
  результатом. Начальный запуск worker, потеря carrier/ping и ошибка refresh
  по-прежнему инвалидируют кэш; writer, KILL и команды движения не создаются.
- В cold RUN `prepare=33,561 s` робот не был медленным: от первого статуса
  writer до `local_kill_latch=reset` прошло `28,846 s`, тогда как от reset до
  подтверждённых StandUp/FSM 4 и Start/FSM 811 — `2,013 s`. До снятия KILL
  последовательно создавались отдельные DDS readers для head status/command,
  physical head seed, locomotion, arms и Deadman.
- Эти шесть наблюдений объединены с уже обязательным двухсекундным пассивным
  preflight. Требования к calibrated head, arm joint set, active=true, пяти
  стабильным q/dq samples и всем command topics не ослаблены. Финальный
  `2,25 s` ArmSdk traffic gate, оба KILL latch и стабильный FSM остаются после
  observer и перед готовностью RUN.
- Физический повтор нового control fast-path выполнен. В журнале есть marker
  `single discovery participant`, обязательный `2,25 s` ArmSdk traffic gate,
  `local_kill_latch=reset` и стабильные FSM 4/811. Полный путь от рождения
  manager-журнала до завершения prepare занял около `36 s`; этапы
  `ready_control=5,725 s` и `prepare=22,917 s`. Это аппаратный результат, а не
  перенос времени ROS mock на робот.
- Во время этого RUN выявлено обратное физическое боковое направление. Pico и
  bridge уже выдавали стандартный ROS-знак; инверсия требовалась только для
  Unitree pilot `lx`. Исправление внесено на границе writer transport, не меняя
  симуляцию, RPC и `lateral_sign=-1` в bridge. Оператор подтвердил правильное
  движение в обе стороны.
- После одновременного утреннего запуска ноутбука и робота 02.10 кнопка СТОЙКА
  была нажата до готовности background warmup. Панель остановила уже идущий
  read-only worker, поэтому manager повторил полный preflight с нуля. Новый
  handoff сохраняет прогресс фазы `CHECKING` до `READY`, затем ждёт штатного
  завершения владельца reader. Ожидание асинхронное и ограничено 45 секундами;
  timeout лишь передаёт работу прежнему authoritative manager preflight и не
  считается успехом.
- UI timeout не убивает физический процесс и не считается успехом. Для
  LOCK/СТОЙКИ/RUN оставлено 300 секунд, потому что холодный путь может законно
  выполнять полный SDK/FSM preflight.

### Переподключить всё

- Команда больше не запускает СТОЙКУ и не меняет режим робота.
- Исправный writer, VR/ROS graph и POV не перезапускаются.
- При падении только POV перезапускается только POV.
- ROS launch автоматически поднимает POV через 2 секунды. Если reconnect нажат
  в этот момент, manager ждёт process-local восстановление до 8 секунд и лишь
  затем перезапускает всю offline-службу как fallback.
- Исчезнувший writer не создаётся автоматически: после проверки условий нужен
  явный RUN.
- CLI ждёт новое подтверждение конкретного reconnect, а не принимает старый
  state-файл за успех.
- После потери Ethernet живой процесс writer больше не считается исправным
  только потому, что его PID сохранился: читается свежий `writer_armed/KILL`.
- Защёлкнутый `KILL` переводит панель в красный `degraded`; reconnect сохраняет
  процессы, но требует явный RUN и никогда сам не снимает защиту.

### Открыть Robot POV

- Открывается существующий локальный URL.
- Полный discovery/preflight не запускается.
- Второй видеосервер не создаётся.
- Если поток ещё стартует, окно панели остаётся отзывчивым и ждёт ready-событие.

### ZERO TORQUE

- Остановка вспомогательных QProcess выполняется без блокировки UI.
- HTTP preview получает неблокирующий stop вместо ожидания до 1,5 секунды в
  Qt-потоке.
- Обязательный offline-handoff и подтверждение освобождения портов перед
  Damping/FSM 1 → Zero Torque/FSM 0 сохранены.

### Калибровка рук

По решению оператора все кнопки калибровки рук удалены со всех экранов.
Случайно перезаписать готовый профиль на выставке нельзя. Для будущей точной
настройки оставлена только явная commissioning-команда:

```bash
make exhibition-calibrate-arms
```

Она допустима только при уже работающем control-сеансе и выполняет
`pause → arms/head calibration → resume`; автоматический RUN не запускается.

## 3. Время кнопок до и после

Числа «до» взяты из сохранённых физических журналов. Software-only измерения
получены в отдельном runtime с поддельными дочерними процессами. Последний
столбец — новая физическая проверка 01.10.2026; там, где точный click timestamp
не сохранился, это явно указано.

| Кнопка | До изменения | После, software-only | Физический статус после изменения |
|---|---:|---:|---|
| НАЙТИ И ПОДКЛЮЧИТЬ | click-to-result не записывался; каждый клик запускал четыре полных preflight | уже исправная связь: медиана `0,008 ms` в 500 mock-вызовах; новый default fast ping выполняется один раз | `0,01 s`, существующий `.161` использован сразу |
| LOCK, тёплый control graph | CLI `1,661–2,620 s`, успешный контрольный цикл `1,980 s`; панель могла добавить до `5 s` poll | `0,257 s` до mock acknowledgement | Ранее `0,922–2,170 s`; в последнем полном сеансе `2,817 s`, после проверки боковой оси `1,567 s`; удержание подтверждено |
| RUN, тёплый control graph | успешные `1,899–2,165 s`; панель могла добавить до `5 s` poll | `0,275 s` до mock acknowledgement | `1,70 s`, тот же graph/PID; последние LOCK → RUN `1,290 s` и `1,650 s` |
| RUN, холодный | недавние полные физические старты примерно `38–75 s` (отдельные старые журналы дольше) | `0,303 s` на полностью поддельных процессах; реальный ROS/DDS mock нового объединённого observer `2,80 s` | Новый объединённый путь физически завершился примерно за `36 s`: `ready_control=5,725 s`, `prepare=22,917 s`. Утренний СТОЙКА → RUN 02.10 занял `41 s` click-to-result: STOP прежней стойки `5,882 s`, затем control manager около `34 s` (`ready_control=6,588 s`, `prepare=21,645 s`) |
| СТОЙКА, холодная | недавние успешные физические старты `22–79 s`; сопоставимый цикл до ускорения — около `21 s`, prepare `14,779 s` | `0,253 s` mock; повторное нажатие в ready static — `0,0002 s`; reuse offline POV проверен без второго процесса; refresh-race test сохраняет предыдущий кэш; RUN cleanup теперь использует один stop participant и параллельное завершение детей | С готовым кэшем `11,652 s`. После одновременного cold boot 02.10 без кэша — `50 s`: `preflight=3,658 s`, `ready_static=36,696 s`, `prepare=9,602 s`. Последующий RUN → СТОЙКА — `24,15 s`, из них около `10 s` cleanup и новый static manager `12,819 s`; новая cleanup-оптимизация ещё не измерена физически |
| ZERO TORQUE | единого click-to-result замера не было; preview мог блокировать UI до `1,5 s` | после подтверждения оператора dispatch: медиана `0,0104 ms`, p95 `0,0116 ms` | Проверен на поддержанном роботе: STOP `2,400 s`, writer и offline-служба завершены, пять независимых read-only выборок подтвердили FSM 0; полный click timestamp не сохранялся |
| Переподключить всё | мог выполняться полный STOP/preflight/rebuild, то есть десятки секунд | исправный manager: `0,209 s` mock; без manager локальный status ack: медиана `0,0098 ms` | В новой static-сессии: `0,305 s`; после живого POV crash поток auto-recovered за `5,716 s`, последующий reconnect ничего не перезапустил |
| Открыть Robot POV | запускал auto-connect и полный `Проверить всё` | существующий поток: медиана `0,0098 ms`, p95 `0,0115 ms` | окно открылось, свежий кадр и единственный сервер `:8080`; точный click timestamp не сохранился |
| Перекалибровать руки | была отдельная кнопка | кнопки нет | Будущий commissioning вне выставочной панели |

Mock-время не включает движение механики, DDS discovery реального робота,
операторский диалог Zero Torque и аппаратные FSM-переходы.

## 4. Mock/dry-run проверки

- Целевой набор панели/status/orchestrator после исправления Ethernet:
  `131 passed in 21.30s`.
- Полный набор после исправления в отдельном network namespace только с
  loopback: `350 passed, 4 skipped in 49.54s`.
- После ускорения static prepare: целевой набор `79 passed`; финальный полный
  набор в отдельном loopback-only namespace — `360 passed in 48.72s`, включая
  четыре ROS integration-теста в изолированном domain `231`.
- После объединения финальных ROS-вызовов: целевой набор `90 passed`; полный
  loopback-only набор — `370 passed in 50.91s`. Поддельный writer подтвердил
  точный порядок `release → не менее 0,3 s → reset → prepare → FSM status`.
- После добавления conditional reuse offline POV: расширенный целевой набор
  `171 passed, 1 skipped in 22.83s` в отдельном network namespace только с
  loopback. Проверены отсутствие второго POV, отсутствие stop/start исправной
  службы и отдельное восстановление только systemd POV при reconnect.
- Финальный полный набор после этого изменения с загруженным ROS 2 и
  включёнными isolated integration-тестами в том же loopback-only namespace:
  `372 passed in 51.38s`.
- После process-local POV recovery: Robot POV/launch `75 passed`; manager
  `62 passed`; итоговый полный loopback-only набор в обязательном domain 231 —
  `375 passed in 52.17s`.
- После исправления cold recovered-link warmup: `69 passed` для всей панели и
  `39 passed` для соседних handoff/attestation/STOP/live-ack тестов. Полный
  прогон исходников и Robot POV в отдельном user+network namespace только с
  loopback, `ROS_DOMAIN_ID=231`, `ROS_LOCALHOST_ONLY=1` и закрытым actuation:
  `450 passed, 5 skipped in 52.69s`.
- После объединения control prepare observation: связанный набор
  prepare/traffic/orchestrator — `158 passed`; root desktop/operator suite в
  отдельном user+network namespace — `377 passed, 5 skipped in 43.32s`;
  функциональные ROS package tests по пакетам — ещё `427 passed, 1 skipped`.
  Один process-level ROS/DDS mock с `KILL=true`, без services и writer подтвердил
  все обязательные control-сигналы одним participant за `2,80 s`.
- После устранения periodic refresh race полный набор панели, warmup и
  attestation: `78 passed in 2.79s`; отдельный process-level тест прерывает
  worker во время `REFRESHING` и подтверждает, что недавний исправный кэш
  сохраняется только для немедленного handoff. Расширенный прогон в PID+
  loopback namespace дал `355 passed, 5 skipped`; четыре оставшихся отказа
  воспроизводятся в signal/QProcess cleanup самого PID namespace, тогда как
  затронутые тесты вне него проходят.
- После добавления bounded handoff уже выполняющегося cold warmup:
  `105 passed in 3.87s` для панели, cleanup, repeated STOP и cancellation.
  Проверены ожидание
  `CHECKING → READY`, отсутствие запуска до завершения владельца reader,
  45-секундный fallback и немедленное использование overlap-cache.
- После замера RUN → СТОЙКА `24,15 s` объединён stop path и распараллелена
  очистка независимых process groups. Тесты manager/session gate:
  `89 passed`; расширенный набор панели, manager, warmup и static prepare в
  network namespace только с loopback: `148 passed, 1 skipped`; полный
  loopback-only regression с ROS 2 workspace: `385 passed, 5 skipped in
  51.87s`. Первый полный прогон с выключенным `lo` дал только ожидаемые четыре
  `Network is unreachable` в localhost USB-тестах; после включения loopback те
  же четыре теста и весь набор прошли.
- `ament_flake8` для четырёх изменённых Python-файлов: `0` ошибок. Глобальные
  package flake8/pep257 по-прежнему ошибочно сканируют весь workspace/build и
  падают на старых несвязанных файлах; это отдельная baseline-проблема lint,
  не отказ runtime-тестов.
- Новые тесты подтверждают `OFFLINE → OK → clean stop → один свежий warmup`,
  отсутствие restart на первом `OK`, блокировку при физическом сеансе и
  приоритет ожидающего RUN/СТОЙКИ над фоновым restart.
- Process-level mock подтвердил crash/respawn: PID POV изменился, PID
  родительского ROS launch сохранился, новый `/readyz` стал доступен.
- Быстрый status-path с поддельными `ping/curl/ros2`: control endpoint вызван
  ровно один раз, PC2 пропущен после успеха, `ros2` не вызван.
- Process-level mock без сети:
  - cold RUN → ready: `0,303 s`;
  - warm LOCK → ack: `0,257 s`;
  - warm RUN → ack: `0,275 s`;
  - healthy reconnect → ack: `0,209 s`;
  - cold mock STAND → ready: `0,253 s`;
  - повторная STAND: `0,0002 s`.
- Проверены повторные и быстрые разные нажатия, смена назначения ожидающего
  перехода, явный STOP во время очереди, process-already-running, failed start,
  потеря POV, исчезновение writer, внешний manager после перезапуска панели и
  cold SDK warmup после перезагрузки ноутбука.
- Проверено, что reconnect не создаёт writer, а Robot POV не запускает второй
  сервер.
- Проверено, что read-only reconnect-health не вызывает ни одного ROS service,
  а защёлкнутый writer оставляет manager живым в `degraded` для последующего
  явного RUN.
- Python compile, `bash -n` и `git diff --check` проходят.

Ни один из этих тестов не отправлял RUN, StandUp, Zero Torque, writer-команды
или SDK physical probe реальному роботу.

## 5. Результаты физической проверки

Выполнены **НАЙТИ И ПОДКЛЮЧИТЬ**, Robot POV, холодная СТОЙКА, холодный RUN,
RUN → LOCK, LOCK → RUN, повторный LOCK и healthy reconnect. Физическое
отключение USB восстановилось без смены PID и без auto-RUN; режим сохранился
LOCK.

Повторная холодная СТОЙКА 01.10 после первого ускорения завершилась успешно:
полный manager-сеанс около `17 s`, `ready_static=5,575 s`, `prepare=9,013 s`.
После установки единого final prepare ROS-клиента выполнен ещё один аппаратный
замер: полный переход около `14 s`, offline handoff `2,311 s`, preflight
`0,077 s`, `ready_static=6,361 s`, `prepare=5,139 s`. Затем проверен conditional
reuse offline POV: системная служба сохранила PID `568705`, POV — `568805`,
bridge — `568803`; дубликатов не появилось. Новый manager подтвердил стабильный
FSM 4 за `10,460 s`: preflight `0,065 s`, `ready_static=5,261 s`, prepare
`4,455 s`. Состояние записано в
`logs/exhibition/static-20261001-130420.log`.

Healthy «Переподключить всё» в этом же сеансе завершилось за `0,305 s` со
строкой `all managed components already healthy; nothing restarted`.
`reconnect_count=1`, `pov_restarts=0`; PID manager, writer, offline-службы,
POV и VR bridge не изменились.

После перезапуска робота повторная СТОЙКА подтвердила FSM 4 примерно за
`13,3 s`: preflight `0,077 s`, readiness `5,691 s`, prepare `7,442 s`.
Затем offline-служба один раз перезапущена для загрузки нового launch-файла;
static manager и writer при этом сохранились. Принудительно завершён только
живой POV PID `1062105`; новый PID `1062956` стал ready за `5,716 s`.
Offline service, родительский ROS launch, VR bridge, static manager и writer
сохранили PID, `NRestarts=0`. Последующее нажатие **Переподключить всё**
вернуло `all managed components already healthy; nothing restarted`.

ZERO TORQUE проверен на поддержанном роботе. Панель сначала штатно завершила
static-сеанс: центральный KILL установлен, ArmSdk освобождён, STOP подтверждён
за `2,400 s`, writer исчез. Затем была остановлена точная offline-служба и
освобождены её порты. Вывод короткоживущего Zero Torque helper сохранялся
только в UI, поэтому финальное состояние дополнительно проверено отдельным
read-only SDK-запросом: пять последовательных выборок вернули `FSM 0` с кодом
`0`. Команд движения во время независимой проверки не отправлялось.

Ethernet был физически отключён на `10,397 s`. LowState стал недоступен, writer
защёлкнул fail-closed через `1,977 s`: locomotion обнулена, ArmSdk отпущен,
`KILL=true`. После подключения carrier и ping восстановились одновременно,
свежий feedback вернулся через `2,661 s`; все процессы и LOCK сохранились,
но движение автоматически не возобновилось. Это требуемое fail-closed
поведение.

После полной перезагрузки ноутбука выполнен отдельный cold-start. СТОЙКА
подтвердила FSM 4 примерно за `46 s`; журнал
`logs/exhibition/static-20261001-152011.log` показывает отсутствующий
`sdk-preflight-attestation.json`, `preflight=3,314 s`,
`ready_static=35,696 s`, `prepare=7,147 s`. Это доказало не гипотезу о
тайм-ауте, а конкретную потерю фонового кэша после OFFLINE/recovery.

Следующий cold RUN завершился примерно за `46 s`; журнал
`logs/exhibition/control-20261001-152402.log` показывает уже исправный SDK-cache
path (`preflight=0,086 s`), но отдельную задержку `prepare=33,561 s`.
RUN → LOCK занял `0,922 s`, LOCK → RUN — `1,290 s`, финальный LOCK —
`2,170 s`; оператор подтвердил удержание стойки. Один manager/writer был
сохранён. Поэтому warmup-исправление не выдаётся за решение независимой
control-prepare задержки.

Разбор временных меток локализовал её точнее: `28,846 s` ушли до reset локальной
KILL-защёлки на последовательные ROS/DDS readers, а сам подтверждённый
StandUp/Start после reset занял `2,013 s`. Новый control fast-path собирает те же
VR/head/arm/locomotion/deadman/head-seed доказательства одним пассивным
participant. Software-only ROS mock занял `2,80 s`; реальное новое время RUN
пока не измерено и из этой цифры не выводится.

По этим данным панель теперь программно перезапускает только read-only warmup
после `OFFLINE → OK`. После подтверждённого STOP панель была безопасно
перезапущена с новым кодом 01.10 в 15:52:48 MSK. Один read-only warmup создал
свежий `sdk-preflight-attestation.json` для интерфейса
`enxb4b024be59fe`, control IP `192.168.123.161` и ROS domain `88`; manager и
writer при этом не запускались.

Физический повтор СТОЙКИ начался в 16:20:54 и успешно подтвердил FSM 4 в
16:21:40. Журнал `logs/exhibition/static-20261001-162054.log` даёт полный
результат `46,696 s`: preflight `3,657 s`, `ready_static=35,130 s`, prepare
`7,502 s`. После reset локальной KILL-защёлки сам StandUp и пять стабильных
FSM 4 заняли `0,585 s`; существующий offline POV был переиспользован, второй
видеосервер не появился. Нажатие совпало с периодическим refresh: прежняя
версия worker уже удалила аттестацию, но ещё не записала новую. Поэтому
задержка локализована именно в cache race, а не в механике робота. После этого
`scripts/r1-sdk-warmup` переведён на перекрывающийся атомарный refresh; новый
вариант загружен после отдельного штатного STOP, но новая СТОЙКА ещё не
запускалась.

STOP текущего static-сеанса завершился за `2,303 s`; state получил
`safe_stop_confirmed=true`, manager и writer исчезли. Панель запустила ровно
один новый read-only warmup. Во время первого планового refresh в 16:39:57
прежний кэш продолжал существовать с теми же inode `271` и SHA256
`a949005139d881e9056da5512fee0350bec5d059ccb00cf8834bea6d4d5f3232` и прошёл
штатный `check` в возрасте `124,6 s`. В 16:40:27 новый результат атомарно
заменил его (inode `272`, другой SHA256), после чего снова прошёл `check`.
На всём интервале manager/writer отсутствовали; движения, KILL release и
service call не выполнялись. Тем самым прежнее пустое cache-window устранено
не только mock-тестом, но и read-only наблюдением на реальном Ethernet.

После нового подтверждения оператора выполнен финальный cached-переход. Файл
`logs/exhibition/static-20261001-164224.log`: log birth
16:42:24.836929 → state `ready` 16:42:36.488655, то есть `11,652 s` от старта
manager до подтверждённого FSM 4. Этапы: preflight `0,068 s`,
`ready_static=6,195 s`, prepare `5,350 s`. Аттестация была принята в возрасте
`118,3 s` и повторно в live launcher в возрасте `121,8 s`, то есть именно на
границе прежнего refresh-window. После `local_kill_latch=reset` prepare получил
пять стабильных FSM 4 за `0,432 s`. Сохранились один systemd POV PID `723600`,
один VR bridge и один writer; второй `:8080` не появился. По сравнению с
дефектным циклом `46,696 s` устранено `35,044 s`, около 75% полного времени.

Новый холодный control-сеанс также выполнен. В
`logs/exhibition/control-20261001-174715.log` manager стартовал в 17:47:15,
`ready_control` завершился за `5,725 s`, prepare — за `22,917 s`, а полный путь
до готовности занял около `36 s`. Журнал содержит marker
`single discovery participant`, обязательный `2,25 s` ArmSdk traffic gate,
`local_kill_latch=reset` и подтверждение стабильных FSM 4/811. В предыдущем
сеансе тёплый LOCK занял `2,817 s`, LOCK → RUN — `1,650 s`; финальный LOCK после
проверки направления занял `1,567 s`.

При физической проверке оператор обнаружил, что левый стик был зеркален по
боковой оси. Логика Pico и bridge уже формировала стандартный ROS-знак
`+Y = влево`; на этом R1 положительный Unitree pilot `lx` физически ведёт
вправо. Инверсия добавлена только в `r1_live_writer` при переводе ROS velocity
в physical pilot frame. Оператор повторно проверил оба направления и сообщил,
что всё работает. После LOCK выполнен разрешённый STOP; manager/writer
завершены, `safe_stop_confirmed=true`, offline video-only служба восстановлена.

Все запланированные физические проверки версии 01.10 выполнены. Добавленный по
утреннему замеру 02.10 handoff незавершённого warmup пока проверен только
software-only. Его аппаратный cold замер остаётся отдельным будущим действием;
любая новая команда требует актуального запроса и подтверждения оператора.

После замера RUN → СТОЙКА `24,15 s` отдельно устранена последовательная
программная очистка. Нормальный control STOP теперь сохраняет порядок
`disarm → central emergency_stop → writer stop`, но выполняет его одним ROS
participant; если хотя бы один stop-ответ не подтверждён, manager запускает
прежний независимый STOP/KILL fallback. Только после подтверждённого STOP POV и
control launch получают TERM одновременно. По журналу 09:25 это должно убрать
повторное DDS discovery в stop path и примерно один последовательный интервал
process cleanup, но ожидаемое время намеренно не подменяет будущий физический
click-to-result замер.

## 6. Действия оператора для будущей физической проверки

Перед тестом оператор должен заново письменно подтвердить:

- робот снят с зарядки и устойчив;
- вокруг свободная зона;
- Ethernet и USB-C не натянуты;
- управление Unitree Explore закрыто;
- аварийная кнопка и выключение батареи доступны;
- для RUN очки надеты, оба контроллера видны, руки спокойны, стики по центру.

Повторные проверки выполняются по одной и только по короткой письменной
подсказке. Zero Torque проверяется отдельно: робот должен лежать или быть
надёжно поддержан. Голосовые подсказки для этих действий не используются.

## 7. Краткий запуск на выставке

1. Подключить робот к ноутбуку существующим Ethernet, Pico — USB-C ↔ USB-C.
2. Включить робот, очки и контроллеры; интернет и Wi-Fi не требуются.
3. Запустить ярлык **Unitree R1 Панель оператора**.
4. Нажать **НАЙТИ И ПОДКЛЮЧИТЬ** и дождаться зелёных статусов робота, VR,
   контроллеров и видео.
5. Проверить Robot POV. Второй видеосервер вручную не запускать.
6. Для отдельной устойчивой позы использовать **СТОЙКА**.
7. Для полного VR-управления при выполненных физических условиях нажать
   **RUN**. Для паузы — **LOCK**, для продолжения — снова **RUN**.
8. При потере только видео нажать **Переподключить всё**. Если исчез writer,
   проверить условия и использовать явный RUN; reconnect сам движение не
   включает.
9. При опасности нажать `B` на правом контроллере или использовать аппаратное
   отключение. Zero Torque применять только на опоре/страховке.

## 8. Оставшиеся ограничения

- Cold-start после перезагрузки ноутбука и ещё один физический переход СТОЙКИ
  выполнены. Второй замер выявил более узкую причину: периодический refresh
  удалял исправную warmup-attestation до готовности замены. Перекрывающийся
  refresh прошёл process-level mock, реальную read-only проверку атомарной
  замены и физический cached-переход СТОЙКИ за `11,652 s`. ZERO TORQUE и
  принудительный POV-only crash также успешно проверены.
- Сокращённый путь СТОЙКИ физически проверен (`~21 → ~17 → ~14 → 10,460 s`).
  Обязательные preflight, KILL, writer-count,
  LowState/motor-health, traffic gate и FSM-подтверждения не удалялись.
- PC2 `192.168.123.164` недоступен, поэтому файлы PC2/робота не собраны и
  локальная копия не является образом робота.
- Полный USB/Ethernet-сеанс физически работал с выключенным Wi-Fi и без
  интернета. Это не доказывает поведение при повреждённом кабеле или отказе
  аппаратного Ethernet-адаптера.
- Панель перезапускалась после подтверждённого STOP и использовала новый код.
  Перезапуск не отправлял физическую команду; следующий запуск СТОЙКИ
  выполнялся отдельным нажатием оператора.
- Абсолютное отсутствие будущих аппаратных ошибок гарантировать нельзя;
  панель теперь выдаёт конкретную причину, ограниченный timeout и безопасный
  повтор вместо зависания или скрытого запуска движения.
- До объединения observer холодный control `prepare` занимал `33,561 s`, из
  которых `28,846 s` проходили до KILL reset. Новый путь сначала прошёл ROS mock
  за `2,80 s`, затем физический cold RUN: около `36 s` полностью,
  `ready_control=5,725 s`, `prepare=22,917 s`. Аппаратный результат зависит от
  DDS/SDK и не гарантирует одинаковое время каждого запуска.
- Cold boot 02.10 показал, что нажатие до завершения фонового SDK warmup всё ещё
  давало `50 s` для первой СТОЙКИ. Новый bounded handoff устраняет повтор уже
  идущего preflight, но пока имеет только software-only приёмку (`105 passed`),
  поэтому ожидаемое сокращение не выдаётся за аппаратно измеренный результат.

## 9. Исправление handoff после неудачного RUN 02.10

После загрузки варианта с bounded warmup handoff оператор нажал RUN. Панель
безопасно вернулась в `Остановлен` за `28,12 s`, manager/writer не остались,
но показала только общий `manager завершился с кодом 2`. Нового control-журнала
не появилось, поэтому отказ произошёл до создания manager log (в USB wrapper
или на самом раннем этапе запуска).

Одновременно был обнаружен воспроизводимый дефект панели: обработчик warmup
разбирал каждый QProcess chunk отдельно. Если строка
`SDK_WARMUP state=READY` делилась между двумя событиями Qt, состояние оставалось
`CHECKING`, хотя attestation уже существовала. Исправление собирает полные
строки для warmup, structured manager state и диагностик, сбрасывает буфер при
новом процессе и дочитывает хвост при завершении. Последняя полная
`[BLOCKED]/[FAIL]` причина теперь переносится в click-to-result строку.

Software-only проверки после исправления: целевой набор `82 passed`,
manager/session-gate набор `171 passed`, полный корневой прогон
`387 passed, 5 skipped in 51.56s`; `ament_flake8`, `py_compile` и
`git diff --check` успешны. После подтверждённого `stopped` без manager/writer
панель перезапущена с новым кодом; read-only warmup создал свежую attestation
02.10 в 10:16:07. Физический повтор RUN ещё не выполнен и требует нового
подтверждения оператора.

## 10. Постоянное USB-видео при RUN → СТОЙКА

Последующий физический RUN успешно включил управление и ходьбу. После LOCK
оператор выключил Wi-Fi ноутбука и выполнил RUN → СТОЙКА. СТОЙКА достигла
`ready/static` за `20,20 s`, но приложение очков было закрыто. Ручной запуск
приложения не вернул изображение. Read-only снимок показал одновременно:

- static manager/writer и FSM 4 готовы;
- единственный offline Robot POV работает на локальном `:8080`;
- Pico виден по USB;
- `adb reverse --list` пуст.

Причина находилась не в камере или Ethernet: старый `usb_link.control` считал
и `19092`, и `8080`, и APK собственностью RUN, а cleanup удалял оба mapping и
выполнял `am force-stop`. Новый путь разделяет владельцев:

- `8080 → 8080` — постоянный read-only video mapping панели;
- `19092 → 19092` — эксклюзивный mapping только текущего RUN;
- фоновый helper восстанавливает `8080` и USB-mode APK при старте панели,
  в СТОЙКЕ и после переподключения кабеля;
- общий `usb-transport.lock` не позволяет helper конкурировать с RUN;
- cleanup RUN сохраняет приложение и `8080`, удаляя только `19092`;
- marker serial/PID не перезапускает исправный APK, но исправляет ручной
  запуск без `r1_usb=true`.

Изолированная проверка: `45 passed`; `py_compile`, `bash -n`, lint без E501 и
`git diff --check` успешны. Активная физическая СТОЙКА во время редактирования
не перезапускалась. После отдельного разрешённого STOP оператор выключил робота
и поставил его на зарядку. Панель перезапущена с новой версией 02.10 в
11:13:12; offline-служба поднялась в 11:13:16. Подтверждены один POV, один
USB-mode APK, только reverse `8080 → 8080`, отсутствие `19092`, manager и
writer. Искусственно удалённый `8080` watchdog восстановил за `4,258 s`, PID
APK сохранился. Остался физический Wi-Fi-off повтор: видео должно сохраниться
в СТОЙКЕ, а RUN должен добавить `19092` без второго APK/POV процесса.

После зарядки этот путь дополнительно проверен физически с новой панелью:
оператор выполнил СТОЙКА → RUN → LOCK, переходы завершились штатно, а видео
в очках после LOCK сохранилось. Журналы дали `ready_static=5,303 s`, static
prepare `6,694 s`, `ready_control=5,778 s` и control prepare `16,559 s`;
FSM 4 и FSM 811 подтверждены. Отдельный повтор именно при выключенном Wi-Fi
оставался для финальной проверки полностью автономного режима.

## 11. Гонка очистки перед USB RUN и финальная Wi-Fi-off приёмка

При следующем СТОЙКА → RUN панель получила конкретную причину раннего отказа:
`USB RUN: Stop the active robot control session before USB diagnostics`.
Предыдущий static manager уже прошёл штатный STOP, но завершающийся writer ещё
кратко присутствовал в `/proc`. Это было окно очистки, а не второй разрешённый
сеанс управления.

В `usb_link/control.py` добавлено ограниченное ожидание исчезновения
предыдущего writer перед запуском control manager. Значение по умолчанию —
`12 s`, допустимый диапазон `R1_USB_LIVE_CLEANUP_WAIT_SEC=0…30`; опрос — каждые
`50 ms`. До успешного завершения проверки ADB/relay/manager не запускаются.
Если writer остаётся активным до тайм-аута, RUN блокируется с явной причиной.
KILL, deadman, watchdog, traffic gate и writer-count gate не менялись.

Проверки исправления:

- USB/handoff pytest: `50 passed`;
- полный прогон в отдельном loopback-only namespace: `405 passed in 55.08s`;
- `py_compile`, lint без E501 и `git diff --check`: успешно;
- реальный `/proc`-тест с искусственно завершающимся writer: ожидание `394 ms`,
  затем RUN-gate прошёл;
- физические повторы:
  `static-20261002-122838.log` (`ready=5,245 s`, `prepare=7,783 s`),
  `control-20261002-122914.log` (`ready=5,805 s`, `prepare=16,505 s`),
  `static-20261002-123008.log` (`ready=5,197 s`, `prepare=4,642 s`),
  `control-20261002-123036.log` (`ready=5,713 s`, `prepare=20,603 s`) и
  `static-20261002-123136.log` (`ready=5,512 s`, `prepare=4,668 s`).

Во всех новых журналах отсутствуют `[BLOCKED]` и `[FAIL]`, подтверждены FSM
4/811 и единственный writer. Оператор отдельно подтвердил, что все циклы
выполнены при выключенном Wi-Fi, видео в очках сохранялось и ошибок панели не
было. Тем самым постоянный USB-видеоканал и СТОЙКА → RUN → LOCK физически
приняты в полностью автономной схеме. Это не отменяет необходимость нового
разрешения оператора для каждого будущего физического запуска.
