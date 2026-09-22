# Robot POV: локальное видео глазами Unitree R1

`r1_robot_pov` — отдельный video-only контур. Он не запускает VR-мост,
инверсную кинематику, `/cmd_vel`, ros2_control или аппаратный адаптер. Viewer
работает в браузере внутри LAN без облака, внешнего signaling, STUN/TURN и
доступа в интернет.

```text
mock | ROS Image | USB/V4L2 | RTSP | Unitree videohub
                         │
                         ▼
        latest-frame-only hub
        crop / rotate / flip / eye swap
                 │
       ┌─────────┴─────────┐
       ▼                   ▼
 LAN WebRTC            MJPEG fallback
 local signaling       no-signal frames
       └─────────┬─────────┘
                 ▼
       Pico/browser fullscreen SBS
       status + reconnect + QR
```

## Один раз до поездки

На компьютере с интернетом сначала установить ROS/system-зависимости,
затем Python-зависимости и создать локальный wheel-кэш:

```bash
cd /home/unitree/Unitree_Project/R1_Teleoperation
source /opt/ros/humble/setup.bash
cd ros2_ws
rosdep install --from-paths src/r1_robot_pov --ignore-src \
  --skip-keys ament_python -r -y
cd ..

./scripts/install-robot-pov-deps
./scripts/cache-robot-pov-deps

cd ros2_ws
colcon build --symlink-install --packages-select r1_robot_pov
```

Каталог `.cache/robot-pov-wheelhouse` нужно скопировать вместе с проектом. Кэш
содержит checksum-manifest и привязан к текущим `requirements.txt`, OS, CPU и
версии Python. На выставочной машине с той же платформой проверить установку
с полным запретом сети:

```bash
./scripts/install-robot-pov-deps --offline
```

Ключ `--offline` не имеет сетевого fallback: при неполном, повреждённом или
устаревшем кэше скрипт завершится с ошибкой. Запущенный Robot POV никогда
не скачивает пакеты.

## Mock/demo без робота и камеры

```bash
cd /home/unitree/Unitree_Project/R1_Teleoperation
./scripts/robot-pov mock --profile balanced
```

Открыть один из адресов, напечатанных при запуске:

```text
http://robot-pov.local:8080/
http://<LAN-IP-ноутбука>:8080/
```

Числовой IP — обязательный fallback: некоторые Android/Pico сети не разрешают
`.local`. QR доступен на `http://<LAN-IP>:8080/qr.png` и показан на стартовой
странице.

Mock содержит движущуюся сетку, счётчик кадров и timecode. Поэтому зависший
кадр заметен сразу, даже без диагностической панели.

Для быстрого smoke-теста без робота доступны Make-алиасы:

```bash
make pov-demo
make pov-stereo-demo
```

`pov-stereo-demo` передаёт два mock-глаза в side-by-side. Для одной камеры
выберите `Mono`: сервер отдаёт один кадр, а browser/Unity viewer показывает его
обоим глазам и сообщает mono fallback. Layout можно проверить также напрямую:

```bash
./scripts/robot-pov mock --layout top-bottom --profile balanced
```

Профили качества: `low-latency` (приоритет задержки), `balanced`, `high`
(разрешение/bitrate) и `bad-wifi` (меньший поток и jitter tolerance). Текущие
FPS, разрешение, layout, dropped frames, latency и reconnect count доступны в
overlay viewer и `/api/status`.

## Как открыть viewer в Pico/очках

1. Подключить очки и ноутбук к одному локальному Wi-Fi; WAN/интернет не нужен.
2. Запустить mock и сначала открыть в браузере очков числовой URL,
   напечатанный скриптом, либо отсканировать QR со стартовой страницы.
3. Проверить, что timecode движется, затем включить `На весь экран` или `VR`.

Отдельное приложение на очки устанавливать не нужно. Для обычного локального
HTTP гарантирован fullscreen-SBS fallback; immersive WebXR зависит от
поддержки браузера и доверенного HTTPS-сертификата.

Тот же mock через установленный ROS 2 launch (также без камеры):

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch r1_robot_pov robot_pov.launch.py \
  env_file:="$(pwd)/ros2_ws/src/r1_robot_pov/config/robot_pov.mock.env" \
  enable_mdns:=false
```

## Выставочный offline-профиль

Скопировать и отредактировать:

```bash
cp ros2_ws/src/r1_robot_pov/config/robot_pov.exhibition.env \
  ros2_ws/src/r1_robot_pov/config/robot_pov.venue.env
```

`robot_pov.venue.env` и `robot_pov.local.env` исключены из Git и не попадают в ROS
install-space. Храните в них локальные IP/логины, но не копируйте такой файл в
публичный архив или отчёт.

Запуск:

```bash
./scripts/robot-pov exhibition \
  --env-file ros2_ws/src/r1_robot_pov/config/robot_pov.venue.env
```

Альтернативно после сборки можно запустить launch; пустые launch-overrides не
заменяют значения из `.env`:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch r1_robot_pov robot_pov.launch.py \
  env_file:="$(pwd)/ros2_ws/src/r1_robot_pov/config/robot_pov.venue.env"
```

Для разовой замены можно добавить `source:=`, `profile:=`, `transport:=`,
`layout:=`, `host:=` или `port:=`. Без них launch строго использует `.env`.

Упрощённый профиль плохого Wi-Fi:

```bash
./scripts/robot-pov bad-wifi
```

Рекомендуемая локальная сеть:

- отдельный 5 GHz роутер без client/AP isolation;
- DHCP reservations либо статические адреса, например ноутбук `.10`, робот
  `.20`, IP-камера `.30` в одной `/24` подсети;
- интернет/WAN не требуется;
- TCP-порт viewer по умолчанию `8080`;
- WebRTC signaling и MJPEG идут через TCP `8080`, а media — через динамический
  локальный UDP-порт aiortc; STUN/TURN не используются. В firewall нужно разрешить
  приложение Python только для private/LAN profile;
- сохранить числовой URL рядом с QR, даже если mDNS работает.

## Источники камеры

### Физический Unitree R1 через videohub

Этот путь подтверждён на подключённом R1. Ноутбук использует адрес
`192.168.123.162/24` на интерфейсе `enxb4b024be59fe`, управляющий компьютер
робота доступен по `192.168.123.161`. Сначала выполнить read-only
инвентаризацию, затем запустить отдельный video-only профиль:

```bash
cd /home/unitree/Unitree_Project/R1_Teleoperation
./scripts/r1-camera-preflight
./scripts/robot-pov r1
```

Viewer напечатает числовой Wi-Fi URL, например
`http://192.168.8.9:8080/`. DHCP-адрес может измениться, поэтому нужно
использовать адрес из текущего вывода, а не копировать пример буквально.

После перезапуска робота сервис камеры может оставаться выключенным. В
официальном **Unitree R1 App** откройте `Settings → Service Status` и запустите
только `video_hub`. Если активен `Stereo patch PC1` / `R1 push service`, сначала
остановите его: этот сервис конфликтует с `video_hub`. Для video-only проверки
не включайте motion, debug, sport или low-level control services. После этого
повторите `./scripts/r1-camera-preflight --dds-seconds 10`: пары
`request-reader=true` и `response-writer=true` подтверждают, что RPC-сервер
`videohub` доступен через DDS.

Параметры находятся в
`ros2_ws/src/r1_robot_pov/config/robot_pov.r1.env`:

```dotenv
ROBOT_POV_SOURCE=unitree
ROBOT_POV_LAYOUT=mono
ROBOT_POV_UNITREE_INTERFACE=enxb4b024be59fe
ROBOT_POV_UNITREE_TIMEOUT_SEC=3.0
ROBOT_POV_UNITREE_FPS=15.0
ROBOT_POV_ROBOT_IP=192.168.123.161
```

Источник лениво создаёт только `unitree_sdk2py.go2.video.VideoClient` и
вызывает video-only API `GetImageSample`. Он не импортирует motor, sport,
locomotion, trajectory или low-level command API. На проверенной прошивке R1
получен JPEG `1280x720` со скоростью примерно `14 FPS`; выход сохраняет
соотношение сторон 16:9 и раздаётся как mono WebRTC/MJPEG. SDK-запрос имеет
ограниченный timeout, очередь всегда содержит один последний кадр, а после
ошибки выполняется автоматическое переподключение. Повторные ошибки используют
ограниченный backoff `0.5 → 1 → 2 → 4 → 5` секунд, чтобы недоступный
`video_hub` не создавал поток RPC-ошибок; после первого успешного кадра задержка
сбрасывается.

Объявленный DDS writer `rt/frontvideostream` был виден в discovery, но не
передал ни одного sample даже после запуска `videohub`. Поэтому текущая
реализация следует официальному примеру SDK и опрашивает
`GetImageSample`. Если будущая прошивка начнёт публиковать непрерывные кадры,
можно добавить subscriber-only backend без изменения web/Unity-контракта.

Проверка запущенного физического источника:

```bash
./scripts/robot-pov r1-preflight --live
```

Нормальный результат содержит `source: Unitree videohub interface is up`,
`viewer.readyz: ready`, FPS около 14/15 и малый `frame age`. Предупреждение
только о недоступном `robot-pov.local` допустимо: используйте числовой URL.

### ROS mono/stereo

```dotenv
ROBOT_POV_SOURCE=ros
ROBOT_POV_LAYOUT=stereo
ROBOT_POV_LEFT_TOPIC=/r1/camera/left/left_eye/image_raw
ROBOT_POV_RIGHT_TOPIC=/r1/camera/right/right_eye/image_raw
```

Для mono установить `ROBOT_POV_LAYOUT=mono`; используется только левый топик.
Текущая модель Gazebo публикует два `640x480 RGB8` потока, 30 FPS nominal,
HFOV 80°, baseline/IPD 64 мм.

### USB/V4L2

```dotenv
ROBOT_POV_SOURCE=usb
ROBOT_POV_LAYOUT=mono
ROBOT_POV_LEFT_SOURCE=/dev/v4l/by-id/REPLACE_WITH_STABLE_CAMERA_ID
```

Не использовать `/dev/video0` как постоянную выставочную настройку: номер может
измениться после перезагрузки. Два `/dev/video*` одного USB-девайса нельзя
автоматически считать стереопарой.

### RTSP

```dotenv
ROBOT_POV_SOURCE=rtsp
ROBOT_POV_LAYOUT=stereo
ROBOT_POV_LEFT_SOURCE=rtsp://user:password@192.168.50.30/left
ROBOT_POV_RIGHT_SOURCE=rtsp://user:password@192.168.50.30/right
```

Пароли RTSP маскируются в диагностическом отчёте. Для R1 RTSP URL не найден;
используется отдельный подтверждённый источник `unitree`, описанный выше.
Переменная `ROBOT_POV_ROBOT_IP` добавляет только сетевую проверку в preflight;
она не открывает actuator/control API. Для ROS-источника драйвер камеры должен
отдельно публиковать настроенные `sensor_msgs/Image` topics.

## Геометрия изображения

Все параметры находятся в `.env`:

```dotenv
ROBOT_POV_SWAP_EYES=false
ROBOT_POV_CROP=0,0,0,0
ROBOT_POV_ROTATION=0
ROBOT_POV_FLIP_HORIZONTAL=false
ROBOT_POV_FLIP_VERTICAL=false
ROBOT_POV_FOV_DEG=80
ROBOT_POV_IPD_MM=64
ROBOT_POV_DISTORTION_K1=0.0
```

`CROP` задаётся как `top,right,bottom,left` в пикселях. Поворот: `0`, `90`,
`180` или `270`. Eye swap можно дополнительно переключить во viewer без
перезапуска источника.

## Профили качества

| Профиль | SBS-размер | FPS | Целевой WebRTC bitrate | MJPEG quality |
|---|---:|---:|---:|---:|
| `high` | 1280×480 | 30 | 3500 kbit/s | 85 |
| `balanced` | 960×360 | 24 | 1800 kbit/s | 72 |
| `low-latency` | 960×360 | 30 | 1200 kbit/s | 62 |
| `bad-wifi` | 640×240 | 15 | 550 kbit/s | 48 |

`auto` сначала использует WebRTC с локальным HTTP signaling. При ошибке ICE,
потере потока или долгом no-signal viewer переподключается и переходит на
MJPEG. Очередь кадров всегда имеет глубину один: задержка не накапливается.
Профиль передаёт WebRTC bitrate-limit через SDP; встроенный REMB дополнительно
адаптирует кодек. После трёх устойчиво плохих RTT/jitter/loss/drop samples
или трёх reconnect за 20 с viewer понижает качество по цепочке
`high → balanced → low-latency → bad-wifi`; cooldown — 30 с, автоповышения нет.

Принудительный MJPEG для диагностики:

```text
http://<LAN-IP>:8080/?transport=mjpeg&profile=bad-wifi
```

Либо запустить весь viewer с `--transport mjpeg`. В чистой LAN без WAN
состояние определяется по реальному потоку, а не по ненадёжному `navigator.onLine`.

## Viewer и VR

- `Fullscreen` открывает безрамочный mono/SBS viewer.
- `Enter VR` сначала пробует WebXR `immersive-vr`, затем безопасно переходит в
  fullscreen SBS.
- WebXR в браузере обычно требует HTTPS/secure context. Обычный fullscreen SBS
  работает по локальному HTTP и остаётся основным offline fallback.
- Status overlay показывает source, transport, profile, FPS, frame age/latency,
  bitrate, dropped frames, network и reconnect count.
- При stale source показывается меняющийся `NO SIGNAL`, а не последний кадр.

### Native Unity WebRTC client

Нативный Unity/OpenXR-клиент может использовать тот же локальный signaling без
облака, STUN и TURN. После завершения ICE gathering он отправляет SDP offer:

```json
{
  "sdp": "v=0...",
  "type": "offer",
  "profile": "low-latency",
  "preferredCodec": "h264",
  "clientTag": "unity-pico-r1"
}
```

`preferredCodec` необязателен и принимает только `h264` или `vp8`
без учёта регистра; формы `video/H264` и `H.264` также нормализуются. Сервер
ставит запрошенный кодек первым, но он всё равно должен присутствовать в SDP
клиента. Если поле отсутствует, сохраняется браузерное поведение: первым идёт
VP8. Неизвестное значение возвращает HTTP 400 до создания peer connection.

`clientTag` (также принимается совместимый ключ `client`) — только безопасная
диагностическая метка: максимум 64 ASCII-символа из букв, цифр, `.`, `_`, `:`,
`-`. Она не включает управление, авторизацию или дополнительные разрешения.
Возможности этого контракта опубликованы в `/api/config` в поле `nativeOffer`.

Для настоящего WebXR по LAN заранее настройте локальный сертификат, которому
доверяет Pico, и задайте обе переменные:

```dotenv
ROBOT_POV_TLS_CERT=/absolute/path/robot-pov.crt
ROBOT_POV_TLS_KEY=/absolute/path/robot-pov.key
```

## Preflight

### Физический R1: безопасная инвентаризация

До настройки источника можно отдельно проверить подключение R1 и объявления
камерного контура:

```bash
./scripts/r1-camera-preflight
# либо
make r1-camera-preflight
```

По умолчанию проверяются интерфейс `enxb4b024be59fe`, native DDS-компьютер
`192.168.123.161` и PC2 `192.168.123.164`. Значения можно изменить через
`--interface`, `--control-ip` и `--pc2-ip`; полный список показывает `--help`.

Скрипт только читает локальные метаданные сети и `/dev/video*`, выполняет
ограниченные ICMP/TCP connect-пробы и слушает встроенные DDS discovery topics с
помощью `ddsls`. Он не открывает V4L2-устройства, не подписывается на кадры, не
публикует DDS application data, не вызывает `videohub`, не запускает TeleImager
Camera Finder и не обращается к motor/locomotion topics. Сам DDS discovery
participant отправляет только стандартные RTPS-анонсы; исключить даже их можно
флагом `--skip-dds`.

В отчёте `/dev/video*` всегда означает устройства компьютера, на котором
выполнен скрипт, а не автоматически камеры R1. Топики
`/r1/camera/left/left_eye/image_raw` и
`/r1/camera/right/right_eye/image_raw` созданы Gazebo-моделью и также не
доказывают наличие физической камеры. Для native DDS отдельно показываются
писатель `rt/frontvideostream` и серверная пара
`rt/api/videohub/{request,response}`. Обнаруженные имена без writer/responder
означают, что контракт виден, но видеобэкенд не готов. В этом случае preflight
печатает точное действие: в Unitree R1 App открыть `Settings → Service Status`,
запустить `video_hub` и остановить конфликтующий `Stereo patch PC1` /
`R1 push service`, если он включён.

Проверки портов PC2 ограничены SSH (`22`), NoMachine (`4000`) и стандартными для
TeleImager портами ZMQ/WebRTC (`55555`/`60001`); сканирование диапазонов не
выполняется. Коды завершения совпадают с основным preflight:
`0=OK`, `2=WARN`, `1=FAIL`.

### Viewer и настроенный источник

До запуска viewer:

```bash
./scripts/robot-pov preflight
```

При уже работающем viewer:

```bash
./scripts/robot-pov preflight --live
```

Для физической камеры R1 обе команды должны использовать её отдельный профиль:

```bash
./scripts/robot-pov r1-preflight
./scripts/robot-pov r1-preflight --live
```

Проверяются video-only invariant, конфиг, Python/codec dependencies, LAN IPv4,
порт, mDNS и числовой URL, камера/ROS/RTSP источник, freshness/FPS, `/healthz`,
`/readyz`, `/api/status`, свободное место и необязательная доступность робота.
Результат `OK/WARN/FAIL` сохраняется локально в `logs/robot_pov` вместе с JSON.
Коды завершения: `0=OK`, `2=WARN`, `1=FAIL`. В mock ожидаемы WARN, если не
задан IP робота или не запущен mDNS; это не ошибка видеоконтура.

## Checklist перед поездкой

- Запустить mock минимум на 15 минут, открыть viewer одновременно на ноутбуке и
  очках.
- Проверить WebRTC, затем открыть URL с `?transport=mjpeg`.
- Отключить источник на 3 секунды: должен появиться `NO SIGNAL`, после возврата
  поток должен восстановиться без reload страницы.
- Проверить каждый профиль, eye swap, rotation и fullscreen.
- Запустить `preflight --live`, сохранить отчёт.
- Скопировать проект, `.env`, wheelhouse и QR на запасной носитель.
- Взять роутер, питание, Ethernet/USB-кабели и запасной USB-адаптер.
- Не обновлять ОС, browser или зависимости непосредственно перед выставкой.

## Checklist на площадке

- Поднять отдельную LAN и отключить AP/client isolation.
- Проверить адрес ноутбука и сначала открыть числовой URL.
- Запустить mock, затем реальный источник.
- Выполнить `preflight --live`.
- Убедиться, что timecode движется и frame age не растёт.
- При помехах выбрать `bad-wifi`; при проблеме WebRTC добавить
  `?transport=mjpeg` к URL.
- Video viewer не должен запускаться через launch, содержащий IK/locomotion.

## Если видео пропало

1. Убедиться, что overlay показывает `NO SIGNAL`, а не замороженную картинку.
2. Проверить `/readyz` и `/api/status` по числовому IP.
3. Для ROS проверить `ros2 topic hz <image_topic>`.
4. Для USB проверить стабильный `/dev/v4l/by-id` путь и питание камеры.
5. Для RTSP проверить IP/порт камеры в той же LAN.
6. Переключить профиль на `bad-wifi`, затем открыть URL с
   `?transport=mjpeg&profile=bad-wifi`.
7. Проверить, что Pico и ноутбук не разнесены guest/AP-isolated сетями.

Runtime и preflight logs находятся только на локальном компьютере. Основной
video-only процесс не подписывается на `/vr/*`, не публикует `/cmd_vel` и не
имеет trajectory/controller endpoints.
