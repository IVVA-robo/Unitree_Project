# Native Unity Robot POV для Pico

Этот документ описывает отдельный нативный Unity-клиент, который показывает
локальный поток `r1_robot_pov` в Pico как изображение «глазами робота». Клиент
работает только с видео: он не запускает VR-телеуправление, не публикует ROS 2
команды и не активирует робота.

Основной сценарий полностью локальный:

```text
mock / ROS Image / USB / RTSP
              │
              ▼
      r1_robot_pov на ноутбуке
      HTTP signaling + WebRTC, LAN only
      без облака, STUN и TURN
              │
      ┌───────┴────────┐
      ▼                ▼
Unity/Pico APK      браузерный viewer
H.264 → VP8         WebRTC → MJPEG
head-locked SBS     диагностический fallback
```

## Почему Unity — основной Pico-клиент

Нативный Unity-клиент предпочтителен для постоянной выставочной установки:

- PicoXR и Unity управляют XR-камерой и стереорендерингом напрямую;
- head-locked экран не зависит от поддержки WebXR и HTTPS secure context в
  браузере Pico;
- APK имеет фиксированный интерфейс без адресной строки и браузерных окон;
- клиент запрашивает H.264, подходящий для аппаратного декодирования Pico, но
  сохраняет VP8 в согласовании как запасной кодек;
- состояние источника, FPS, RTT и число переподключений видно прямо в очках;
- при потере источника показывается `NO SIGNAL`, а соединение восстанавливается
  с exponential backoff.

Браузерный viewer остаётся обязательным fallback. Он не требует установки APK,
быстро проверяется с любого устройства в LAN и умеет перейти с WebRTC на MJPEG.
Если нативный WebRTC-клиент не проходит физический тест на конкретной прошивке
Pico, откройте `http://<LAN-IP-ноутбука>:8080/` в браузере очков. Полное описание
браузерного контура находится в `docs/robot_pov.md`.

## Канонические пути и версии

Использовать нужно только следующие пути:

| Назначение | Путь / версия |
|---|---|
| Активный проект | `/home/unitree/Unitree_Project/R1_Teleoperation` |
| Unity-проект | `/home/unitree/Unitree_Project/Unity_Projects/Unitree_VR_Controller` |
| Резервная копия Unity source/settings | `/home/unitree/Unitree_Project/Backups/Unity/Unitree_VR_Controller.pre-native-pov.2026-09-11` |
| Unity Editor | `/home/unitree/Unity/Hub/Editor/2022.3.44f1/Editor/Unity` |
| Unity version | `2022.3.44f1` |
| Pico SDK | `/home/unitree/Unitree_Project/SDK/Pico/Pico_SDK` |
| Unity WebRTC | `com.unity.webrtc 3.0.0-pre.8` |
| ADB | `/usr/bin/adb` |

Android Build Support, SDK, NDK и OpenJDK уже находятся в Unity
`2022.3.44f1/Editor/Data/PlaybackEngines/AndroidPlayer`.

## Реализованные Unity-компоненты

Код расположен во внешнем Unity-проекте:

- `Assets/RobotPov/Runtime/RobotPovVideoOnlyBootstrap.cs` — явная точка входа
  только для video-only сцены;
- `Assets/RobotPov/Runtime/RobotPovRuntimeConfig.cs` — LAN endpoint, профиль,
  кодек, таймауты, layout и display-настройки;
- `Assets/RobotPov/Runtime/RobotPovWebRtcReceiver.cs` — WebRTC receiver для
  desktop и Pico-safe MJPEG receiver для Android, status polling, метрики и
  reconnect;
- `Assets/RobotPov/Runtime/RobotPovSbsPresenter.cs` — head-locked/world-space
  экран, curved immersive surface, `NO SIGNAL` и диагностический overlay;
- `Assets/RobotPov/Runtime/Resources/RobotPovSbs.shader` — GPU-разделение
  mono/SBS/top-bottom текстуры между левым и правым глазом;
- `Assets/Editor/RobotPovVideoOnlyBuild.cs` — validation и изолированная Android
  сборка;
- `Assets/Plugins/Android/AndroidManifest.xml` — объявленный приложением
  `INTERNET`, разрешение local HTTP и стандартная Unity launcher activity.

`RobotPov.RobotPovVideoOnlyBootstrap` создаёт runtime только когда компонент
присутствует в отдельной сцене. Глобального auto-bootstrap нет.

Значения по умолчанию:

| Параметр | Значение |
|---|---:|
| server URL | `http://192.168.8.9:8080` (override for a changed LAN address) |
| profile | `low-latency` |
| client tag | `unity-pico-r1` |
| preferred codec | `h264`, затем negotiated VP8 fallback |
| layout | stereo side-by-side |
| display surface | `CurvedImmersive`, 140° × 100° |
| screen mode | head-locked |
| screen distance / size | радиус `2 m`; `FlatQuad` сохраняет `3 × 2.25 m` |
| ICE / HTTP / first-frame timeout | `5 / 10 / 10 s` |
| decoded-frame stale timeout | `4 s` |
| reconnect delay | `0.5 s`, экспоненциально до `8 s` |
| status / metrics interval | `1 / 2 s` |

Поддерживаются `high`, `balanced`, `low-latency` и `bad-wifi`. Для выставки
лучше начинать с `low-latency`, а при потерях заранее собрать вариант
`bad-wifi`.

### Immersive, stereo и mono fallback

`CurvedImmersive` создаёт вокруг XR-камеры head-locked mesh 140° × 100°.
Изображение занимает поле зрения и не выглядит как маленькая карточка. Для
отладки можно выбрать `FlatQuad`. В shader layout выбирает источник для каждого
глаза: `Mono` использует весь кадр обоими глазами, `StereoSideBySide` берёт
левую/правую половины, `StereoTopBottom` — верхнюю/нижнюю половины. Реальная
камера R1 сейчас mono, поэтому её поток намеренно дублируется на оба глаза;
true stereo требует двух синхронных источников.

В browser viewer `VR` вызывает `immersive-vr`, а `На весь экран` переключает
полноэкранный fallback. Диагностика скрывается кнопкой `Оверлей` или клавишей
`O`. На Pico Unity-клиент по умолчанию использует тот же mono/SBS/top-bottom
контракт и оставляет overlay выключенным.

## Video-only safety boundary

Сборщик создаёт временную сцену только с одной XR Camera и одним
`RobotPovVideoOnlyBootstrap`. Перед сборкой он запрещает компоненты
`VRUdpSender` и `UDP_Controller`, собирает сцену с define
`ROBOT_POV_VIDEO_ONLY`, а затем удаляет generated scene. Существующая teleop
сцена и её `EditorBuildSettings` не включаются.

У APK отдельная идентичность:

```text
package ID: com.ionosrobots.unitree.robotpov
product:    Unitree Robot POV
```

Android-сборка принудительно использует ARM64, IL2CPP и OpenGLES3. Настройки
Unity временно меняются на время validation/build и восстанавливаются после
завершения. Собственный manifest не добавляет микрофон, геолокацию, Bluetooth
или USB. Однако итоговые разрешения нужно проверять уже в собранном APK:
PicoXR/Unity manifest merge может добавить платформенные разрешения, которых
нет в исходном manifest. Они не создают путь к моторике, но не должны оставаться
незамеченными в safety-аудите.

Unity runtime обращается только к следующим video endpoints:

```text
POST /offer
GET  /api/status
POST /api/client-metrics
```

В нём нет адресов actuator API, ROS publishers, `/cmd_vel`, trajectory или
ros2_control. Запуск Robot POV не является разрешением на движение робота.

## Native offer contract

После полного завершения ICE gathering клиент отправляет один non-trickle
offer. Частичный SDP на сервер не отправляется:

```json
{
  "sdp": "v=0...",
  "type": "offer",
  "profile": "low-latency",
  "preferredCodec": "h264",
  "clientTag": "unity-pico-r1"
}
```

`preferredCodec` принимает только `h264` или `vp8`; сервер также нормализует
`video/H264`, `H.264` и регистр. H.264 выбирается первым только если он есть в
client SDP. VP8 остаётся fallback. Если поле отсутствует, сервер сохраняет
browser default VP8. Неверный кодек возвращает HTTP 400 до создания peer.

`clientTag` используется только в диагностике. Допустимо не более 64 ASCII
символов: буквы, цифры, `.`, `_`, `:`, `-`. Сервер публикует этот контракт в
`GET /api/config`, поле `nativeOffer`. `RTCConfiguration` клиента и сервера не
содержит ICE servers: облако, STUN и TURN не используются.

## 1. Сначала запустить безопасный mock

Робот, камера и очки для этого шага не нужны:

```bash
cd /home/unitree/Unitree_Project/R1_Teleoperation
./scripts/robot-pov mock --profile low-latency
```

В другом терминале проверить локальный сервер:

```bash
curl --fail http://127.0.0.1:8080/healthz
curl --fail http://127.0.0.1:8080/api/config
```

Перед сборкой APK узнайте IPv4 интерфейса, подключённого к выставочному
роутеру:

```bash
ip -4 -br addr
```

В APK нужно записать числовой адрес ноутбука, доступный из Pico, например
`http://192.168.8.9:8080`. Нельзя использовать `127.0.0.1`: внутри Pico он
указывает на сами очки. На выставке закрепите IP ноутбука через DHCP reservation
или статическую настройку. `.local` поддерживается как вариант, но числовой IP
надёжнее на Android/Pico.

## 2. Validation и сборка в Unity Editor

Unity-проект можно открыть так:

```bash
/home/unitree/Unity/Hub/Editor/2022.3.44f1/Editor/Unity \
  -projectPath /home/unitree/Unitree_Project/Unity_Projects/Unitree_VR_Controller
```

После окончания импорта доступны пункты меню:

```text
Robot POV/Validate video-only build
Robot POV/Build video-only Android APK
```

Build-команда сама создаёт и удаляет временную сцену. Добавлять teleop-сцену в
Build Settings не нужно.

## 3. Повторяемая batch-сборка

Перед batch-командой закройте этот Unity-проект в GUI, иначе его lock-файл не
даст открыть второй Editor process.

```bash
UNITY_BIN=/home/unitree/Unity/Hub/Editor/2022.3.44f1/Editor/Unity
UNITY_PROJECT=/home/unitree/Unitree_Project/Unity_Projects/Unitree_VR_Controller
POV_LOG_DIR="$UNITY_PROJECT/Logs/RobotPovVideoOnly"

mkdir -p "$POV_LOG_DIR"

"$UNITY_BIN" \
  -batchmode -nographics -quit \
  -projectPath "$UNITY_PROJECT" \
  -buildTarget Android \
  -executeMethod RobotPov.Editor.RobotPovVideoOnlyBuild.ValidateVideoOnlyBuildBatch \
  -logFile "$POV_LOG_DIR/validate.log"
```

Для APK задайте сервер, который будет доступен очкам, и явный output path:

```bash
UNITY_BIN=/home/unitree/Unity/Hub/Editor/2022.3.44f1/Editor/Unity
UNITY_PROJECT=/home/unitree/Unitree_Project/Unity_Projects/Unitree_VR_Controller
POV_LOG_DIR="$UNITY_PROJECT/Logs/RobotPovVideoOnly"
POV_APK="$UNITY_PROJECT/Builds/RobotPovVideoOnly/RobotPovVideoOnly.apk"
POV_SERVER=http://192.168.8.9:8080

mkdir -p "$POV_LOG_DIR" "$(dirname "$POV_APK")"

ROBOT_POV_SERVER_URL="$POV_SERVER" \
ROBOT_POV_PROFILE=low-latency \
ROBOT_POV_APK_PATH="$POV_APK" \
"$UNITY_BIN" \
  -batchmode -nographics -quit \
  -projectPath "$UNITY_PROJECT" \
  -buildTarget Android \
  -executeMethod RobotPov.Editor.RobotPovVideoOnlyBuild.BuildAndroidBatch \
  -logFile "$POV_LOG_DIR/build.log"

test -s "$POV_APK"
sha256sum "$POV_APK" | tee "$POV_APK.sha256"
```

Если `ROBOT_POV_APK_PATH` не задан, default равен
`Builds/RobotPovVideoOnly/RobotPovVideoOnly.apk`. Явный абсолютный путь
предпочтительнее. `ROBOT_POV_SERVER_URL` принимает только HTTP(S) loopback,
private IP или `.local`; внешний/cloud URL validation не пройдёт.

Проверить package ID, launcher, архитектуру и фактически merged permissions:

```bash
AAPT=/home/unitree/Unity/Hub/Editor/2022.3.44f1/Editor/Data/PlaybackEngines/AndroidPlayer/SDK/build-tools/34.0.0/aapt
POV_APK=/home/unitree/Unitree_Project/Unity_Projects/Unitree_VR_Controller/Builds/RobotPovVideoOnly/RobotPovVideoOnly.apk

"$AAPT" dump badging "$POV_APK"
"$AAPT" dump permissions "$POV_APK"
```

### Проверенный локальный артефакт

Batch validation и сборка от 2026-09-11 завершились успешно. Generated scene
после сборки удалена, а в `EditorBuildSettings` не добавлена.

```text
APK:    /home/unitree/Unitree_Project/Unity_Projects/Unitree_VR_Controller/Builds/RobotPovVideoOnly/RobotPovVideoOnly.apk
size:   60,920,679 bytes
SHA-256: 9e61b1494ad8fa976f4e8279eec023e09648e6124e24914cc960c507a6dbb5
```

`aapt` подтвердил package `com.ionosrobots.unitree.robotpov`, label
`Unitree Robot POV`, launcher `UnityPlayerActivity`, `arm64-v8a` и OpenGL ES
3.0. Кроме `INTERNET`, PicoXR/Unity merge текущей сборки добавил
`WRITE_SETTINGS`, `com.pvr.tobactivate.permission.AUTH_CHECK`,
`WRITE_EXTERNAL_STORAGE`, `READ_PHONE_STATE` и `READ_EXTERNAL_STORAGE`.
Последние три появились как legacy implied permissions из XR manifest с очень
старым target SDK; перед выставочным release их следует удалить или явно
обосновать и затем повторить `aapt`-аудит. Этот APK собран, но **не установлен
в Pico**.

## Offline package strategy

WebRTC уже vendored внутри проекта:

```text
Assets/Editor/OfflinePackages~/com.unity.webrtc-3.0.0-pre.8.tgz
SHA-256: 9a36d45121ff6f5cef3e4c77e9f1fb263f88f7567f19ecd9250e8beef9f28d54
```

Проверка архива:

```bash
cd /home/unitree/Unitree_Project/Unity_Projects/Unitree_VR_Controller/Assets/Editor/OfflinePackages~
sha256sum -c com.unity.webrtc-3.0.0-pre.8.tgz.sha256
```

`Packages/manifest.json` ссылается на локальный tarball WebRTC и на абсолютный
локальный Pico SDK:

```json
"com.unity.webrtc": "file:../Assets/Editor/OfflinePackages~/com.unity.webrtc-3.0.0-pre.8.tgz",
"com.unity.xr.picoxr": "file:/home/unitree/Unitree_Project/SDK/Pico/Pico_SDK"
```

Путь Pico SDK намеренно абсолютный. На запасной машине нужно воспроизвести этот
канонический путь либо заранее обновить обе записи `manifest.json` и
`packages-lock.json`, затем снова выполнить online import и offline build test.

До поездки сохранить на отдельный носитель:

- готовый APK и его `.sha256`;
- весь Unity-проект, включая уже заполненный `Library/PackageCache`;
- Pico SDK по каноническому пути;
- Unity Editor `2022.3.44f1` вместе с AndroidPlayer, SDK, NDK и OpenJDK;
- активированную offline-доступную Unity license для выставочной машины;
- исходный WebRTC tarball и checksum;
- `r1_robot_pov`, Python wheelhouse и выставочный `.env`, как описано в
  `docs/robot_pov.md`.

Локальный WebRTC tarball не делает автоматически офлайн-доступными все
транзитивные Unity registry packages. Поэтому после успешного online import
обязательно сохранить project `Library/PackageCache` и выполнить отдельный
build при отключённом WAN. Во время этого теста оставить локальную LAN или
loopback, запустить mock server и убедиться, что Unity ничего не скачивает.

## 4. Editor → APK → Pico

До физического подключения допустимы только следующие шаги:

1. Запустить mock server.
2. Выполнить batch validation.
3. Собрать APK с правильным числовым LAN URL.
4. Проверить размер APK и SHA-256.
5. Проверить browser fallback с ноутбука.

> **STOP 1 — установка APK.** Перед `adb install`, включением Developer Mode /
> USB debugging или физическим подключением Pico остановиться и дождаться
> явного подтверждения пользователя. В текущем этапе эти действия не
> выполняются.

Только после подтверждения пользователя:

```bash
/usr/bin/adb devices -l
/usr/bin/adb install -r \
  /home/unitree/Unitree_Project/Unity_Projects/Unitree_VR_Controller/Builds/RobotPovVideoOnly/RobotPovVideoOnly.apk
/usr/bin/adb shell monkey \
  -p com.ionosrobots.unitree.robotpov \
  -c android.intent.category.LAUNCHER 1
```

Pico и ноутбук должны находиться в одной LAN без guest/client/AP isolation.
Интернет и WAN не нужны. При первом USB-подключении пользователь должен сам
подтвердить RSA fingerprint в очках.

## 5. NO SIGNAL и reconnect tests

После разрешённой установки APK тестировать сначала только на mock:

1. Запустить `./scripts/robot-pov mock --profile low-latency`.
2. Запустить `Unitree Robot POV`; дождаться `LIVE`, движущегося timecode и
   ненулевого FPS.
3. Остановить сервер `Ctrl+C`. Экран обязан перейти в `NO SIGNAL`, не оставляя
   замороженный кадр; причина должна стать `ROBOT POV SERVER UNREACHABLE` или
   сообщением reconnect.
4. Запустить ту же mock-команду снова. APK должен восстановить видео без
   перезапуска, а `RECONNECT` увеличиться.
5. Повторить на изолированном 5 GHz роутере без WAN.
6. Проверить `bad-wifi`, собрав APK с
   `ROBOT_POV_PROFILE=bad-wifi`.
7. Параллельно открыть browser fallback и проверить WebRTC, затем
   `http://<LAN-IP>:8080/?transport=mjpeg&profile=bad-wifi`.

После native connection серверный лог должен содержать диагностическую метку и
H.264 preference:

```bash
cd /home/unitree/Unitree_Project/R1_Teleoperation
rg 'unity-pico-r1|"preferred_codec": "h264"' logs/robot_pov
```

Это подтверждает запрос кодека, но не доказывает аппаратный decode path.
Фактически выбранный кодек, цвет, eye order и OpenGLES3 нужно подтвердить на
реальном Pico визуально и по Android/Unity diagnostics.

Для server-side eye swap без пересборки APK установите в выставочном `.env`:

```dotenv
ROBOT_POV_SWAP_EYES=true
```

Локальные `swapEyes`, `flipVertical`, mono/SBS и world-space параметры также
есть в `RobotPovRuntimeConfig`, но batch build использует безопасные defaults.

> **STOP 2 — реальная камера или робот.** Перед подключением физической камеры,
> запуском camera SDK/RTSP/ROS driver или доступом к самому Unitree R1
> остановиться и дождаться явного подтверждения пользователя. Сначала нужно
> назвать кабель/интерфейс, питание, LAN-параметры и следующую безопасную
> команду. Robot POV не должен запускаться вместе с motor/teleop launch.

После такого подтверждения отдельно проверить:

- переход `NO SIGNAL → LIVE` при остановке и возврате только camera driver;
- mono/SBS, left/right swap, rotation, crop и flip;
- фактический FPS, RTT, jitter, dropped frames и reconnect count;
- `preflight --live` из `docs/robot_pov.md`;
- отсутствие `/cmd_vel`, trajectory и actuator endpoints в процессе viewer.

## Ограничения текущей версии

- На Pico/Android Unity-клиент автоматически использует LAN MJPEG fallback:
  это обходит crash `libwebrtc.so` в `com.unity.webrtc 3.0.0-pre.8` на Pico 4
  Ultra. На desktop сохраняется WebRTC путь. Серверный endpoint —
  `/stream.mjpg?profile=<profile>`.
- Первый APK ещё содержит legacy permissions, добавленные PicoXR/Unity manifest
  merge; перед release требуется их минимизация или документированное
  обоснование и повторная проверка через `aapt`.
- Server URL и профиль записываются в generated scene при сборке; для другой
  площадки рекомендуется новый APK либо стабильный DHCP reservation.
- H.264 — предпочтение, а не гарантия: он должен присутствовать в SDP Unity и
  пройти согласование с aiortc; иначе используется VP8.
- Аппаратное декодирование, цветовой формат, eye order, производительность и
  thermal behavior требуют физического теста Pico.
- Реальный camera endpoint Unitree R1 пока не подтверждён; нельзя подставлять
  API других моделей Unitree без документации или безопасной read-only
  идентификации.
- Native adaptive profile switching пока не реализован. При плохой сети
  используйте заранее собранный `bad-wifi` APK или browser/MJPEG fallback.
