# Восстановление и резервные копии

Это восстановление файлов, не автоматический запуск физического робота.
Сначала отключите аппаратное управление и прочитайте ../AGENTS.md.
Исходный компьютер использовал /home/unitree/Unitree_Project.

## Клон

```bash
git lfs install
git clone https://github.com/IVVA-robo/Unitree_Project.git
cd Unitree_Project
git lfs pull
git lfs fsck
cd Artifacts/laptop-2026-09-30
sha256sum -c SHA256SUMS
```

На слабом канале сначала можно скачать только исходники:

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/IVVA-robo/Unitree_Project.git
cd Unitree_Project
git lfs pull --include='SDK/**,ROS2_WS/**,Legacy_Robotics/**,Unity_Projects/**,Artifacts/laptop-2026-09-30/apk/CombinedDemo/UnitreeR1TelepresenceDemo_usb.apk'
```

До полной загрузки проверка всех SHA256SUMS ожидаемо не пройдёт.
Файл на несколько строк с version https://git-lfs.github.com/spec/v1 —
указатель, а не APK/модель/архив.

## Зависимости и сборка

Ubuntu 22.04, ROS 2 Humble, Python 3.10, colcon/CMake, Qt/PyQt5,
ADB; Unity 2022.3.44f1 с Android build support и Pico SDK 3.4.0.
Файлы wheelhouse помогают восстановить Python-видео зависимости, но не
заменяют пакеты ОС, ROS, Android SDK, Unity Editor и лицензию Unity.
Полное развёртывание на чистом ноутбуке без заранее установленных зависимостей
ещё не аттестовано.

Если путь/пользователь другой, проверить operator_panel.json, systemd/desktop,
shell-wrapper пути и Unity Packages/manifest.json. Калибровки зависят от
конкретного оператора/робота; наличие сохранённого JSON не заменяет калибровку.

```bash
cd /home/unitree/Unitree_Project/R1_Teleoperation
source /opt/ros/humble/setup.bash
make r1-teleoperation-build
```

Deployment/Laptop содержит справочные копии установленных файлов.
Не копируйте их поверх работающего окружения и не выполняйте enable/restart
служб без проверки. Сохранённый профиль имеет allow_live=true, dry_run=false.

## Офлайн-артефакты

В Artifacts/laptop-2026-09-30/offline:

- robot-pov-wheelhouse.tar.gz — Python wheels; распаковывать в новую папку,
  затем устанавливать по requirements нужного компонента с --no-index.
- voice-models.tar.gz — полный Vosk zip и подготовительный bundle.
- google_gemma-3-4b-it-Q4_K_M.gguf — 512 MiB части и manifest.json.
  Для восстановления предусмотрен Tools/GitHub/restore_model.py:
  он проверяет части, не перезаписывает существующий файл и проверяет полный SHA256.

```bash
python3 Tools/GitHub/restore_model.py \
  Artifacts/laptop-2026-09-30/offline/google_gemma-3-4b-it-Q4_K_M.gguf \
  /путь/к/новой/google_gemma-3-4b-it-Q4_K_M.gguf
```

Сначала ознакомьтесь с лицензиями моделей. Эти веса не нужны для USB-ходьбы:
это сохранённый материал голосового/офлайн-подпроекта.

## Старые рабочие версии

source-backups содержит перепакованные архивы до/после этапов разработки.
Сначала tar -tf, затем распаковка в **новый** временный каталог и сравнение.
Не распаковывать поверх работающего проекта. Архив не должен автоматически
устанавливать службы, запускать скрипты или снимать защитные блокировки.

diagnostics.tar.gz содержит исторические логи/протоколы, а не актуальные
команды запуска. Полезные итоговые отчёты отдельно доступны в Docs/Test_Reports.
Пути внутри отчётов привязаны к исходной структуре logs.

Локальные оригиналы архивов и предшествующий Git bundle описаны в
[TRANSFER_STATUS](../TRANSFER_STATUS.md). Если они недоступны, использовать
опубликованные перепакованные копии с их новыми контрольными суммами.

## Проверки после восстановления

Сначала статический просмотр настроек, импорты/сборка, unit/mocks в отдельной
сети/ROS-домене. Полный r1-offline-commissioning-check не запускать поверх
активного управляющего графа. После программной проверки оператор заново
подтверждает физические условия; только затем допускаются стойка/RUN.
