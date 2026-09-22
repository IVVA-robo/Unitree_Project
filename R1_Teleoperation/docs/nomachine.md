# NoMachine для R1: безопасная памятка

Этот документ относится только к удалённому рабочему столу PC2 робота. Он не
запускает ROS, телеприсутствие или команды приводам.

## Проверенная совместимая связка

- R1 PC2: `192.168.123.164`, NX TCP port `4000`.
- Ноутбук на robot-facing Ethernet: `192.168.123.162/24`.
- Клиент NoMachine: portable Player **9.7.3**.
- Сервер на PC2: **8.9.1**.

Portable-клиент уже находится в проекте:

```text
/home/unitree/Unitree_Project/Tools/NoMachine/9.7.3-portable/
```

Запуск:

```bash
/home/unitree/Unitree_Project/Tools/NoMachine/9.7.3-portable/connect-r1.sh
```

Пакет проверен как Debian `amd64`, версия `9.7.3-1`, SHA-256:
`81e0f8b48c7a4d3c16dac0401b9d188353e4a237498284d267a05c5176fd95fc`.
Старый официальный URL 9.7.3 сейчас перенаправляет на главную страницу; не
следует скачивать файл, если MIME-тип не `application/vnd.debian.binary-package`.

## Если подключение даёт чёрный экран

1. В Player выберите `Machines`, затем правой кнопкой по R1 откройте
   **Server admin**. Это read-only проверка, не требующая SSH-команд.
2. Проверьте, что сервер отвечает на порт 4000:

   ```bash
   nc -vz -w 2 192.168.123.164 4000
   ```

3. Не выполняйте вслепую на R1:

   ```bash
   sudo systemctl stop display-manager
   sudo /etc/NX/nxserver --restart
   ```

   Эти команды могут остановить vendor GUI и оборвать текущие сессии. Они нужны
   только после согласованного плана восстановления headless X-сервера.

Причина чёрного экрана обычно в том, что NoMachine подключается к физическому
`:0`, а GDM/X11 не получил активный видеовыход (headless/EDID). Безопасные
варианты исправления: временный HDMI/dummy-адаптер, отдельный headless Xorg или
настройка `CreateDisplay`/`DisplayOwner` сервером с подтверждёнными правами.
Wayland physical-desktop без видеовыхода не считается поддержанным режимом.
Официальные справочные статьи: [Headless Linux](https://kb.nomachine.com/AR03P00973),
[server configuration](https://kb.nomachine.com/AR07Q01037) и
[удаление старых версий](https://kb.nomachine.com/AR08C00250).

## Граница безопасности

NoMachine не является каналом управления приводами и не заменяет штатный
E-stop. Пока не проверены IP, версия клиента и экран, оставляйте deadman
отпущенным и используйте только локальные ROS/Gazebo dry-run сценарии.
