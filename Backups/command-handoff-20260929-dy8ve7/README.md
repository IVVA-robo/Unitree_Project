# Before command handoff repair — 2026-09-29

`source-before.tar` contains the pre-edit usb_link, exhibition, operator_panel,
scripts, tests, docs/usb_mode_ru.md and config/operator_panel.json from
`/home/unitree/Unitree_Project/R1_Teleoperation`.

SHA256: `a4fc69e177a699e8436b389c6f0fcdd12cf8c239811c2eb9d02b9e9a5da3e84d`.

This is a scoped source/configuration backup, not a complete robot/Unity snapshot.
Earlier full backups remain in the adjacent pre-usb and usb-working-before-live
directories. Extract into a new directory and compare before restoring files;
do not overwrite later user work blindly.

The repair removes the need for a global ExecStop callback on an external USB
RUN service, waits for the previous transport owner's cleanup before handoff,
and keeps the reviewed robot STOP/KILL path and explicit physical permissions.
