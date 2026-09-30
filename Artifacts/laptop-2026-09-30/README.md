# Снимок материалов ноутбука — 30.09.2026

Большие файлы хранятся через Git LFS. После git lfs pull:
`sha256sum -c SHA256SUMS` из этого каталога.

- apk — все сохранённые APK Unity Builds; текущий USB:
  apk/CombinedDemo/UnitreeR1TelepresenceDemo_usb.apk.
- source-backups — перепакованные исторические снимки исходников.
- backup-supporting-files.tar.gz — сопроводительные материалы старых backup.
- diagnostics.tar.gz — историческая диагностика; не текущие команды/состояние.
- offline — wheels, voice-архивы и части GGUF (условия рядом с моделью).
- inventory.json — исходный workspace до окончательного оформления переноса.
- export-review.json — исключения и редактирования при экспорте.

Нет файлов PC2. Это не полный образ диска и не гарантия побайтового
восстановления старых raw .tar. Полный охват/исключения:
[TRANSFER_STATUS](../../TRANSFER_STATUS.md).
