#!/usr/bin/env bash
# Launch the system Qt application without GTK plugin paths inherited from Snap.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
app="$root/build-release/inertial_navigation"
if [[ ! -x "$app" ]]; then
    printf 'Приложение не собрано: %s\n' "$app" >&2
    printf 'Соберите: cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release && cmake --build build-release\n' >&2
    exit 1
fi
exec env -u GTK_PATH -u GTK_EXE_PREFIX -u GTK_IM_MODULE_FILE \
    -u GIO_MODULE_DIR -u GTK_MODULES \
    QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}" "$app" "$@"
