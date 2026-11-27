#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 4 ]]; then
    echo "Usage: $0 LINUX_SRC OPENSBI_SRC ROOTFSIMG OUTPUT_DIR" >&2
    exit 2
fi
board=$(cd "$(dirname "$0")" && pwd)
linux=$(realpath -e "$1")
sbi=$(realpath -e "$2")
rootfs=$(realpath -e "$3")
mkdir -p "$4/linux"
out=$(realpath "$4")
export ARCH=riscv CROSS_COMPILE=${CROSS_COMPILE:-riscv64-unknown-linux-gnu-}
jobs=${JOBS:-8}

python3 - "$board" "$rootfs" "$out" <<'PY'
from pathlib import Path
from string import Template
import sys

board, rootfs, out = map(Path, sys.argv[1:])
listing = Template((board / "initramfs.list.in").read_text()).substitute(BOARD=board, ROOTFS=rootfs)
for line in listing.splitlines():
    if line.startswith("file "):
        path = Path(line.split()[2])
        if not path.is_file():
            sys.exit(f"Missing initramfs input: {path}")
(out / "initramfs.list").write_text(listing)
PY
dtc -I dts -O dtb -o "$out/XSTop-vnic.dtb" "$board/XSTop-vnic.dts"
cp "$board/kernel.config" "$out/linux/.config"
"$linux/scripts/config" --file "$out/linux/.config" --enable VNIC --disable IP_PNP \
    --set-str INITRAMFS_SOURCE "$out/initramfs.list" --set-str LOCALVERSION '-vnic-test' \
    --disable LOCALVERSION_AUTO
make -C "$linux" O="$out/linux" olddefconfig
grep -qx 'CONFIG_VNIC=y' "$out/linux/.config"
make -C "$linux" O="$out/linux" -j"$jobs" Image
make -C "$sbi" O="$out/opensbi" PLATFORM=ict OPENSBI_PAYLOAD="$out/linux/arch/riscv/boot/Image" \
    FW_FDT_PATH="$out/XSTop-vnic.dtb" SERVE_PLAT=h HART_COUNT=1 RV_TARGET=xiangshan WITH_SM=n -j"$jobs"
cp "$out/opensbi/platform/ict/firmware/fw_payload.bin" "$out/fw_payload.bin"
