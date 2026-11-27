# VCU128 上复现主机与 FPGA 虚拟网卡

在连接 VCU128 的 Linux 主机上运行命令。将三个绝对路径分别设为本仓库、已构建的 xdma-pcie-util，以及存有 `system.bit`、`bootrom.bin`、`fw_payload.bin` 的目录。后面两个主机终端使用相同的变量。

```bash
export VNIC=/path/to/vnic
export UTIL=/path/to/xdma-pcie-util
export IMAGES=/path/to/vcu128-images
```

## 1. 确认 bit 和软件

实测组合为 NexST `1fba14d` 的单核 `vcu128_orig`、Vivado 2020.2、xdma-pcie-util `54b5019`，以及主机内核 `6.12.24-xdma`。主机通过 H2C 把启动镜像送到 DDR 的 `0x80000000`。Linux 启动后，两端通过 BAR2 bypass 和 FPGA CPU 的 coherent DMA 接口访问共享队列。

| 实测产物 | SHA-256 |
|---|---|
| system.bit | `acb0297010579c7f0b9aedb9988e08f790a57eb0e74c73bf0af3ec2f1d6f89a9` |
| bootrom.bin | `a3360212cf7bb784119c1719c9258362f8567803601b2359aee2c104864fa994` |
| fw_payload.bin | `fe4507036ba836838745b9e0087c1eb9d24f7ef93f489ebf9bb5dbec05351664` |

bit 来自 NexST 构建目录中的 `work_farm/hw_plat/target_nanhu-g_proto_vcu128/system.bit`，BootROM 使用同平台跳到 `0x80000000` 的 12 字节版本。已有上述产物时可直接进入第 2 步。需要重建 Linux 镜像时，使用 DASICS-ICT/riscv-linux 的 `5.16.0-nexst-vnic` 分支提交 `586fae475420`、DASICS-ICT/OpenSBI 的 `ac061bbc8ce5`，以及 NexST 准备好的 RISC-V `rootfsimg`（BusyBox 和动态库清单见同目录的 `initramfs.list.in`）。将 RISC-V GCC 加入 PATH，并安装 dtc 和 Python 3。

```bash
"$VNIC/boards/vcu128/build-image.sh" /path/to/riscv-linux /path/to/OpenSBI \
    /path/to/rootfsimg "$PWD/vcu128-build"
cp "$PWD/vcu128-build/fw_payload.bin" "$IMAGES/"
```

脚本打开 `CONFIG_VNIC=y`，将设备树和带静态 IP 的 BusyBox initramfs 打入镜像。设备树的 `my_reserved@f8000000` 预留 128 MiB。FPGA 发送队列位于 BAR2 偏移 `0x08000000`，主机发送队列位于 `0x0c000000`。重新构建的镜像含内核构建信息，哈希可能不同。

## 2. 烧写并加载主机驱动

在板卡空闲时用 Vivado 烧写 bit；板卡已运行上述 bit 时可直接检查 PCIe 枚举。

```bash
cd "$UTIL"
vivado -mode batch -source scripts/program_fpga.tcl -tclargs xcvu37p_0 "$IMAGES/system.bit"
lspci -Dnn -d 10ee:903f
```

如果没有枚举到端点，先执行 `echo 1 | sudo tee /sys/bus/pci/rescan` 再查。若仍未出现，本次实测是在保持板卡供电的情况下，协调重启连板主机后枚举成功的。

主机需要与运行内核匹配、提供 `xdma_read64_bar` 和 `xdma_write64_bar` 的专用模块。以下步骤假设从未加载 XDMA 的状态开始，并且当前用户对 XDMA 设备节点有读写权限。若已经加载同一专用模块，可跳过 `insmod`。已有实验室配套的 `xdma.ko` 和 `Module.symvers` 时，可把 `XDMA` 指向其目录并跳过第一条 make。

```bash
XDMA="$VNIC/xdma/xdma"
make -C "$XDMA"
make -C "$VNIC/kernel" KBUILD_EXTRA_SYMBOLS="$XDMA/Module.symvers"
sudo insmod "$XDMA/xdma.ko"
ls -l /dev/xdma0_{user,h2c_0,bypass}
```

## 3. 启动 FPGA，验证两个方向

**主机终端 A** 运行普通加载脚本，从 H2C 启动后会自动进入 FPGA 串口。

```bash
cd "$UTIL"
bash -e scripts/load_and_run.sh xdma0 "$IMAGES/bootrom.bin" "$IMAGES/fw_payload.bin"
```

等待串口出现 `VNIC_DUT_READY`。这表示板端已初始化队列，且 `vnic0` 已配置 `192.0.2.2/30`。

**主机终端 B** 加载主机 vNIC 模块并测试主机到 FPGA。

```bash
sudo insmod "$VNIC/kernel/vnic.ko"
sudo ip addr add 192.0.2.1/30 dev vnic0
sudo ip link set dev vnic0 up
ping -I vnic0 -c 4 -W 5 192.0.2.2
```

**回到终端 A 的 FPGA shell**，验证 FPGA 到主机。

```sh
ping -I vnic0 -c 4 -W 5 192.0.2.1
```

2026-11-27 实板测试中，两个方向均发送 4 包、收到 4 包。主机发起的平均 RTT 为 25.2 ms，FPGA 发起为 12.9 ms。串口用 Ctrl+反斜杠退出，FPGA 上的 Linux 和网络接口可继续使用。
