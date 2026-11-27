# VCU128 虚拟网卡使用说明

主机和 FPGA 上的 Linux 各使用一个 `vnic0` 接口，通过共享内存交换以太网帧。以下步骤从板端 Linux 已启动、主机已枚举 XDMA PCIe 端点开始。

## 板端配置

板端 Linux 使用 `DASICS-ICT/riscv-linux` 的 `5.16.0-nexst-vnic` 分支，启用 `CONFIG_VNIC=y`。驱动按名称查找 `my_reserved` 节点。设备树须在 `/reserved-memory` 下为它预留从 `0xf8000000` 开始的 128 MiB，并设置 `no-map`。主机的 BAR2 bypass 连接 FPGA CPU 的 coherent DMA 接口，BAR2 偏移 0 映射到 FPGA 地址 `0xf0000000`。[设备树示例](XSTop-vnic.dts)

| 共享队列 | FPGA 物理起始地址 | 主机 BAR2 偏移 |
|---|---|---|
| FPGA 发送，主机接收 | `0xf8000000` | `0x08000000` |
| 主机发送，FPGA 接收 | `0xfc000000` | `0x0c000000` |

先在 **FPGA 的 Linux shell** 中配置接口。板端驱动在初始化时会清空共享队列，因此应等板端就绪后再启用主机接口。

```sh
ip addr add 192.0.2.2/30 dev vnic0
ip link set dev vnic0 up
ip addr show dev vnic0
```

使用本目录的示例镜像时，看到 `VNIC_DUT_READY` 就表示上述配置已完成，可以直接配置主机。

## 主机配置

主机需安装与运行内核匹配的内核头文件和模块构建工具。XDMA 接口头文件已随本仓库提供。从仓库根目录先编译专用 `xdma.ko`，再用它生成的 `Module.symvers` 编译 `vnic.ko`。

```bash
XDMA="$PWD/xdma/xdma"
make -C "$XDMA"
make -C kernel KBUILD_EXTRA_SYMBOLS="$XDMA/Module.symvers"
```

在板端就绪后，在**主机**上依次加载模块并配置地址。如果专用 XDMA 模块已经加载，跳过第一行。

```bash
sudo insmod "$XDMA/xdma.ko"
sudo insmod kernel/vnic.ko
sudo ip addr add 192.0.2.1/30 dev vnic0
sudo ip link set dev vnic0 up
```

## 验证双向通信

| 执行位置 | 命令 |
|---|---|
| 主机 | `ping -I vnic0 -c 4 -W 5 192.0.2.2` |
| FPGA Linux | `ping -I vnic0 -c 4 -W 5 192.0.2.1` |

两端都应收到对端应答。VCU128 实板验证中，两个方向均为 4 包发送、4 包收到、零丢包。
