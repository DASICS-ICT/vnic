# S2C VU19P 虚拟网卡硬件参考

NexST 标签 `s2c-vu19p-vnic-coherent-v1`（`7959856`）记录了生成 19P bit 时使用的 SoC。本目录的 Tcl 与该版本的 `shell/s2c_vu19p/fpga/scripts/xiangshan.tcl` 逐字节一致。[板级 Tcl](xiangshan.tcl) · [NexST 硬件版本](https://github.com/DASICS-ICT/nexst/tree/s2c-vu19p-vnic-coherent-v1)

| XDMA 接口 | 19P 上的连接和用途 |
|---|---|
| `M_AXI_BYPASS` | 主机 BAR2 经 `axi_ic_dma`、`axi_ic_dma_reg_slice` 和 `axi_ic_dma_cross_domain` 接至 `u_role/s_axi_dma`，访问与香山 CPU 保持一致性的共享内存。 |
| `M_AXI` | 经 `axi_ic_ddr_mem_pcie` 所在的 DDR 通路接至 DDR4 MIG，供常规 H2C/C2H 传输使用。 |

BAR2 的 256 MiB 窗口从 FPGA 地址 `0xf0000000` 开始。FPGA 发送队列和主机发送队列的 BAR2 偏移分别是 `0x08000000` 和 `0x0c000000`，与 VCU128 使用的布局相同。

19P 已生成 bit，布线后时序报告给出的 WNS 为 +0.086 ns。本版本尚无 19P 实板双向 ping 结果。软件配置和已验证的 VCU128 测试步骤见 [VCU128 使用说明](../vcu128/README.md)。
