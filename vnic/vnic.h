#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/gfp.h>
#include <asm/page.h>
#include <linux/kthread.h>
#include <linux/delay.h>

#define DRV_NAME "vnic"

#ifdef FPGA
    #include <asm/kdasics.h>
#else
    #include "xdma.h"
#endif

/*
    share mem layout
    totally 128MB, 64MB for fpga and 64MB for host
    fpga tx == host rx, fpga rx == host tx
    +---------------------------+ base (0xfc000000)
    |         fpga tx tail        |
    |           64-bit            |
    +---------------------------+ base + 8B (0xfc000008)
    |         fpga tx head        |
    |           64-bit            |
    +---------------------------+ base + 16B (0xfc000010)
    |     fpga tx metadata fifo   |
    |          32B * 1024         |
    +---------------------------+ base + 16KB (0xfc004000)
    |       fpga tx data fifo     |
    |          4KB * 1024         |
    +---------------------------+ base + 64MB (0x100000000)
    |         host tx tail        |
    |           64-bit            |
    +---------------------------+ base + 64MB + 8B (0x100000008)
    |         host tx head        |
    |           64-bit            |
    +---------------------------+ base + 64MB + 16B (0x100000010)
    |       host tx metadata fifo |
    |          32B * 1024         |
    +---------------------------+ base + 64MB + 16KB (0x100004000)
    |       host tx data fifo     |
    |          4KB * 1024         |
    +---------------------------+ base + 128MB (0x104000000)
*/

/* metadata format
static struct metadata {
    uint64_t buf;
    uint64_t len;
    uint64_t valid;
    uint64_t reserved;
} medata;
*/

#define FPGA_TX_TAIL_OFFSET 0x0
#define FPGA_TX_HEAD_OFFSET 0x8
#define FPGA_TX_MD_FIFO_OFFSET 0x10
#define FPGA_TX_D_FIFO_OFFSET (1 << 14) // 16KB
#define HOST_TX_TAIL_OFFSET (1 << 26) // 64MB
#define HOST_TX_HEAD_OFFSET (1 << 26) + 0x8
#define HOST_TX_MD_FIFO_OFFSET (1 << 26) + 0x10
#define HOST_TX_D_FIFO_OFFSET (1 << 26) + (1 << 14) // 64MB + 16KB

#ifdef FPGA
    #define TX_TAIL_OFFSET FPGA_TX_TAIL_OFFSET
    #define TX_HEAD_OFFSET FPGA_TX_HEAD_OFFSET
    #define TX_MD_FIFO_OFFSET FPGA_TX_MD_FIFO_OFFSET
    #define TX_D_FIFO_OFFSET FPGA_TX_D_FIFO_OFFSET
    #define RX_TAIL_OFFSET HOST_TX_TAIL_OFFSET
    #define RX_HEAD_OFFSET HOST_TX_HEAD_OFFSET
    #define RX_MD_FIFO_OFFSET HOST_TX_MD_FIFO_OFFSET
    #define RX_D_FIFO_OFFSET HOST_TX_D_FIFO_OFFSET
#else
    #define SHARE_MEM_BASE 0xfc000000 // FPGA reserved memory base address
    #define TX_TAIL_OFFSET HOST_TX_TAIL_OFFSET
    #define TX_HEAD_OFFSET HOST_TX_HEAD_OFFSET
    #define TX_MD_FIFO_OFFSET HOST_TX_MD_FIFO_OFFSET
    #define TX_D_FIFO_OFFSET HOST_TX_D_FIFO_OFFSET
    #define RX_TAIL_OFFSET FPGA_TX_TAIL_OFFSET
    #define RX_HEAD_OFFSET FPGA_TX_HEAD_OFFSET
    #define RX_MD_FIFO_OFFSET FPGA_TX_MD_FIFO_OFFSET
    #define RX_D_FIFO_OFFSET FPGA_TX_D_FIFO_OFFSET
#endif

#define MAX_DESC 1024
#define MAX_PACKET_SIZE (1 << 12)

static bool vnic_opened = false;

static void __iomem *share_mem_virt;
static uint64_t *tx_tail;
static uint64_t *tx_head;
static uint64_t *tx_md;
static void *tx_data;
static uint64_t *rx_tail;
static uint64_t *rx_head;
static uint64_t *rx_md;
static void *rx_data;

static struct net_device *vnic_dev;
static struct task_struct *polling_thread;

static void vnic_setup(struct net_device *dev);
static int vnic_open(struct net_device *dev);
static int vnic_close(struct net_device *dev);
static netdev_tx_t vnic_start_xmit(struct sk_buff *skb, struct net_device *dev);
static int vnic_poll_rx(void *data);

static uint64_t vnic_read_share_mem(uint64_t *addr)
{
    #ifdef FPGA
        return *addr;
    #else
        return vnic_xdma_read((uint64_t)addr);
    #endif
}

static void vnic_write_share_mem(uint64_t *addr, uint64_t val)
{
    #ifdef FPGA
        *addr = val;
    #else
        vnic_xdma_write((uint64_t)addr, val);
    #endif
}

static void vnic_memcpy_to_share_mem(uint64_t addr, void *data, size_t len)
{
    #ifdef FPGA
        memcpy((void*)addr, data, len);
    #else
        vnic_xdma_memcpy_to_share_mem(addr, data, len);
    #endif
}

static void vnic_memcpy_from_share_mem(uint64_t addr, void *data, size_t len)
{
    #ifdef FPGA
        memcpy(data, (void*)addr, len);
    #else
        vnic_xdma_memcpy_from_share_mem(addr, data, len);
    #endif
}

static int vnic_init_share_mem(void)
{
    #ifdef FPGA
        uint64_t phys_addr;
        uint32_t size;
        struct device_node *np;
        struct resource res;

        np = of_find_node_by_name(NULL, "my_reserved");
        if (!np) {
            pr_err("Device tree node 'my_reserved' not found\n");
            return -ENODEV;
        }
        if (of_address_to_resource(np, 0, &res)) {
            pr_err("Failed to get resource\n");
            return -EINVAL;
        }
        phys_addr = res.start;
        size = resource_size(&res);

        share_mem_virt = ioremap(phys_addr, size);
        if (!share_mem_virt) {
            pr_err("ioremap failed\n");
            return -ENOMEM;
        }
        memset(share_mem_virt, 0x0, size);
        pr_info("Reserved memory mapped at virtual address: 0x%llx -> 0x%llx\n",
                (uint64_t)phys_addr, share_mem_virt);
    #else
        share_mem_virt = (void *)SHARE_MEM_BASE;
        pr_info("fpga reserved memory at fpga physical address: 0x%llx\n",
                (uint64_t)share_mem_virt);
    #endif

    tx_tail = (uint64_t *)(share_mem_virt + TX_TAIL_OFFSET);
    tx_head = (uint64_t *)(share_mem_virt + TX_HEAD_OFFSET);
    tx_md = (uint64_t *)(share_mem_virt + TX_MD_FIFO_OFFSET);
    tx_data = (void *)(share_mem_virt + TX_D_FIFO_OFFSET);
    rx_tail = (uint64_t *)(share_mem_virt + RX_TAIL_OFFSET);
    rx_head = (uint64_t *)(share_mem_virt + RX_HEAD_OFFSET);
    rx_md = (uint64_t *)(share_mem_virt + RX_MD_FIFO_OFFSET);
    rx_data = (void *)(share_mem_virt + RX_D_FIFO_OFFSET);

    return 0;
}

static int vnic_tx_full(void)
{
    return (vnic_read_share_mem(tx_tail) + 1) % MAX_DESC == vnic_read_share_mem(tx_head);
}

static int vnic_rx_empty(void)
{
    return vnic_read_share_mem(rx_tail) == vnic_read_share_mem(rx_head);
}

static uint64_t *vnic_get_tx_buf_addr(bool is_metadata)
{
    if (vnic_tx_full()) {
        pr_err("TX FIFO is full, cannot get metadata\n");
        return NULL;
    }
    if (is_metadata) {
        uint64_t *md_buf = tx_md + (vnic_read_share_mem(tx_tail) % MAX_DESC);
        if (md_buf[2] == 1) {
            pr_err("Metadata buffer is already in use\n");
            return NULL; // Metadata buffer is already in use
        }
        return md_buf;
    } else {
        uint64_t *data_buf = (uint64_t *)(tx_data + (vnic_read_share_mem(tx_tail) % MAX_DESC) * MAX_PACKET_SIZE);
        return data_buf;
    }
}

static int vnic_send_packet(struct sk_buff *skb)
{
    pr_info("send packet: %p\n", skb);
    
    if (vnic_tx_full()) {
        pr_err("TX FIFO is full, cannot send packet\n");
        return NETDEV_TX_BUSY; // No space in TX FIFO
    }

    uint64_t *dst_buf = vnic_get_tx_buf_addr(false);
    uint64_t *md_buf = vnic_get_tx_buf_addr(true);
    if (!dst_buf || !md_buf) {
        pr_err("Failed to get TX buffers\n");
        return NETDEV_TX_BUSY; // No space in TX FIFO
    }
    vnic_memcpy_to_share_mem((uint64_t)dst_buf, skb->data, skb->len);
    vnic_write_share_mem(md_buf, (uint64_t)dst_buf); // Set buffer address
    vnic_write_share_mem(md_buf + 1, skb->len);      // Set length of the packet
    vnic_write_share_mem(md_buf + 2, 1);             // Set valid flag
    vnic_write_share_mem(md_buf + 3, 0);             // Reserved
    vnic_write_share_mem(tx_tail, (vnic_read_share_mem(tx_tail) + 1) % MAX_DESC); // Update tail pointer
    vnic_dev->stats.tx_packets++;
    vnic_dev->stats.tx_bytes += skb->len;
    return NETDEV_TX_OK;
}

static int vnic_recv_packet(void)
{
    if (vnic_rx_empty()) {
        pr_info("RX FIFO is empty, no packet to receive\n");
        return 0; // No packet to receive
    }

    uint64_t *md_buf = rx_md + (vnic_read_share_mem(rx_head) % MAX_DESC);
    if (!md_buf) {
        pr_err("Failed to get RX metadata buffer\n");
        return -EIO; // Error in getting metadata buffer
    }

    BUG_ON(md_buf[2] != 1); // Ensure metadata is valid
    uint64_t buf = md_buf[0]; // Get buffer address
    uint64_t len = md_buf[1]; // Get length of the packet

    struct sk_buff *skb = netdev_alloc_skb(vnic_dev, len + NET_IP_ALIGN);
    if (!skb) {
        pr_err("Failed to allocate skb\n");
        return -ENOMEM;
    }
    skb_reserve(skb, NET_IP_ALIGN); // Align the skb data
    skb_put(skb, len); // Set the length of the skb
    skb->protocol = eth_type_trans(skb, vnic_dev);
    skb->dev = vnic_dev;
    skb->tstamp = ktime_get_real();
    vnic_memcpy_from_share_mem(buf, skb->data, len);

    netif_rx(skb);

    vnic_write_share_mem(md_buf + 2, 0);
    vnic_write_share_mem(rx_head, (vnic_read_share_mem(rx_head) + 1) % MAX_DESC);
    vnic_dev->stats.rx_packets++;
    vnic_dev->stats.rx_bytes += len;

    return len;
}

static void vnic_deinit_share_mem(void)
{
    #ifdef FPGA
        iounmap(share_mem_virt);
    #endif
}