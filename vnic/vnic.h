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
#include <linux/ip.h>

#define DRV_NAME "vnic"

uint64_t fail_count = 0;

bool debug = true;

#define VNIC_DBG(fmt, ...) \
    do { \
        if (debug) { \
            pr_info(DRV_NAME ": " fmt, ##__VA_ARGS__); \
        } \
    } while (0)

#ifdef FPGA
    #include <asm/kdasics.h>
#else
    #include "xdma.h"
#endif

/*
    share mem layout
    totally 128MB, 64MB for fpga and 64MB for host
    fpga tx == host rx, fpga rx == host tx
    +---------------------------+ base (0xf8000000)
    |         fpga tx tail        |
    |           64-bit            |
    +---------------------------+ base + 8B (0xf8000400)
    |         fpga tx head        |
    |           64-bit            |
    +---------------------------+ base + 16B (0xf8000800)
    |     fpga tx metadata fifo   |
    |          64B * 1024         |
    +---------------------------+ base + 16KB (0xf8020000)
    |       fpga tx data fifo     |
    |          4KB * 1024         |
    +---------------------------+ base + 64MB (0xfc000000)
    |         host tx tail        |
    |           64-bit            |
    +---------------------------+ base + 64MB + 8B (0xfc000400)
    |         host tx head        |
    |           64-bit            |
    +---------------------------+ base + 64MB + 16B (0xfc000800)
    |       host tx metadata fifo |
    |          64B * 1024         |
    +---------------------------+ base + 64MB + 16KB (0xfc020000)
    |       host tx data fifo     |
    |          4KB * 1024         |
    +---------------------------+ base + 128MB (0x100000000)
*/

/* metadata format
static struct metadata {
    uint64_t buf;
    uint64_t len;
    uint64_t valid;
    uint64_t reserved;
} metadata;
*/

#define FPGA_TX_TAIL_OFFSET 0x0ULL
#define FPGA_TX_HEAD_OFFSET 0x400ULL
#define FPGA_TX_MD_FIFO_OFFSET 0x800ULL
#define FPGA_TX_D_FIFO_OFFSET 0x20000ULL
#define HOST_TX_TAIL_OFFSET 0x4000000ULL
#define HOST_TX_HEAD_OFFSET (HOST_TX_TAIL_OFFSET + FPGA_TX_HEAD_OFFSET)
#define HOST_TX_MD_FIFO_OFFSET (HOST_TX_TAIL_OFFSET + FPGA_TX_MD_FIFO_OFFSET)
#define HOST_TX_D_FIFO_OFFSET (HOST_TX_TAIL_OFFSET + FPGA_TX_D_FIFO_OFFSET)

#define XDMA_BAR_BASE 0x50000000UL

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
    //#define SHARE_MEM_BASE 0xf8000000 // FPGA reserved memory base address
    #define SHARE_MEM_BASE (XDMA_BAR_BASE + 0x8000000UL)
    #define TX_TAIL_OFFSET HOST_TX_TAIL_OFFSET
    #define TX_HEAD_OFFSET HOST_TX_HEAD_OFFSET
    #define TX_MD_FIFO_OFFSET HOST_TX_MD_FIFO_OFFSET
    #define TX_D_FIFO_OFFSET HOST_TX_D_FIFO_OFFSET
    #define RX_TAIL_OFFSET FPGA_TX_TAIL_OFFSET
    #define RX_HEAD_OFFSET FPGA_TX_HEAD_OFFSET
    #define RX_MD_FIFO_OFFSET FPGA_TX_MD_FIFO_OFFSET
    #define RX_D_FIFO_OFFSET FPGA_TX_D_FIFO_OFFSET
#endif

#define MAX_DESC 0x400ULL
#define DESC_SIZE 0x40ULL
#define MAX_PACKET_SIZE 0x1000ULL

#define X86_MAC "\x11\x22\x33\x44\x55\x66"
#define RV_MAC "\x01\x02\x03\x04\x05\x06"
#ifdef FPGA
    #define SRC_MAC RV_MAC
    #define DST_MAC X86_MAC
#else
    #define SRC_MAC X86_MAC
    #define DST_MAC RV_MAC
#endif

static bool vnic_opened = false;

static void __iomem *share_mem_virt;
static volatile uint64_t *tx_tail;
static volatile uint64_t *tx_head;
static volatile void *tx_md;
static volatile void *tx_data;
static volatile uint64_t *rx_tail;
static volatile uint64_t *rx_head;
static volatile void *rx_md;
static volatile void *rx_data;

spinlock_t tx_lock;
spinlock_t rx_lock;
spinlock_t read_lock;
spinlock_t write_lock;

static struct net_device *vnic_dev;
static struct task_struct *polling_thread;

static void vnic_setup(struct net_device *dev);
static int vnic_open(struct net_device *dev);
static int vnic_close(struct net_device *dev);
static netdev_tx_t vnic_start_xmit(struct sk_buff *skb, struct net_device *dev);
static int vnic_poll_rx(void *data);

#ifdef FPFA
#else
static int vnic_get_sharemem_offset(uint64_t *addr)
{
    uint64_t offset = (uint64_t)addr - (uint64_t)XDMA_BAR_BASE;
    if (offset >= (1 << 28)) { // Check if the address is within the valid range
        pr_err("Address 0x%llx is out of bounds\n", (unsigned long long)addr);
        return -EINVAL;
    }
    return offset;
}
#endif

static void vnic_flush_cache(void)
{
    #ifdef FPGA
        return;
        uint64_t cache_block_size = 64;
        uint64_t cache_sets = 128;
        uint64_t cache_size = 32768;
    #else
        uint64_t cache_block_size = 64;
        uint64_t cache_sets = 64;
        uint64_t cache_size = 49152;
    #endif
    uint64_t cache_associativity = cache_size / (cache_block_size * cache_sets);
    uint8_t *buffer = kmalloc(2 * cache_size, GFP_KERNEL);
    if (!buffer) {
        pr_err("Failed to allocate memory for cache flush\n");
        return;
    }
    volatile uint8_t *v_buffer = buffer;
    int set, way;
    volatile uint8_t *ptr;
    for (set = 0; set < cache_sets; set++) {
        for (way = 0; way < cache_associativity; way++) {
            // 计算当前目标地址：基址 + 块偏移 + 组偏移 + 路偏移
            ptr = v_buffer + (set * cache_block_size) + (way * cache_sets * cache_block_size);
            *ptr = 1;
        }
    }
    kfree(buffer);
}

static uint64_t vnic_read_share_mem(uint64_t *addr)
{
    uint32_t flags;
    spin_lock_irqsave(&read_lock, flags);
    uint64_t val;
    #ifdef FPGA
    /*
        uint32_t lo = ioread32((u8 __iomem *)addr);
	    uint32_t hi = ioread32((u8 __iomem *)addr + 4);
	    val = (((uint64_t)hi & 0xFFFFFFFFUL) << 32) | (lo & 0xFFFFFFFFUL);
    */
        vnic_flush_cache();
        val = readq((void __iomem *)addr);
        mb();
    #else
        int off = vnic_get_sharemem_offset(addr);
        if (off < 0) {
            return -EINVAL; // Invalid address
        }
        //VNIC_DBG("vnic_read_share_mem: off = 0x%lx, addr = 0x%llx\n", 
        //    (unsigned long)off, (unsigned long long)addr);
        vnic_flush_cache();
        val = vnic_xdma_read((uint32_t)off);
        mb();
        if (val == (uint64_t)-1) {
            pr_err("Failed to read from address 0x%llx\n", (unsigned long long)addr);
        }
        //VNIC_DBG("vnic_read_share_mem: pa = 0x%llx, val = 0x%llx\n", 
        //    (unsigned long long)addr, (unsigned long long)val);
    #endif
    //VNIC_DBG("vnic_read_share_mem: addr = 0x%llx, val = 0x%llx\n",
    //    (unsigned long long)addr, (unsigned long long)val);
    spin_unlock_irqrestore(&read_lock, flags);
    return val;
}

static void vnic_write_share_mem(uint64_t *addr, uint64_t val)
{
    uint32_t flags;
    spin_lock_irqsave(&write_lock, flags);
    #ifdef FPGA
    /*
        uint32_t lo = (uint32_t)(val & 0xFFFFFFFFUL);
	    uint32_t hi = (uint32_t)((val >> 32) & 0xFFFFFFFFUL);
	    iowrite32(lo, (u8 __iomem *)addr);
	    iowrite32(hi, (u8 __iomem *)addr + 4);
    */
        writeq(val, (void __iomem *)addr);
        mb();
        vnic_flush_cache();
    #else
        int off = vnic_get_sharemem_offset(addr);
        if (off < 0) {
            return; // Invalid address
        }
        vnic_xdma_write((uint32_t)off, val);
        mb();
        vnic_flush_cache();
    #endif
    //VNIC_DBG("vnic_write_share_mem: addr = 0x%llx, val = 0x%llx\n",
    //    (unsigned long long)addr, (unsigned long long)val);
    spin_unlock_irqrestore(&write_lock, flags);
    return;
}

static void vnic_memcpy_to_share_mem(void *addr, void *data, size_t len)
{
    uint32_t flags;
    spin_lock_irqsave(&write_lock, flags);
    #ifdef FPGA
        memcpy(addr, data, len);
        mb();
        vnic_flush_cache();
    #else
        int off = vnic_get_sharemem_offset(addr);
        if (off < 0) {
            return;
        }
        vnic_xdma_memcpy_to_share_mem((uint32_t)off, data, len);
        mb();
        vnic_flush_cache();
    #endif
    spin_unlock_irqrestore(&write_lock, flags);
}

static void vnic_memcpy_from_share_mem(void *addr, void *data, size_t len)
{
    uint32_t flags;
    spin_lock_irqsave(&read_lock, flags);
    #ifdef FPGA
        vnic_flush_cache();
        memcpy(data, addr, len);
        mb();
    #else
        int off = vnic_get_sharemem_offset(addr);
        if (off < 0) {
            return;
        }
        vnic_flush_cache();
        vnic_xdma_memcpy_from_share_mem((uint32_t)off, data, len);
        mb();
    #endif
    spin_unlock_irqrestore(&read_lock, flags);
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
    #else
        share_mem_virt = (void *)SHARE_MEM_BASE;
    #endif

    tx_tail = (uint64_t *)(share_mem_virt + TX_TAIL_OFFSET);
    tx_head = (uint64_t *)(share_mem_virt + TX_HEAD_OFFSET);
    tx_md = (void *)(share_mem_virt + TX_MD_FIFO_OFFSET);
    tx_data = (void *)(share_mem_virt + TX_D_FIFO_OFFSET);
    rx_tail = (uint64_t *)(share_mem_virt + RX_TAIL_OFFSET);
    rx_head = (uint64_t *)(share_mem_virt + RX_HEAD_OFFSET);
    rx_md = (void *)(share_mem_virt + RX_MD_FIFO_OFFSET);
    rx_data = (void *)(share_mem_virt + RX_D_FIFO_OFFSET);

    VNIC_DBG("share mem base address: 0x%llx\n", (uint64_t)share_mem_virt);
    VNIC_DBG("TX tail address: 0x%llx\n", (uint64_t)tx_tail);
    VNIC_DBG("TX head address: 0x%llx\n", (uint64_t)tx_head);
    VNIC_DBG("TX metadata FIFO address: 0x%llx\n", tx_md);
    VNIC_DBG("TX data FIFO address: 0x%llx\n", (uint64_t)tx_data);
    VNIC_DBG("RX tail address: 0x%llx\n", (uint64_t)rx_tail);
    VNIC_DBG("RX head address: 0x%llx\n", (uint64_t)rx_head);
    VNIC_DBG("RX metadata FIFO address: 0x%llx\n", rx_md);
    VNIC_DBG("RX data FIFO address: 0x%llx\n", (uint64_t)rx_data);

    return 0;
}

static int vnic_tx_full(void)
{
    uint64_t tail = vnic_read_share_mem(tx_tail);
    uint64_t head = vnic_read_share_mem(tx_head);
    //("vnic_tx_full: tx_tail = %llx, tx_head = %llx\n", tail, head);
    return ((tail + 1) % MAX_DESC == head);
}

static int vnic_rx_empty(void)
{
    uint64_t tail = vnic_read_share_mem(rx_tail);
    uint64_t head = vnic_read_share_mem(rx_head);
    //VNIC_DBG("vnic_rx_empty: rx_tail = %llu, rx_head = %llu\n", tail, head);
    return (tail == head);
}

static void *vnic_get_tx_buf_addr(bool is_metadata)
{
    if (vnic_tx_full()) {
        pr_err("TX FIFO is full, cannot get metadata\n");
        return NULL;
    }
    uint64_t txtail = vnic_read_share_mem(tx_tail);
    if (is_metadata) {
        //void *md_buf = tx_md + (vnic_read_share_mem(tx_tail) * DESC_SIZE);
        void *md_buf = tx_md + txtail * DESC_SIZE;
        VNIC_DBG("vnic_get_tx_buf_addr: md_buf = %llx, tx_tail = %llx cpu = %x\n", md_buf, txtail, smp_processor_id());
        if (vnic_read_share_mem(md_buf + 2 * 8) == 0x1ULL) {
            pr_err("Metadata buffer is already in use\n");
            return NULL; // Metadata buffer is already in use
        }
        return md_buf;
    } else {
        void *data_buf = tx_data + txtail * MAX_PACKET_SIZE;
        return data_buf;
    }
}

static void *vnic_get_rx_buf_addr(bool is_metadata)
{
    if (vnic_rx_empty()) {
        pr_err("RX FIFO is empty, no metadata to read\n");
        return NULL;
    }
    uint64_t rxhead = vnic_read_share_mem(rx_head);
    if (is_metadata) {
        //void *md_buf = rx_md + (vnic_read_share_mem(rx_head) * DESC_SIZE);
        void *md_buf = rx_md + rxhead * DESC_SIZE;
        VNIC_DBG("vnic_get_rx_buf_addr: md_buf = %llx, rx_head = %llx, cpu = %x\n", md_buf, vnic_read_share_mem(rx_head), smp_processor_id());
        if (vnic_read_share_mem(md_buf + 2 * 8) == 0x0ULL) {
            pr_err("Metadata buffer is not in use\n");
            return NULL;
        }
        return md_buf;
    } else {
        void *data_buf = rx_data + rxhead * MAX_PACKET_SIZE;
        return data_buf;
    }
}

static void vnic_dump_64B(void *addr)
{
    uint64_t *data = (uint64_t *)addr;
    int i;
    VNIC_DBG("Dumping 64B at address %llx:\n", (unsigned long long)addr);
    for (i = 0; i < 8; i++) {
        VNIC_DBG("0x%016llx %s", vnic_read_share_mem(data + i), (i == 7) ? "\n" : " ");
    }
}

static int vnic_send_packet(struct sk_buff *skb)
{
    if (!vnic_opened)
        return NETDEV_TX_OK; // Device is not opened

    if (vnic_tx_full()) {
        pr_err("TX FIFO is full, cannot send packet\n");
        return NETDEV_TX_BUSY; // No space in TX FIFO
    }

    void *dst_buf = vnic_get_tx_buf_addr(false);
    void *md_buf = vnic_get_tx_buf_addr(true);
    //VNIC_DBG("vnic_send_packet: dst_buf = %p, md_buf = %p sk_buf = %p\n", dst_buf, md_buf, skb->data);
    if (!md_buf || !dst_buf) {
        pr_err("0x%llx times Failed to get TX buffers\n", ++fail_count);
        return -EIO; // Error in reading RX buffers
    }
    if(fail_count > 100) debug = false; // Disable debug after 10 failures
    uint64_t txtail = vnic_read_share_mem(tx_tail);
    uint64_t txhead = vnic_read_share_mem(tx_head);
    uint64_t next_txtail;
    memcpy(eth_hdr(skb)->h_source, SRC_MAC, ETH_ALEN);
    memcpy(eth_hdr(skb)->h_dest, DST_MAC, ETH_ALEN); 
    vnic_memcpy_to_share_mem(dst_buf, skb->data, skb->len);
    //vnic_write_share_mem(md_buf, (uint64_t)dst_buf); // Set buffer address
    //we do not need to set buf addr, rx can calculate it from tx_tail
    vnic_write_share_mem(md_buf + 8, skb->len);      // Set length of the packet
    vnic_write_share_mem(md_buf + 2 * 8, 0x1ULL);             // Set valid flag
    //vnic_write_share_mem(md_buf + 3 * 8, 0UL);             // Reserved
    vnic_write_share_mem(tx_tail, (txtail + 1) % MAX_DESC); // Update tail pointer
    next_txtail = vnic_read_share_mem(tx_tail);
    vnic_dev->stats.tx_packets++;
    vnic_dev->stats.tx_bytes += skb->len;
    //uint64_t txtail = vnic_read_share_mem(tx_tail);
    //uint64_t first_8B = vnic_read_share_mem((uint64_t*)dst_buf);
    uint64_t len = vnic_read_share_mem(md_buf + 8); // Get length of the packet
    VNIC_DBG("send packet: buf = 0x%llx, len = 0x%llx, md = 0x%llx, txtail = 0x%llx, txhead = 0x%llx, next txtail = 0x%llx\n", 
        dst_buf, len, md_buf, txtail, txhead, next_txtail);
    VNIC_DBG("send packet: src mac = %pM dst mac = %pM\n",
        eth_hdr(skb)->h_source, eth_hdr(skb)->h_dest);
    vnic_dump_64B(dst_buf);
    dev_kfree_skb(skb); // Free the skb after processing
    return NETDEV_TX_OK;
}

static int vnic_recv_packet(void)
{
    if (vnic_rx_empty()) {
        //VNIC_DBG("RX FIFO is empty, no packet to receive\n");
        return 0; // No packet to receive
    }
    uint64_t rxtail = vnic_read_share_mem(rx_tail);
    uint64_t rxhead = vnic_read_share_mem(rx_head);
    //uint64_t *md_buf = rx_md + (rxhead * (DESC_SIZE / sizeof(uint64_t)));
    void *md_buf = vnic_get_rx_buf_addr(true);
    void *data_buf = vnic_get_rx_buf_addr(false);
    if (!md_buf || !data_buf) {
        pr_err("0x%llx times Failed to get RX buffers\n", ++fail_count);
        return -EIO; // Error in reading RX buffers
    }
    if(fail_count > 100) debug = false; // Disable debug after 10 failures
    void *buf = rx_data + rxhead * MAX_PACKET_SIZE; // Get buffer address
    //uint64_t first_8B = vnic_read_share_mem((uint64_t*)buf);
    uint64_t len = vnic_read_share_mem((uint64_t*)(md_buf + 8)); // Get length of the packet
    uint64_t next_rxhead;

    vnic_dev->stats.rx_packets++;
    vnic_dev->stats.rx_bytes += len;

    struct sk_buff *skb = netdev_alloc_skb(vnic_dev, len + NET_IP_ALIGN);
    if (!skb) {
        pr_err("Failed to allocate skb\n");
        return -ENOMEM;
    }
    skb_reserve(skb, NET_IP_ALIGN); // Align the skb data
    skb_put(skb, len); // Set the length of the skb
    vnic_memcpy_from_share_mem(buf, skb->data, len);
    skb->protocol = eth_type_trans(skb, vnic_dev);
    skb->dev = vnic_dev;
    skb->pkt_type = PACKET_HOST; 
    skb->tstamp = ktime_get_real();

    netif_rx(skb);

    vnic_write_share_mem(md_buf + 2 * 8, 0x0ULL);
    vnic_write_share_mem(rx_head, (rxhead + 1) % MAX_DESC);
    next_rxhead = vnic_read_share_mem(rx_head);
    VNIC_DBG("recv packet: buf = 0x%llx, len = 0x%llx, md = 0x%llx, rxtail = 0x%llx, rxhead = 0x%llx, next_rxhead = 0x%llx\n",
        buf, len, md_buf, rxtail, rxhead, next_rxhead);
    VNIC_DBG("recv packet: src mac = %pM dst mac = %pM\n",
        eth_hdr(skb)->h_source, eth_hdr(skb)->h_dest);
    vnic_dump_64B(buf);
    //dev_kfree_skb(skb); // Free the skb after processing
    return len;
}

static void vnic_deinit_share_mem(void)
{
    #ifdef FPGA
        iounmap(share_mem_virt);
    #endif
}