#include <xdma/libxdma_api.h>
#include <linux/string.h>
#include <linux/io.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))

uint64_t vnic_xdma_read(uint32_t off)
{
    return xdma_read64_bar(off);
}

void vnic_xdma_write(uint32_t off, uint64_t val)
{
    xdma_write64_bar(off, val);
}

void vnic_xdma_memcpy_from_share_mem(uint32_t off, void *host, size_t len)
{
    //pr_info("vnic_xdma_memcpy_from_share_mem start: fpga = %lx, host = %p, len = %zu\n", off, host, len);
    uintptr_t fpga_addr = (uintptr_t)off;
    u8 *dst = (u8 *)host;
    
    // 处理起始非对齐部分
    if (len > 0 && (fpga_addr & 0x7)) {
        uintptr_t aligned_addr = fpga_addr & ~0x7UL;
        u64 val = vnic_xdma_read(aligned_addr);
        
        u8 *pval = (u8 *)&val;
        size_t offset = fpga_addr & 0x7;
        size_t copy_bytes = MIN(8 - offset, len);
        
        for (size_t i = 0; i < copy_bytes; i++) {
            dst[i] = pval[offset + i];
        }
        
        fpga_addr += copy_bytes;
        dst += copy_bytes;
        len -= copy_bytes;
    }
    
    // 处理对齐块 (64位拷贝)
    while (len >= sizeof(u64)) {
        u64 val = vnic_xdma_read(fpga_addr);
        
        // 逐字节复制到目标缓冲区
        u8 *pval = (u8 *)&val;
        for (size_t i = 0; i < sizeof(u64); i++) {
            dst[i] = pval[i];
        }
        
        fpga_addr += sizeof(u64);
        dst += sizeof(u64);
        len -= sizeof(u64);
    }
    
    // 处理剩余尾部数据
    if (len > 0) {
        u64 val = vnic_xdma_read(fpga_addr);
        u8 *pval = (u8 *)&val;
        
        for (size_t i = 0; i < len; i++) {
            dst[i] = pval[i];
        }
    }
    //pr_info("vnic_xdma_memcpy_from_share_mem end\n");
}

// Host memory -> FPGA DRAM 拷贝
void vnic_xdma_memcpy_to_share_mem(uint32_t off, void *host, size_t len)
{
    //pr_info("vnic_xdma_memcpy_to_share_mem start: fpga = %lx, host = %p, len = %zu\n", off, host, len);
    uintptr_t fpga_addr = (uintptr_t)off;
    u8 *src = (u8 *)host;
    
    // 处理起始非对齐部分
    if (len > 0 && (fpga_addr & 0x7)) {
        uintptr_t aligned_addr = fpga_addr & ~0x7UL;
        u64 val = vnic_xdma_read(aligned_addr);
        
        u8 *pval = (u8 *)&val;
        size_t offset = fpga_addr & 0x7;
        size_t copy_bytes = MIN(8 - offset, len);
        
        for (size_t i = 0; i < copy_bytes; i++) {
            pval[offset + i] = src[i];
        }
        
        vnic_xdma_write(aligned_addr, val);
        
        fpga_addr += copy_bytes;
        src += copy_bytes;
        len -= copy_bytes;
    }
    
    // 处理对齐块 (64位拷贝)
    while (len >= sizeof(u64)) {
        u64 tmp = 0;
        
        // 从源缓冲区逐字节复制构建64位值
        for (size_t i = 0; i < sizeof(u64); i++) {
            *((u8 *)&tmp + i) = src[i];
        }
        
        vnic_xdma_write(fpga_addr, tmp);
        
        fpga_addr += sizeof(u64);
        src += sizeof(u64);
        len -= sizeof(u64);
    }
    
    // 处理剩余尾部数据
    if (len > 0) {
        u64 val = vnic_xdma_read(fpga_addr);
        u8 *pval = (u8 *)&val;
        
        for (size_t i = 0; i < len; i++) {
            pval[i] = src[i];
        }
        
        vnic_xdma_write(fpga_addr, val);
    }
    //pr_info("vnic_xdma_memcpy_to_share_mem end\n");
}
/*
static ssize_t vnic_xdma_read_write_chrdev(void *buf, size_t count, loff_t offset, bool write)
{
    struct sg_table *sgt;
    struct scatterlist *sg;
    int ret;

    sgt = kmalloc(sizeof(struct sg_table), GFP_KERNEL);
    if (!sgt)
        return -ENOMEM;

    ret = sg_alloc_table(sgt, 1, GFP_KERNEL);
    if (ret) {
        kfree(sgt);
        return -ENOMEM;
    }

    sg = sgt->sgl;
    sg_set_buf(sg, buf, count);

    ret = xdma_xfer_submit(xdma_get_dev_hndl(), 0, write, offset, sgt, 0, 1000000);
    if (ret < 0) {
        pr_err("xdma_xfer_submit failed: %d\n", ret);
        sg_free_table(sgt);
        kfree(sgt);
        return ret;
    }

    if (ret != count) {
        pr_err("xdma_xfer_submit transferred %d bytes, expected %zu bytes\n", ret, count);
        sg_free_table(sgt);
        kfree(sgt);
        return -EIO;
    }
    
    sg_free_table(sgt);
    kfree(sgt);
    return ret;
}

static ssize_t vnic_xdma_read_chrdev(void *buf, size_t count, loff_t offset)
{
    return vnic_xdma_read_write_chrdev(buf, count, offset, 0);
}

static ssize_t vnic_xdma_write_chrdev(const void *buf, size_t count, loff_t offset)
{
    return vnic_xdma_read_write_chrdev((void *)buf, count, offset, 1);
}

static uint64_t vnic_xdma_read(uint64_t addr)
{
    uint64_t *val = kmalloc(sizeof(uint64_t), GFP_KERNEL);
    if (!val)
        return -ENOMEM;

    if (vnic_xdma_read_chrdev((void*)val, sizeof(uint64_t), addr) < 0) {
        kfree(val);
        return -EIO;
    }

    uint64_t ret = *val;
    kfree(val);
    return ret;
}

static void vnic_xdma_write(uint64_t addr, uint64_t data)
{
    uint64_t *val = kmalloc(sizeof(uint64_t), GFP_KERNEL);
    if (!val) {
        pr_err("Failed to allocate memory for write operation\n");
        return;
    }

    *val = data;
    if (vnic_xdma_write_chrdev((void*)val, sizeof(uint64_t), addr) < 0) {
        pr_err("Failed to write to address 0x%llx\n", addr);
    }
}

static void vnic_xdma_memcpy_to_share_mem(void *addr, void *data, size_t len)
{
    if (vnic_xdma_write_chrdev(data, len, (uint64_t)addr) < 0) {
        pr_err("Failed to memcpy to share memory at address 0x%llx\n", addr);
    }
}

static void vnic_xdma_memcpy_from_share_mem(void *addr, void *data, size_t len)
{
    if (vnic_xdma_read_chrdev(data, len, (uint64_t)addr) < 0) {
        pr_err("Failed to memcpy from share memory at address 0x%llx\n", addr);
    }
}
*/