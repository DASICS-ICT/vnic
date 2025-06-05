#include <xdma/libxdma_api.h>

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

    ret = xdma_xfer_submit(xdma_get_dev_hndl(), 0, write, offset, sgt, 0, 10000);
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

static void vnic_xdma_memcpy_to_share_mem(uint64_t addr, void *data, size_t len)
{
    if (vnic_xdma_write_chrdev(data, len, addr) < 0) {
        pr_err("Failed to memcpy to share memory at address 0x%llx\n", addr);
    }
}

static void vnic_xdma_memcpy_from_share_mem(uint64_t addr, void *data, size_t len)
{
    if (vnic_xdma_read_chrdev(data, len, addr) < 0) {
        pr_err("Failed to memcpy from share memory at address 0x%llx\n", addr);
    }
}