#include "vnic.h"

/* 网络设备操作函数 */
static netdev_tx_t vnic_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
    uint32_t flags;
    spin_lock_irqsave(&tx_lock, flags);
    skb_tx_timestamp(skb);
    
    if (skb_is_nonlinear(skb)) {
        if (skb_linearize(skb) != 0) {
            //dev_kfree_skb(skb);
            pr_err("skb_linearize failed\n");
            return NETDEV_TX_OK;
        }
    }
    //pr_info("vnic_start_xmit: dev = %llx, tx_packets %llx, tx_bytes = %llx\n", 
    //       (unsigned long long)dev, (unsigned long long)dev->stats.tx_packets,
    //       (unsigned long long)dev->stats.tx_bytes);
    //if (skb)
    //    dev_kfree_skb(skb);
    int ret = vnic_send_packet(skb);
    spin_unlock_irqrestore(&tx_lock, flags);
    return ret;
}

static int vnic_poll_rx(void *data)
{
    int ret;
    while (!kthread_should_stop()) { // 检查线程是否应停止
        //pr_info("vnic poll one time\n");
        if (!vnic_opened) {
            //pr_info("vnic is not opened, skipping poll\n");
            #ifdef FPGA
                msleep(100);
            #else
                msleep(1000); // 如果设备未打开，休眠5毫秒降低CPU占用
            #endif
            continue; // 跳过本次循环
        }
        uint32_t flags;
        spin_lock_irqsave(&rx_lock, flags);
        ret = vnic_recv_packet();
        spin_unlock_irqrestore(&rx_lock, flags);       
        if (ret < 0) {
            pr_err("Failed to receive packet: %d\n", ret);
            return -EIO; // 如果接收失败，继续下一次循环
        }
        #ifdef FPGA
            msleep(100);
        #else
            msleep(1000); // 如果设备未打开，休眠5毫秒降低CPU占用
        #endif
    }
    return 0;
}

static int vnic_open(struct net_device *dev)
{
    pr_info("open vnic\n");
    netif_start_queue(dev);
    vnic_opened = true;
    return 0;
}

static int vnic_close(struct net_device *dev)
{
    pr_info("close vnic\n");
    netif_stop_queue(dev);
    vnic_opened = false;
    return 0;
}

/* 网络设备操作结构体 */
static const struct net_device_ops vnic_ops = {
    .ndo_open = vnic_open,
    .ndo_stop = vnic_close,
    .ndo_start_xmit = vnic_start_xmit,
};

/* 初始化网络设备 */
static void vnic_setup(struct net_device *dev)
{
    ether_setup(dev);  // 设置以太网相关参数
    spin_lock_init(&tx_lock); 
    spin_lock_init(&rx_lock);
    spin_lock_init(&read_lock);
    spin_lock_init(&write_lock);
    dev->netdev_ops = &vnic_ops;
    dev->num_tx_queues = 1;
    dev->real_num_tx_queues = 1;
    
    /* random mac */
    //eth_random_addr(dev->dev_addr);
    /* manually set mac address */
    
    #ifdef FPGA
        memcpy(dev->dev_addr, RV_MAC, ETH_ALEN);
    #else
        memcpy(dev->dev_addr, X86_MAC, ETH_ALEN);
    #endif
    
    if (vnic_init_share_mem()) {
        pr_err("Failed to initialize shared memory\n");
        return;
    }
    pr_info("share_mem_virt: 0x%llx\n", (uint64_t)share_mem_virt);
}

static int __init vnic_init(void)
{
    int ret;
    
    /* 分配网络设备结构体 */
    vnic_dev = alloc_netdev(0, "vnic%d", NET_NAME_ENUM, vnic_setup);
    if (!vnic_dev)
        return -ENOMEM;
    
    /* 注册网络设备 */
    ret = register_netdev(vnic_dev);
    if (ret) {
        free_netdev(vnic_dev);
        return ret;
    }

    polling_thread = kthread_run(vnic_poll_rx, NULL, "vnic_poll");
    if (IS_ERR(polling_thread)) {
        pr_err("Failed to start polling thread\n");
        return PTR_ERR(polling_thread);
    }
    //kthread_bind(polling_thread, 0); // Bind to CPU 0
    pr_info("Virtual network device %s registered\n", vnic_dev->name);
    return 0;
}

static void __exit vnic_exit(void)
{
    unregister_netdev(vnic_dev);
    free_netdev(vnic_dev);
    if (polling_thread) {
        kthread_stop(polling_thread);
        polling_thread = NULL;
    }
    vnic_deinit_share_mem();
    pr_info("Virtual network device unregistered\n");
}

module_init(vnic_init);
module_exit(vnic_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Guofeng Li");