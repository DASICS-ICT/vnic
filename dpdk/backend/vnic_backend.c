#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#define VNIC_REG_PHYS_BASE      0xa1000000ULL
#define VNIC_REG_RX_DESC        0x0ULL
#define VNIC_REG_RX_DATA        0x8ULL
#define VNIC_REG_TX_DESC        0x10ULL
#define VNIC_REG_TX_DATA        0x18ULL
#define VNIC_REG_RX_TAIL        0x20ULL
#define VNIC_REG_RX_HEAD        0x28ULL
#define VNIC_REG_TX_TAIL        0x30ULL
#define VNIC_REG_TX_HEAD        0x38ULL
#define VNIC_REG_STATUS         0x40ULL
#define VNIC_REGS_SIZE          0x100ULL

#define VNIC_DESC_SIZE          0x20ULL
#define VNIC_DATA_SIZE          0x1000ULL
#define VNIC_MAX_QUEUE_DEPTH    0x1000ULL

#define FPGA_RESV_MEM_PHYS     0xf8000000ULL
#define VNIC_RESV_MEM_PHYS     0x98000000ULL
#define VNIC_RESV_MEM_SIZE     0x8000000ULL
#define VNIC_RESV_RX_DESC      0x0ULL
#define VNIC_RESV_RX_DATA      VNIC_RESV_RX_DESC + VNIC_DESC_SIZE * VNIC_MAX_QUEUE_DEPTH
#define VNIC_RESV_TX_DESC      VNIC_RESV_RX_DATA + VNIC_DATA_SIZE * VNIC_MAX_QUEUE_DEPTH
#define VNIC_RESV_TX_DATA      VNIC_RESV_TX_DESC + VNIC_DESC_SIZE * VNIC_MAX_QUEUE_DEPTH

#define VNIC_STATUS_IDLE 0x0
#define VNIC_STATUS_BUSY 0x80000000ULL

#define VNIC_DESC_SIZE         0x20ULL
#define VNIC_DESC_STATUS_DONE  0x80000000ULL
#define VNIC_DESC_STATUS_VALID 0x40000000ULL

#define DEBUG 0
#define VNIC_DEBUG(fmt, args...) \
	do { \
		if (DEBUG) \
			printf(fmt, ##args); \
	} while (0)

static volatile void *vnic_regs_virt;
static volatile void *vnic_resv_mem_virt;
static volatile uint64_t *vnic_rx_desc_base;
static volatile uint64_t *vnic_rx_data_base;
static volatile uint64_t *vnic_tx_desc_base;
static volatile uint64_t *vnic_tx_data_base;
static volatile uint64_t *vnic_rx_tail;
static volatile uint64_t *vnic_rx_head;
static volatile uint64_t *vnic_tx_tail;
static volatile uint64_t *vnic_tx_head;
static volatile uint64_t *vnic_status;

struct vnic_rxtx_desc {
	uint64_t buf;    // Buffer address
	uint64_t len;    // Length of the received packet
	uint64_t id;   // RX/TX identifier
	uint32_t status; // Status flags
	uint32_t reserved;
};

static uint64_t vnic_read_reg64(uint32_t offset)
{
	uint64_t val;
	val = *(volatile uint64_t *)((void *)vnic_regs_virt + offset);
	//pread(xdma_c2h_fd, &val, sizeof(uint64_t), offset);
	return val;
}

static void vnic_write_reg64(uint32_t offset, uint64_t val)
{
	//*(volatile uint64_t *)((void *)vnic_regs_virt + offset) = val;
	*((volatile uint64_t *)(vnic_regs_virt + offset)) = val;
	//pwrite(xdma_h2c_fd, &val, sizeof(uint64_t), offset);
}

static uint32_t vnic_read_reg32(uint32_t offset)
{
	uint32_t val;
	val = *(volatile uint32_t *)((void *)vnic_regs_virt + offset);
	//pread(xdma_c2h_fd, &val, sizeof(uint32_t), offset);
	return val;
}

static void vnic_write_reg32(uint32_t offset, uint32_t val)
{
	*(volatile uint32_t *)((void *)vnic_regs_virt + offset) = val;
	//pwrite(xdma_h2c_fd, &val, sizeof(uint32_t), offset);
}

static void vnic_dump_regs(void)
{
	printf("VNIC RX DESC BASE: 0x%lx\n", vnic_read_reg64(VNIC_REG_RX_DESC));
	printf("VNIC RX DATA BASE: 0x%lx\n", vnic_read_reg64(VNIC_REG_RX_DATA));
	printf("VNIC TX DESC BASE: 0x%lx\n", vnic_read_reg64(VNIC_REG_TX_DESC));
	printf("VNIC TX DATA BASE: 0x%lx\n", vnic_read_reg64(VNIC_REG_TX_DATA));
	printf("VNIC RX TAIL: 0x%lx\n", vnic_read_reg64(VNIC_REG_RX_TAIL));
	printf("VNIC RX HEAD: 0x%lx\n", vnic_read_reg64(VNIC_REG_RX_HEAD));
	printf("VNIC TX TAIL: 0x%lx\n", vnic_read_reg64(VNIC_REG_TX_TAIL));
	printf("VNIC TX HEAD: 0x%lx\n", vnic_read_reg64(VNIC_REG_TX_HEAD));
	printf("VNIC STATUS: 0x%lx\n", vnic_read_reg64(VNIC_REG_STATUS));
}

void process_packet(uint64_t buf_addr, uint64_t len) {
	// 计算缓冲区在映射内存中的位置
	void *packet = malloc(len);
	void *buf_addr_virt = vnic_resv_mem_virt + (buf_addr - FPGA_RESV_MEM_PHYS);
	if (!packet) {
		printf("Failed to allocate memory for packet");
		return;
	}

	VNIC_DEBUG("Processing packet @ 0x%lx, size: %lu bytes\n", buf_addr, len);

	memcpy(packet, buf_addr_virt, len);

	// 打印数据包内容（前64字节）
	/*
	VNIC_DEBUG("Packet data (first 64 bytes):\n");
	for (int i = 0; i < 64 && i < len; i++) {
		VNIC_DEBUG("%02x ", ((uint8_t *)packet)[i]);
		if ((i + 1) % 16 == 0) VNIC_DEBUG("\n");
	}
	*/
	free(packet);
}

/* TX队列处理线程 */
void tx_process(void) {
	void *tx_desc_base = vnic_resv_mem_virt + VNIC_RESV_TX_DESC;
	uint64_t last_head = vnic_read_reg64(VNIC_REG_TX_HEAD);
	//sleep(1);
	uint64_t tx_tail = vnic_read_reg64(VNIC_REG_TX_TAIL);
	struct vnic_rxtx_desc *desc;

	VNIC_DEBUG("TX Process started. TX base: 0x%lx\n", tx_desc_base);
	//printf("Initial TX_HEAD: %lx, TX_TAIL: %lx\n", last_head, tx_tail);
	while (1) {
		// 检查是否有新的描述符需要处理
		if (last_head != tx_tail) {
			// 计算描述符在映射内存中的位置
			desc = (struct vnic_rxtx_desc *)(tx_desc_base + last_head * VNIC_DESC_SIZE);
			// 检查描述符是否有效
			if (desc->status & VNIC_DESC_STATUS_VALID) {
				// 处理数据包
				process_packet(desc->buf, desc->len);
				// 标记描述符为已完成
				desc->status = VNIC_DESC_STATUS_DONE;
				VNIC_DEBUG("Processed TX descriptor %u\n\n", last_head);
				last_head = (last_head + 1) % VNIC_MAX_QUEUE_DEPTH;
				__sync_synchronize();
				vnic_write_reg64(VNIC_REG_TX_HEAD, last_head);
				//printf("Updated TX_HEAD to %lx\n", last_head);
			}
		}
		tx_tail = vnic_read_reg64(VNIC_REG_TX_TAIL);
	}
	//vnic_write_reg64(VNIC_REG_TX_HEAD, last_head);
}

int main()
{
	int fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd < 0) {
		perror("Failed to open /dev/mem");
		return -1;
	}

	vnic_regs_virt = mmap(NULL, VNIC_REGS_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, VNIC_REG_PHYS_BASE);
	if (vnic_regs_virt == MAP_FAILED) {
		perror("Failed to mmap /dev/mem");
		close(fd);
		return -1;
	}

	vnic_resv_mem_virt = mmap(NULL, VNIC_RESV_MEM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, VNIC_RESV_MEM_PHYS);
	if (vnic_resv_mem_virt == MAP_FAILED) {
		perror("Failed to mmap reserved memory");
		munmap((void *)vnic_regs_virt, VNIC_REGS_SIZE);
		close(fd);
		return -1;
	}

	while (vnic_read_reg64(VNIC_REG_STATUS) == VNIC_STATUS_IDLE)
		;

	vnic_dump_regs();

	tx_process();
    
	munmap((void *)vnic_regs_virt, VNIC_REGS_SIZE);
	close(fd);

	return 0;
}