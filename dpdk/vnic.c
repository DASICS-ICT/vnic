#include "vnic_rxtx.h"

#include <rte_mbuf.h>
#include <rte_cycles.h>

#define NUM_MBUFS 8192
#define MBUF_CACHE_SIZE 250
#define DEFAULT_BURST_SIZE 1
#define DEFAULT_PKT_SIZE 64

uint64_t burst_size = DEFAULT_BURST_SIZE;
uint64_t pkt_size = DEFAULT_PKT_SIZE;

struct perf_stats {
	uint64_t total_bytes;      // 总传输字节数
	uint64_t total_packets;    // 总传输包数
	uint64_t start_tsc;        // 开始时间戳计数器
	uint64_t end_tsc;          // 结束时间戳计数器
	double duration_sec;       // 持续时间(秒)
	double bandwidth_mbps;     // 带宽(Gbps)
	double packets_per_sec;    // 包/秒
};

void parse_args(int argc, char** argv) {
	for (int i = 0; i < argc; i++) {
		if (strcmp(argv[i], "--burst") == 0 && i + 1 < argc) {
			burst_size = atoi(argv[i + 1]);
		}
		if (strcmp(argv[i], "--size") == 0 && i + 1 < argc) {
			pkt_size = atoi(argv[i + 1]);
		}
	}
}


int main(int argc, char *argv[])
{
	int ret = 0;
	int i;
	struct rte_mempool *mbuf_pool;
	struct vnic_rxtx_queue *rxq, *txq;
	uint64_t nb_tx = 0, nb_tx_cmpl = 0;

	ret = rte_eal_init(argc, argv);
	if (ret < 0) {
		rte_exit(EXIT_FAILURE, "Error with EAL initialization\n");
	}

	parse_args(argc, argv);
	ret = vnic_init_buf();
	if (ret < 0) {
		rte_exit(EXIT_FAILURE, "Error with VNIC buffer initialization\n");
	}

	ret = vnic_init_regs();
	if (ret < 0) {
		rte_exit(EXIT_FAILURE, "Error with VNIC register initialization\n");
	}

	rxq = vnic_create_rxtx_queue();
	txq = vnic_create_rxtx_queue();

	if (rxq == NULL || txq == NULL) {
		rte_exit(EXIT_FAILURE, "Error with VNIC RX/TX queue initialization\n");
	}

	vnic_dump_regs();

	vnic_write_reg32(VNIC_REG_STATUS, VNIC_STATUS_BUSY);
	rte_wmb();

	// reset timer
	struct perf_stats *stats = malloc(sizeof(struct perf_stats));
	memset(stats, 0, sizeof(struct perf_stats));
	stats->start_tsc = rte_rdtsc();

	// RX/TX main loop
	nb_tx = vnic_tx_burst(txq, burst_size, pkt_size);
	VNIC_DEBUG("submit %lu packets for tx\n", nb_tx);
	do { 
		//VNIC_DEBUG("nb_tx = %lu, nb_tx_cmpl = %lu\n", nb_tx, nb_tx_cmpl);
		nb_tx_cmpl += vnic_process_tx_completion(txq);
	} while (nb_tx_cmpl != nb_tx);
	stats->end_tsc = rte_rdtsc();
	stats->total_packets = nb_tx_cmpl;
	stats->total_bytes = nb_tx_cmpl * pkt_size;
	stats->duration_sec = (double)(stats->end_tsc - stats->start_tsc) / rte_get_tsc_hz();
	stats->bandwidth_mbps = (double)(stats->total_bytes * 8) / (stats->duration_sec * 1e6);
	stats->packets_per_sec = (double)stats->total_packets / stats->duration_sec;
	printf("cmpl %lu packets for tx\n", nb_tx_cmpl);
	printf("\n===== Performance Statistics =====\n");
	printf("Total packets: %" PRIu64 "\n", stats->total_packets);
	printf("Total bytes: %" PRIu64 " bytes\n", stats->total_bytes);
	printf("Duration: %.6f seconds\n", stats->duration_sec);
	printf("Bandwidth: %.3f mbps\n", stats->bandwidth_mbps);
	printf("Throughput: %.2f packets/sec\n", stats->packets_per_sec);
	printf("===================================\n");

	vnic_cleanup_regs();
	vnic_free_rxtx_queues(rxq);
	vnic_free_rxtx_queues(txq);

	rte_eal_cleanup();

	return 0;
}
