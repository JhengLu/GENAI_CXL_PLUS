#ifndef __MONITOR_H__
#define __MONITOR_H__

#include <linux/perf_event.h>
#include <vector>

// Event ID: UMask + EventSel. (https://perfmon-events.intel.com/skylake_server.html)
#define EVENT_RxC_OCCUPANCY_IRQ 0x0111UL
#define EVENT_RxC_INSERTS_IRQ 0x0113UL
#define EVENT_CAS_COUNT_RD 0x0304UL
#define EVENT_CAS_COUNT_WR 0x0201UL
//#define EVENT_CAS_COUNT_ALL 0x0F04UL


#define PAGE_SIZE 4096UL
#define NUM_PERF_EVENT_MMAP_PAGES 64UL      // NOTE: might consider increase this value later on
#define SAMPLING_PERIOD_EVENT 500UL         // in # of events
//#define SAMPLING_PERIOD_MS 50UL             // in ms
#define SAMPLING_PERIOD_MS 500UL             // in ms
#define EWMA_ALPHA 0.5

#define NUM_SOCKETS 2
#define PROCESSOR_GHZ 2.4   // CloudLab c6420

// TODO: read the numbers from path
// perf_event_attr.type value for each individual cha unit found in /sys/bus/event_source/devices/uncore_cha_*/type
const uint32_t PMU_CHA_TYPE[] = {25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40};   // CloudLab c6420

// perf_event_attr.type value for each individual imc unit found in /sys/bus/event_source/devices/uncore_imc_*/type
const uint32_t PMU_IMC_TYPE[] = {12, 13, 14, 15, 16, 17};   // CloudLab c6420


class Monitor {
  public:
    Monitor();
    ~Monitor();

    void perf_event_reset(int fd);
    void perf_event_enable(int fd);
    void perf_event_disable(int fd);
    int perf_event_setup(int pid, int cpu, int group_fd, uint32_t type, uint64_t event_id);
    double sleep_ms(int time);
    void measure_latency();
    void perf_event_setup_mem_bw(int opcode);
    void perf_event_enable_mem_bw(int opcode);
    void perf_event_disable_mem_bw(int opcode);
    void perf_event_read_mem_bw(int opcode, double elapsed);
    void measure_bandwidth(int opcode);
    void measure_bandwidth_read();
    void measure_bandwidth_write();
    void measure_bandwidth_all();

  private:
    uint32_t num_sockets_;
    int sampling_period_ms_;
    double ewma_alpha_;
    std::vector<uint32_t> pmu_cha_type_;
    std::vector<uint32_t> pmu_imc_type_;
    // file_descriptor[socket][cha]
    std::vector<std::vector<int>> fd_rxc_occ_;
    std::vector<std::vector<int>> fd_rxc_ins_;
    std::vector<std::vector<int>> fd_cas_rd_;
    std::vector<std::vector<int>> fd_cas_wr_;
    std::vector<std::vector<int>> fd_cas_all_;
    std::vector<std::vector<uint64_t>> curr_count_occ_;
    std::vector<std::vector<uint64_t>> curr_count_ins_;
    std::vector<std::vector<uint64_t>> curr_count_rd_;
    std::vector<std::vector<uint64_t>> curr_count_wr_;
    std::vector<std::vector<double>> bw_read_;
    std::vector<std::vector<double>> bw_write_;

};


#endif
