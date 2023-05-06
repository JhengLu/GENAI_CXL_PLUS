#ifndef __MONITOR_H__
#define __MONITOR_H__

#include <linux/perf_event.h>
#include <vector>

// Event ID: UMask + EventSel. (https://perfmon-events.intel.com/skylake_server.html)
#define EVENT_RxC_OCCUPANCY_IRQ 0x0111UL
#define EVENT_RxC_INSERTS_IRQ 0x0113UL

#define PAGE_SIZE 4096UL
#define NUM_PERF_EVENT_MMAP_PAGES 64UL      // NOTE: might consider increase this value later on
#define SAMPLING_PERIOD 500UL               // in # of events

#define NUM_SOCKETS 2
// perf_event_attr.type value for each individual cha unit found in /sys/bus/event_source/devices/uncore_cha*/type
const uint32_t PMU_TYPE[] = {25, 26};


class Monitor {
  public:
    Monitor();
    ~Monitor();

    void perf_event_reset(int fd);
    void perf_event_enable(int fd);
    void perf_event_disable(int fd);
    int perf_event_setup(int pid, int cpu, int group_fd, uint32_t type, uint64_t event_id);
    void measure_latency();

  private:
    uint32_t _num_sockets;
    std::vector<uint32_t> _pmu_type;
    std::vector<std::vector<int>> _fd;     // file_descriptor[socket][cha]

};


#endif
