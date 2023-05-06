#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <linux/perf_event.h>
#include <iostream>
#include <assert.h>
#include <errno.h>

#include "monitor.h"

// Takeaway (for uncore monitoring):
// (1) For perf_event_attr.type, specify the integer value for individual PMU instead of PERF_TYPE_RAW
// (2) Do not enable perf_event_attr.exclude_kernel
// (3) In the hardware I use, I need to set precise_ip = 0 (i.e., PEBS won't work).
//     Newer hardware may support PEBS for uncore monitoring.

static int perf_event_open(struct perf_event_attr *hw_event, pid_t pid,
                            int cpu, int group_fd, unsigned long flags) {
    int ret;
    ret = syscall(__NR_perf_event_open, hw_event, pid, cpu,
                    group_fd, flags);
    return ret;
}

Monitor::Monitor() {
    _num_sockets = NUM_SOCKETS;
    for (const auto &x : PMU_TYPE) {
        _pmu_type.push_back(x);
    }
    _fd.resize(NUM_SOCKETS);
}

Monitor::~Monitor() {
}

void Monitor::perf_event_reset(int fd) {
    int ret = ioctl(fd, PERF_EVENT_IOC_RESET, 0);
    if (ret < 0) {
        std::cout << "[Error] perf_event_reset: " << strerror(errno) << std::endl;
    }
}

void Monitor::perf_event_enable(int fd) {
    int ret = ioctl(fd, PERF_EVENT_IOC_ENABLE, 0);
    if (ret < 0) {
        std::cout << "[Error] perf_event_enable: " << strerror(errno) << std::endl;
    }
}

void Monitor::perf_event_disable(int fd) {
    int ret = ioctl(fd, PERF_EVENT_IOC_DISABLE, 0);
    if (ret < 0) {
        std::cout << "[Error] perf_event_disable: " << strerror(errno) << std::endl;
    }
}

//int perf_event_setup(int cpu, int pid, int group_fd, int sampling_period) {
int Monitor::perf_event_setup(int pid, int cpu, int group_fd, uint32_t type, uint64_t event_id) {
    struct perf_event_attr event_attr;
	memset(&event_attr, 0, sizeof(event_attr));
	////event_attr.type = PERF_TYPE_RAW;
	event_attr.type = 25;
	event_attr.size = sizeof(event_attr);
	event_attr.config = event_id;
	//event_attr.sample_period = sampling_period;
	//event_attr.sample_type = PERF_SAMPLE_TID | PERF_SAMPLE_ADDR;
	event_attr.disabled = 1;
	//event_attr.read_format = PERF_FORMAT_GROUP | PERF_FORMAT_ID;
	event_attr.precise_ip = 0;

    int ret = perf_event_open(&event_attr, pid, cpu, group_fd, 0);
    if (ret < 0) {
        std::cout << "[Error] perf_event_open: " << strerror(errno) << std::endl;
    }
    return ret;
}

void Monitor::measure_latency() {
    // setup fd for each PMU and per socket
    for (int i = 0; i < _num_sockets; i++) {
        for (int j = 0; j < _pmu_type.size(); j++) {
            int fd = perf_event_setup(-1, i, -1, _pmu_type[j], EVENT_RxC_INSERTS_IRQ);
            _fd[i].push_back(fd);
            perf_event_reset(fd);
        }
    }

    // monitor all PMUs
    uint64_t count;     // TODO: need to construct a custom struct when we specify more read format later
    while (true) {
        for (int i = 0; i < _num_sockets; i++) {
            for (int j = 0; j < _pmu_type.size(); j++) {
                perf_event_enable(_fd[i][j]);
            }
        }

        sleep(5);       // TODO: change

        for (int i = 0; i < _num_sockets; i++) {
            for (int j = 0; j < _pmu_type.size(); j++) {
                perf_event_disable(_fd[i][j]);
            }
        }

        for (int i = 0; i < _num_sockets; i++) {
            for (int j = 0; j < _pmu_type.size(); j++) {
                read(_fd[i][j], &count, sizeof(count));
                std::cout << "socket[" << i << "]: cha" << j << " count = " << count << std::endl;
            }
        }
    }
}

int main (int argc, char *argv[]) {
    Monitor monitor = Monitor();
    monitor.measure_latency();
}
