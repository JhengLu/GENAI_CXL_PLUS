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
#include <chrono>
#include <thread>

#include "monitor.h"

// Takeaway (for uncore monitoring):
// (1) For perf_event_attr.type, specify the integer value for individual PMU instead of PERF_TYPE_RAW
// (2) Do not enable perf_event_attr.exclude_kernel
// (3) In the hardware I use, I need to set precise_ip = 0 (i.e., PEBS won't work).
//     Newer hardware may support PEBS for uncore monitoring.

static int perf_event_open(struct perf_event_attr *hw_event, pid_t pid,
                            int cpu, int group_fd, unsigned long flags) {
    int ret = syscall(__NR_perf_event_open, hw_event, pid, cpu,
                    group_fd, flags);
    return ret;
}

Monitor::Monitor() {
    _num_sockets = NUM_SOCKETS;
    _sampling_period_ms = SAMPLING_PERIOD_MS;
    _ewma_alpha = EWMA_ALPHA;
    for (const auto &x : PMU_CHA_TYPE) {
        _pmu_cha_type.push_back(x);
    }
    for (const auto &x : PMU_IMC_TYPE) {
        _pmu_imc_type.push_back(x);
    }
    _fd_rxc_occ.resize(NUM_SOCKETS);
    _fd_rxc_ins.resize(NUM_SOCKETS);
    _fd_cas_rd.resize(NUM_SOCKETS);
    _fd_cas_wr.resize(NUM_SOCKETS);
    _fd_cas_all.resize(NUM_SOCKETS);
    _curr_count_occ = std::vector<std::vector<uint64_t>>(_num_sockets,
        std::vector<uint64_t>(_pmu_cha_type.size(), 0));
    _curr_count_ins = std::vector<std::vector<uint64_t>>(_num_sockets,
        std::vector<uint64_t>(_pmu_cha_type.size(), 0));
    _curr_count_rd = std::vector<std::vector<uint64_t>>(_num_sockets,
        std::vector<uint64_t>(_pmu_imc_type.size(), 0));
    _curr_count_wr = std::vector<std::vector<uint64_t>>(_num_sockets,
        std::vector<uint64_t>(_pmu_imc_type.size(), 0));
    _bw_read = std::vector<std::vector<double>>(_num_sockets,
        std::vector<double>(_pmu_imc_type.size(), 0));
    _bw_write = std::vector<std::vector<double>>(_num_sockets,
        std::vector<double>(_pmu_imc_type.size(), 0));
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
	event_attr.type = type;
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

double Monitor::sleep_ms(int time) {
    auto start = std::chrono::high_resolution_clock::now();
    std::this_thread::sleep_for(std::chrono::milliseconds(_sampling_period_ms));
    std::chrono::duration<double, std::milli> elapsed = std::chrono::high_resolution_clock::now() - start;
    return elapsed.count();
}

void Monitor::measure_latency() {
    // setup fd for each event on each PMU per socket
    for (int i = 0; i < _num_sockets; i++) {
        for (int j = 0; j < _pmu_cha_type.size(); j++) {
            int fd = perf_event_setup(-1, i, -1, _pmu_cha_type[j], EVENT_RxC_OCCUPANCY_IRQ);
            _fd_rxc_occ[i].push_back(fd);
            perf_event_reset(fd);

            fd = perf_event_setup(-1, i, -1, _pmu_cha_type[j], EVENT_RxC_INSERTS_IRQ);
            _fd_rxc_ins[i].push_back(fd);
            perf_event_reset(fd);
        }
    }

    // monitor all PMUs
    //std::vector<uint64_t> count_sum(2, 0);
    while (true) {
        for (int i = 0; i < _num_sockets; i++) {
            for (int j = 0; j < _pmu_cha_type.size(); j++) {
                perf_event_enable(_fd_rxc_occ[i][j]);
                perf_event_enable(_fd_rxc_ins[i][j]);
            }
        }

        sleep_ms(_sampling_period_ms);

        for (int i = 0; i < _num_sockets; i++) {
            for (int j = 0; j < _pmu_cha_type.size(); j++) {
                perf_event_disable(_fd_rxc_occ[i][j]);
                perf_event_disable(_fd_rxc_ins[i][j]);
            }
        }

        for (int i = 0; i < _num_sockets; i++) {
            for (int j = 0; j < _pmu_cha_type.size(); j++) {
                uint64_t count_occ = 0, count_ins = 0;     // TODO: need to construct a custom struct when we specify more read format later
                read(_fd_rxc_occ[i][j], &count_occ, sizeof(count_occ));
                read(_fd_rxc_ins[i][j], &count_ins, sizeof(count_ins));
                double latency_cycles = (double) (count_occ - _curr_count_occ[i][j]) / (count_ins - _curr_count_ins[i][j]);
                _curr_count_occ[i][j] = count_occ;
                _curr_count_ins[i][j] = count_ins;
                double latency_ns = latency_cycles / PROCESSOR_GHZ;

                std::cout << "socket[" << i << "]: cha" << j << "  \tRxC_OCCUPANCY.IRQ = " << count_occ
                    << ", RxC_INSERTS.IRQ = " << count_ins << ", latency = " << latency_ns << " ns" << std::endl;
                //count_sum[i] += count;
            }
            //std::cout << "socket[" << i << "]: total count = " << count_sum[i] << std::endl;
        }
        std::cout << std::endl;
    }
}

// opcode - 0: read, 1: write, 2: both
void Monitor::perf_event_setup_mem_bw(int opcode) {
    if (opcode == 0) {
        for (int i = 0; i < _num_sockets; i++) {
            for (int j = 0; j < _pmu_imc_type.size(); j++) {
                int fd = perf_event_setup(-1, i, -1, _pmu_imc_type[j], EVENT_CAS_COUNT_RD);
                _fd_cas_rd[i].push_back(fd);
                perf_event_reset(fd);
            }
        }
    } else if (opcode == 1) {
        for (int i = 0; i < _num_sockets; i++) {
            for (int j = 0; j < _pmu_imc_type.size(); j++) {
                int fd = perf_event_setup(-1, i, -1, _pmu_imc_type[j], EVENT_CAS_COUNT_WR);
                _fd_cas_wr[i].push_back(fd);
                perf_event_reset(fd);
            }
        }
    } else if (opcode == 2) {
        for (int i = 0; i < _num_sockets; i++) {
            for (int j = 0; j < _pmu_imc_type.size(); j++) {
                int fd = perf_event_setup(-1, i, -1, _pmu_imc_type[j], EVENT_CAS_COUNT_RD);
                _fd_cas_rd[i].push_back(fd);
                perf_event_reset(fd);
                fd = perf_event_setup(-1, i, -1, _pmu_imc_type[j], EVENT_CAS_COUNT_WR);
                _fd_cas_wr[i].push_back(fd);
                perf_event_reset(fd);
            }
        }
    } else {
        assert(false);
    }
}

// opcode - 0: read, 1: write, 2: both
void Monitor::perf_event_enable_mem_bw(int opcode) {
    if (opcode == 0) {
        for (int i = 0; i < _num_sockets; i++) {
            for (int j = 0; j < _pmu_imc_type.size(); j++) {
                perf_event_enable(_fd_cas_rd[i][j]);
            }
        }
    } else if (opcode == 1) {
        for (int i = 0; i < _num_sockets; i++) {
            for (int j = 0; j < _pmu_imc_type.size(); j++) {
                perf_event_enable(_fd_cas_wr[i][j]);
            }
        }
    } else if (opcode == 2) {
        for (int i = 0; i < _num_sockets; i++) {
            for (int j = 0; j < _pmu_imc_type.size(); j++) {
                perf_event_enable(_fd_cas_rd[i][j]);
                perf_event_enable(_fd_cas_wr[i][j]);
            }
        }
    } else {
        assert(false);
    }
}

// opcode - 0: read, 1: write, 2: both
void Monitor::perf_event_disable_mem_bw(int opcode) {
    if (opcode == 0) {
        for (int i = 0; i < _num_sockets; i++) {
            for (int j = 0; j < _pmu_imc_type.size(); j++) {
                perf_event_disable(_fd_cas_rd[i][j]);
            }
        }
    } else if (opcode == 1) {
        for (int i = 0; i < _num_sockets; i++) {
            for (int j = 0; j < _pmu_imc_type.size(); j++) {
                perf_event_disable(_fd_cas_wr[i][j]);
            }
        }
    } else if (opcode == 2) {
        for (int i = 0; i < _num_sockets; i++) {
            for (int j = 0; j < _pmu_imc_type.size(); j++) {
                perf_event_disable(_fd_cas_rd[i][j]);
                perf_event_disable(_fd_cas_wr[i][j]);
            }
        }
    } else {
        assert(false);
    }
}

// opcode - 0: read, 1: write, 2: both
void Monitor::perf_event_read_mem_bw(int opcode, double elapsed_ms) {
    if (opcode == 0) {
        for (int i = 0; i < _num_sockets; i++) {
            for (int j = 0; j < _pmu_imc_type.size(); j++) {
                uint64_t count_rd = 0;     // TODO: need to construct a custom struct when we specify more read format later
                read(_fd_cas_rd[i][j], &count_rd, sizeof(count_rd));
                double curr_bw_rd = (count_rd - _curr_count_rd[i][j]) * 64 / 1024 / 1024 / (elapsed_ms / 1000);     // in MBps
                _curr_count_rd[i][j] = count_rd;
                _bw_read[i][j] = _ewma_alpha * curr_bw_rd + (1 - _ewma_alpha) * _bw_read[i][j];
                std::cout << "socket[" << i << "]: imc" << j << "  \t BW_read = " << _bw_read[i][j] << " MBps" << std::endl;
            }
        }
        std::cout << std::endl;
    } else if (opcode == 1) {
        for (int i = 0; i < _num_sockets; i++) {
            for (int j = 0; j < _pmu_imc_type.size(); j++) {
                uint64_t count_wr = 0;     // TODO: need to construct a custom struct when we specify more read format later
                read(_fd_cas_wr[i][j], &count_wr, sizeof(count_wr));
                double curr_bw_wr = (count_wr - _curr_count_wr[i][j]) * 64 / 1024 / 1024 / (elapsed_ms / 1000);     // in MBps
                _curr_count_wr[i][j] = count_wr;
                _bw_write[i][j] = _ewma_alpha * curr_bw_wr + (1 - _ewma_alpha) * _bw_write[i][j];

                std::cout << "socket[" << i << "]: imc" << j << "  \t BW_write = " << _bw_write[i][j] << " MBps" << std::endl;
            }
        }
        std::cout << std::endl;
    } else if (opcode == 2) {
        for (int i = 0; i < _num_sockets; i++) {
            for (int j = 0; j < _pmu_imc_type.size(); j++) {
                uint64_t count_rd = 0, count_wr = 0, count_all = 0;     // TODO: need to construct a custom struct when we specify more read format later
                read(_fd_cas_rd[i][j], &count_rd, sizeof(count_rd));
                double curr_bw_rd = (count_rd - _curr_count_rd[i][j]) * 64 / 1024 / 1024 / (elapsed_ms / 1000);     // in MBps
                _curr_count_rd[i][j] = count_rd;
                _bw_read[i][j] = _ewma_alpha * curr_bw_rd + (1 - _ewma_alpha) * _bw_read[i][j];

                read(_fd_cas_wr[i][j], &count_wr, sizeof(count_wr));
                double curr_bw_wr = (count_wr - _curr_count_wr[i][j]) * 64 / 1024 / 1024 / (elapsed_ms / 1000);     // in MBps
                _curr_count_wr[i][j] = count_wr;
                _bw_write[i][j] = _ewma_alpha * curr_bw_wr + (1 - _ewma_alpha) * _bw_write[i][j];

                std::cout << "socket[" << i << "]: imc" << j << "  \t BW_read = " << _bw_read[i][j]
                    << "MBps, BW_write = " << _bw_write[i][j] << "MBps, BW_all = " << _bw_read[i][j] + _bw_write[i][j] << " MBps" << std::endl;
            }
        }
        std::cout << std::endl;
    } else {
        assert(false);
    }
}

// opcode - 0: read, 1: write, 2: both
void Monitor::measure_bandwidth(int opcode) {
    // setup fd for each event on each PMU per socket
    perf_event_setup_mem_bw(opcode);

    // monitor all PMUs
    while (true) {
        perf_event_enable_mem_bw(opcode);

        double elapsed = sleep_ms(_sampling_period_ms);

        perf_event_disable_mem_bw(opcode);
        perf_event_read_mem_bw(opcode, elapsed);
    }
}

void Monitor::measure_bandwidth_read() {
    measure_bandwidth(0);
}

void Monitor::measure_bandwidth_write() {
    measure_bandwidth(1);
}

void Monitor::measure_bandwidth_all() {
    measure_bandwidth(2);
}

int main (int argc, char *argv[]) {
    Monitor monitor = Monitor();
    monitor.measure_latency();
    //monitor.measure_bandwidth_all();
}
