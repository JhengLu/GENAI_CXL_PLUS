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
    num_sockets_ = NUM_SOCKETS;
    sampling_period_ms_ = SAMPLING_PERIOD_MS;
    ewma_alpha_ = EWMA_ALPHA;
    for (const auto &x : PMU_CHA_TYPE) {
        pmu_cha_type_.push_back(x);
    }
    for (const auto &x : PMU_IMC_TYPE) {
        pmu_imc_type_.push_back(x);
    }
    fd_rxc_occ_.resize(NUM_SOCKETS);
    fd_rxc_ins_.resize(NUM_SOCKETS);
    fd_cas_rd_.resize(NUM_SOCKETS);
    fd_cas_wr_.resize(NUM_SOCKETS);
    fd_cas_all_.resize(NUM_SOCKETS);
    curr_count_occ_ = std::vector<std::vector<uint64_t>>(num_sockets_,
        std::vector<uint64_t>(pmu_cha_type_.size(), 0));
    curr_count_ins_ = std::vector<std::vector<uint64_t>>(num_sockets_,
        std::vector<uint64_t>(pmu_cha_type_.size(), 0));
    curr_count_rd_ = std::vector<std::vector<uint64_t>>(num_sockets_,
        std::vector<uint64_t>(pmu_imc_type_.size(), 0));
    curr_count_wr_ = std::vector<std::vector<uint64_t>>(num_sockets_,
        std::vector<uint64_t>(pmu_imc_type_.size(), 0));
    bw_read_ = std::vector<std::vector<double>>(num_sockets_,
        std::vector<double>(pmu_imc_type_.size(), 0));
    bw_write_ = std::vector<std::vector<double>>(num_sockets_,
        std::vector<double>(pmu_imc_type_.size(), 0));
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
    std::this_thread::sleep_for(std::chrono::milliseconds(sampling_period_ms_));
    std::chrono::duration<double, std::milli> elapsed = std::chrono::high_resolution_clock::now() - start;
    return elapsed.count();
}

void Monitor::measure_latency() {
    // setup fd for each event on each PMU per socket
    for (int i = 0; i < num_sockets_; i++) {
        for (int j = 0; j < pmu_cha_type_.size(); j++) {
            int fd = perf_event_setup(-1, i, -1, pmu_cha_type_[j], EVENT_RxC_OCCUPANCY_IRQ);
            fd_rxc_occ_[i].push_back(fd);
            perf_event_reset(fd);

            fd = perf_event_setup(-1, i, -1, pmu_cha_type_[j], EVENT_RxC_INSERTS_IRQ);
            fd_rxc_ins_[i].push_back(fd);
            perf_event_reset(fd);
        }
    }

    // monitor all PMUs
    //std::vector<uint64_t> count_sum(2, 0);
    while (true) {
        for (int i = 0; i < num_sockets_; i++) {
            for (int j = 0; j < pmu_cha_type_.size(); j++) {
                perf_event_enable(fd_rxc_occ_[i][j]);
                perf_event_enable(fd_rxc_ins_[i][j]);
            }
        }

        sleep_ms(sampling_period_ms_);

        for (int i = 0; i < num_sockets_; i++) {
            for (int j = 0; j < pmu_cha_type_.size(); j++) {
                perf_event_disable(fd_rxc_occ_[i][j]);
                perf_event_disable(fd_rxc_ins_[i][j]);
            }
        }

        for (int i = 0; i < num_sockets_; i++) {
            for (int j = 0; j < pmu_cha_type_.size(); j++) {
                uint64_t count_occ = 0, count_ins = 0;     // TODO: need to construct a custom struct when we specify more read format later
                read(fd_rxc_occ_[i][j], &count_occ, sizeof(count_occ));
                read(fd_rxc_ins_[i][j], &count_ins, sizeof(count_ins));
                double latency_cycles = (double) (count_occ - curr_count_occ_[i][j]) / (count_ins - curr_count_ins_[i][j]);
                curr_count_occ_[i][j] = count_occ;
                curr_count_ins_[i][j] = count_ins;
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
        for (int i = 0; i < num_sockets_; i++) {
            for (int j = 0; j < pmu_imc_type_.size(); j++) {
                int fd = perf_event_setup(-1, i, -1, pmu_imc_type_[j], EVENT_CAS_COUNT_RD);
                fd_cas_rd_[i].push_back(fd);
                perf_event_reset(fd);
            }
        }
    } else if (opcode == 1) {
        for (int i = 0; i < num_sockets_; i++) {
            for (int j = 0; j < pmu_imc_type_.size(); j++) {
                int fd = perf_event_setup(-1, i, -1, pmu_imc_type_[j], EVENT_CAS_COUNT_WR);
                fd_cas_wr_[i].push_back(fd);
                perf_event_reset(fd);
            }
        }
    } else if (opcode == 2) {
        for (int i = 0; i < num_sockets_; i++) {
            for (int j = 0; j < pmu_imc_type_.size(); j++) {
                int fd = perf_event_setup(-1, i, -1, pmu_imc_type_[j], EVENT_CAS_COUNT_RD);
                fd_cas_rd_[i].push_back(fd);
                perf_event_reset(fd);
                fd = perf_event_setup(-1, i, -1, pmu_imc_type_[j], EVENT_CAS_COUNT_WR);
                fd_cas_wr_[i].push_back(fd);
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
        for (int i = 0; i < num_sockets_; i++) {
            for (int j = 0; j < pmu_imc_type_.size(); j++) {
                perf_event_enable(fd_cas_rd_[i][j]);
            }
        }
    } else if (opcode == 1) {
        for (int i = 0; i < num_sockets_; i++) {
            for (int j = 0; j < pmu_imc_type_.size(); j++) {
                perf_event_enable(fd_cas_wr_[i][j]);
            }
        }
    } else if (opcode == 2) {
        for (int i = 0; i < num_sockets_; i++) {
            for (int j = 0; j < pmu_imc_type_.size(); j++) {
                perf_event_enable(fd_cas_rd_[i][j]);
                perf_event_enable(fd_cas_wr_[i][j]);
            }
        }
    } else {
        assert(false);
    }
}

// opcode - 0: read, 1: write, 2: both
void Monitor::perf_event_disable_mem_bw(int opcode) {
    if (opcode == 0) {
        for (int i = 0; i < num_sockets_; i++) {
            for (int j = 0; j < pmu_imc_type_.size(); j++) {
                perf_event_disable(fd_cas_rd_[i][j]);
            }
        }
    } else if (opcode == 1) {
        for (int i = 0; i < num_sockets_; i++) {
            for (int j = 0; j < pmu_imc_type_.size(); j++) {
                perf_event_disable(fd_cas_wr_[i][j]);
            }
        }
    } else if (opcode == 2) {
        for (int i = 0; i < num_sockets_; i++) {
            for (int j = 0; j < pmu_imc_type_.size(); j++) {
                perf_event_disable(fd_cas_rd_[i][j]);
                perf_event_disable(fd_cas_wr_[i][j]);
            }
        }
    } else {
        assert(false);
    }
}

// opcode - 0: read, 1: write, 2: both
void Monitor::perf_event_read_mem_bw(int opcode, double elapsed_ms) {
    if (opcode == 0) {
        for (int i = 0; i < num_sockets_; i++) {
            for (int j = 0; j < pmu_imc_type_.size(); j++) {
                uint64_t count_rd = 0;     // TODO: need to construct a custom struct when we specify more read format later
                read(fd_cas_rd_[i][j], &count_rd, sizeof(count_rd));
                double curr_bw_rd = (count_rd - curr_count_rd_[i][j]) * 64 / 1024 / 1024 / (elapsed_ms / 1000);     // in MBps
                curr_count_rd_[i][j] = count_rd;
                bw_read_[i][j] = ewma_alpha_ * curr_bw_rd + (1 - ewma_alpha_) * bw_read_[i][j];
                std::cout << "socket[" << i << "]: imc" << j << "  \t BW_read = " << bw_read_[i][j] << " MBps" << std::endl;
            }
        }
        std::cout << std::endl;
    } else if (opcode == 1) {
        for (int i = 0; i < num_sockets_; i++) {
            for (int j = 0; j < pmu_imc_type_.size(); j++) {
                uint64_t count_wr = 0;     // TODO: need to construct a custom struct when we specify more read format later
                read(fd_cas_wr_[i][j], &count_wr, sizeof(count_wr));
                double curr_bw_wr = (count_wr - curr_count_wr_[i][j]) * 64 / 1024 / 1024 / (elapsed_ms / 1000);     // in MBps
                curr_count_wr_[i][j] = count_wr;
                bw_write_[i][j] = ewma_alpha_ * curr_bw_wr + (1 - ewma_alpha_) * bw_write_[i][j];

                std::cout << "socket[" << i << "]: imc" << j << "  \t BW_write = " << bw_write_[i][j] << " MBps" << std::endl;
            }
        }
        std::cout << std::endl;
    } else if (opcode == 2) {
        for (int i = 0; i < num_sockets_; i++) {
            for (int j = 0; j < pmu_imc_type_.size(); j++) {
                uint64_t count_rd = 0, count_wr = 0, count_all = 0;     // TODO: need to construct a custom struct when we specify more read format later
                read(fd_cas_rd_[i][j], &count_rd, sizeof(count_rd));
                double curr_bw_rd = (count_rd - curr_count_rd_[i][j]) * 64 / 1024 / 1024 / (elapsed_ms / 1000);     // in MBps
                curr_count_rd_[i][j] = count_rd;
                bw_read_[i][j] = ewma_alpha_ * curr_bw_rd + (1 - ewma_alpha_) * bw_read_[i][j];

                read(fd_cas_wr_[i][j], &count_wr, sizeof(count_wr));
                double curr_bw_wr = (count_wr - curr_count_wr_[i][j]) * 64 / 1024 / 1024 / (elapsed_ms / 1000);     // in MBps
                curr_count_wr_[i][j] = count_wr;
                bw_write_[i][j] = ewma_alpha_ * curr_bw_wr + (1 - ewma_alpha_) * bw_write_[i][j];

                std::cout << "socket[" << i << "]: imc" << j << "  \t BW_read = " << bw_read_[i][j]
                    << "MBps, BW_write = " << bw_write_[i][j] << "MBps, BW_all = " << bw_read_[i][j] + bw_write_[i][j] << " MBps" << std::endl;
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

        double elapsed = sleep_ms(sampling_period_ms_);

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
