#include "monitor.h"

#include <assert.h>
#include <errno.h>
#include <getopt.h>
#include <linux/perf_event.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>
#include <tuple>

Monitor monitor = Monitor();
std::vector<int> cores_g;

// Takeaways (for uncore monitoring):
// (1) For perf_event_attr.type, specify the integer value for individual PMU instead of PERF_TYPE_RAW
// (2) Do not enable perf_event_attr.exclude_kernel
// (3) In the hardware I use, I need to set precise_ip = 0 (i.e., PEBS won't work).
//     Newer hardware may support PEBS for uncore monitoring.
// (4) uncore monitoring is per-socket (i.e., no per-core or per-process monitoring)

static int perf_event_open(struct perf_event_attr *hw_event, pid_t pid,
                           int cpu, int group_fd, unsigned long flags) {
    int ret = syscall(__NR_perf_event_open, hw_event, pid, cpu,
                      group_fd, flags);
    return ret;
}

ApplicationInfo::ApplicationInfo(std::string app_name) {
    pid = -1;
    process_exists = false;
    name = app_name;
}

ApplicationInfo::~ApplicationInfo() {
}

LatencyInfoPerCore::LatencyInfoPerCore(int cpu_id) {
    cpu_id = cpu_id;
    fd_l1d_pend_miss = -1;
    fd_retired_l3_miss = -1;
    curr_count_l1d_pend_miss = 0;
    curr_count_retired_l3_miss = 0;
}

LatencyInfoPerCore::~LatencyInfoPerCore() {
}

LatencyInfoPerProcess::LatencyInfoPerProcess() {
    pid = -1;
    fd_occupancy_ia_miss = -1;
    fd_inserts_ia_miss = -1;
    curr_count_occupancy_ia_miss = 0;
    curr_count_inserts_ia_miss = 0;
}



LatencyInfoPerProcess::LatencyInfoPerProcess(int pid) {
    pid = pid;
    fd_occupancy_ia_miss = -1;
    fd_inserts_ia_miss = -1;
    curr_count_occupancy_ia_miss = 0;
    curr_count_inserts_ia_miss = 0;
}

LatencyInfoPerProcess::~LatencyInfoPerProcess() {
}

BWInfoPerCore::BWInfoPerCore(int cpu_id) {
    cpu_id = cpu_id;
    fd_offcore_all_reqs = -1;
    curr_count_offcore_all_reqs = 0;
    curr_bw = 0;
}

BWInfoPerCore::~BWInfoPerCore() {
}

PageTempInfoPerCore::PageTempInfoPerCore(int cpu_id, int num_events) {
    cpu_id = cpu_id;
    fds.resize(num_events, -1);
    perf_m_pages.resize(num_events, NULL);
}

// TODO: consider calling munmap for perf pages
PageTempInfoPerCore::~PageTempInfoPerCore() {
}

template<typename A, typename B>
std::pair<B,A> flip_pair(const std::pair<A,B> &p)
{
    return std::pair<B,A>(p.second, p.first);
}

template<typename A, typename B>
std::multimap<B,A> flip_map(const std::map<A,B> &src)
{
    std::multimap<B,A> dst;
    std::transform(src.begin(), src.end(), std::inserter(dst, dst.begin()),
                   flip_pair<A,B>);
    return dst;
}

Monitor::Monitor() {
    num_sockets_ = NUM_SOCKETS;
    sampling_period_ms_ = SAMPLING_PERIOD_MS;
    sampling_period_event_ = SAMPLING_PERIOD_EVENT;
    ewma_alpha_ = EWMA_ALPHA;
    num_cpu_throttle_ = 0;
    num_cpu_unthrottle_ = 0;
    num_local_access_ = 0;
    num_remote_access_ = 0;
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

    for (int i = 0; i < NUM_CORES; i++) {
        lat_info_cpu_.emplace_back(LatencyInfoPerCore(i));
    }

    for (int i = 0; i < NUM_CORES; i++) {
        bw_info_cpu_.emplace_back(BWInfoPerCore(i));
    }

    page_temp_events_ = {EVENT_MEM_LOAD_L3_MISS_RETIRED_LOCAL_DRAM, EVENT_MEM_LOAD_L3_MISS_RETIRED_REMOTE_DRAM};
    for (int i = 0; i < NUM_CORES; i++) {
        page_temp_info_.emplace_back(PageTempInfoPerCore(i, page_temp_events_.size()));
    }

}

Monitor::~Monitor() {
    // TODO: delete ApplicationInfo *
}

void Monitor::add_application(ApplicationInfo *app_info) {
    // find pid by app name
    int pid = -1;
    bool found_pid = false;
    while (!found_pid) {
        pid = get_pid_from_proc_name(app_info->name);
        if (pid != -1) {
            found_pid = true;
        }
    }
    app_info->pid = pid;
    app_info->process_exists = true;

    application_info_[app_info->pid] = app_info;

    // update the core list to monitor b/w
    for (const auto &c : app_info->bw_cores) {
        if (bw_core_list_.contains(c)) {
            std::cout << "[Error] add_application: bw core (" << c
                      << ") already exists in monitor's bw core list" << std::endl;
        }
        bw_core_list_.insert(c);
    }
}

void Monitor::perf_event_reset(int fd) {
    int ret = ioctl(fd, PERF_EVENT_IOC_RESET, 0);
    if (ret < 0) {
        std::cout << "[Error] perf_event_reset: " << strerror(errno) << std::endl;
    }
}

void Monitor::

perf_event_enable(int fd) {
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

// precise_ip (since Linux 2.6.35)
//              This controls the amount of skid.  Skid is how many instructions execute between an
//              event of interest happening and the kernel being able to stop and record the event.
//              Smaller  skid  is  better  and  allows  more  accurate  reporting  of  which events
//              correspond to which instructions, but hardware is often limited with how small this
//              can be.
//
//              The possible values of this field are the following:
//
//              0  SAMPLE_IP can have arbitrary skid.
int Monitor::perf_event_setup(int pid, int cpu, int group_fd, uint32_t type, uint64_t event_id) {
    struct perf_event_attr event_attr;
    memset(&event_attr, 0, sizeof(event_attr));
    event_attr.type = type;
    event_attr.size = sizeof(event_attr);
    event_attr.config = event_id;
    event_attr.disabled = 1;
    event_attr.inherit = 1;     // includes child process
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

int Monitor::get_pid_from_proc_name(std::string proc_name) {
    std::string cmd = "pidof " + proc_name;
    char pidline[1024] = "";
    FILE *fp = popen(cmd.c_str(), "r");
    fgets(pidline, 1024, fp);

    if (pidline && !pidline[0]) {   // check empty c string
        pclose(fp);
        return -1;
    }

    int pid = strtoul(pidline, NULL, 10);
    pclose(fp);

    return pid;
}



void Monitor::perf_event_setup_process_latency(int pid) {
    auto latinfo = LatencyInfoPerProcess(pid);
    lat_info_process_[pid] = latinfo;

    //  pid > 0 and cpu == -1   This measures the specified process/thread on any CPU.
    //The group_fd argument allows event groups to be created.  An event group has one event which is the group leader.  The leader is created first, with group_fd = -1.

    // config: This  specifies  which  event  you  want,  in conjunction with the type field.
    // If type is PERF_TYPE_RAW, then a custom "raw" config value is  needed.
    // Most  CPUs support  events  that  are  not  covered  by  the  "generalized" events.  These are implementation defined; see your CPU  manual
    int fd = perf_event_setup(pid, -1, -1, PERF_TYPE_RAW, EVENT_TOR_OCCUPANCY_IA_MISS_DRD);
    lat_info_process_[pid].fd_occupancy_ia_miss = fd;
    perf_event_reset(fd);

    fd = perf_event_setup(pid, -1, -1, PERF_TYPE_RAW, EVENT_TOR_INSERTS_IA_MISS_DRD);
    lat_info_process_[pid].fd_inserts_ia_miss = fd;
    perf_event_reset(fd);
}

void Monitor::perf_event_enable_process_latency(int pid) {
    perf_event_enable(lat_info_process_[pid].fd_occupancy_ia_miss);
    perf_event_enable(lat_info_process_[pid].fd_inserts_ia_miss);
}

void Monitor::perf_event_disable_process_latency(int pid) {
    perf_event_disable(lat_info_process_[pid].fd_occupancy_ia_miss);
    perf_event_disable(lat_info_process_[pid].fd_inserts_ia_miss);
}

void Monitor::perf_event_read_process_latency(int pid, double GHZ, bool log_latency, ApplicationInfo *app_info) {
    uint64_t count_occupancy_ia_miss = 0, count_inserts_ia_miss = 0;
    read(lat_info_process_[pid].fd_occupancy_ia_miss, &count_occupancy_ia_miss, sizeof(count_occupancy_ia_miss));
    read(lat_info_process_[pid].fd_inserts_ia_miss, &count_inserts_ia_miss, sizeof(count_inserts_ia_miss));
    //double latency_cycles = (double) (count_l1d_pend_miss - lat_info_process_[pid].curr_count_l1d_pend_miss)
     //                       / (count_retired_l3_miss - lat_info_process_[pid].curr_count_retired_l3_miss);
    //lat_info_process_[pid].curr_count_occupancy_ia_miss = count_l1d_pend_miss;
    //lat_info_process_[pid].curr_count_inserts_ia_miss = count_retired_l3_miss;
    // double latency_ns = latency_cycles / GHZ;
    double latency_ns = count_occupancy_ia_miss / count_inserts_ia_miss;
    if (log_latency) {
        sampled_process_lat_.push_back(latency_ns);
    }
    if (app_info) {
        std::cout << "App (\"" << app_info->name <<  "\"): latency = " << latency_ns << " ns" << std::endl;
    } else {
        std::cout << "process [" << pid << "]: latency = " << latency_ns << " ns" << std::endl;
    }
}

void Monitor::measure_process_latency(int pid, double GHZ) {
    perf_event_setup_process_latency(pid);

    for (;;) {
        perf_event_enable_process_latency(pid);

        sleep_ms(sampling_period_ms_);

        perf_event_disable_process_latency(pid);
        perf_event_read_process_latency(pid,GHZ);
    }
}

void Monitor::measure_process_latency(std::string proc_name, double GHZ) {
    // get pid first via process name
    int pid = -1;
    bool found_pid = false;
    while (!found_pid) {
        pid = get_pid_from_proc_name(proc_name);
        if (pid != -1) {
            std::cout << "Start measuring latency for " << proc_name << "(" << pid << ") ..." << std::endl;
            found_pid = true;
        }
    }

    // measure latency given the pid; always check if the process still exists
    bool process_exists = true;

    perf_event_setup_process_latency(pid);

    while (process_exists) {
        perf_event_enable_process_latency(pid);

        sleep_ms(sampling_period_ms_);

        perf_event_disable_process_latency(pid);
        perf_event_read_process_latency(pid, GHZ);

        if (get_pid_from_proc_name(proc_name) == -1) {
            process_exists = false;
        }
    }

    if (!sampled_process_lat_.empty()) {
        double sum_lat = 0;
        std::cout << "sampled latency = [";
        int i = 0;
        for (; i < sampled_process_lat_.size() - 1; i++) {
            std::cout << int(sampled_process_lat_[i]) << ",";
            sum_lat += sampled_process_lat_[i];
        }
        std::cout << int(sampled_process_lat_[i]) << "]" << std::endl;
        sum_lat += sampled_process_lat_[i];
        std::sort(sampled_process_lat_.begin() + sampled_process_lat_.size() * 0.1, sampled_process_lat_.begin() + sampled_process_lat_.size() * 0.9);
        double avg_lat = sum_lat / sampled_process_lat_.size();
        int medium = sampled_process_lat_[sampled_process_lat_.size() * 0.5];
        std::cout << "avg sampled latency = " << avg_lat << std::endl;
        std::cout << "Medium Sampled Latency = " << medium << std::endl;
    }

    std::cout << proc_name << " no longer exists. Stop measuring." << std::endl;
}

void Monitor::measure_application_latency() {
    for (const auto &[pid, app] : application_info_) {
        perf_event_setup_process_latency(pid);
    }

    for (;;) {
        for (const auto &[pid, app] : application_info_) {
            if (app->process_exists) {
                perf_event_enable_process_latency(pid);
            }
        }

        sleep_ms(sampling_period_ms_);

        for (const auto &[pid, app] : application_info_) {
            if (app->process_exists) {
                perf_event_disable_process_latency(pid);
                perf_event_read_process_latency(pid, false, app);
            }
        }

        for (const auto &[pid, app] : application_info_) {
            if (get_pid_from_proc_name(app->name) == -1) {
                app->process_exists = false;
            }
            //} else {
            //    app->process_exists = true;     // shall we do this?
            //}
        }
    }
}



int main (int argc, char *argv[]) {
    ////Monitor monitor = Monitor();        // moved to global to make signal handler work
    ////std::vector<int> cores = {0};       // moved to global to make signal handler work

    for (int i = 0; i < NUM_CORES; i++) {
        cores_g.push_back(i);
    }

    // Adding a variable for GHZ
    double ghz = -1.0; // Default to an invalid value





    int processId = -1; // Default to an invalid process ID
    std::string processName = "";

    // Iterate over all arguments to find --pid
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--pid") == 0) {
            if (i + 1 < argc) { // Make sure there's an argument after --pid
                processId = std::atoi(argv[i + 1]); // Convert the next argument to an integer
                std::cout<<"pid: "<<processId<<std::endl;

            } else {
                // Handle error: --pid flag is present, but no value is specified
                std::cerr << "Error: --pid flag requires a value." << std::endl;
                return 1; // Exit with an error code
            }
        }
        else if (strcmp(argv[i], "--pname") == 0){
            if (i + 1 < argc) { // Make sure there's an argument after --pname
                processName = argv[i + 1]; // Convert the next argument to an integer
                std::cout<<"pname: "<<processName<<std::endl;

            } else {
                // Handle error: --processid flag is present, but no value is specified
                std::cerr << "Error: --pname flag requires a value." << std::endl;
                return 1; // Exit with an error code
            }
        }
        else if (strcmp(argv[i], "--GHZ") == 0) {
            if (i + 1 < argc) { // Make sure there's an argument after --GHZ
                ghz = std::atof(argv[i + 1]); // Convert the next argument to a double
                if (ghz <= 0.0) {
                    std::cerr << "Error: Invalid value for --GHZ. Must be a positive number." << std::endl;
                    return 1; // Exit with an error code
                }
                std::cout << "GHZ: " << ghz << std::endl;

            } else {
                // Handle error: --GHZ flag is present, but no value is specified
                std::cerr << "Error: --GHZ flag requires a numeric value." << std::endl;
                return 1; // Exit with an error code
            }
        }
    }

    // Check if GHZ value was provided
    if (ghz == -1.0) {
        std::cerr << "Error: --GHZ argument is required. use lscpu | grep CPU to find the value" << std::endl;
        return 1; // Exit with an error code
    }

    if (processId > -1) {
        monitor.measure_process_latency(processId,ghz);
    } else if (!processName.empty()) {
        monitor.measure_process_latency(processName,ghz);
    } else {
        std::cerr << "No valid process identifier provided." << std::endl;
        return 1;
    }


    // monitor.measure_process_latency("memtier_benchmark");
    //monitor.measure_process_latency("redis-server");
    //monitor.measure_process_latency("bc");


}

