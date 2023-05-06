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
#include <vector>

// A very simple sample code to make uncore monitoring work.
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

void perf_event_reset(int fd) {
    int ret = ioctl(fd, PERF_EVENT_IOC_RESET, 0);
    if (ret < 0) {
        std::cout << "[Error] perf_event_reset: " << strerror(errno) << std::endl;
    }
}

void perf_event_enable(int fd) {
    int ret = ioctl(fd, PERF_EVENT_IOC_ENABLE, 0);
    if (ret < 0) {
        std::cout << "[Error] perf_event_enable: " << strerror(errno) << std::endl;
    }
}

void perf_event_disable(int fd) {
    int ret = ioctl(fd, PERF_EVENT_IOC_DISABLE, 0);
    if (ret < 0) {
        std::cout << "[Error] perf_event_disable: " << strerror(errno) << std::endl;
    }
}

//int perf_event_setup(int cpu, int pid, int group_fd, int sampling_period) {
int perf_event_setup(int pid, int cpu, int group_fd, uint64_t event_id) {
    struct perf_event_attr event_attr;
	memset(&event_attr, 0, sizeof(event_attr));
	////event_attr.type = PERF_TYPE_RAW;
	event_attr.type = 25;
	//event_attr.type = PERF_TYPE_HARDWARE;
	event_attr.size = sizeof(event_attr);
	event_attr.config = event_id;
	////event_attr.config = PERF_COUNT_HW_INSTRUCTIONS;
	//event_attr.sample_period = sampling_period;
	//event_attr.sample_type = PERF_SAMPLE_TID | PERF_SAMPLE_ADDR;
	event_attr.disabled = 1;
	////event_attr.exclude_kernel = 1;
	////event_attr.inherit = 1;
	//event_attr.read_format = PERF_FORMAT_GROUP | PERF_FORMAT_ID;
	event_attr.precise_ip = 0;

    int ret = perf_event_open(&event_attr, pid, cpu, group_fd, 0);
    //assert(ret >= 0);
    if (ret < 0) {
        std::cout << "[Error] perf_event_open: " << strerror(errno) << std::endl;
    }
    return ret;
}

int main (int argc, char *argv[]) {
    int fd = perf_event_setup(-1, 0, -1, 0x0111);
    ////int fd = perf_event_setup(-1, 1, -1, 0x0111);
    //int fd = perf_event_setup(-1, 0, -1, 0x0113);
    //int fd = perf_event_setup(-1, 0, -1, 0x0000);
    //int fd = perf_event_setup(-1, 0, -1, 0x20D1);
    //int fd = perf_event_setup(-1, 0, -1, 0x0159);
    perf_event_reset(fd);
    //perf_event_enable(fd);
    //std::vector<int> array(1000);
    //for (int i = 0; i < array.size(); i++) {
    //    array[i] = i;
    //}
    //perf_event_disable(fd);

    // read results
    uint64_t count;     // TODO: need to construct a custom struct when we specify more read format later
    while (true) {
        perf_event_enable(fd);
        sleep(5);
        perf_event_disable(fd);
        read(fd, &count, sizeof(count));
        std::cout << "counter value = " << count << std::endl;
    }

    close(fd);

}
