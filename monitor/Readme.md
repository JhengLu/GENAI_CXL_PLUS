## setup

```sh
mkdir build
cd build
cmake ..

```

## run
### memory monitor
We can use either process name or process id to get the memory access latency for specific process.
```sh
./monitor --mode la --pname memtier_benchmark --GHZ 2.0
./monitor --mode la <process_id> --GHZ 2.0
```
### B/W monitor
Prepare
```shell
pip install pqos==4.3.0
```
Run
```sh
./monitor --mode bw --pid <process_id>
./monitor --mode bw --pname redis-server
```
Detail for functions
```shell
# This function continuously prints the bandwidth (B/W) for a given process ID.
void print_process_bw(int processId);
# This function retrieves the bandwidth (B/W) information for a given process ID once.
BandwidthData detect_process_bw(int processId);
```
