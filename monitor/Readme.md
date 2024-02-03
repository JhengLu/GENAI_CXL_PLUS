## setup

```sh
mkdir build
cd build
cmake ..

```

## run
We can use either process name or process id to get the memory access latency for specific process.
```sh
./monitor_new --pname memtier_benchmark --GHZ 2.0
./monitor_new --pid 1234567 --GHZ 2.0
```
