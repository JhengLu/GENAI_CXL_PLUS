'''
this file consider node 1 as the local DRAM, since using ./mlc --latency_matrix, the result is
Using buffer size of 2000.000MiB
Measuring idle latencies (in ns)...
		Numa node
Numa node	     0	     1
       0	 139.1	 202.8
       1	 231.5	  84.3

GRUB_CMDLINE_LINUX="isolcpus=0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83 memmap=110G!82G memmap=182G!202G"

this is for our lab's script
isolate the node 1 as the remote memory

'''

if __name__ == '__main__':
    sysReserveCap = 2
    WSSCapicaty = 200  # the WSS
    oneDRAM_capacity = 512

    remoteDRAM_capacity = 400
    totalDRAM_capacity = int(512 * 2)
    localDRAM_endLocation = oneDRAM_capacity

    #remoteDRAM_stopLocation = localDRAM_endLocation + remoteDRAM_capacity
    remoteDRAM_stopLocation = remoteDRAM_capacity + oneDRAM_capacity
    remoteDRAM_leftCapacity = totalDRAM_capacity - remoteDRAM_stopLocation

    for ratio in [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1]: # set the local DRAM according to the ratio of WSS
        localDRAM_Capacity = int(ratio * WSSCapicaty)
        localDRAM_stopLocation = int(sysReserveCap + localDRAM_Capacity)
        localDRAM_leftCapacity = localDRAM_endLocation - localDRAM_stopLocation
        currentCommand = f'''GRUB_CMDLINE_LINUX="isolcpus=28-55,84-111 memmap={localDRAM_leftCapacity}G!{localDRAM_stopLocation}G memmap={remoteDRAM_leftCapacity}G!{remoteDRAM_stopLocation}G" '''
        print("current capacity: " + str(localDRAM_Capacity)+ " GB")
        print("ratio of WSS:" + str(ratio))
        print("current total capacity:"+ str(localDRAM_stopLocation-sysReserveCap))
        print(currentCommand)
        print()

