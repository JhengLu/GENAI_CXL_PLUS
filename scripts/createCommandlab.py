'''
GRUB_CMDLINE_LINUX="isolcpus=28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111 memmap=110G!82G memmap=182G!202G"

this is for our lab's script
isolate the node 1 as the remote memory

node 0 cpus: 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 56 57 58 59 60 61 62 63 64 65 66 67 68 69 70 71 72 73 74 75 76 77 78 79 80 81 82 83
node 0 size: 515449 MB
node 0 free: 513508 MB
node 1 cpus: 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55 84 85 86 87 88 89 90 91 92 93 94 95 96 97 98 99 100 101 102 103 104 105 106 107 108 109 110 111
node 1 size: 515510 MB
node 1 free: 513447 MB

'''

if __name__ == '__main__':
    sysReserveCap = 2 * 1024
    WSSCapicaty = 22*1024  # the WSS
    localDRAM_endLocation = int(512 * 1024)
    remoteDRAM_capacity = 400 * 1024  # how much you want to allocate
    totalDRAM_capacity = 512 * 1024 * 2
    remoteDRAM_stopLocation = localDRAM_endLocation + remoteDRAM_capacity
    remoteDRAM_leftcapacity = totalDRAM_capacity - remoteDRAM_stopLocation
    for ratio in [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1]: # set the local DRAM according to the ratio of WSS
        localDRAM_Capacity = int(ratio * WSSCapicaty)
        localDRAM_stopLocation = int(0 + sysReserveCap + localDRAM_Capacity)
        localDRAM_leftCapacity = localDRAM_endLocation - localDRAM_stopLocation
        currentCommand = f'''GRUB_CMDLINE_LINUX="isolcpus=28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111 memmap={localDRAM_leftCapacity}M!{localDRAM_stopLocation}M memmap={remoteDRAM_leftcapacity}M!{remoteDRAM_stopLocation}M" '''
        print("current capacity: " + str(localDRAM_Capacity)+ " MB")
        print("node0 capacity: " + str(localDRAM_Capacity+sysReserveCap)+ " MB")
        print("ratio of WSS:" + str(ratio))
        print(currentCommand)
        print()

