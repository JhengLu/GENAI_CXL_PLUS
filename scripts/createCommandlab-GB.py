'''
GRUB_CMDLINE_LINUX="isolcpus=28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111 memmap=110G!82G memmap=182G!202G"

this is for our lab's script
isolate the node 1 as the remote memory

'''

if __name__ == '__main__':
    sysReserveCap = 2
    WSSCapicaty = 500  # the WSS
    localDRAM_endLocation = int(512)
    remoteDRAM_capacity = 400
    totalDRAM_capacity = 512 * 2
    remoteDRAM_stopLocation = localDRAM_endLocation + remoteDRAM_capacity
    remoteDRAM_leftcapacity = totalDRAM_capacity - remoteDRAM_stopLocation
    for ratio in [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1]: # set the local DRAM according to the ratio of WSS
        localDRAM_Capacity = int(ratio * WSSCapicaty)
        localDRAM_stopLocation = int(0 + sysReserveCap + localDRAM_Capacity)
        localDRAM_leftCapacity = localDRAM_endLocation - localDRAM_stopLocation
        currentCommand = f'''GRUB_CMDLINE_LINUX="isolcpus=28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111 memmap={localDRAM_leftCapacity}G!{localDRAM_stopLocation}G memmap={remoteDRAM_leftcapacity}G!{remoteDRAM_stopLocation}G rd.driver.blacklist=grub.nouveau" '''
        print("current capacity: " + str(localDRAM_Capacity)+ " GB")
        print("ratio of WSS:" + str(ratio))
        print("current total capacity:"+ str(localDRAM_stopLocation))
        print(currentCommand)
        print()

