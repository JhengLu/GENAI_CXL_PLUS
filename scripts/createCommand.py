
'''
GRUB_CMDLINE_LINUX="isolcpus=0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60,62 memmap=110G!82G memmap=182G!202G"

the socket 1 starts from 192, the first 2 GB belongs to the system
'''

if __name__ == '__main__':
    startLocation = 192
    difference = 3
    initialCapicaty = 17
    iterationNumber = 9
    endLocation = 384
    for currentCapacity in range(initialCapicaty,initialCapicaty+difference*iterationNumber+1,difference):
        stopLocation = currentCapacity + startLocation +2
        leftCapacity = endLocation - stopLocation
        currentCommand = '''GRUB_CMDLINE_LINUX="isolcpus=0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60,62 memmap=110G!82G memmap=''' + str(leftCapacity) + 'G!' + str(stopLocation) + 'G\"'
        print("current capacity: " + str(currentCapacity))
        print(currentCommand)
        print()

