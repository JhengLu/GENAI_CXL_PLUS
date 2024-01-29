import re
import numpy as np

# Sample data
data = """
TIME 2024-01-26 19:15:54
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.72      62818k     40040.0      4264.2         1.9
TIME 2024-01-26 19:15:55
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.68      61458k     40432.0      4172.9         0.3
TIME 2024-01-26 19:15:56
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.74      63965k     39760.0      4332.0         1.5
TIME 2024-01-26 19:15:57
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.67      61741k     40432.0      4203.5         7.6
TIME 2024-01-26 19:15:58
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.74      64861k     40152.0      4370.2         0.1
TIME 2024-01-26 19:15:59
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.66      60376k     40376.0      4119.2         1.9
TIME 2024-01-26 19:16:00
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.74      66392k     40488.0      4517.8         2.4
TIME 2024-01-26 19:16:01
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.68      58345k     40096.0      3977.3         0.1
TIME 2024-01-26 19:16:02
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.74      66877k     39984.0      4494.4         1.8
TIME 2024-01-26 19:16:03
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.67      57384k     40320.0      3910.5         1.9
TIME 2024-01-26 19:16:04
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.75      67308k     39984.0      4525.4         0.1
TIME 2024-01-26 19:16:05
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.68      56344k     39816.0      3872.3         1.8
TIME 2024-01-26 19:16:06
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.77      63518k     42952.0      4288.8         1.6
TIME 2024-01-26 19:16:07
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.64      61586k     39760.0      4203.6         0.0
TIME 2024-01-26 19:16:08
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.84      58486k     41664.0      4001.6         1.8
TIME 2024-01-26 19:16:09
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.56      67121k     40488.0      4535.1         1.8
TIME 2024-01-26 19:16:10
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.87      57405k     42224.0      3992.6         0.4
TIME 2024-01-26 19:16:11
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.53      68434k     40488.0      4623.1         1.9
TIME 2024-01-26 19:16:12
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.85      57898k     41608.0      4060.2         1.9
TIME 2024-01-26 19:16:13
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.57      65814k     39928.0      4387.4         0.1
TIME 2024-01-26 19:16:14
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.81      59296k     40824.0      4175.9         1.7
TIME 2024-01-26 19:16:15
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.58      66705k     40208.0      4437.0         1.9
TIME 2024-01-26 19:16:16
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.78      59990k     41216.0      4204.5         0.0
TIME 2024-01-26 19:16:17
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.64      63753k     39872.0      4248.3         1.8
TIME 2024-01-26 19:16:18
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.74      61869k     41160.0      4325.0         2.0
TIME 2024-01-26 19:16:19
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.67      62052k     39704.0      4108.2         0.0
TIME 2024-01-26 19:16:20
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.70      63626k     42000.0      4420.6         2.4
TIME 2024-01-26 19:16:21
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.71      61940k     40600.0      4165.3         1.8
TIME 2024-01-26 19:16:22
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.69      62518k     40488.0      4287.0         0.0
TIME 2024-01-26 19:16:23
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.74      62391k     39984.0      4242.4         1.9
TIME 2024-01-26 19:16:24
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.68      61879k     40376.0      4240.7         0.3
TIME 2024-01-26 19:16:25
    CORE         IPC      MISSES     LLC[KB]   MBL[MB/s]   MBR[MB/s]
28-55,84        1.73      63934k     40824.0      4368.5         1.5





"""

# Regular expression to extract the MBL[MB/s] data
mbl_pattern = re.compile(r"28-55,84.*?\s+[\d.]+\s+[\d.]+k+\s+[\d.]+\s+([\d.]+)+\s+[\d.]+")


# Extracting the MBL values
mbl_values = [float(match) for match in mbl_pattern.findall(data)]

# Calculating the median
median_mbl = np.median(mbl_values)
print("median: " + str(median_mbl))

# Calculate the average of MBL values
average_mbl = np.mean(mbl_values)

print(f"Average MBL: {average_mbl} MB/s")

