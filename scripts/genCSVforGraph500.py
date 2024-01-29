import re
import csv

# Sample data
data = """
Time for BFS 1 is 24.335974
Current Time of Time for BFS: Mon Jan  8 17:29:57 2024

TEPS for BFS 1 is 3.52971e+08
Running BFS 2
Current Time of Running BFS: Mon Jan  8 17:30:00 2024

Time for BFS 2 is 25.540105
Current Time of Time for BFS: Mon Jan  8 17:30:25 2024

TEPS for BFS 2 is 3.3633e+08
Running BFS 3
Current Time of Running BFS: Mon Jan  8 17:30:28 2024

Time for BFS 3 is 26.128252
Current Time of Time for BFS: Mon Jan  8 17:30:54 2024

TEPS for BFS 3 is 3.28759e+08
Running BFS 4
Current Time of Running BFS: Mon Jan  8 17:30:56 2024

Time for BFS 4 is 21.556163
Current Time of Time for BFS: Mon Jan  8 17:31:18 2024

TEPS for BFS 4 is 3.98489e+08
Running BFS 5
Current Time of Running BFS: Mon Jan  8 17:31:20 2024

Time for BFS 5 is 21.220935
Current Time of Time for BFS: Mon Jan  8 17:31:42 2024

TEPS for BFS 5 is 4.04784e+08
Running BFS 6
Current Time of Running BFS: Mon Jan  8 17:31:44 2024

Time for BFS 6 is 22.706019
Current Time of Time for BFS: Mon Jan  8 17:32:07 2024

TEPS for BFS 6 is 3.78309e+08
Running BFS 7
Current Time of Running BFS: Mon Jan  8 17:32:09 2024

Time for BFS 7 is 23.987361
Current Time of Time for BFS: Mon Jan  8 17:32:33 2024

TEPS for BFS 7 is 3.58101e+08
Running BFS 8
Current Time of Running BFS: Mon Jan  8 17:32:36 2024

Time for BFS 8 is 26.479181
Current Time of Time for BFS: Mon Jan  8 17:33:02 2024

TEPS for BFS 8 is 3.24402e+08
Running BFS 9
Current Time of Running BFS: Mon Jan  8 17:33:05 2024

Time for BFS 9 is 24.674694
Current Time of Time for BFS: Mon Jan  8 17:33:29 2024

TEPS for BFS 9 is 3.48126e+08
Running BFS 10
Current Time of Running BFS: Mon Jan  8 17:33:32 2024

Time for BFS 10 is 22.504436
Current Time of Time for BFS: Mon Jan  8 17:33:54 2024

TEPS for BFS 10 is 3.81698e+08
Running BFS 11
Current Time of Running BFS: Mon Jan  8 17:33:57 2024

Time for BFS 11 is 21.671113
Current Time of Time for BFS: Mon Jan  8 17:34:19 2024

TEPS for BFS 11 is 3.96376e+08
Running BFS 12
Current Time of Running BFS: Mon Jan  8 17:34:21 2024

Time for BFS 12 is 25.079201
Current Time of Time for BFS: Mon Jan  8 17:34:46 2024

TEPS for BFS 12 is 3.42511e+08
Running BFS 13
Current Time of Running BFS: Mon Jan  8 17:34:49 2024

Time for BFS 13 is 27.885794
Current Time of Time for BFS: Mon Jan  8 17:35:17 2024

TEPS for BFS 13 is 3.08039e+08
Running BFS 14
Current Time of Running BFS: Mon Jan  8 17:35:19 2024

Time for BFS 14 is 21.897004
Current Time of Time for BFS: Mon Jan  8 17:35:41 2024

TEPS for BFS 14 is 3.92287e+08
Running BFS 15
Current Time of Running BFS: Mon Jan  8 17:35:43 2024

Time for BFS 15 is 23.705660
Current Time of Time for BFS: Mon Jan  8 17:36:07 2024

TEPS for BFS 15 is 3.62357e+08
Running BFS 16
Current Time of Running BFS: Mon Jan  8 17:36:09 2024

Time for BFS 16 is 21.793645
Current Time of Time for BFS: Mon Jan  8 17:36:31 2024

TEPS for BFS 16 is 3.94147e+08
Running BFS 17
Current Time of Running BFS: Mon Jan  8 17:36:34 2024

Time for BFS 17 is 22.923866
Current Time of Time for BFS: Mon Jan  8 17:36:57 2024

TEPS for BFS 17 is 3.74714e+08
Running BFS 18
Current Time of Running BFS: Mon Jan  8 17:36:59 2024

Time for BFS 18 is 26.424811
Current Time of Time for BFS: Mon Jan  8 17:37:26 2024

TEPS for BFS 18 is 3.2507e+08
Running BFS 19
Current Time of Running BFS: Mon Jan  8 17:37:28 2024

Time for BFS 19 is 26.166497
Current Time of Time for BFS: Mon Jan  8 17:37:54 2024

TEPS for BFS 19 is 3.28279e+08
Running BFS 20
Current Time of Running BFS: Mon Jan  8 17:37:57 2024

Time for BFS 20 is 23.820353
Current Time of Time for BFS: Mon Jan  8 17:38:21 2024

TEPS for BFS 20 is 3.60612e+08
Running BFS 21
Current Time of Running BFS: Mon Jan  8 17:38:23 2024

Time for BFS 21 is 24.958683
Current Time of Time for BFS: Mon Jan  8 17:38:48 2024

TEPS for BFS 21 is 3.44165e+08
Running BFS 22
Current Time of Running BFS: Mon Jan  8 17:38:50 2024

Time for BFS 22 is 26.444801
Current Time of Time for BFS: Mon Jan  8 17:39:17 2024

TEPS for BFS 22 is 3.24824e+08
Running BFS 23
Current Time of Running BFS: Mon Jan  8 17:39:19 2024

Time for BFS 23 is 22.152352
Current Time of Time for BFS: Mon Jan  8 17:39:42 2024

TEPS for BFS 23 is 3.87765e+08
Running BFS 24
Current Time of Running BFS: Mon Jan  8 17:39:44 2024

Time for BFS 24 is 24.069428
Current Time of Time for BFS: Mon Jan  8 17:40:08 2024

TEPS for BFS 24 is 3.5688e+08
Running BFS 25
Current Time of Running BFS: Mon Jan  8 17:40:11 2024

Time for BFS 25 is 21.416342
Current Time of Time for BFS: Mon Jan  8 17:40:32 2024

TEPS for BFS 25 is 4.01091e+08
Running BFS 26
Current Time of Running BFS: Mon Jan  8 17:40:34 2024

Time for BFS 26 is 24.678582
Current Time of Time for BFS: Mon Jan  8 17:40:59 2024

TEPS for BFS 26 is 3.48071e+08
Running BFS 27
Current Time of Running BFS: Mon Jan  8 17:41:02 2024

Time for BFS 27 is 22.733503
Current Time of Time for BFS: Mon Jan  8 17:41:24 2024

TEPS for BFS 27 is 3.77852e+08
Running BFS 28
Current Time of Running BFS: Mon Jan  8 17:41:27 2024

Time for BFS 28 is 22.315193
Current Time of Time for BFS: Mon Jan  8 17:41:49 2024

TEPS for BFS 28 is 3.84935e+08
Running BFS 29
Current Time of Running BFS: Mon Jan  8 17:41:51 2024

Time for BFS 29 is 25.009425
Current Time of Time for BFS: Mon Jan  8 17:42:16 2024

TEPS for BFS 29 is 3.43467e+08
Running BFS 30
Current Time of Running BFS: Mon Jan  8 17:42:19 2024

Time for BFS 30 is 23.696292
Current Time of Time for BFS: Mon Jan  8 17:42:43 2024

TEPS for BFS 30 is 3.625e+08
Running BFS 31
Current Time of Running BFS: Mon Jan  8 17:42:45 2024

Time for BFS 31 is 25.703268
Current Time of Time for BFS: Mon Jan  8 17:43:11 2024

TEPS for BFS 31 is 3.34195e+08
Running BFS 32
Current Time of Running BFS: Mon Jan  8 17:43:13 2024

Time for BFS 32 is 25.725191
Current Time of Time for BFS: Mon Jan  8 17:43:39 2024

TEPS for BFS 32 is 3.3391e+08
Running BFS 33
Current Time of Running BFS: Mon Jan  8 17:43:42 2024

Time for BFS 33 is 26.948200
Current Time of Time for BFS: Mon Jan  8 17:44:08 2024

TEPS for BFS 33 is 3.18756e+08
Running BFS 34
Current Time of Running BFS: Mon Jan  8 17:44:11 2024

Time for BFS 34 is 24.924053
Current Time of Time for BFS: Mon Jan  8 17:44:36 2024

TEPS for BFS 34 is 3.44643e+08
Running BFS 35
Current Time of Running BFS: Mon Jan  8 17:44:38 2024

Time for BFS 35 is 24.460226
Current Time of Time for BFS: Mon Jan  8 17:45:03 2024

TEPS for BFS 35 is 3.51178e+08
Running BFS 36
Current Time of Running BFS: Mon Jan  8 17:45:05 2024

Time for BFS 36 is 26.362999
Current Time of Time for BFS: Mon Jan  8 17:45:32 2024

TEPS for BFS 36 is 3.25832e+08
Running BFS 37
Current Time of Running BFS: Mon Jan  8 17:45:34 2024

Time for BFS 37 is 21.304244
Current Time of Time for BFS: Mon Jan  8 17:45:55 2024

TEPS for BFS 37 is 4.03201e+08
Running BFS 38
Current Time of Running BFS: Mon Jan  8 17:45:58 2024

Time for BFS 38 is 24.386982
Current Time of Time for BFS: Mon Jan  8 17:46:22 2024

TEPS for BFS 38 is 3.52233e+08
Running BFS 39
Current Time of Running BFS: Mon Jan  8 17:46:25 2024

Time for BFS 39 is 22.689305
Current Time of Time for BFS: Mon Jan  8 17:46:48 2024

TEPS for BFS 39 is 3.78588e+08
Running BFS 40
Current Time of Running BFS: Mon Jan  8 17:46:50 2024

Time for BFS 40 is 18.809218
Current Time of Time for BFS: Mon Jan  8 17:47:08 2024

TEPS for BFS 40 is 4.56686e+08
Running BFS 41
Current Time of Running BFS: Mon Jan  8 17:47:10 2024

Time for BFS 41 is 18.999012
Current Time of Time for BFS: Mon Jan  8 17:47:30 2024

TEPS for BFS 41 is 4.52124e+08
Running BFS 42
Current Time of Running BFS: Mon Jan  8 17:47:32 2024

Time for BFS 42 is 18.869305
Current Time of Time for BFS: Mon Jan  8 17:47:50 2024

TEPS for BFS 42 is 4.55232e+08
Running BFS 43
Current Time of Running BFS: Mon Jan  8 17:47:52 2024

Time for BFS 43 is 17.441587
Current Time of Time for BFS: Mon Jan  8 17:48:10 2024

TEPS for BFS 43 is 4.92495e+08
Running BFS 44
Current Time of Running BFS: Mon Jan  8 17:48:12 2024

Time for BFS 44 is 14.626895
Current Time of Time for BFS: Mon Jan  8 17:48:27 2024

TEPS for BFS 44 is 5.87268e+08
Running BFS 45
Current Time of Running BFS: Mon Jan  8 17:48:29 2024

Time for BFS 45 is 14.616461
Current Time of Time for BFS: Mon Jan  8 17:48:43 2024

TEPS for BFS 45 is 5.87687e+08
Running BFS 46
Current Time of Running BFS: Mon Jan  8 17:48:45 2024

Time for BFS 46 is 18.862801
Current Time of Time for BFS: Mon Jan  8 17:49:04 2024

TEPS for BFS 46 is 4.55388e+08
Running BFS 47
Current Time of Running BFS: Mon Jan  8 17:49:06 2024

Time for BFS 47 is 18.956969
Current Time of Time for BFS: Mon Jan  8 17:49:25 2024

TEPS for BFS 47 is 4.53126e+08
Running BFS 48
Current Time of Running BFS: Mon Jan  8 17:49:27 2024

Time for BFS 48 is 18.815691
Current Time of Time for BFS: Mon Jan  8 17:49:46 2024

TEPS for BFS 48 is 4.56529e+08
Running BFS 49
Current Time of Running BFS: Mon Jan  8 17:49:48 2024

Time for BFS 49 is 14.637225
Current Time of Time for BFS: Mon Jan  8 17:50:03 2024

TEPS for BFS 49 is 5.86853e+08
Running BFS 50
Current Time of Running BFS: Mon Jan  8 17:50:05 2024

Time for BFS 50 is 14.665308
Current Time of Time for BFS: Mon Jan  8 17:50:19 2024

TEPS for BFS 50 is 5.85729e+08
Running BFS 51
Current Time of Running BFS: Mon Jan  8 17:50:22 2024

Time for BFS 51 is 18.663199
Current Time of Time for BFS: Mon Jan  8 17:50:40 2024

TEPS for BFS 51 is 4.60259e+08
Running BFS 52
Current Time of Running BFS: Mon Jan  8 17:50:42 2024

Time for BFS 52 is 19.169489
Current Time of Time for BFS: Mon Jan  8 17:51:01 2024

TEPS for BFS 52 is 4.48103e+08
Running BFS 53
Current Time of Running BFS: Mon Jan  8 17:51:03 2024

Time for BFS 53 is 18.742041
Current Time of Time for BFS: Mon Jan  8 17:51:22 2024

TEPS for BFS 53 is 4.58323e+08
Running BFS 54
Current Time of Running BFS: Mon Jan  8 17:51:24 2024

Time for BFS 54 is 19.275146
Current Time of Time for BFS: Mon Jan  8 17:51:44 2024

TEPS for BFS 54 is 4.45647e+08
Running BFS 55
Current Time of Running BFS: Mon Jan  8 17:51:46 2024

Time for BFS 55 is 18.984978
Current Time of Time for BFS: Mon Jan  8 17:52:05 2024

TEPS for BFS 55 is 4.52458e+08
Running BFS 56
Current Time of Running BFS: Mon Jan  8 17:52:07 2024

Time for BFS 56 is 18.532989
Current Time of Time for BFS: Mon Jan  8 17:52:25 2024

TEPS for BFS 56 is 4.63493e+08
Running BFS 57
Current Time of Running BFS: Mon Jan  8 17:52:27 2024

Time for BFS 57 is 18.846527
Current Time of Time for BFS: Mon Jan  8 17:52:46 2024

TEPS for BFS 57 is 4.55782e+08
Running BFS 58
Current Time of Running BFS: Mon Jan  8 17:52:48 2024

Time for BFS 58 is 16.704196
Current Time of Time for BFS: Mon Jan  8 17:53:05 2024

TEPS for BFS 58 is 5.14236e+08
Running BFS 59
Current Time of Running BFS: Mon Jan  8 17:53:07 2024

Time for BFS 59 is 14.629670
Current Time of Time for BFS: Mon Jan  8 17:53:21 2024

TEPS for BFS 59 is 5.87156e+08
Running BFS 60
Current Time of Running BFS: Mon Jan  8 17:53:24 2024

Time for BFS 60 is 14.557026
Current Time of Time for BFS: Mon Jan  8 17:53:38 2024

TEPS for BFS 60 is 5.90086e+08
Running BFS 61
Current Time of Running BFS: Mon Jan  8 17:53:40 2024

Time for BFS 61 is 14.729560
Current Time of Time for BFS: Mon Jan  8 17:53:55 2024

TEPS for BFS 61 is 5.83174e+08
Running BFS 62
Current Time of Running BFS: Mon Jan  8 17:53:57 2024

Time for BFS 62 is 18.874205
Current Time of Time for BFS: Mon Jan  8 17:54:16 2024

TEPS for BFS 62 is 4.55113e+08
Running BFS 63
Current Time of Running BFS: Mon Jan  8 17:54:18 2024

Time for BFS 63 is 14.483646
Current Time of Time for BFS: Mon Jan  8 17:54:32 2024

TEPS for BFS 63 is 5.93076e+08
Running BFS 64
Current Time of Running BFS: Mon Jan  8 17:54:34 2024

Time for BFS 64 is 19.241218
Current Time of Time for BFS: Mon Jan  8 17:54:54 2024

TEPS for BFS 64 is 4.46432e+08
Running BFS 65
Current Time of Running BFS: Mon Jan  8 17:54:56 2024

Time for BFS 65 is 19.095683
Current Time of Time for BFS: Mon Jan  8 17:55:15 2024

TEPS for BFS 65 is 4.49835e+08
Running BFS 66
Current Time of Running BFS: Mon Jan  8 17:55:17 2024

Time for BFS 66 is 18.400783
Current Time of Time for BFS: Mon Jan  8 17:55:35 2024

TEPS for BFS 66 is 4.66823e+08
Running BFS 67
Current Time of Running BFS: Mon Jan  8 17:55:37 2024

Time for BFS 67 is 14.653983
Current Time of Time for BFS: Mon Jan  8 17:55:52 2024

TEPS for BFS 67 is 5.86182e+08
Running BFS 68
Current Time of Running BFS: Mon Jan  8 17:55:54 2024

Time for BFS 68 is 14.643703
Current Time of Time for BFS: Mon Jan  8 17:56:09 2024

TEPS for BFS 68 is 5.86594e+08
Running BFS 69
Current Time of Running BFS: Mon Jan  8 17:56:11 2024

Time for BFS 69 is 18.851397
Current Time of Time for BFS: Mon Jan  8 17:56:30 2024

TEPS for BFS 69 is 4.55664e+08
Running BFS 70
Current Time of Running BFS: Mon Jan  8 17:56:32 2024

Time for BFS 70 is 14.609985
Current Time of Time for BFS: Mon Jan  8 17:56:46 2024


"""

# Extracting BFS times and validation times
pattern = r"Running BFS (\d+).*?Time for BFS \1 is ([\d.]+).*?Validating BFS \1.*?Validate time for BFS \1 is ([\d.]+)"
matches = re.findall(pattern, data, re.DOTALL)

# Writing to CSV
csv_file = "data/BFS_times.csv"
with open(csv_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["BFS Number", "Running Time", "Validation Time"])
    for match in matches:
        writer.writerow(match)


