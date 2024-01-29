from datetime import datetime
import pandas as pd
import re

# Text input
text = """


Running BFS 0
Current Time of Running BFS: Mon Jan  8 20:27:07 2024

Time for BFS 0 is 14.495563
Current Time of Time for BFS: Mon Jan  8 20:27:21 2024

TEPS for BFS 0 is 5.92588e+08
Running BFS 1
Current Time of Running BFS: Mon Jan  8 20:27:23 2024

Time for BFS 1 is 14.486303
Current Time of Time for BFS: Mon Jan  8 20:27:38 2024

TEPS for BFS 1 is 5.92967e+08
Running BFS 2
Current Time of Running BFS: Mon Jan  8 20:27:40 2024

Time for BFS 2 is 14.627100
Current Time of Time for BFS: Mon Jan  8 20:27:55 2024

TEPS for BFS 2 is 5.87259e+08
Running BFS 3
Current Time of Running BFS: Mon Jan  8 20:27:57 2024

Time for BFS 3 is 14.357006
Current Time of Time for BFS: Mon Jan  8 20:28:11 2024

TEPS for BFS 3 is 5.98307e+08
Running BFS 4
Current Time of Running BFS: Mon Jan  8 20:28:13 2024

Time for BFS 4 is 14.702506
Current Time of Time for BFS: Mon Jan  8 20:28:28 2024

TEPS for BFS 4 is 5.84248e+08
Running BFS 5
Current Time of Running BFS: Mon Jan  8 20:28:30 2024

Time for BFS 5 is 14.420708
Current Time of Time for BFS: Mon Jan  8 20:28:44 2024

TEPS for BFS 5 is 5.95664e+08
Running BFS 6
Current Time of Running BFS: Mon Jan  8 20:28:46 2024

Time for BFS 6 is 15.051595
Current Time of Time for BFS: Mon Jan  8 20:29:01 2024

TEPS for BFS 6 is 5.70697e+08
Running BFS 7
Current Time of Running BFS: Mon Jan  8 20:29:03 2024

Time for BFS 7 is 14.468694
Current Time of Time for BFS: Mon Jan  8 20:29:18 2024

TEPS for BFS 7 is 5.93689e+08
Running BFS 8
Current Time of Running BFS: Mon Jan  8 20:29:20 2024

Time for BFS 8 is 14.765362
Current Time of Time for BFS: Mon Jan  8 20:29:35 2024

TEPS for BFS 8 is 5.8176e+08
Running BFS 9
Current Time of Running BFS: Mon Jan  8 20:29:37 2024

Time for BFS 9 is 14.521558
Current Time of Time for BFS: Mon Jan  8 20:29:51 2024

TEPS for BFS 9 is 5.91528e+08
Running BFS 10
Current Time of Running BFS: Mon Jan  8 20:29:53 2024

Time for BFS 10 is 14.529450
Current Time of Time for BFS: Mon Jan  8 20:30:08 2024

TEPS for BFS 10 is 5.91206e+08
Running BFS 11
Current Time of Running BFS: Mon Jan  8 20:30:10 2024

Time for BFS 11 is 14.523168
Current Time of Time for BFS: Mon Jan  8 20:30:24 2024

TEPS for BFS 11 is 5.91462e+08
Running BFS 12
Current Time of Running BFS: Mon Jan  8 20:30:26 2024

Time for BFS 12 is 14.608618
Current Time of Time for BFS: Mon Jan  8 20:30:41 2024

TEPS for BFS 12 is 5.88002e+08
Running BFS 13
Current Time of Running BFS: Mon Jan  8 20:30:43 2024

Time for BFS 13 is 15.445632
Current Time of Time for BFS: Mon Jan  8 20:30:58 2024

TEPS for BFS 13 is 5.56138e+08
Running BFS 14
Current Time of Running BFS: Mon Jan  8 20:31:00 2024

Time for BFS 14 is 14.578329
Current Time of Time for BFS: Mon Jan  8 20:31:15 2024

TEPS for BFS 14 is 5.89224e+08
Running BFS 15
Current Time of Running BFS: Mon Jan  8 20:31:17 2024

Time for BFS 15 is 15.137716
Current Time of Time for BFS: Mon Jan  8 20:31:32 2024

TEPS for BFS 15 is 5.6745e+08
Running BFS 16
Current Time of Running BFS: Mon Jan  8 20:31:34 2024

Time for BFS 16 is 14.415001
Current Time of Time for BFS: Mon Jan  8 20:31:48 2024

TEPS for BFS 16 is 5.959e+08
Running BFS 17
Current Time of Running BFS: Mon Jan  8 20:31:50 2024

Time for BFS 17 is 14.655865
Current Time of Time for BFS: Mon Jan  8 20:32:05 2024

TEPS for BFS 17 is 5.86107e+08
Running BFS 18
Current Time of Running BFS: Mon Jan  8 20:32:07 2024

Time for BFS 18 is 14.638414
Current Time of Time for BFS: Mon Jan  8 20:32:22 2024

TEPS for BFS 18 is 5.86806e+08
Running BFS 19
Current Time of Running BFS: Mon Jan  8 20:32:24 2024

Time for BFS 19 is 14.540410
Current Time of Time for BFS: Mon Jan  8 20:32:38 2024

TEPS for BFS 19 is 5.90761e+08
Running BFS 20
Current Time of Running BFS: Mon Jan  8 20:32:40 2024

Time for BFS 20 is 14.509839
Current Time of Time for BFS: Mon Jan  8 20:32:55 2024

TEPS for BFS 20 is 5.92005e+08
Running BFS 21
Current Time of Running BFS: Mon Jan  8 20:32:57 2024

Time for BFS 21 is 14.704721
Current Time of Time for BFS: Mon Jan  8 20:33:12 2024

TEPS for BFS 21 is 5.8416e+08
Running BFS 22
Current Time of Running BFS: Mon Jan  8 20:33:14 2024

Time for BFS 22 is 14.456668
Current Time of Time for BFS: Mon Jan  8 20:33:28 2024

TEPS for BFS 22 is 5.94183e+08
Running BFS 23
Current Time of Running BFS: Mon Jan  8 20:33:30 2024

Time for BFS 23 is 14.559324
Current Time of Time for BFS: Mon Jan  8 20:33:45 2024

TEPS for BFS 23 is 5.89993e+08
Running BFS 24
Current Time of Running BFS: Mon Jan  8 20:33:47 2024

Time for BFS 24 is 14.590468
Current Time of Time for BFS: Mon Jan  8 20:34:01 2024

TEPS for BFS 24 is 5.88734e+08
Running BFS 25
Current Time of Running BFS: Mon Jan  8 20:34:03 2024

Time for BFS 25 is 14.450083
Current Time of Time for BFS: Mon Jan  8 20:34:18 2024

TEPS for BFS 25 is 5.94454e+08
Running BFS 26
Current Time of Running BFS: Mon Jan  8 20:34:20 2024

Time for BFS 26 is 14.566869
Current Time of Time for BFS: Mon Jan  8 20:34:34 2024

TEPS for BFS 26 is 5.89688e+08
Running BFS 27
Current Time of Running BFS: Mon Jan  8 20:34:36 2024

Time for BFS 27 is 14.419225
Current Time of Time for BFS: Mon Jan  8 20:34:51 2024

TEPS for BFS 27 is 5.95726e+08
Running BFS 28
Current Time of Running BFS: Mon Jan  8 20:34:53 2024

Time for BFS 28 is 14.567613
Current Time of Time for BFS: Mon Jan  8 20:35:07 2024

TEPS for BFS 28 is 5.89658e+08
Running BFS 29
Current Time of Running BFS: Mon Jan  8 20:35:09 2024

Time for BFS 29 is 14.640973
Current Time of Time for BFS: Mon Jan  8 20:35:24 2024

TEPS for BFS 29 is 5.86703e+08
Running BFS 30
Current Time of Running BFS: Mon Jan  8 20:35:26 2024

Time for BFS 30 is 14.568600
Current Time of Time for BFS: Mon Jan  8 20:35:41 2024

TEPS for BFS 30 is 5.89618e+08
Running BFS 31
Current Time of Running BFS: Mon Jan  8 20:35:43 2024

Time for BFS 31 is 14.679750
Current Time of Time for BFS: Mon Jan  8 20:35:57 2024

TEPS for BFS 31 is 5.85153e+08
Running BFS 32
Current Time of Running BFS: Mon Jan  8 20:35:59 2024

Time for BFS 32 is 14.612791
Current Time of Time for BFS: Mon Jan  8 20:36:14 2024

TEPS for BFS 32 is 5.87834e+08
Running BFS 33
Current Time of Running BFS: Mon Jan  8 20:36:16 2024

Time for BFS 33 is 14.902745
Current Time of Time for BFS: Mon Jan  8 20:36:31 2024

TEPS for BFS 33 is 5.76397e+08
Running BFS 34
Current Time of Running BFS: Mon Jan  8 20:36:33 2024

Time for BFS 34 is 14.604337
Current Time of Time for BFS: Mon Jan  8 20:36:48 2024

TEPS for BFS 34 is 5.88175e+08
Running BFS 35
Current Time of Running BFS: Mon Jan  8 20:36:50 2024

Time for BFS 35 is 14.546976
Current Time of Time for BFS: Mon Jan  8 20:37:04 2024

TEPS for BFS 35 is 5.90494e+08
Running BFS 36
Current Time of Running BFS: Mon Jan  8 20:37:06 2024

Time for BFS 36 is 14.663675
Current Time of Time for BFS: Mon Jan  8 20:37:21 2024

TEPS for BFS 36 is 5.85795e+08
Running BFS 37
Current Time of Running BFS: Mon Jan  8 20:37:23 2024

Time for BFS 37 is 14.911973
Current Time of Time for BFS: Mon Jan  8 20:37:38 2024

TEPS for BFS 37 is 5.76041e+08
Running BFS 38
Current Time of Running BFS: Mon Jan  8 20:37:40 2024

Time for BFS 38 is 16.608737
Current Time of Time for BFS: Mon Jan  8 20:37:56 2024

TEPS for BFS 38 is 5.17192e+08
Running BFS 39
Current Time of Running BFS: Mon Jan  8 20:37:58 2024

Time for BFS 39 is 17.754717
Current Time of Time for BFS: Mon Jan  8 20:38:16 2024

TEPS for BFS 39 is 4.8381e+08
Running BFS 40
Current Time of Running BFS: Mon Jan  8 20:38:18 2024

Time for BFS 40 is 16.853077
Current Time of Time for BFS: Mon Jan  8 20:38:35 2024

TEPS for BFS 40 is 5.09693e+08
Running BFS 41
Current Time of Running BFS: Mon Jan  8 20:38:37 2024

Time for BFS 41 is 14.628004
Current Time of Time for BFS: Mon Jan  8 20:38:52 2024

TEPS for BFS 41 is 5.87223e+08
Running BFS 42
Current Time of Running BFS: Mon Jan  8 20:38:54 2024

Time for BFS 42 is 14.664873
Current Time of Time for BFS: Mon Jan  8 20:39:08 2024

TEPS for BFS 42 is 5.85747e+08
Running BFS 43
Current Time of Running BFS: Mon Jan  8 20:39:10 2024

Time for BFS 43 is 14.546549
Current Time of Time for BFS: Mon Jan  8 20:39:25 2024

TEPS for BFS 43 is 5.90511e+08
Running BFS 44
Current Time of Running BFS: Mon Jan  8 20:39:27 2024

Time for BFS 44 is 14.748061
Current Time of Time for BFS: Mon Jan  8 20:39:42 2024

TEPS for BFS 44 is 5.82443e+08
Running BFS 45
Current Time of Running BFS: Mon Jan  8 20:39:44 2024

Time for BFS 45 is 14.665622
Current Time of Time for BFS: Mon Jan  8 20:39:58 2024

TEPS for BFS 45 is 5.85717e+08
Running BFS 46
Current Time of Running BFS: Mon Jan  8 20:40:00 2024

Time for BFS 46 is 14.601453
Current Time of Time for BFS: Mon Jan  8 20:40:15 2024

TEPS for BFS 46 is 5.88291e+08
Running BFS 47
Current Time of Running BFS: Mon Jan  8 20:40:17 2024

Time for BFS 47 is 14.663516
Current Time of Time for BFS: Mon Jan  8 20:40:32 2024

TEPS for BFS 47 is 5.85801e+08
Running BFS 48
Current Time of Running BFS: Mon Jan  8 20:40:34 2024

Time for BFS 48 is 14.595278
Current Time of Time for BFS: Mon Jan  8 20:40:48 2024

TEPS for BFS 48 is 5.8854e+08
Running BFS 49
Current Time of Running BFS: Mon Jan  8 20:40:50 2024

Time for BFS 49 is 14.664081
Current Time of Time for BFS: Mon Jan  8 20:41:05 2024

TEPS for BFS 49 is 5.85778e+08
Running BFS 50
Current Time of Running BFS: Mon Jan  8 20:41:07 2024

Time for BFS 50 is 14.785676
Current Time of Time for BFS: Mon Jan  8 20:41:22 2024

TEPS for BFS 50 is 5.80961e+08
Running BFS 51
Current Time of Running BFS: Mon Jan  8 20:41:24 2024

Time for BFS 51 is 14.719122
Current Time of Time for BFS: Mon Jan  8 20:41:39 2024

TEPS for BFS 51 is 5.83588e+08
Running BFS 52
Current Time of Running BFS: Mon Jan  8 20:41:41 2024

Time for BFS 52 is 14.656830
Current Time of Time for BFS: Mon Jan  8 20:41:55 2024

TEPS for BFS 52 is 5.86068e+08
Running BFS 53
Current Time of Running BFS: Mon Jan  8 20:41:57 2024

Time for BFS 53 is 14.818870
Current Time of Time for BFS: Mon Jan  8 20:42:12 2024

TEPS for BFS 53 is 5.7966e+08
Running BFS 54
Current Time of Running BFS: Mon Jan  8 20:42:14 2024

Time for BFS 54 is 14.805405
Current Time of Time for BFS: Mon Jan  8 20:42:29 2024

TEPS for BFS 54 is 5.80187e+08
Running BFS 55
Current Time of Running BFS: Mon Jan  8 20:42:31 2024

Time for BFS 55 is 17.043423
Current Time of Time for BFS: Mon Jan  8 20:42:48 2024

TEPS for BFS 55 is 5.04001e+08
Running BFS 56
Current Time of Running BFS: Mon Jan  8 20:42:50 2024

Time for BFS 56 is 16.855066
Current Time of Time for BFS: Mon Jan  8 20:43:07 2024

TEPS for BFS 56 is 5.09633e+08
Running BFS 57
Current Time of Running BFS: Mon Jan  8 20:43:09 2024

Time for BFS 57 is 14.618148
Current Time of Time for BFS: Mon Jan  8 20:43:24 2024

TEPS for BFS 57 is 5.87619e+08
Running BFS 58
Current Time of Running BFS: Mon Jan  8 20:43:26 2024

Time for BFS 58 is 14.567049
Current Time of Time for BFS: Mon Jan  8 20:43:40 2024

TEPS for BFS 58 is 5.8968e+08
Running BFS 59
Current Time of Running BFS: Mon Jan  8 20:43:42 2024

Time for BFS 59 is 14.681033
Current Time of Time for BFS: Mon Jan  8 20:43:57 2024

TEPS for BFS 59 is 5.85102e+08
Running BFS 60
Current Time of Running BFS: Mon Jan  8 20:43:59 2024

Time for BFS 60 is 14.657058
Current Time of Time for BFS: Mon Jan  8 20:44:14 2024

TEPS for BFS 60 is 5.86059e+08
Running BFS 61
Current Time of Running BFS: Mon Jan  8 20:44:16 2024

Time for BFS 61 is 15.596317
Current Time of Time for BFS: Mon Jan  8 20:44:31 2024

TEPS for BFS 61 is 5.50765e+08
Running BFS 62
Current Time of Running BFS: Mon Jan  8 20:44:33 2024

Time for BFS 62 is 14.620313
Current Time of Time for BFS: Mon Jan  8 20:44:48 2024

TEPS for BFS 62 is 5.87532e+08
Running BFS 63
Current Time of Running BFS: Mon Jan  8 20:44:50 2024

Time for BFS 63 is 14.515498
Current Time of Time for BFS: Mon Jan  8 20:45:04 2024

TEPS for BFS 63 is 5.91775e+08
Running BFS 64
Current Time of Running BFS: Mon Jan  8 20:45:06 2024

Time for BFS 64 is 14.568781
Current Time of Time for BFS: Mon Jan  8 20:45:21 2024

TEPS for BFS 64 is 5.8961e+08
Running BFS 65
Current Time of Running BFS: Mon Jan  8 20:45:23 2024

Time for BFS 65 is 14.683139
Current Time of Time for BFS: Mon Jan  8 20:45:38 2024

TEPS for BFS 65 is 5.85018e+08
Running BFS 66
Current Time of Running BFS: Mon Jan  8 20:45:40 2024

Time for BFS 66 is 14.653035
Current Time of Time for BFS: Mon Jan  8 20:45:54 2024

TEPS for BFS 66 is 5.8622e+08
Running BFS 67
Current Time of Running BFS: Mon Jan  8 20:45:56 2024

Time for BFS 67 is 14.707810
Current Time of Time for BFS: Mon Jan  8 20:46:11 2024

TEPS for BFS 67 is 5.84037e+08
Running BFS 68
Current Time of Running BFS: Mon Jan  8 20:46:13 2024

Time for BFS 68 is 14.676669
Current Time of Time for BFS: Mon Jan  8 20:46:28 2024

TEPS for BFS 68 is 5.85276e+08
Running BFS 69
Current Time of Running BFS: Mon Jan  8 20:46:30 2024

Time for BFS 69 is 14.526738
Current Time of Time for BFS: Mon Jan  8 20:46:44 2024

TEPS for BFS 69 is 5.91317e+08
Running BFS 70
Current Time of Running BFS: Mon Jan  8 20:46:46 2024

Time for BFS 70 is 14.781460
Current Time of Time for BFS: Mon Jan  8 20:47:01 2024

TEPS for BFS 70 is 5.81127e+08
Running BFS 71
Current Time of Running BFS: Mon Jan  8 20:47:03 2024

Time for BFS 71 is 14.649430
Current Time of Time for BFS: Mon Jan  8 20:47:18 2024

TEPS for BFS 71 is 5.86364e+08
Running BFS 72
Current Time of Running BFS: Mon Jan  8 20:47:20 2024

Time for BFS 72 is 14.670675
Current Time of Time for BFS: Mon Jan  8 20:47:34 2024

TEPS for BFS 72 is 5.85515e+08
Running BFS 73
Current Time of Running BFS: Mon Jan  8 20:47:36 2024

Time for BFS 73 is 14.564280
Current Time of Time for BFS: Mon Jan  8 20:47:51 2024

TEPS for BFS 73 is 5.89792e+08
Running BFS 74
Current Time of Running BFS: Mon Jan  8 20:47:53 2024

Time for BFS 74 is 14.593779
Current Time of Time for BFS: Mon Jan  8 20:48:08 2024

TEPS for BFS 74 is 5.886e+08
Running BFS 75
Current Time of Running BFS: Mon Jan  8 20:48:10 2024

Time for BFS 75 is 17.133332
Current Time of Time for BFS: Mon Jan  8 20:48:27 2024

TEPS for BFS 75 is 5.01356e+08
Running BFS 76
Current Time of Running BFS: Mon Jan  8 20:48:29 2024

Time for BFS 76 is 20.801189
Current Time of Time for BFS: Mon Jan  8 20:48:50 2024

TEPS for BFS 76 is 4.12952e+08
Running BFS 77
Current Time of Running BFS: Mon Jan  8 20:48:52 2024

Time for BFS 77 is 20.892043
Current Time of Time for BFS: Mon Jan  8 20:49:12 2024

TEPS for BFS 77 is 4.11157e+08
Running BFS 78
Current Time of Running BFS: Mon Jan  8 20:49:14 2024

Time for BFS 78 is 24.238367
Current Time of Time for BFS: Mon Jan  8 20:49:39 2024

TEPS for BFS 78 is 3.54393e+08
Running BFS 79
Current Time of Running BFS: Mon Jan  8 20:49:41 2024

Time for BFS 79 is 25.062791
Current Time of Time for BFS: Mon Jan  8 20:50:06 2024

TEPS for BFS 79 is 3.42735e+08
Running BFS 80
Current Time of Running BFS: Mon Jan  8 20:50:09 2024

Time for BFS 80 is 26.995641
Current Time of Time for BFS: Mon Jan  8 20:50:36 2024

TEPS for BFS 80 is 3.18196e+08
Running BFS 81
Current Time of Running BFS: Mon Jan  8 20:50:39 2024

Time for BFS 81 is 25.383141
Current Time of Time for BFS: Mon Jan  8 20:51:04 2024

TEPS for BFS 81 is 3.3841e+08
Running BFS 82
Current Time of Running BFS: Mon Jan  8 20:51:07 2024

Time for BFS 82 is 24.622573
Current Time of Time for BFS: Mon Jan  8 20:51:31 2024

TEPS for BFS 82 is 3.48863e+08
Running BFS 83
Current Time of Running BFS: Mon Jan  8 20:51:34 2024

Time for BFS 83 is 25.617022
Current Time of Time for BFS: Mon Jan  8 20:52:00 2024

TEPS for BFS 83 is 3.3532e+08
Running BFS 84
Current Time of Running BFS: Mon Jan  8 20:52:02 2024

Time for BFS 84 is 26.744085
Current Time of Time for BFS: Mon Jan  8 20:52:29 2024

TEPS for BFS 84 is 3.21189e+08
Running BFS 85
Current Time of Running BFS: Mon Jan  8 20:52:31 2024

Time for BFS 85 is 23.528292
Current Time of Time for BFS: Mon Jan  8 20:52:55 2024

TEPS for BFS 85 is 3.65088e+08
Running BFS 86
Current Time of Running BFS: Mon Jan  8 20:52:58 2024

Time for BFS 86 is 23.333310
Current Time of Time for BFS: Mon Jan  8 20:53:21 2024

TEPS for BFS 86 is 3.68139e+08
Running BFS 87
Current Time of Running BFS: Mon Jan  8 20:53:24 2024

Time for BFS 87 is 26.543975
Current Time of Time for BFS: Mon Jan  8 20:53:50 2024

TEPS for BFS 87 is 3.2361e+08
Running BFS 88
Current Time of Running BFS: Mon Jan  8 20:53:53 2024

Time for BFS 88 is 26.694487
Current Time of Time for BFS: Mon Jan  8 20:54:19 2024

TEPS for BFS 88 is 3.21786e+08
Running BFS 89
Current Time of Running BFS: Mon Jan  8 20:54:22 2024

Time for BFS 89 is 22.190713
Current Time of Time for BFS: Mon Jan  8 20:54:44 2024

TEPS for BFS 89 is 3.87094e+08
Running BFS 90
Current Time of Running BFS: Mon Jan  8 20:54:47 2024

Time for BFS 90 is 24.473471
Current Time of Time for BFS: Mon Jan  8 20:55:11 2024

TEPS for BFS 90 is 3.50988e+08
Running BFS 91
Current Time of Running BFS: Mon Jan  8 20:55:14 2024

Time for BFS 91 is 25.651803
Current Time of Time for BFS: Mon Jan  8 20:55:40 2024

TEPS for BFS 91 is 3.34865e+08
Running BFS 92
Current Time of Running BFS: Mon Jan  8 20:55:42 2024

Time for BFS 92 is 23.644153
Current Time of Time for BFS: Mon Jan  8 20:56:06 2024

TEPS for BFS 92 is 3.63299e+08
Running BFS 93
Current Time of Running BFS: Mon Jan  8 20:56:08 2024

Time for BFS 93 is 25.222726
Current Time of Time for BFS: Mon Jan  8 20:56:34 2024

TEPS for BFS 93 is 3.40562e+08
Running BFS 94
Current Time of Running BFS: Mon Jan  8 20:56:36 2024

Time for BFS 94 is 23.374209
Current Time of Time for BFS: Mon Jan  8 20:57:00 2024

TEPS for BFS 94 is 3.67495e+08
Running BFS 95
Current Time of Running BFS: Mon Jan  8 20:57:02 2024

Time for BFS 95 is 22.551541
Current Time of Time for BFS: Mon Jan  8 20:57:25 2024

TEPS for BFS 95 is 3.80901e+08
Running BFS 96
Current Time of Running BFS: Mon Jan  8 20:57:27 2024

Time for BFS 96 is 22.480700
Current Time of Time for BFS: Mon Jan  8 20:57:50 2024

TEPS for BFS 96 is 3.82101e+08
Running BFS 97
Current Time of Running BFS: Mon Jan  8 20:57:52 2024

Time for BFS 97 is 24.002943
Current Time of Time for BFS: Mon Jan  8 20:58:16 2024

TEPS for BFS 97 is 3.57869e+08
Running BFS 98
Current Time of Running BFS: Mon Jan  8 20:58:19 2024

Time for BFS 98 is 21.154412
Current Time of Time for BFS: Mon Jan  8 20:58:40 2024

TEPS for BFS 98 is 4.06057e+08
Running BFS 99
Current Time of Running BFS: Mon Jan  8 20:58:43 2024

Time for BFS 99 is 23.012843
Current Time of Time for BFS: Mon Jan  8 20:59:06 2024

TEPS for BFS 99 is 3.73266e+08
Running BFS 100
Current Time of Running BFS: Mon Jan  8 20:59:09 2024

Time for BFS 100 is 22.785152
Current Time of Time for BFS: Mon Jan  8 20:59:31 2024

TEPS for BFS 100 is 3.76996e+08
Running BFS 101
Current Time of Running BFS: Mon Jan  8 20:59:34 2024

Time for BFS 101 is 23.626386
Current Time of Time for BFS: Mon Jan  8 20:59:58 2024

TEPS for BFS 101 is 3.63572e+08
Running BFS 102
Current Time of Running BFS: Mon Jan  8 21:00:00 2024






"""

# Extracting relevant data

# Extracting BFS times
BFSTimePattern = r"Time for BFS \d+ is ([\d.]+)"
BFSTimeMatches = re.findall(BFSTimePattern, text)

BFSIndexPattern = r'Current Time of Time for BFS: (.+)'
BFSIndexMatches = re.findall(BFSIndexPattern, text)

# Extracting validation times

ValidateTimePattern = r"Validate time for BFS \d+ is ([\d.]+)"
ValidateTimeMatches = re.findall(ValidateTimePattern, text)

ValidateTimeIndexPattern = r'Current Time of Validate time for BFS: (.+)'
ValidateTimeIndexMatches = re.findall(ValidateTimeIndexPattern, text)

# Beginning time
beginning_time = datetime(2024, 1, 8, 20, 27, 7)


# Processing data
data = []
Number_of_BasicTime = len(BFSTimeMatches)
Number_of_BFSTime = len(BFSIndexMatches)
if (Number_of_BasicTime != Number_of_BFSTime):
    print("number is different")
    exit()
else:
    for number in range(Number_of_BasicTime):
        # Extract data
        # time_spent, inference_time, current_time = match
        current_time_original = BFSIndexMatches[number]
        current_time = datetime.strptime(current_time_original, '%a %b %d %H:%M:%S %Y')

        if (current_time < beginning_time):
            continue
        # Calculate index (seconds difference)
        index = (current_time - beginning_time).seconds
        BFS_time = BFSTimeMatches[number]

        # Add to data list
        data.append([index, float(BFS_time)])

        # Creating DataFrame
        df = pd.DataFrame(data, columns=["Index (s)", "BFS Time (s)"])

        # Generate CSV
        csv_file = "data/optane01_0-69cores_BFS_itf_uncoreFalse_check2.csv"
        df.to_csv(csv_file, index=False)

exit()
ValidateData = []
Number_of_ValidateTime = len(ValidateTimeMatches)
Number_of_ValidateIndexTime = len(ValidateTimeIndexMatches)
if (Number_of_ValidateTime != Number_of_ValidateIndexTime):
    print("number is different")
    exit()
else:
    for number in range(Number_of_ValidateTime):
        # time_spent, inference_time, current_time = match
        validate_current_time_original = ValidateTimeIndexMatches[number]
        validate_current_time = datetime.strptime(validate_current_time_original, '%a %b %d %H:%M:%S %Y')

        if (validate_current_time < beginning_time):
            continue
        # Calculate index (seconds difference)
        Validate_index = (validate_current_time - beginning_time).seconds
        Validate_BFS_time = ValidateTimeMatches[number]

        # Add to data list
        ValidateData.append([Validate_index, float(Validate_BFS_time)])

        # Creating DataFrame
        Validate_df = pd.DataFrame(ValidateData, columns=["Validate Index (s)", "Validate BFS Time (s)"])

        # Generate CSV
        csv_file = "data/optane01_28cores_Validate_BFS_interference.csv"
        Validate_df.to_csv(csv_file, index=False)





