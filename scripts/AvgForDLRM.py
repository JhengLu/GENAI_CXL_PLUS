import re

data = '''

Finished training it 20/1000 of epoch 0, 509.31 ms/it, loss 0.219017 (16:20:26.776), wall time duration 71.551 s
Finished training it 40/1000 of epoch 0, 420.12 ms/it, loss 0.201076 (16:20:35.204), wall time duration 8.428 s
Finished training it 60/1000 of epoch 0, 429.60 ms/it, loss 0.174162 (16:20:55.318), wall time duration 20.114 s
Finished training it 80/1000 of epoch 0, 427.46 ms/it, loss 0.132446 (16:21:04.124), wall time duration 8.806 s
Finished training it 100/1000 of epoch 0, 442.85 ms/it, loss 0.093828 (16:21:13.419), wall time duration 9.295 s
Finished training it 120/1000 of epoch 0, 394.60 ms/it, loss 0.084764 (16:21:22.478), wall time duration 9.060 s
Finished training it 140/1000 of epoch 0, 432.49 ms/it, loss 0.084098 (16:21:31.776), wall time duration 9.298 s
Finished training it 160/1000 of epoch 0, 424.33 ms/it, loss 0.084203 (16:21:48.526), wall time duration 16.749 s
Finished training it 180/1000 of epoch 0, 413.24 ms/it, loss 0.084223 (16:22:00.367), wall time duration 11.842 s
Finished training it 200/1000 of epoch 0, 442.35 ms/it, loss 0.084117 (16:22:15.345), wall time duration 14.978 s
Finished training it 220/1000 of epoch 0, 409.97 ms/it, loss 0.084074 (16:22:28.246), wall time duration 12.901 s
Finished training it 240/1000 of epoch 0, 438.22 ms/it, loss 0.084083 (16:22:38.530), wall time duration 10.284 s
Finished training it 260/1000 of epoch 0, 419.72 ms/it, loss 0.084087 (16:22:52.308), wall time duration 13.778 s
Finished training it 280/1000 of epoch 0, 438.14 ms/it, loss 0.084225 (16:23:03.827), wall time duration 11.519 s
Finished training it 300/1000 of epoch 0, 379.87 ms/it, loss 0.084004 (16:23:15.835), wall time duration 12.008 s
Finished training it 320/1000 of epoch 0, 386.45 ms/it, loss 0.084185 (16:23:29.189), wall time duration 13.354 s
Finished training it 340/1000 of epoch 0, 392.94 ms/it, loss 0.084016 (16:23:41.975), wall time duration 12.786 s
Finished training it 360/1000 of epoch 0, 393.18 ms/it, loss 0.084069 (16:23:53.964), wall time duration 11.989 s
Finished training it 380/1000 of epoch 0, 395.81 ms/it, loss 0.083943 (16:24:07.484), wall time duration 13.520 s
Finished training it 400/1000 of epoch 0, 426.16 ms/it, loss 0.084199 (16:24:20.001), wall time duration 12.517 s
Finished training it 420/1000 of epoch 0, 407.17 ms/it, loss 0.084175 (16:24:32.711), wall time duration 12.710 s
Finished training it 440/1000 of epoch 0, 402.46 ms/it, loss 0.083959 (16:24:44.834), wall time duration 12.123 s
Finished training it 460/1000 of epoch 0, 367.27 ms/it, loss 0.084076 (16:24:57.294), wall time duration 12.459 s
Finished training it 480/1000 of epoch 0, 369.44 ms/it, loss 0.084059 (16:25:11.365), wall time duration 14.071 s
Finished training it 500/1000 of epoch 0, 428.43 ms/it, loss 0.084119 (16:25:22.993), wall time duration 11.628 s
Finished training it 520/1000 of epoch 0, 380.40 ms/it, loss 0.084188 (16:25:35.384), wall time duration 12.391 s
Finished training it 540/1000 of epoch 0, 392.54 ms/it, loss 0.084063 (16:25:47.892), wall time duration 12.508 s
Finished training it 560/1000 of epoch 0, 394.85 ms/it, loss 0.084151 (16:26:00.121), wall time duration 12.229 s
Finished training it 580/1000 of epoch 0, 372.97 ms/it, loss 0.084073 (16:26:12.772), wall time duration 12.651 s
Finished training it 600/1000 of epoch 0, 394.04 ms/it, loss 0.084159 (16:26:25.849), wall time duration 13.077 s
Finished training it 620/1000 of epoch 0, 377.04 ms/it, loss 0.084032 (16:26:37.430), wall time duration 11.581 s
Finished training it 640/1000 of epoch 0, 368.05 ms/it, loss 0.083966 (16:26:49.888), wall time duration 12.458 s
Finished training it 660/1000 of epoch 0, 401.42 ms/it, loss 0.084068 (16:27:02.794), wall time duration 12.906 s
Finished training it 680/1000 of epoch 0, 384.37 ms/it, loss 0.083916 (16:27:14.627), wall time duration 11.833 s
Finished training it 700/1000 of epoch 0, 377.15 ms/it, loss 0.084040 (16:27:27.286), wall time duration 12.659 s
Finished training it 720/1000 of epoch 0, 389.84 ms/it, loss 0.084039 (16:27:40.469), wall time duration 13.182 s
Finished training it 740/1000 of epoch 0, 387.19 ms/it, loss 0.084012 (16:27:53.155), wall time duration 12.686 s
Finished training it 760/1000 of epoch 0, 395.95 ms/it, loss 0.084124 (16:28:06.406), wall time duration 13.252 s
Finished training it 780/1000 of epoch 0, 378.88 ms/it, loss 0.084023 (16:28:18.887), wall time duration 12.481 s
Finished training it 800/1000 of epoch 0, 414.71 ms/it, loss 0.084054 (16:28:30.760), wall time duration 11.873 s
Finished training it 820/1000 of epoch 0, 367.50 ms/it, loss 0.084079 (16:28:43.487), wall time duration 12.727 s
Finished training it 840/1000 of epoch 0, 382.01 ms/it, loss 0.083989 (16:28:55.830), wall time duration 12.343 s
Finished training it 860/1000 of epoch 0, 392.11 ms/it, loss 0.084003 (16:29:08.734), wall time duration 12.904 s
Finished training it 880/1000 of epoch 0, 394.82 ms/it, loss 0.083898 (16:29:21.816), wall time duration 13.082 s
Finished training it 900/1000 of epoch 0, 411.96 ms/it, loss 0.084076 (16:29:33.240), wall time duration 11.424 s
Finished training it 920/1000 of epoch 0, 387.62 ms/it, loss 0.084024 (16:29:44.707), wall time duration 11.467 s
Finished training it 940/1000 of epoch 0, 387.86 ms/it, loss 0.083919 (16:29:56.323), wall time duration 11.616 s
Finished training it 960/1000 of epoch 0, 383.77 ms/it, loss 0.083991 (16:30:07.620), wall time duration 11.297 s
Finished training it 980/1000 of epoch 0, 365.42 ms/it, loss 0.083946 (16:30:19.329), wall time duration 11.709 s
Finished training it 1000/1000 of epoch 0, 388.72 ms/it, loss 0.083927 (16:30:30.798), wall time duration 11.468 s


'''

# Extract ms/it values using regular expression
ms_per_it_values = re.findall(r'(\d+\.\d+) ms/it', data)

# Convert values to float and calculate the average
ms_per_it_values = [float(value) for value in ms_per_it_values]
average_ms_per_it = sum(ms_per_it_values) / len(ms_per_it_values)

print(f'Average ms/it: {average_ms_per_it:.2f} ms/it')
