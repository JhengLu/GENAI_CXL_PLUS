import re
from datetime import datetime

log_data = """
time/loss/accuracy (if enabled):
Finished training it 20/1000 of epoch 0, 545.37 ms/it, loss 0.088643 (20:43:44.602), wall time duration 214.370 s
Finished training it 40/1000 of epoch 0, 522.82 ms/it, loss 0.085613 (20:47:15.906), wall time duration 211.303 s
Finished training it 60/1000 of epoch 0, 467.58 ms/it, loss 0.084396 (20:50:18.991), wall time duration 183.086 s
Finished training it 80/1000 of epoch 0, 469.72 ms/it, loss 0.083794 (20:53:50.238), wall time duration 211.247 s
Finished training it 100/1000 of epoch 0, 448.87 ms/it, loss 0.083538 (20:57:19.978), wall time duration 209.740 s
Finished training it 120/1000 of epoch 0, 489.27 ms/it, loss 0.083489 (21:00:24.943), wall time duration 184.965 s
Finished training it 140/1000 of epoch 0, 457.51 ms/it, loss 0.083331 (21:03:55.289), wall time duration 210.346 s
Finished training it 160/1000 of epoch 0, 449.24 ms/it, loss 0.083470 (21:07:26.426), wall time duration 211.137 s
Finished training it 180/1000 of epoch 0, 449.54 ms/it, loss 0.083466 (21:10:30.547), wall time duration 184.121 s
Finished training it 200/1000 of epoch 0, 449.15 ms/it, loss 0.083413 (21:14:01.511), wall time duration 210.965 s
Finished training it 220/1000 of epoch 0, 449.59 ms/it, loss 0.083471 (21:17:33.346), wall time duration 211.835 s
Finished training it 240/1000 of epoch 0, 466.18 ms/it, loss 0.083466 (21:20:36.634), wall time duration 183.287 s
Finished training it 260/1000 of epoch 0, 451.39 ms/it, loss 0.083290 (21:24:08.733), wall time duration 212.100 s
Finished training it 280/1000 of epoch 0, 451.46 ms/it, loss 0.083395 (21:27:40.701), wall time duration 211.968 s
Finished training it 300/1000 of epoch 0, 449.26 ms/it, loss 0.083346 (21:30:43.571), wall time duration 182.871 s
Finished training it 320/1000 of epoch 0, 448.87 ms/it, loss 0.083311 (21:34:15.850), wall time duration 212.279 s
Finished training it 340/1000 of epoch 0, 450.19 ms/it, loss 0.083348 (21:37:48.189), wall time duration 212.339 s
Finished training it 360/1000 of epoch 0, 487.11 ms/it, loss 0.083304 (21:40:51.018), wall time duration 182.830 s
Finished training it 380/1000 of epoch 0, 449.96 ms/it, loss 0.083424 (21:44:22.938), wall time duration 211.920 s
Finished training it 400/1000 of epoch 0, 449.81 ms/it, loss 0.083420 (21:47:55.341), wall time duration 212.403 s
Finished training it 420/1000 of epoch 0, 448.07 ms/it, loss 0.083423 (21:50:58.327), wall time duration 182.986 s
Finished training it 440/1000 of epoch 0, 449.40 ms/it, loss 0.083351 (21:54:30.247), wall time duration 211.920 s
Finished training it 460/1000 of epoch 0, 460.46 ms/it, loss 0.083315 (21:58:02.488), wall time duration 212.242 s
Finished training it 480/1000 of epoch 0, 449.09 ms/it, loss 0.083376 (22:01:05.353), wall time duration 182.864 s
Finished training it 500/1000 of epoch 0, 448.26 ms/it, loss 0.083390 (22:04:37.151), wall time duration 211.799 s
Finished training it 520/1000 of epoch 0, 449.44 ms/it, loss 0.083452 (22:08:09.101), wall time duration 211.950 s
Finished training it 540/1000 of epoch 0, 449.98 ms/it, loss 0.083303 (22:11:12.060), wall time duration 182.959 s
Finished training it 560/1000 of epoch 0, 448.50 ms/it, loss 0.083441 (22:14:44.019), wall time duration 211.959 s
Finished training it 580/1000 of epoch 0, 449.04 ms/it, loss 0.083375 (22:18:15.603), wall time duration 211.583 s
Finished training it 600/1000 of epoch 0, 502.47 ms/it, loss 0.083329 (22:21:18.551), wall time duration 182.949 s
Finished training it 620/1000 of epoch 0, 460.19 ms/it, loss 0.083352 (22:24:50.623), wall time duration 212.071 s
Finished training it 640/1000 of epoch 0, 460.18 ms/it, loss 0.083342 (22:28:22.675), wall time duration 212.052 s
Finished training it 660/1000 of epoch 0, 449.65 ms/it, loss 0.083403 (22:31:26.323), wall time duration 183.648 s
Finished training it 680/1000 of epoch 0, 503.71 ms/it, loss 0.083390 (22:35:00.023), wall time duration 213.700 s
Finished training it 700/1000 of epoch 0, 449.73 ms/it, loss 0.083376 (22:38:34.467), wall time duration 214.444 s
Finished training it 720/1000 of epoch 0, 465.47 ms/it, loss 0.083442 (22:41:39.505), wall time duration 185.038 s
Finished training it 740/1000 of epoch 0, 448.40 ms/it, loss 0.083408 (22:45:13.868), wall time duration 214.363 s
Finished training it 760/1000 of epoch 0, 456.21 ms/it, loss 0.083428 (22:48:46.832), wall time duration 212.963 s
Finished training it 780/1000 of epoch 0, 462.53 ms/it, loss 0.083475 (22:51:50.421), wall time duration 183.589 s
Finished training it 800/1000 of epoch 0, 448.24 ms/it, loss 0.083402 (22:55:22.978), wall time duration 212.557 s
Finished training it 820/1000 of epoch 0, 447.90 ms/it, loss 0.083434 (22:58:55.751), wall time duration 212.773 s
Finished training it 840/1000 of epoch 0, 447.97 ms/it, loss 0.083331 (23:01:58.968), wall time duration 183.217 s
Finished training it 860/1000 of epoch 0, 448.26 ms/it, loss 0.083360 (23:05:31.047), wall time duration 212.079 s
Finished training it 880/1000 of epoch 0, 449.42 ms/it, loss 0.083401 (23:09:02.927), wall time duration 211.880 s
Finished training it 900/1000 of epoch 0, 474.55 ms/it, loss 0.083474 (23:12:06.459), wall time duration 183.532 s
Finished training it 920/1000 of epoch 0, 448.50 ms/it, loss 0.083366 (23:15:37.661), wall time duration 211.202 s
Finished training it 940/1000 of epoch 0, 466.63 ms/it, loss 0.083419 (23:19:09.688), wall time duration 212.027 s
Finished training it 960/1000 of epoch 0, 450.64 ms/it, loss 0.083443 (23:22:12.726), wall time duration 183.038 s
Finished training it 980/1000 of epoch 0, 460.99 ms/it, loss 0.083357 (23:25:45.243), wall time duration 212.516 s
Finished training it 1000/1000 of epoch 0, 448.51 ms/it, loss 0.083276 (23:29:17.258), wall time duration 212.015 s





"""

# Regular expression to extract ms/it and wall time duration
pattern = re.compile(r"(\d+\.\d+) ms/it.*?wall time duration (\d+\.\d+) s")

# Extract values from the log data
matches = pattern.findall(log_data)

# Calculate averages
ms_per_iteration_values = [float(match[0]) for match in matches]
wall_time_duration_values = [float(match[1]) for match in matches]

average_ms_per_iteration = sum(ms_per_iteration_values) / len(ms_per_iteration_values)
average_wall_time_duration = sum(wall_time_duration_values) / len(wall_time_duration_values)

print(f"Average ms/it: {average_ms_per_iteration:.2f} ms/it")
print(f"Average wall time duration: {average_wall_time_duration:.2f} s")
