from datetime import datetime
import pandas as pd
import re

# Text input
text = """
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:   1%|          | 1/88 [00:22<32:15, 22.24s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 20:50:11
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:   2%|▏         | 2/88 [00:44<31:46, 22.17s/it]dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:   3%|▎         | 3/88 [01:06<31:17, 22.08s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 20:50:33
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:   5%|▍         | 4/88 [01:28<30:50, 22.03s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 20:50:54
dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 20:51:17
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:   6%|▌         | 5/88 [01:50<30:29, 22.04s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 20:51:39
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:   7%|▋         | 6/88 [02:12<30:08, 22.05s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 20:52:01
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:   8%|▊         | 7/88 [02:34<29:43, 22.02s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 20:52:23
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:   9%|▉         | 8/88 [02:56<29:21, 22.02s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 20:52:45
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  10%|█         | 9/88 [03:18<28:57, 21.99s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 20:53:07
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  11%|█▏        | 10/88 [03:40<28:34, 21.99s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 20:53:28
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  12%|█▎        | 11/88 [04:02<28:11, 21.96s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 20:53:50
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  14%|█▎        | 12/88 [04:24<27:47, 21.95s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 20:54:12
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  15%|█▍        | 13/88 [04:46<27:30, 22.01s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 20:54:34
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  16%|█▌        | 14/88 [05:08<27:07, 21.99s/it]dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  17%|█▋        | 15/88 [05:30<26:45, 21.99s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 20:54:56
dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 20:55:18
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  18%|█▊        | 16/88 [05:52<26:24, 22.00s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 20:55:40
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  19%|█▉        | 17/88 [06:14<26:01, 21.99s/it]dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  20%|██        | 18/88 [06:36<25:39, 21.99s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 20:56:02
dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 20:56:24
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  22%|██▏       | 19/88 [06:58<25:15, 21.97s/it]dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  23%|██▎       | 20/88 [07:20<24:53, 21.97s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 20:56:46
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  24%|██▍       | 21/88 [07:42<24:30, 21.95s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 20:57:08
dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 20:57:30
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  25%|██▌       | 22/88 [08:03<24:07, 21.93s/it]dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  26%|██▌       | 23/88 [08:25<23:41, 21.86s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 20:57:52
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  27%|██▋       | 24/88 [08:47<23:20, 21.88s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 20:58:14
dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 20:58:35
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  28%|██▊       | 25/88 [09:09<22:55, 21.84s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 20:58:57
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  30%|██▉       | 26/88 [09:31<22:35, 21.86s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 20:59:19
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  31%|███       | 27/88 [09:53<22:15, 21.89s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 20:59:41
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  32%|███▏      | 28/88 [10:14<21:53, 21.89s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:00:03
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  33%|███▎      | 29/88 [10:36<21:32, 21.91s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:00:18
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  34%|███▍      | 30/88 [10:52<19:12, 19.88s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:00:32
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  35%|███▌      | 31/88 [11:06<17:15, 18.17s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:00:47
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  36%|███▋      | 32/88 [11:20<15:50, 16.98s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:01:01
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  38%|███▊      | 33/88 [11:34<14:47, 16.14s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:01:15
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  39%|███▊      | 34/88 [11:49<14:05, 15.65s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:01:30
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  40%|███▉      | 35/88 [12:03<13:28, 15.26s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:01:44
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  41%|████      | 36/88 [12:17<12:56, 14.93s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:01:58
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  42%|████▏     | 37/88 [12:31<12:30, 14.71s/it]dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  43%|████▎     | 38/88 [12:46<12:07, 14.55s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:02:12
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  44%|████▍     | 39/88 [13:00<11:49, 14.47s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:02:27
dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:02:41
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  45%|████▌     | 40/88 [13:14<11:32, 14.43s/it]dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  47%|████▋     | 41/88 [13:28<11:14, 14.36s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:02:55
dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:03:09
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  48%|████▊     | 42/88 [13:43<10:57, 14.30s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:03:23
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  49%|████▉     | 43/88 [13:57<10:42, 14.27s/it]dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  50%|█████     | 44/88 [14:11<10:26, 14.24s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:03:38
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  51%|█████     | 45/88 [14:25<10:11, 14.22s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:03:52
dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:04:06
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  52%|█████▏    | 46/88 [14:39<09:56, 14.21s/it]dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  53%|█████▎    | 47/88 [14:54<09:43, 14.23s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:04:20
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  55%|█████▍    | 48/88 [15:08<09:28, 14.21s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:04:34
dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:04:49
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  56%|█████▌    | 49/88 [15:22<09:13, 14.20s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:05:03
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  57%|█████▋    | 50/88 [15:36<08:59, 14.19s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:05:17
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  58%|█████▊    | 51/88 [15:50<08:45, 14.22s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:05:31
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  59%|█████▉    | 52/88 [16:05<08:31, 14.20s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:05:45
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  60%|██████    | 53/88 [16:19<08:16, 14.19s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:06:00
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  61%|██████▏   | 54/88 [16:33<08:02, 14.18s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:06:14
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  62%|██████▎   | 55/88 [16:47<07:51, 14.28s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:06:28
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  64%|██████▎   | 56/88 [17:02<07:36, 14.27s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:06:42
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  65%|██████▍   | 57/88 [17:16<07:21, 14.24s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:06:57
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  66%|██████▌   | 58/88 [17:30<07:06, 14.22s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:07:11
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  67%|██████▋   | 59/88 [17:44<06:51, 14.20s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:07:25
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  68%|██████▊   | 60/88 [17:58<06:37, 14.19s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:07:39
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  69%|██████▉   | 61/88 [18:12<06:23, 14.19s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:07:53
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  70%|███████   | 62/88 [18:27<06:08, 14.19s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:08:08
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  72%|███████▏  | 63/88 [18:41<05:54, 14.19s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:08:22
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  73%|███████▎  | 64/88 [18:55<05:40, 14.18s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:08:36
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  74%|███████▍  | 65/88 [19:09<05:26, 14.18s/it]dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  75%|███████▌  | 66/88 [19:23<05:11, 14.18s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:08:50
dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:09:04
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  76%|███████▌  | 67/88 [19:38<04:57, 14.18s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:09:18
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  77%|███████▋  | 68/88 [19:52<04:43, 14.18s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:09:33
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  78%|███████▊  | 69/88 [20:06<04:29, 14.18s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:09:47
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  80%|███████▉  | 70/88 [20:20<04:15, 14.18s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:10:01
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  81%|████████  | 71/88 [20:34<04:00, 14.18s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:10:15
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  82%|████████▏ | 72/88 [20:48<03:46, 14.18s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:10:29
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  83%|████████▎ | 73/88 [21:03<03:32, 14.17s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:10:43
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  84%|████████▍ | 74/88 [21:17<03:18, 14.19s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:10:58
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  85%|████████▌ | 75/88 [21:31<03:04, 14.19s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-08 21:11:12
dlrm_testonly_display_cpu/0 [0]:

"""

# Extracting relevant data
pattern = r'.*Current Time: (\d+-\d+-\d+ \d+:\d+:\d+)'
CurrentTimeMatches = re.findall(pattern, text)

# Beginning time
beginning_time = datetime(2024, 1, 8, 20, 27, 7)

# Use regular expression to extract times in the format "12.54s/it"
InferenceTimeMatches = re.findall(r'(\d+\.\d+)s/it', text)

# Convert times to float
Inference_times_in_seconds = [float(time) for time in InferenceTimeMatches]



# Processing data
data = []
Number_of_CTime = len(CurrentTimeMatches)
Number_of_ITime = len(Inference_times_in_seconds)
if (Number_of_CTime != Number_of_ITime):
    print("number is different")
    exit()

for number in range(Number_of_CTime):
    # Extract data
    #time_spent, inference_time, current_time = match
    current_time_original = CurrentTimeMatches[number]
    current_time = datetime.strptime(current_time_original, "%Y-%m-%d %H:%M:%S")

    if (current_time<beginning_time):
        continue
    # Calculate index (seconds difference)
    index = (current_time - beginning_time).seconds
    inference_time = Inference_times_in_seconds[number]

    # Add to data list
    data.append([index, float(inference_time)])

# Creating DataFrame
df = pd.DataFrame(data, columns=["Index (s)", "Inference Time (s)"])

# Generate CSV
csv_file = "data/optane01_14cores_inference_itf_uncoreFalse_check2.csv"
df.to_csv(csv_file, index=False)


