import re
import statistics

data = '''

dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-16 12:57:37
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:   1%|          | 1/88 [00:21<31:40, 21.84s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-16 12:57:59
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:   2%|▏         | 2/88 [00:43<31:10, 21.75s/it]dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:   3%|▎         | 3/88 [01:04<30:17, 21.39s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-16 12:58:20
dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-16 12:58:41
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:   5%|▍         | 4/88 [01:25<29:41, 21.21s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-16 12:59:02
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:   6%|▌         | 5/88 [01:46<29:16, 21.16s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-16 12:59:23
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:   7%|▋         | 6/88 [02:07<28:49, 21.09s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-16 12:59:44
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:   8%|▊         | 7/88 [02:28<28:24, 21.05s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-16 13:00:05
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:   9%|▉         | 8/88 [02:49<28:12, 21.15s/it]dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  10%|█         | 9/88 [03:10<27:46, 21.09s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-16 13:00:26
dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-16 13:00:47
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  11%|█▏        | 10/88 [03:31<27:21, 21.05s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-16 13:01:08
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  12%|█▎        | 11/88 [03:52<27:01, 21.06s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-16 13:01:29
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  14%|█▎        | 12/88 [04:13<26:38, 21.04s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-16 13:01:51
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  15%|█▍        | 13/88 [04:35<26:38, 21.32s/it]dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  16%|█▌        | 14/88 [04:56<26:11, 21.23s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-16 13:02:12
dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-16 13:02:33
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  17%|█▋        | 15/88 [05:17<25:43, 21.15s/it]dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  18%|█▊        | 16/88 [05:38<25:21, 21.13s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-16 13:02:54
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  19%|█▉        | 17/88 [05:59<24:56, 21.07s/it]dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-16 13:03:15
dlrm_testonly_display_cpu/0 [0]:Current Time: 2024-01-16 13:03:36
dlrm_testonly_display_cpu/0 [0]:
dlrm_testonly_display_cpu/0 [0]:Evaluating test set:  20%|██        | 18/88 [06:20<24:32, 21.04s/it]
'''

# Use regular expression to extract times in the format "12.54s/it"
times = re.findall(r'(\d+\.\d+)s/it', data)

# Convert times to float
times_in_seconds = [float(time) for time in times]

# Calculate the median
median_time = statistics.median(times_in_seconds)
print(f"Median time per iteration: {median_time} seconds/it")

# Calculate the average
average_time = sum(times_in_seconds) / len(times_in_seconds)
print(f"Average time per iteration: {average_time} seconds/it")