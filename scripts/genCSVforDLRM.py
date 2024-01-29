import csv
import re

# Sample text data
data = '''


Finished training it 20/1000 of epoch 0, 559.64 ms/it, loss 0.088643 (22:33:57.182), wall time duration 192.553 s
Finished training it 40/1000 of epoch 0, 621.20 ms/it, loss 0.085613 (22:37:05.561), wall time duration 188.380 s
Finished training it 60/1000 of epoch 0, 473.23 ms/it, loss 0.084396 (22:39:52.267), wall time duration 166.706 s
Finished training it 80/1000 of epoch 0, 543.95 ms/it, loss 0.083794 (22:43:01.704), wall time duration 189.437 s
Finished training it 100/1000 of epoch 0, 511.03 ms/it, loss 0.083538 (22:46:08.563), wall time duration 186.859 s
Finished training it 120/1000 of epoch 0, 524.94 ms/it, loss 0.083489 (22:48:59.369), wall time duration 170.807 s
Finished training it 140/1000 of epoch 0, 519.97 ms/it, loss 0.083331 (22:52:08.065), wall time duration 188.696 s
Finished training it 160/1000 of epoch 0, 511.24 ms/it, loss 0.083470 (22:55:11.720), wall time duration 183.655 s
Finished training it 180/1000 of epoch 0, 541.59 ms/it, loss 0.083466 (22:58:04.145), wall time duration 172.425 s
Finished training it 200/1000 of epoch 0, 514.66 ms/it, loss 0.083413 (23:01:13.942), wall time duration 189.798 s
Finished training it 220/1000 of epoch 0, 528.91 ms/it, loss 0.083471 (23:04:14.607), wall time duration 180.664 s
Finished training it 240/1000 of epoch 0, 587.97 ms/it, loss 0.083466 (23:07:11.092), wall time duration 176.485 s
Finished training it 260/1000 of epoch 0, 563.23 ms/it, loss 0.083290 (23:10:20.408), wall time duration 189.316 s
Finished training it 280/1000 of epoch 0, 544.84 ms/it, loss 0.083395 (23:13:24.514), wall time duration 184.106 s
Finished training it 300/1000 of epoch 0, 511.56 ms/it, loss 0.083346 (23:16:26.455), wall time duration 181.941 s
Finished training it 320/1000 of epoch 0, 482.89 ms/it, loss 0.083311 (23:19:31.092), wall time duration 184.637 s
Finished training it 340/1000 of epoch 0, 506.95 ms/it, loss 0.083348 (23:22:35.163), wall time duration 184.071 s
Finished training it 360/1000 of epoch 0, 536.12 ms/it, loss 0.083304 (23:25:41.289), wall time duration 186.127 s
Finished training it 380/1000 of epoch 0, 554.95 ms/it, loss 0.083424 (23:28:42.634), wall time duration 181.344 s
Finished training it 400/1000 of epoch 0, 590.79 ms/it, loss 0.083420 (23:31:50.431), wall time duration 187.797 s
Finished training it 420/1000 of epoch 0, 632.92 ms/it, loss 0.083423 (23:34:54.935), wall time duration 184.504 s
Finished training it 440/1000 of epoch 0, 519.11 ms/it, loss 0.083351 (23:37:53.011), wall time duration 178.077 s






'''

# Initialize lists to store extracted data
iterations = []
ms_per_iteration = []
losses = []
wall_time_durations = []

# Define a regular expression pattern to extract information from each line
#pattern = re.compile(r'Finished training it (\d+)/\d+ of epoch \d+, ([\d.]+) ms/it, loss ([\d.]+) \([\d:]+\), wall time duration ([\d.]+) s')
pattern = re.compile(r'Finished training it (\d+)/\d+ of epoch \d+, ([\d.]+) ms/it, loss ([\d.]+) \([^)]+\), wall time duration ([\d.]+) s')

# Match the pattern in each line and extract information
matches = pattern.findall(data)
for match in matches:
    iterations.append(int(match[0]))
    ms_per_iteration.append(float(match[1]))
    losses.append(float(match[2]))
    wall_time_durations.append(float(match[3]))

# Create a list of dictionaries for each row
rows = [
    {'Iteration': iteration, 'ms/it': ms, 'Loss': loss, 'Wall Time Duration (s)': duration}
    for iteration, ms, loss, duration in zip(iterations, ms_per_iteration, losses, wall_time_durations)
]

# Specify the CSV file path
csv_file_path = 'unlimited.csv'

# Write the data to a CSV file
with open(csv_file_path, 'w', newline='') as csv_file:
    fieldnames = ['Iteration', 'ms/it', 'Loss', 'Wall Time Duration (s)']
    writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

    # Write the header
    writer.writeheader()

    # Write the rows
    writer.writerows(rows)

print(f'Data has been successfully written to {csv_file_path}')
