import re
import csv

# Input data
data = """
process [3496]: latency = 278.699 ns
process [3496]: latency = 151.458 ns
process [3496]: latency = 137.5 ns
process [3496]: latency = 156.729 ns
process [3496]: latency = 296.856 ns
process [3496]: latency = 194.04 ns
process [3496]: latency = 256.417 ns
process [3496]: latency = 343.333 ns
process [3496]: latency = 216.094 ns
process [3496]: latency = 455.833 ns
process [3496]: latency = 196.667 ns
process [3496]: latency = 485.104 ns
process [3496]: latency = 397.583 ns
process [3496]: latency = 392.639 ns
process [3496]: latency = 416.083 ns
process [3496]: latency = 455.556 ns
process [3496]: latency = 442.917 ns
process [3496]: latency = 652.917 ns
process [3496]: latency = 690 ns
process [3496]: latency = 412.778 ns
process [3496]: latency = 303.75 ns
process [3496]: latency = 220.595 ns
process [3496]: latency = 323.274 ns
process [3496]: latency = 367.167 ns
process [3496]: latency = 328.929 ns
process [3496]: latency = 265.278 ns
process [3496]: latency = 402.333 ns
process [3496]: latency = 299.844 ns
process [3496]: latency = 321.25 ns
process [3496]: latency = 493.021 ns
process [3496]: latency = 290.069 ns
process [3496]: latency = 136.215 ns
process [3496]: latency = 231.012 ns
process [3496]: latency = 419.333 ns
process [3496]: latency = 407.986 ns
process [3496]: latency = 139.256 ns
process [3496]: latency = 398.542 ns
process [3496]: latency = 303.929 ns
process [3496]: latency = 504.688 ns
process [3496]: latency = 168.333 ns
process [3496]: latency = 425.167 ns
process [3496]: latency = 738.333 ns
process [3496]: latency = 304.653 ns
process [3496]: latency = 405.972 ns
process [3496]: latency = 93.0589 ns
process [3496]: latency = 351.806 ns
process [3496]: latency = 201.814 ns
process [3496]: latency = 312.024 ns
process [3496]: latency = 393.542 ns
process [3496]: latency = 130.59 ns
process [3496]: latency = 223.565 ns
process [3496]: latency = 188.657 ns
process [3496]: latency = 388.167 ns
process [3496]: latency = 198.69 ns
process [3496]: latency = 301.845 ns
process [3496]: latency = 1047.5 ns
process [3496]: latency = 199.583 ns
process [3496]: latency = 371.417 ns
process [3496]: latency = 202.5 ns
process [3496]: latency = 180.208 ns
process [3496]: latency = 158.083 ns
process [3496]: latency = 900.208 ns
process [3496]: latency = 444.833 ns
process [3496]: latency = 220.119 ns
process [3496]: latency = 239.01 ns
process [3496]: latency = 155.88 ns
process [3496]: latency = 143.056 ns
process [3496]: latency = 372.619 ns
process [3496]: latency = 127.593 ns
process [3496]: latency = 167.083 ns
process [3496]: latency = 125.91 ns
process [3496]: latency = 320.179 ns
process [3496]: latency = 326.071 ns
process [3496]: latency = 285.75 ns
process [3496]: latency = 383.083 ns
process [3496]: latency = 220.521 ns
process [3496]: latency = 293.333 ns
process [3496]: latency = 182.188 ns
"""

# Extract latency values using regular expressions
latency_values = re.findall(r'latency = (\d+\.\d+) ns', data)

# Define the output CSV file
output_file = 'memcached_latency_data.csv'

# Write the latency values to a CSV file
with open(output_file, 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)

    # Write a header
    csv_writer.writerow(['Latency (ns)'])

    # Write the latency values
    for value in latency_values:
        csv_writer.writerow([value])

print(f"Latency values have been saved to {output_file}")
