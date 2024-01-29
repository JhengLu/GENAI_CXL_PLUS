import re

text = """
1.	Average Latency: 277.100 ns
2.	Average Latency: 280.891 ns
3.	Average Latency: 276.206 ns
4.	Average Latency: 274.753 ns
5.	Average Latency: 276.460 ns
6.	Average Latency: 274.695 ns
7.	Average Latency: 273.253 ns
Average Latency: 281.842 ns
Average Latency: 280.081 ns
Average Latency: 279.717 ns










"""

# Define a regular expression pattern to match average latency values
latency_pattern = r'Average Latency: (\d+\.\d+) ns'

# Use re.findall to find all matching average latency values in the text
latency_values = re.findall(latency_pattern, text)

# Convert the extracted values to floating-point numbers
latencies = [float(value) for value in latency_values]

# Calculate and print the average latency
average_latency = sum(latencies) / len(latencies)
print(f"Avg for Avg latency: {average_latency:.3f} ns")
