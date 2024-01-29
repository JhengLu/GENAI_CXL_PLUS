import re


def calculate_median(values):
    n = len(values)
    values.sort()
    if n % 2 == 0:
        median = (values[n//2 - 1] + values[n//2]) / 2
    else:
        median = values[n//2]
    return median


if __name__ == '__main__':


    text = """
GHZ: 2
process [127459]: latency = 89.5163 ns
count_cycles_l3_miss: 899184412
count_retired_l3_miss: 5022464

process [127459]: latency = 89.3629 ns
count_cycles_l3_miss: 1794368460
count_retired_l3_miss: 10031165

process [127459]: latency = 89.2717 ns
count_cycles_l3_miss: 2688014408
count_retired_l3_miss: 15036370

process [127459]: latency = 89.3337 ns
count_cycles_l3_miss: 3581940994
count_retired_l3_miss: 20039670

process [127459]: latency = 89.6348 ns
count_cycles_l3_miss: 4474872206
count_retired_l3_miss: 25020611

process [127459]: latency = 89.6989 ns
count_cycles_l3_miss: 5368307872
count_retired_l3_miss: 30000803

process [127459]: latency = 89.568 ns
count_cycles_l3_miss: 6262175072
count_retired_l3_miss: 34990683

process [127459]: latency = 89.6613 ns
count_cycles_l3_miss: 7153629698
count_retired_l3_miss: 39961919

process [127459]: latency = 89.5331 ns
count_cycles_l3_miss: 8042497886
count_retired_l3_miss: 44925829

process [127459]: latency = 89.5657 ns
count_cycles_l3_miss: 8932856254
count_retired_l3_miss: 49896251

process [127459]: latency = 89.5318 ns
count_cycles_l3_miss: 9822953980
count_retired_l3_miss: 54867098

process [127459]: latency = 89.3434 ns
count_cycles_l3_miss: 10718780594
count_retired_l3_miss: 59880489

process [127459]: latency = 89.3436 ns
count_cycles_l3_miss: 11612158162
count_retired_l3_miss: 64880160

process [127459]: latency = 89.4958 ns
count_cycles_l3_miss: 12503733776
count_retired_l3_miss: 69861265

process [127459]: latency = 89.4076 ns
count_cycles_l3_miss: 13109391558
count_retired_l3_miss: 73248326



    """

    # Define a regular expression pattern to match latency values
    latency_pattern = r'latency = (\d+\.\d+) ns'

    # Use re.findall to find all matching latency values in the text
    latency_values = re.findall(latency_pattern, text)

    # Convert the extracted values to floating-point numbers
    latencies = [float(value) for value in latency_values]

    # Calculate and print the average latency
    average_latency = sum(latencies) / len(latencies)
    print(f"Average Latency: {average_latency:.3f} ns")

    # Calculate and print the median latency
    median_latency = calculate_median(latencies)
    print(f"Median Latency: {median_latency:.3f} ns")




