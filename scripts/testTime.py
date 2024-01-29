import time


current_time_1 = time.time()  # Define and assign current_time here
time.sleep(1)
current_time_2 = time.time()
print(current_time_1)
print(current_time_2)
print(current_time_2-current_time_1)
# wall_time = " ({})".format(time.strftime("%H:%M:%S", time.localtime(current_time)) + f".{int((current_time % 1) * 1000):03d}")
# print(current_time)


from datetime import datetime
current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
print(f"Current Time: {current_time}")