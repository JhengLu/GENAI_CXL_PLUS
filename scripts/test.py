import os
import sys
import string
import random
import argparse
from multiprocessing import Pool, Manager

def store_data(_):
    key = ''.join(random.choices(string.ascii_letters + string.digits, k=100))
    value = os.urandom(1024)  # creates a 1KB data
    return key, value

def store_data_batch(args):
    start, end, q = args
    data_batch = {}
    for _ in range(start, end):
        key = ''.join(random.choices(string.ascii_letters + string.digits, k=100))
        value = os.urandom(1024)  # creates a 1KB data
        data_batch[key] = value
    q.update(data_batch)

# create a parser object
parser = argparse.ArgumentParser()

# defining an argument for the parser object
parser.add_argument('--size', type=int, help='The size of data you want to store in memory in GB')

# parse the arguments
args = parser.parse_args()

# create manager object and dictionary
manager = Manager()
my_data = manager.dict()

# specify pool size equal to the number of cores
pool = Pool()

# specify the batch size
batch_size = 1000
size = 1

# generate random key-value pairs in batches and store them in the dictionary
for i in range(0, size * (10**6), batch_size):
    pool.apply_async(store_data_batch, args=((i, i + batch_size, my_data),))

# close the pool to prevent further tasks
pool.close()

# wait for all tasks to complete
pool.join()

# Avoid printing memory usage in the main process
# print('Memory size of my_data: ', sys.getsizeof(my_data) / (1024**3), 'GB') # prints the size in GB

# keep the program running to keep data in memory
while True:
    pass
