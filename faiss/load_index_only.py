import numpy as np
import faiss
import os
import time
import argparse  # Import argparse for command line parsing
import concurrent.futures  # Import for parallel execution
import multiprocessing
from functools import partial


def load_index(index_file_path, use_gpu=False):
    """
    Load an index from disk with an option to use GPU.
    """
    if not os.path.exists(index_file_path):
        print(f"Index file {index_file_path} not found.")
        return None

    index = faiss.read_index(index_file_path)
    if use_gpu:
        # Assuming you have a GPU compatible FAISS installed
        gpu_index = faiss.index_cpu_to_gpu(faiss.StandardGpuResources(), 0, index)
        index = gpu_index
        print(f"Index loaded on GPU from {index_file_path}")
    else:
        print(f"Index loaded from {index_file_path} with {index.ntotal} vectors.")

    return index

def query_index(index, dimension, n_queries, n_neighbors):
    """
    Perform queries on a loaded index and display performance metrics.
    This function is designed to be run in a separate process.
    """
    query_vectors = np.random.rand(n_queries, dimension).astype('float32')
    start_time = time.time()
    D, I = index.search(query_vectors, n_neighbors)  # D: distances, I: indices
    end_time = time.time()

    total_query_time = end_time - start_time
    average_query_time = total_query_time / n_queries
    average_distance = np.mean(D)
    print(f"Process {os.getpid()}: Total query time = {total_query_time:.4f} seconds")
    print(f"Process {os.getpid()}: Average query time per vector = {average_query_time:.4f} seconds")
    print(f"Process {os.getpid()}: Average distance of the nearest neighbors = {average_distance:.4f}")
    print(f"Process {os.getpid()}: Indices of nearest neighbors for the first query vector = {I[0]}")

def run_parallel_queries(index_path, dimension, n_queries, n_neighbors, num_processes):
    """
    Set up the environment to run parallel queries on the index independently in each process.
    """
    if not os.path.exists(index_path):
        print("Index file does not exist.")
        return

    index = faiss.read_index(index_path)  # Load the index

    processes = []
    for _ in range(num_processes):
        p = multiprocessing.Process(target=query_index, args=(index, dimension, n_queries, n_neighbors))
        processes.append(p)
        p.start()

    for p in processes:
        p.join()



def parse_arguments():
    """
    Parse command line arguments.
    """
    parser = argparse.ArgumentParser(description="FAISS Index Querying")
    parser.add_argument('--index_path', type=str, default='faiss_index20.index',
                        help='Path to the FAISS index file')
    parser.add_argument('--dimension', type=int, default=512,
                        help='Dimension of the vectors')
    parser.add_argument('--n_queries', type=int, default=10,
                        help='Number of query vectors')
    parser.add_argument('--n_neighbors', type=int, default=10,
                        help='Number of nearest neighbors to search for each query vector')
    parser.add_argument('--use_gpu', action='store_true',
                        help='Flag to use GPU for querying')
    parser.add_argument('--n_processes', type=int, default=1, help='Number of parallel processes to use')

    return parser.parse_args()



# def main():
#     args = parse_arguments()
#     print(args.n_queries)
#     index = load_index(args.index_path, args.use_gpu)
#     if index is None:
#         return

#     try:
#         while True:
#             # run_parallel_queries(args.index_path, args.dimension, args.n_queries, args.n_neighbors,1)
#             query_index(index, args.dimension, args.n_queries, args.n_neighbors)
#     except KeyboardInterrupt:
#         print("Stopped querying.")

def worker(index_path, use_gpu, dimension, n_queries, n_neighbors):
    index = load_index(index_path, use_gpu)
    if index is None:
        print("Failed to load index.")
        return

   # try:
    #    while True:
    #        query_index(index, dimension, n_queries, n_neighbors)
   # except KeyboardInterrupt:
    #    print("Process interrupted.")

def main():
    args = parse_arguments()
    print(f"Number of queries: {args.n_queries}")
    
    processes = []
    for _ in range(args.n_processes):
        p = multiprocessing.Process(target=worker, args=(args.index_path, args.use_gpu, args.dimension, args.n_queries, args.n_neighbors))
        processes.append(p)
        p.start()

    try:
        for p in processes:
            p.join()
    except KeyboardInterrupt:
        print("Stopped querying.")
        for p in processes:
            p.terminate()
        for p in processes:
            p.join()



if __name__ == "__main__":
    main()
