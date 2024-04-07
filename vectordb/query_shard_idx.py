'''
Generate random queries and query shard index files
'''

import os
import time
import faiss
import argparse
import numpy as np

from utils.vdb_utils import load_index, random_floats, random_normal_vectors, query_index_file, random_queries_mix_distribs
from utils.search_by_topology import search_outterloop_index, search_outterloop_query, reverse_stopology

# fix random seed
np.random.seed(0)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Query shard index for vector database")
    parser.add_argument("-idx", "--idx_root", required=False, default="shards/idxs/", help="dir to index files", type=str,)
    parser.add_argument("-np", "--nprobe", default=10, help="a small number (nprobe) of subsets to visit", type=int,)
    parser.add_argument("-k", default=3, help="top k results", type=int,)
    parser.add_argument("-nq", "--num_query", default=5, help="number of queries", type=int,)
    parser.add_argument("-mr", "--mixtures_ratio", default=0., help="mixtures ratio for random queries", type=float,)
    # hardcode args for now
    parser.add_argument("-d", "--dim", default=128, help="dimension of embeddings", type=int,)
    args = parser.parse_args()

    # args
    print(args)
    index_root = args.idx_root
    nprobe = args.nprobe
    k = args.k
    dim = args.dim
    num_queries = args.num_query
    mixtures_ratio = args.mixtures_ratio

    # idx_k: k for each index search
    # idx_k = (k // nprobe) + k
    idx_k = k
    
    # idx_paths = [os.path.join(index_root, f) for f in os.listdir(index_root)]
    idx_paths = []
    centriod_idx_paths = ""
    for f in os.listdir(index_root):
        if "centroid" in f:
            centriod_idx_paths = os.path.join(index_root, f)
        else:
            idx_paths.append(os.path.join(index_root, f))    

    # Generate random queries from normal distribution with mean 0 and std 1
    # random_mean = random_floats(1, low=-1, high=1)
    # random_std = [0.5]
    # queries = random_normal_vectors(num_queries, dim, random_mean[0], random_std[0])
    queries = random_queries_mix_distribs(num_queries, dim, mixtures_ratio=mixtures_ratio, low=-1, high=.1)

    # knn find top centroids
    D, I = query_index_file(centriod_idx_paths, queries, nprobe)
    print("top-{} centroids:".format(nprobe))
    print(I)

    # create search topology
    stopology = {}
    for i in range(num_queries):
        stopology[i] = list(I[i])
    
    # reverse search topology
    rstopology = reverse_stopology(stopology)

    # search queries
    start_time = time.perf_counter()
    D_matrix, I_matrix, file_idx_matrix = search_outterloop_index(rstopology, queries, idx_k, k, idx_paths)
    end_time = time.perf_counter()
    qb_runtime = end_time - start_time

    print()
    print("Distances")
    print(D_matrix)
    print()
    print("Index")
    print(I_matrix)
    print()
    print("Index file")
    print(file_idx_matrix)

    print(f"Search time: {qb_runtime:.8f}s")
