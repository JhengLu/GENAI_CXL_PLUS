
import os
import time
import faiss
import argparse
import numpy as np

from utils.vdb_utils import load_index, random_queries

# query index
def query_index(index, queries, k):
    D, I = index.search(queries, k)
    return D, I

def query_index_file(index_path, queries, k):
    index = load_index(index_path)
    D, I = query_index(index, queries, k)
    return D, I

def batch_query_index(index, queries, k, batch_size=1000):
    D = np.array([])
    I = np.array([])
    for i in range(0, len(queries), batch_size):
        D_batch, I_batch = query_index(index, queries[i:i+batch_size], k)
        D = np.concatenate((D, D_batch), axis=0) if D.size else D_batch
        I = np.concatenate((I, I_batch), axis=0) if I.size else I_batch
    return D, I


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Query shard index for vector database")
    parser.add_argument("-idx", "--idx_root", required=False, default="shards/idxs/", help="dir to index files", type=str,)
    parser.add_argument("-np", "--nprobe", default=10, help="a small number (nprobe) of subsets to visit", type=int,)
    args = parser.parse_args()

    # args
    index_root = args.idx_root
    nprobe = args.nprobe
    # idx_paths = [os.path.join(index_root, f) for f in os.listdir(index_root)]
    idx_paths = []
    centriod_idx_paths = ""
    for f in os.listdir(index_root):
        if "centroid" in f:
            centriod_idx_paths = os.path.join(index_root, f)
        else:
            idx_paths.append(os.path.join(index_root, f))


    # some initializations
    k = 5
    dim = 64
    num_shards = 50
    num_queries = 100

    # process queries
    queries = random_queries(num_queries, dim)

    # knn find top centroids
    idx_k = (k // nprobe) + k

    D, I = query_index_file(centriod_idx_paths, queries, nprobe)
    print(I)
    print(I.shape)

    # convert I to idx_paths: embeds_{idx}.index, do NOT change dimension


    exit()

    # for now randomly select index file to visit
    idxs = list(np.random.choice(idx_paths, nprobe, replace=False))

    # intelligent batching

    # query indexs and merge results
    D_concat = np.array([])
    I_concat = np.array([])
    file_idx_concat = np.array([])

    for i, idx_path in enumerate(idxs):
        start_time = time.perf_counter()
        D, I = query_index_file(idx_path, queries, idx_k)
        end_time = time.perf_counter()
        query_idx_time = end_time - start_time
        # print query time 5digits
        print(f"{i:3} - {idx_path:30}: {query_idx_time:.8f}s")
        # print(D, I)
        # make idx_matrix: [[i, i], [i,i]...], shape same as D and I
        file_idx = np.ones_like(D) * i
        # print(file_idx)
        D_concat = np.concatenate((D_concat, D), axis=1) if D_concat.size else D
        I_concat = np.concatenate((I_concat, I), axis=1) if I_concat.size else I
        file_idx_concat = np.concatenate((file_idx_concat, file_idx), axis=1) if file_idx_concat.size else file_idx

    # sort by distance, and also sort file_idx and I
    sort_idx = np.argsort(D_concat, axis=1)
    D_sorted = np.take_along_axis(D_concat, sort_idx, axis=1)
    I_sorted = np.take_along_axis(I_concat, sort_idx, axis=1)
    file_idx_sorted = np.take_along_axis(file_idx_concat, sort_idx, axis=1).astype(int)

    top_k_idx = I_sorted[:, :k]
    top_k_dist = D_sorted[:, :k]
    top_k_file_idx = file_idx_sorted[:, :k]

    # save results to file    
