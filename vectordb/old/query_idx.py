import os
import time
import argparse
from pprint import pprint
import faiss
import numpy as np
from autofaiss import build_index

def random_embeddings(num_embeds, dim):
    # create random embeddings
    return np.float32(np.random.rand(num_embeds, dim))

def random_queries(num_queries, dim):
    # create random queries
    return np.float32(np.random.rand(num_queries, dim))

# main function
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Query index for vector database")
    parser.add_argument("-idx", "--index", required=True, help="path to index file", type=str,)
    parser.add_argument("--dim", default=512, help="dimension of embeddings", type=int,)
    parser.add_argument("-q", "--query_num", default=100, help="number of queries", type=int,)
    parser.add_argument("-k", "--query_k", default=5, help="number of nearest neighbors", type=int,)
    args = parser.parse_args()

    index_path = args.index
    
    index = faiss.read_index(index_path, faiss.IO_FLAG_MMAP | faiss.IO_FLAG_READ_ONLY)

    # create random queries
    queries = random_queries(args.query_num, args.dim)

    # time it 
    start = time.time()
    D, I = index.search(queries, args.query_k)
    end = time.time()

    # print time per query
    print(f"Performance: {(end - start) / args.query_num} (sec / query)")
    