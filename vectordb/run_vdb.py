import os
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
    parser = argparse.ArgumentParser(description="Standardizing PREFIRE data")
    parser.add_argument("--num_embeds", default=1000, help="number of embeddings", type=int,)
    parser.add_argument("--dim", default=512, help="dimension of embeddings", type=int,)
    parser.add_argument("-q", "--query_num", default=1, help="number of queries", type=int,)
    parser.add_argument("-k", "--query_k", default=1, help="number of nearest neighbors", type=int,)
    parser.add_argument("--save", default="my_idx", metavar="FILE", help="path to save index files", type=str,)
    args = parser.parse_args()

    # print args as a dictionary
    print("args:")
    pprint(vars(args))
    print()

    index_path = os.path.join(args.save, "knn.index")
    index_infos_path = os.path.join(args.save, "index_infos.json")

    # create random embeddings
    embeddings = random_embeddings(args.num_embeds, args.dim)

    index, index_infos = build_index(
        embeddings, 
        save_on_disk=True, 
        index_path=index_path,
        index_infos_path=index_infos_path)

    index = faiss.read_index("my_idx/knn.index", faiss.IO_FLAG_MMAP | faiss.IO_FLAG_READ_ONLY)

    # create random queries
    queries = random_queries(args.query_num, args.dim)
    D, I = index.search(queries, args.query_k)
    
    # print res and their distances side by side
    print("results:")
    for i in range(args.query_num):
        print(f"query {i}:")
        print(f"  distances: {D[i]}")
        print(f"  indices: {I[i]}")
        print()