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

def save_np_to_file(file_path, np_array):
    np.save(file_path, np_array)
    print(f"Saved to {file_path}")

def batch_gen_save_np_to_file(root_file_path, num_embeds, dim, num_batch):
    '''
    Handling very large embeddings gen, we save them in batches to npy files in a directory
    '''
    # create dir if not exist
    os.makedirs(root_file_path, exist_ok=True)
    
    # gen and save batches
    batch_num_embeds = num_embeds // num_batch
    for i in range(num_batch):
        batch_np = random_embeddings(batch_num_embeds, dim)
        save_np_to_file(os.path.join(root_file_path, f"embeddings_{i}.npy"), batch_np)

def apx_num_dp_from_idx_size_gb(idx_size_gb, dim):
    '''
    Note: result index slightly differ from the given size. 

    Get an approximate number of data points from the index size in GB
    1 float32 = 4 bytes, 1024**3 bytes = 1 GB
    idx_size_gb * 1024**3 bytes = idx_size_gb GB
    '''
    byte_factor = 1.1
    return int(idx_size_gb * 1024**3 / (dim * 4 * byte_factor))

# main function
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Create index for vector database")
    parser.add_argument("-isg", "--idx_size_gb", default=1, help="size of the index in GB", type=float,)
    # parser.add_argument("-ne", "--num_embeds", default=1000, help="number of embeddings", type=int,)
    parser.add_argument("--dim", default=512, help="dimension of embeddings", type=int,)
    parser.add_argument("--save", default="tmp_idx", metavar="FILE", help="dir path to save index files", type=str,)
    parser.add_argument("--npy_root", default="tmp_idx/npys", metavar="FILE", help="dir path to save npy files", type=str,)
    args = parser.parse_args()

    num_batch_npy = int(args.idx_size_gb)

    # construct file name based on GB
    idx_size_gb = args.idx_size_gb
    idx_gb_file_str = f"random_{idx_size_gb}gb"
    idx_file_path = os.path.join(args.save, "{}.index".format(idx_gb_file_str))
    index_infos_path = os.path.join(args.save, "{}_infos.json".format(idx_gb_file_str))

    # create random embeddings
    num_embeds = apx_num_dp_from_idx_size_gb(idx_size_gb, args.dim)
    print(f"creating num_embeds: {num_embeds}")
    # embeddings = random_embeddings(num_embeds, args.dim)
    batch_gen_save_np_to_file(args.npy_root, num_embeds, args.dim, num_batch_npy)

    index, index_infos = build_index(
        # embeddings, 
        args.npy_root,
        save_on_disk=True, 
        index_path=idx_file_path,
        index_infos_path=index_infos_path)

    # index = faiss.read_index("my_idx/knn.index", faiss.IO_FLAG_MMAP | faiss.IO_FLAG_READ_ONLY)
