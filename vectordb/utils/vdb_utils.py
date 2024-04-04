
import numpy as np
import faiss

def random_embeddings(num_embeds, dim):
    # create random embeddings
    data = np.random.random((num_embeds, dim)).astype('float32')
    # data[:, 0] += np.arange(num_embeds) / 1000.
    return data

def random_queries(num_queries, dim):
    # create random queries
    queries = np.random.random((num_queries, dim)).astype('float32')
    # queries[:, 0] += np.arange(num_queries) / 1000.
    return queries

def save_index(index, index_path):
    faiss.write_index(index, index_path)

# load index
def load_index(index_path):
    return faiss.read_index(index_path)