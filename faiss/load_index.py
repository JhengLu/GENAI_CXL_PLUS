import numpy as np
import faiss
import os
import time

def load_index_and_query_metrics(index_file_path, dimension, n_queries=10, n_neighbors=10):
    """
    Load an index from disk, perform queries, and display performance metrics.
    - index_file_path: Path to the stored FAISS index file.
    - dimension: Dimension of the vectors indexed.
    - n_queries: Number of query vectors to generate and search for.
    - n_neighbors: Number of nearest neighbors to search for each query vector.
    """
    if not os.path.exists(index_file_path):
        print(f"Index file {index_file_path} not found.")
        return

    # Load the index
    index = faiss.read_index(index_file_path)
    print(f"Index loaded from {index_file_path} with {index.ntotal} vectors.")

    # Generate random query vectors
    query_vectors = np.random.rand(n_queries, dimension).astype('float32')

    # Perform the search and measure the time
    start_time = time.time()
    D, I = index.search(query_vectors, n_neighbors)  # D: distances, I: indices
    end_time = time.time()

    # Calculate and display metrics
    total_query_time = end_time - start_time
    average_query_time = total_query_time / n_queries
    average_distance = np.mean(D)
    print(f"Total query time: {total_query_time:.4f} seconds")
    print(f"Average query time per vector: {average_query_time:.4f} seconds")
    print(f"Average distance of the nearest neighbors: {average_distance:.4f}")
    print(f"Query results (indices of the nearest neighbors for the first query vector): {I[0]}")

# Example usage parameters
index_size_gb = 10
index_file_path = f"faiss_index{index_size_gb}.index"  # Path where the index is stored
dimension = 512  # Dimension of the vectors

# Uncomment the line below to load the index, perform queries, and see the performance metrics
while True:
    load_index_and_query_metrics(index_file_path, dimension)
