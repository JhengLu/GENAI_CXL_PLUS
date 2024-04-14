import numpy as np
import faiss
import os
import time

def load_index(index_file_path):
    """
    Load an index from disk.
    - index_file_path: Path to the stored FAISS index file.
    Returns the loaded index or None if the file does not exist.
    """
    if not os.path.exists(index_file_path):
        print(f"Index file {index_file_path} not found.")
        return None

    index = faiss.read_index(index_file_path)
    print(f"Index loaded from {index_file_path} with {index.ntotal} vectors.")
    return index

def query_index(index, dimension, n_queries=10, n_neighbors=10):
    """
    Perform queries on a loaded index and display performance metrics.
    - index: Loaded FAISS index.
    - dimension: Dimension of the vectors indexed.
    - n_queries: Number of query vectors to generate and search for.
    - n_neighbors: Number of nearest neighbors to search for each query vector.
    """
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

def main():
    # Example usage parameters
    index_size_gb = 20
    index_file_path = f"faiss_index{index_size_gb}.index"  # Path where the index is stored
    dimension = 512  # Dimension of the vectors

    # Load the index once
    index = load_index(index_file_path)
    if index is None:
        return  # Stop if index cannot be loaded

    try:
        while True:
            query_index(index, dimension)
    except KeyboardInterrupt:
        print("Stopped querying.")

if __name__ == "__main__":
    main()
