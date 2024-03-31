import argparse
import os

from collections import defaultdict
from sklearn.cluster import KMeans
import numpy as np
# import pandas as pd

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Split dataset by centroids")

    parser.add_argument("--k", default=2, help="number of cluster centers", type=int,)
    parser.add_argument("--dataset", default="default_dataset.npy", metavar="FILE", help="dir path to save centroid files", type=str,)
    parser.add_argument("--path", default="tmp_centroids", metavar="FILE", help="dir path to save centroid files", type=str,)
    parser.add_argument("--outfile_prefix", default="cluster_", help="prefix for all generated files", type=str)
    args = parser.parse_args()
    k = args.k
    dataset = np.load(args.dataset)

    kmeans = KMeans(n_clusters=k, random_state=0, n_init="auto").fit(dataset)
    print(kmeans.labels_)
    print(kmeans.cluster_centers_)

    center_to_data = defaultdict(list)
    for center, item in zip(kmeans.labels_, dataset):
        center_to_data[center].append(item)
    
    for center, group in center_to_data.items():
        print(f"{center}: {group}")

        if os.path.isdir(args.path):
            # Construct the complete path by joining the path with the filename
            full_file_path = os.path.join(args.path, f'{args.outfile_prefix}{center}.npy')

            # Save the file
            np.save(full_file_path, group)
            print(f'File saved successfully to {full_file_path}')
        else:
            print(f'Error: The path provided "{args.path}" does not exist or is not a directory.')

    context_path = os.path.join(args.path, f'{args.outfile_prefix}context.npy')
    np.save(context_path, kmeans.cluster_centers_)

