"""Extract containers from point cloud and calculate surface corners of each container face"""

import argparse
import os
import pickle

import extraction_utils
from container_point_cloud import ContainerPointCloud


def read_paths():
    """Read input and output paths."""
    if not os.path.exists("files.path"):
        raise Exception("Input and output path files not specified.")
    paths = {}
    with open("files.path", "r") as f:
        for line in f.readlines():
            key, value = line.split('=')
            paths[key] = value.strip()
    return paths


def main():
    """Main method."""

    parser = argparse.ArgumentParser()
    parser.add_argument("-v",
                        "--verbose",
                        help="print outputs of point cloud processing",
                        action="store_true")
    parser.add_argument(
        "-g",
        "--graph",
        help="output graphs to visualize point cloud processing",
        action="store_true")
    args = parser.parse_args()

    paths = read_paths()

    cpc = ContainerPointCloud(verbose=args.verbose)
    cpc.read_pcd(paths["in"])
    cpc.remove_ground_points()
    cpc.cluster_entities()
    cpc.identify_containers()
    cpc.rasterize_surfaces()

    tour, avg_pts, connections = extraction_utils.connect_faces(
        cpc.raster_paths)

    if args.verbose:
        print("Tour...")
        print(tour.shape)
        print(tour)
        print()

        print("Avg_pts...")
        print(avg_pts.shape)
        print(avg_pts[0].shape)
        print(avg_pts)
        print()

        print("Connections...")
        print(connections)
        print()

    final_connections = extraction_utils.make_full_path(tour, avg_pts,
                                                        cpc.raster_paths,
                                                        connections,
                                                        cpc)

    if args.graph:
        extraction_utils.plot_container_surfaces(cpc.containers)
        extraction_utils.graph_surface_containers(cpc.containers)
        extraction_utils.plot_raster_paths(cpc.raster_paths, None)

    if args.verbose:
        print("Final connections...")
        print(final_connections)
        print()

    with open(paths["out"], 'wb') as fout:
        pickle.dump(final_connections, fout)

    return final_connections


if __name__ == '__main__':
    main()
