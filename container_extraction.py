"""Extract containers from point cloud and calculate surface corners of each container face"""

import argparse
import os
import extraction_utils
from container_point_cloud import ContainerPointCloud

def main():
    """Main method."""

    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", help="print outputs of point cloud processing", action="store_true")
    parser.add_argument("-g", "--graph", help="output graphs to visualize point cloud processing", action="store_true")
    args = parser.parse_args()

    search_directory = "../"

    in_pc_path = os.path.join(search_directory,
                              "25d_051_2020_11_25_18_54_50_cleaned.ply")

    cpc = ContainerPointCloud(verbose=args.verbose)
    cpc.read_pcd(in_pc_path)
    cpc.remove_ground_points()
    cpc.cluster_entities()
    cpc.identify_containers()
    cpc.rasterize_surfaces()

    tour, avg_pts, connections = extraction_utils.connect_faces(
        cpc.raster_paths)

    final_connections = extraction_utils.make_full_path(
        tour, avg_pts, cpc.raster_paths, connections,
        cpc.containers)

    if args.graph:
        extraction_utils.plot_container_surfaces(cpc.containers)
        extraction_utils.graph_surface_containers(cpc.containers)
        extraction_utils.plot_raster_paths(cpc.raster_paths, None)

    print()
    print(final_connections)
    return final_connections


if __name__ == '__main__':
    main()
