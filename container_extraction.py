"""Extract containers from point cloud and calculate surface corners of each container face"""

import os
import extraction_utils
from container_point_cloud import ContainerPointCloud


def main():
    """Main method."""
    search_directory = "../"

    in_pc_path = os.path.join(search_directory,
                              "25d_051_2020_11_25_18_54_50_cleaned.ply")
    out_filtered_pc_path = os.path.join(search_directory, "filtered_pc.ply")
    out_surface_corners_path = os.path.join(search_directory,
                                            "surface_corners.pkl")
    out_raster_paths = os.path.join(search_directory, "raster_paths.pkl")

    cpc = ContainerPointCloud(verbose=True)

    cpc.set_out_paths(out_filtered_pc_path, out_surface_corners_path,
                      out_raster_paths)
    cpc.read_pcd(in_pc_path)
    cpc.remove_ground_points()
    cpc.cluster_entities()
    cpc.identify_containers()
    cpc.rasterize_surfaces()
    cpc.write()

    tour, avg_pts, connections = extraction_utils.connect_faces(
        cpc.raster_paths)

    final_connections = extraction_utils.make_full_path(
        tour, avg_pts, cpc.raster_paths, connections,
        cpc.containers)

    extraction_utils.plot_container_surfaces(cpc.containers)
    extraction_utils.graph_surface_containers(cpc.containers)
    extraction_utils.plot_raster_paths(cpc.raster_paths)

    print()
    print(final_connections)
    return final_connections


if __name__ == '__main__':
    main()
