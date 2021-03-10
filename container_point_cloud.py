"""Defines the Container Point Cloud class."""

import pickle
import time
import numpy as np
import open3d as o3d
import extraction_utils
from extraction_utils import Container

class ContainerPointCloud:  # pylint: disable=too-many-instance-attributes
    """Container point cloud class"""

    DOWNSAMPLE_SIZE = 0.1
    VOXEL_SIZE = 0.1
    NORMALS_KDTREE_SEARCH_RADIUS = 0.3

    def __init__(self, verbose=True):
        self.verbose = verbose
        self.pcd = None
        self.filtered_file = None
        self.surfaces_file = None
        self.rasters_file = None
        self.noise_points = None
        self.ground_normal = None
        self.groundless_pcd = None
        self.dbscan_clusters = None
        self.container_candidate_clusters = None
        self.non_container_ids = None
        self.raster_paths = None
        self.containers = []

    def read_pcd(self, in_pc_path):
        """Read PCD from a file path"""
        self.pcd = o3d.io.read_point_cloud(in_pc_path)
        self.pcd = self.pcd.voxel_down_sample(voxel_size=self.DOWNSAMPLE_SIZE)
        if self.verbose:
            print(self.pcd.points)

    def set_pcd(self, pcd):
        """Save a given PCD"""
        self.pcd = pcd
        self.pcd = self.pcd.voxel_down_sample(voxel_size=self.DOWNSAMPLE_SIZE)
        if self.verbose:
            print(self.pcd.points)

    def set_out_paths(self, filtered_file, surfaces_file, rasters_file):
        """Sets the filepaths to save all relevant processed point cloud data."""
        self.filtered_file = filtered_file
        self.surfaces_file = surfaces_file
        self.rasters_file = rasters_file

    def remove_ground_points(self):
        """Identifies and removes ground points from point cloud"""
        search_param = o3d.geometry.KDTreeSearchParamHybrid(
            radius=self.NORMALS_KDTREE_SEARCH_RADIUS, max_nn=50)
        self.pcd.estimate_normals(search_param=search_param)
        ground_normal, inliers = extraction_utils.get_plane_points(
            self.pcd, dist=0.3, verbose=self.verbose)
        groundless_pcd = self.pcd.select_by_index(inliers, invert=True)
        self.groundless_pcd = groundless_pcd
        self.ground_normal = ground_normal

    def cluster_entities(self):
        """Performs DBSCAN and removes noise clusters"""
        start_time = time.time()
        self.dbscan_clusters = np.array(
            self.groundless_pcd.cluster_dbscan(0.6, 200))
        if self.verbose:
            print("Finished DBSCAN in %.2f seconds" %
                  (time.time() - start_time))
            print("Found %d DBSCAN clusters" % len(set(self.dbscan_clusters)))

        self.noise_points = []
        self.container_candidate_clusters = {}

        for cluster_id in set(self.dbscan_clusters):
            point_indices = np.where(self.dbscan_clusters == cluster_id)[0]
            if cluster_id == -1 or extraction_utils.is_noise_cluster(
                    self.groundless_pcd, point_indices):
                if self.verbose:
                    print("Cluster %d is %snoise" %
                          (cluster_id,
                           "definitionally " if cluster_id == -1 else ""))
                self.noise_points.extend(point_indices)
            else:
                self.container_candidate_clusters[cluster_id] = point_indices

    def identify_containers(self):  # pylint: disable=too-many-locals
        """Identify container clusters and calculates corner points per each surface of
        each container."""
        self.groundless_pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.NORMALS_KDTREE_SEARCH_RADIUS, max_nn=50))

        self.non_container_ids = []

        palette = extraction_utils.get_color_palette(
            len(set(self.dbscan_clusters)))

        color_map = {
            cluster_idx: palette[k]
            for k, cluster_idx in enumerate(set(self.dbscan_clusters))
        }

        for container_id in self.container_candidate_clusters:
            point_indices = self.container_candidate_clusters[container_id]
            is_container, surfaces, cluster_obb = extraction_utils.get_container_metadata(
                self.groundless_pcd, point_indices)

            if self.verbose:
                print("Cluster %d: %s, %d planes" %
                      (container_id, str(is_container), len(surfaces)))

            if is_container:
                container = Container(container_id, color_map[container_id])
                container.set_surfaces(surfaces)
                container.set_container_obb(cluster_obb)
                container.filter_surfaces()
                self.containers.append(container)
            else:
                self.non_container_ids.extend(point_indices)

        if self.verbose:
            print("Found %d containers\n" % len(self.containers))

        for container_id in range(len(self.containers)):
            for surface_id in range(len(self.containers[container_id].surfaces)):
                surface = self.containers[container_id].surfaces[surface_id]
                corners = extraction_utils.fit_quadrilateral(surface.points, surface.normal)
                surface.set_corners(corners)

        self.groundless_pcd = self.groundless_pcd.select_by_index(
            self.noise_points + self.non_container_ids, invert=True)

    def rasterize_surfaces(self):
        """Generate raster paths for each of the container surfaces."""
        self.raster_paths = []
        for container_id in range(len(self.containers)):
            for surface_id in range(len(self.containers[container_id].surfaces)):
                surface = self.containers[container_id].surfaces[surface_id]
                self.raster_paths.append(
                    extraction_utils.rasterize_container_face(surface.corners))

    def write(self):
        """Writes the filtered point cloud, container surface corner points, and raster paths
        to their respective appropriate file paths"""
        if self.filtered_file is None or self.surfaces_file is None or self.rasters_file is None:
            raise Exception("One or more out paths not specified")

        containers_json = {}
        for container_id in range(len(self.containers)):
            container = self.containers[container_id]
            containers_json[container.uid] = {}
            for surface_id in range(len(container.surfaces)):
                surface = self.containers[container_id].surfaces[surface_id]
                containers_json[container.uid][surface.uid] = surface.corners.tolist()


        with open(self.surfaces_file, 'wb') as fout:
            pickle.dump(containers_json, fout)
        with open(self.rasters_file, 'wb') as fout:
            pickle.dump(self.raster_paths, fout)
        o3d.io.write_point_cloud(self.filtered_file, self.groundless_pcd)
