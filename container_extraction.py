"""Extract containers from point cloud and calculate surface corners of each container face"""

import os
import math
import time
import pickle
import colorsys
import itertools
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull  # pylint: disable=no-name-in-module
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

DOWNSAMPLE_SIZE = 0.1
VOXEL_SIZE = 0.1
NORMALS_KDTREE_SEARCH_RADIUS = 0.3
CONTAINER_HEIGHT = 8.5 * 0.3048  # 8.5 ft to meters
CONTAINER_EPS = 0.75  # to account for height error
RASTER_TICK_SIZE = 0.5  # in meters


def get_color_palette(num_colors=20):
    """Generate a variable number of different colors"""
    hsv = [(x * 1.0 / num_colors, 0.5, 0.5) for x in range(num_colors)]
    rgb = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv))
    return rgb


def get_plane_points(pcd, dist=0.05, ransac_n=30, num_iters=500, verbose=False):
    """Extract plane with largest support in point cloud"""
    normal, inliers = pcd.segment_plane(distance_threshold=dist,
                                        ransac_n=ransac_n,
                                        num_iterations=num_iters)
    if verbose:
        [a, b, c, d] = normal  # pylint: disable=invalid-name
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
        print("Found %d plane points" % len(inliers))
    return normal[:3], inliers


def is_noise_cluster(pcd, cluster_points):
    """Given a DBSCAN cluster, classify it as noise if the cluster points do not
    form a height of at least 8.5 feet (standard shipping container height)
    """
    cluster_points = np.asarray(pcd.points)[cluster_points]
    min_height, max_height = 1e9, -1e9
    for point in cluster_points:
        min_height = min(point[2], min_height)
        max_height = max(point[2], max_height)
    return max_height - min_height < (CONTAINER_HEIGHT - CONTAINER_EPS)


def is_container(  # pylint: disable=too-many-arguments
        pcd,
        cluster_points,
        dist=0.05,
        ransac_n=30,
        num_iters=500,
        min_points=800,
        min_num_planes=3):
    """
    Fits planes to the surfaces of each cluster, and returns whether the cluster
    is a container (more than 3 surfaces)

    Returns:
    res (Bool): True if cluster is a container
    planes (dict): Dictionary object representing the surfaces of the cluster
    cluster_obb (np.array): Array of bounding box points that encapsulates
        the entire cluster
    """
    cluster_pcd = pcd.select_by_index(cluster_points)
    cluster_obb = o3d.geometry.OrientedBoundingBox()
    cluster_obb = np.asarray(
        cluster_obb.create_from_points(cluster_pcd.points).get_box_points())
    plane_model, inliers = get_plane_points(cluster_pcd,
                                            dist=dist,
                                            ransac_n=ransac_n,
                                            num_iters=num_iters)
    plane_id = 0
    planes = {}
    while len(inliers) > min_points:
        planes[plane_id] = {}
        planes[plane_id]["model"] = plane_model
        planes[plane_id]["points"] = np.asarray(cluster_pcd.points)[inliers]
        plane_id += 1
        cluster_pcd = o3d.geometry.PointCloud(
            cluster_pcd.select_by_index(inliers, invert=True))
        if len(cluster_pcd.points) < min_points:
            break
        plane_model, inliers = get_plane_points(cluster_pcd,
                                                dist=dist,
                                                ransac_n=ransac_n,
                                                num_iters=num_iters)
    res = len(planes) >= min_num_planes
    return res, planes, cluster_obb


def fit_quadrilateral(points, plane_normal):  # pylint: disable=too-many-locals, too-many-statements
    """Projects 3D surface points to 2D, finds the corner points of the 2D quadrilateral,
    and returns thereconstructed corner points in 3D space"""

    # Project points onto plane
    plane_normal /= np.linalg.norm(plane_normal)
    dists = np.dot(points, plane_normal)
    normal_proj = np.zeros_like(points)
    for i, dist in enumerate(dists):
        normal_proj[i, :] = dist * plane_normal

    proj = points - normal_proj
    # Create arbitrary basis on plane and convert to 2D coords
    basis = np.zeros((3, 2))
    while True:
        [i, j, k] = np.random.randint(len(points), size=3)
        u = proj[j] - proj[i]  # pylint: disable=invalid-name
        u /= np.linalg.norm(u)  # pylint: disable=invalid-name
        v = proj[k] - proj[i]  # pylint: disable=invalid-name
        v /= np.linalg.norm(v)  # pylint: disable=invalid-name
        if abs(np.dot(u, v)) < 1:  # make sure u and v are not parallel
            # Gram schmidt
            v -= np.dot(u, v) * u  # pylint: disable=invalid-name
            v /= np.linalg.norm(v)  # pylint: disable=invalid-name
            basis[:, 0] = u
            basis[:, 1] = v
            break
    coords = np.linalg.lstsq(basis, proj.T)[0].T  #(n x 2)
    mean = np.mean(coords, axis=0)
    coords -= mean

    # Calculate principle axes to somewhat unskew quadrilateral
    coords_x = coords[:, 0]
    coords_y = coords[:, 1]
    cov = np.cov(np.vstack((coords_x, coords_y)))
    _, evec = np.linalg.eig(cov)
    evec = evec.T
    rotated_coords = evec @ coords.T  #(2xn)

    # Calculate convex hull of rotated points, giving CCW hull points that can be
    # used to find area
    rotated_hull = ConvexHull(rotated_coords.T)
    rotated_hull_points = rotated_coords.T[rotated_hull.vertices]

    # Find the four convex hull points that encapsulate the largest area
    best_area = 0
    best_combination = None
    all_idxs = range(len(rotated_hull_points))
    for permutation in set(itertools.combinations(all_idxs, 4)):
        idxs = list(permutation)
        ccw_points = rotated_hull_points[idxs]
        ccw_x = ccw_points[:, 0]
        ccw_y = ccw_points[:, 1]
        # Use shoelace theorem
        area = 0.5 * np.abs(
            np.dot(ccw_x, np.roll(ccw_y, 1)) - np.dot(ccw_y, np.roll(ccw_x, 1)))
        if area > best_area:
            best_area = area
            best_combination = idxs

    rotated_hull_corners = rotated_hull_points[best_combination]
    hull_points = (evec.T @ rotated_hull_points.T).T  #(n x 2)
    hull_corners = (evec.T @ rotated_hull_corners.T).T  #(n x 2)

    # Remean points
    hull_points += mean
    hull_corners += mean
    coords += mean

    hull_normal_proj = normal_proj[rotated_hull.vertices]
    hull_corners_proj = normal_proj[rotated_hull.vertices][best_combination]

    # Unproject
    hull_3d = np.matmul(basis, hull_points.T).T  #(n x 3)
    hull_corners_3d = np.matmul(basis, hull_corners.T).T  #(n x 3)

    hull_3d += (hull_normal_proj)
    hull_corners_3d += (hull_corners_proj)

    return hull_corners_3d


def plot_container_surfaces(cluster_planes):
    """Plot each surface and its respective normal"""
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(1, 2, 1, projection='3d')  # pylint: disable=invalid-name
    topdown_ax = fig.add_subplot(1, 2, 2, projection='3d')

    for cluster_id in cluster_planes:
        for plane_id in cluster_planes[cluster_id]["surfaces"]:
            plane = cluster_planes[cluster_id]["surfaces"][plane_id]
            points = plane["points"]
            normal = np.array(plane["model"][:3])
            ax.scatter(points[:, 0], points[:, 1], points[:, 2], alpha=0.1)
            mean = np.mean(points, axis=0)
            ax.quiver(mean[0],
                      mean[1],
                      mean[2],
                      normal[0],
                      normal[1],
                      normal[2],
                      length=5,
                      color='black')
            topdown_ax.scatter(points[:, 0],
                               points[:, 1],
                               points[:, 2],
                               alpha=0.1)
            topdown_ax.quiver(mean[0],
                              mean[1],
                              mean[2],
                              normal[0],
                              normal[1],
                              normal[2],
                              length=5,
                              color='black')

    ax.set_zlim(-2, 10)
    topdown_ax.view_init(azim=0, elev=90)
    plt.show()


def graph_surface_containers(container_surface_corners):
    """Graph container and container surface OBBs"""
    fig = plt.figure(figsize=(10, 6))
    axs = [fig.add_subplot(1, 2, (i + 1), projection='3d') for i in range(2)]

    mins = [1e9, 1e9, 1e9]
    maxs = [-1e9, -1e9, -1e9]

    for shape in container_surface_corners:
        pts, color = shape  # pts is (4x3)
        for dim in range(3):
            sorted_pts = sorted(pts, key=lambda x: x[dim])  #pylint: disable=cell-var-from-loop
            mins[dim] = min(mins[dim], sorted_pts[0][dim])
            maxs[dim] = max(maxs[dim], sorted_pts[-1][dim])
        for i in range(2):
            surface = Poly3DCollection([pts], facecolor=color, alpha=.25)
            axs[i].add_collection3d(surface)

    offset = 1
    for i in range(2):
        axs[i].set_xlim(mins[0] - offset, maxs[0] + offset)
        axs[i].set_ylim(mins[1] - offset, maxs[1] + offset)
        axs[i].set_zlim(mins[2] - offset, 15)

    axs[1].view_init(azim=0, elev=90)
    plt.show()


def rasterize_container_face(corner_points):  # pylint: disable=too-many-locals
    """Constructs a raster path off a container face. The input corner points are
    in CCW order.
    """
    # Rails are two parallel edges of the container going in the same direction.
    # Corresponding tickpoints will be uniformly spaced along these lines. A line
    # segment normal to the rail will then be drawn through the tickpoints, and
    # interior raster points will be extracted from these lines
    corner_points = np.array(corner_points)
    rails = [(corner_points[0], corner_points[1]),
             (corner_points[3], corner_points[2])]

    get_dist = lambda u, v: np.sqrt((u[2] - v[2])**2 + (u[1] - v[1])**2 +
                                    (u[0] - v[0])**2)

    rail_dists = [
        get_dist(rails[0][0], rails[0][1]),
        get_dist(rails[1][0], rails[1][1])
    ]

    diverging_rails_flag = abs(rail_dists[0] - rail_dists[1]) > RASTER_TICK_SIZE

    # Dist between two ticks for each rail
    tickdists = [RASTER_TICK_SIZE, RASTER_TICK_SIZE]
    # Remainder values if ticks aren't even (aligned with edge of rail)
    rail_remainders = [
        rail_dists[0] -
        (math.floor(rail_dists[0] / RASTER_TICK_SIZE) * RASTER_TICK_SIZE),
        rail_dists[1] -
        (math.floor(rail_dists[1] / RASTER_TICK_SIZE) * RASTER_TICK_SIZE)
    ]
    points_per_rail = 1 + math.ceil(min(rail_dists) / RASTER_TICK_SIZE)

    if diverging_rails_flag:
        tickdists = []
        for rail_id in range(2):
            tickdists.append(rail_dists[rail_id] / (points_per_rail - 1))

    parity = 0
    raster_points = []

    r1_dir = rails[0][1] - rails[0][0]
    r1_dir /= np.linalg.norm(r1_dir)
    r2_dir = rails[1][1] - rails[1][0]
    r2_dir /= np.linalg.norm(r2_dir)

    # Get the corresponding tickpoints along the rails
    # Running magnitudes
    running_mag_r1 = 0
    running_mag_r2 = 0

    for r_tick_id in range(points_per_rail):
        # last tick might not be complete RASTER_TICK_SIZE
        if r_tick_id == points_per_rail - 1:
            running_mag_r1 += rail_remainders[0]
            running_mag_r2 += rail_remainders[1]

        # Tickpoint on rail 1
        pt_r1 = rails[0][0] + running_mag_r1 * r1_dir
        # Tickpoint on rail 2
        pt_r2 = rails[1][0] + running_mag_r2 * r2_dir

        running_mag_r1 += tickdists[0]
        running_mag_r2 += tickdists[1]

        # Extract rail-normal interior points
        norm_dir = pt_r2 - pt_r1
        norm_dir /= np.linalg.norm(norm_dir)

        rail_normal_dist = get_dist(pt_r1, pt_r2)
        rail_normal_remainder = rail_normal_dist - math.floor(
            rail_normal_dist / RASTER_TICK_SIZE) * RASTER_TICK_SIZE
        points_per_rail_normal = math.ceil(1 +
                                           rail_normal_dist / RASTER_TICK_SIZE)

        path_segment = []

        running_mag_rail_normal = 0
        for n_tick_id in range(points_per_rail_normal):
            if n_tick_id == points_per_rail_normal - 1:
                running_mag_rail_normal += rail_normal_remainder

            path_segment.append(pt_r1 + running_mag_rail_normal * norm_dir)
            running_mag_rail_normal += RASTER_TICK_SIZE

        # Reverse every other path segment to form raster path
        parity = 1 - parity
        if parity:
            path_segment = path_segment[::-1]
        raster_points.extend(path_segment)
    return raster_points


def plot_raster_paths(raster_paths):
    """Plot surface raster paths"""
    fig = plt.figure(figsize=(15, 8))
    for i, raster_path in enumerate(raster_paths):
        ax = fig.add_subplot(6, 6, (i + 1), projection='3d')  # pylint: disable=invalid-name
        cur_direction = None
        cur_norm = None
        for j, point in enumerate(raster_path):
            direction = None
            norm = None
            if j + 1 >= len(raster_path) and cur_direction is not None:
                direction = cur_direction
                norm = cur_norm
            else:
                direction = raster_path[j + 1] - point
                norm = np.linalg.norm(direction)
                direction /= norm
            cur_direction = direction
            cur_norm = norm
            ax.quiver(point[0],
                      point[1],
                      point[2],
                      direction[0],
                      direction[1],
                      direction[2],
                      length=norm / 3,
                      color='blue')
    plt.show()


class ContainerPointCloud:  # pylint: disable=too-many-instance-attributes
    """Container point cloud class"""

    def __init__( #pylint: disable=too-many-arguments
            self,
            in_pc_path,
            out_filtered_pc_path,
            out_surface_corners_path,
            out_raster_paths,
            verbose=True):
        self.verbose = verbose
        self.out_filtered_pc_path = out_filtered_pc_path
        self.out_surface_corners_path = out_surface_corners_path
        self.out_raster_paths = out_raster_paths
        self.pcd = o3d.io.read_point_cloud(in_pc_path)
        self.pcd = self.pcd.voxel_down_sample(voxel_size=DOWNSAMPLE_SIZE)
        self.noise_points = None
        self.ground_normal = None
        self.groundless_pcd = None
        self.dbscan_clusters = None
        self.container_candidate_clusters = None
        self.cluster_planes = None
        self.non_container_ids = None
        self.container_surface_corners = None
        self.raster_paths = None
        if verbose:
            print(self.pcd.points)

    def remove_ground_points(self):
        """Identifies and removes ground points from point cloud"""
        self.pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=NORMALS_KDTREE_SEARCH_RADIUS, max_nn=50))
        ground_normal, inliers = get_plane_points(self.pcd,
                                                  dist=0.3,
                                                  verbose=self.verbose)
        groundless_pcd = self.pcd.select_by_index(inliers, invert=True)
        self.groundless_pcd = groundless_pcd
        self.ground_normal = ground_normal

    def cluster_entities(self):
        """Performs DBSCAN and removes noise clusters"""
        start_time = time.time()
        dbscan_clusters = np.array(self.groundless_pcd.cluster_dbscan(0.6, 200))
        if self.verbose:
            print("Finished DBSCAN in %.2f seconds" %
                  (time.time() - start_time))
            print("Found %d DBSCAN clusters" % len(set(dbscan_clusters)))
        self.dbscan_clusters = dbscan_clusters
        noise_points = []
        container_candidate_clusters = {}

        for cluster_id in set(self.dbscan_clusters):
            cluster_points = np.where(self.dbscan_clusters == cluster_id)[0]
            if cluster_id == -1 or is_noise_cluster(self.groundless_pcd,
                                                    cluster_points):
                if self.verbose:
                    print("Cluster %d is %snoise" %
                          (cluster_id,
                           "definitionally " if cluster_id == -1 else ""))
                noise_points.extend(cluster_points)
            else:
                container_candidate_clusters[cluster_id] = cluster_points

        self.noise_points = noise_points
        self.container_candidate_clusters = container_candidate_clusters

    def calculate_surfaces(self):  # pylint: disable=too-many-locals
        """Identify container clusters and calculates corner points per each surface of
        each container."""
        self.groundless_pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=NORMALS_KDTREE_SEARCH_RADIUS, max_nn=50))
        cluster_planes = {}
        non_container_ids = []

        for cluster_id in self.container_candidate_clusters:
            cluster_points = self.container_candidate_clusters[cluster_id]
            ret, planes, cluster_obb = is_container(self.groundless_pcd,
                                                    cluster_points)
            if self.verbose:
                print("Cluster %d: %s, %d planes" %
                      (cluster_id, str(ret), len(planes)))
            if ret:
                cluster_planes[cluster_id] = {}
                cluster_planes[cluster_id]["surfaces"] = planes
                cluster_planes[cluster_id]["obb"] = cluster_obb
            else:
                non_container_ids.extend(cluster_points)

        if self.verbose:
            print("Found %d container clusters" % len(cluster_planes))

        self.cluster_planes = cluster_planes
        self.non_container_ids = non_container_ids

        palette = get_color_palette(len(set(self.dbscan_clusters)))

        color_map = {
            cluster_idx: palette[k]
            for k, cluster_idx in enumerate(set(self.dbscan_clusters))
        }

        container_surface_corners = []

        for cluster_id in self.cluster_planes:
            for plane_id in self.cluster_planes[cluster_id]["surfaces"]:
                plane = self.cluster_planes[cluster_id]["surfaces"][plane_id]
                normal = np.array(plane["model"][:3])

                points = plane["points"]
                corners_3d = fit_quadrilateral(points, normal)
                plane["corner_points"] = corners_3d
                container_surface_corners.append(
                    (corners_3d, color_map[cluster_id]))

        self.container_surface_corners = container_surface_corners
        self.groundless_pcd = self.groundless_pcd.select_by_index(
            self.noise_points + self.non_container_ids, invert=True)

    def rasterize_surfaces(self):
        """Generate raster paths for each of the container surfaces."""
        self.raster_paths = []
        for shape in self.container_surface_corners:
            pts, _ = shape
            raster_path = rasterize_container_face(pts)
            self.raster_paths.append(raster_path)

    def write(self):
        """Writes the filtered point cloud, container surface corner points, and raster paths
        to their respective appropriate file paths"""
        containers_json = {}
        for cluster_id in self.cluster_planes:
            containers_json[cluster_id] = {}
            for plane_id in self.cluster_planes[cluster_id]["surfaces"]:
                plane = self.cluster_planes[cluster_id]["surfaces"][plane_id]
                containers_json[cluster_id][plane_id] = plane[
                    "corner_points"].tolist()
        with open(self.out_surface_corners_path, 'wb') as fout:
            pickle.dump(containers_json, fout)
        with open(self.out_raster_paths, 'wb') as fout:
            pickle.dump(self.raster_paths, fout)
        o3d.io.write_point_cloud(self.out_filtered_pc_path, self.groundless_pcd)


def main():
    """Main method."""
    search_directory = "/Users/michaelwan/Desktop/CS/School (CS)/research/"

    in_pc_path = os.path.join(search_directory,
                              "25d_051_2020_11_25_18_54_50_cleaned.ply")
    out_filtered_pc_path = os.path.join(search_directory, "filtered_pc.ply")
    out_surface_corners_path = os.path.join(search_directory,
                                            "surface_corners.pkl")
    out_raster_paths = os.path.join(search_directory, "raster_paths.pkl")

    cpc = ContainerPointCloud(in_pc_path,
                              out_filtered_pc_path,
                              out_surface_corners_path,
                              out_raster_paths,
                              verbose=True)
    cpc.remove_ground_points()
    cpc.cluster_entities()
    cpc.calculate_surfaces()
    cpc.rasterize_surfaces()
    cpc.write()

    plot_container_surfaces(cpc.cluster_planes)
    graph_surface_containers(cpc.container_surface_corners)
    plot_raster_paths(cpc.raster_paths)


if __name__ == '__main__':
    main()
