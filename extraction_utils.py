"""Defines utility functions used for extracting containers from point clouds."""

import colorsys
import math
import pickle
import time
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from surface import Surface
from container import Container

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
    normal, inlier_indices = pcd.segment_plane(distance_threshold=dist,
                                               ransac_n=ransac_n,
                                               num_iterations=num_iters)
    if verbose:
        [a, b, c, d] = normal  # pylint: disable=invalid-name
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
        print("Found %d plane points" % len(inlier_indices))
    return normal[:3], inlier_indices


def is_noise_cluster(pcd, cluster_indices):
    """Given a DBSCAN cluster, classify it as noise if the cluster points do not
    form a height of at least 8.5 feet (standard shipping container height)
    """
    cluster_points = np.asarray(pcd.points)[cluster_indices]
    min_height, max_height = 1e9, -1e9
    for point in cluster_points:
        min_height = min(point[2], min_height)
        max_height = max(point[2], max_height)
    return max_height - min_height < (CONTAINER_HEIGHT - CONTAINER_EPS)


def get_container_metadata(  # pylint: disable=too-many-arguments
        pcd,
        cluster_indices,
        dist=0.05,
        ransac_n=30,
        num_iters=500,
        min_points=800,
        min_num_planes=3):
    """
    Fits planes to the surfaces of each cluster, and returns whether the cluster
    is a container (more than 3 surfaces)

    Returns:
    is_container (bool): True if cluster is a container
    planes (dict): Dictionary object representing the surfaces of the cluster
    cluster_obb (np.array): Array of bounding box points that encapsulates
        the entire cluster
    """
    cluster_pcd = pcd.select_by_index(cluster_indices)
    cluster_obb = o3d.geometry.OrientedBoundingBox()
    cluster_obb = np.asarray(
        cluster_obb.create_from_points(cluster_pcd.points).get_box_points())
    plane_normal, inlier_indices = get_plane_points(cluster_pcd,
                                                    dist=dist,
                                                    ransac_n=ransac_n,
                                                    num_iters=num_iters)
    surface_id = 0
    surfaces = []

    while len(inlier_indices) > min_points:
        surface = Surface(surface_id)
        surface.set_normal(plane_normal)
        surface.set_points(np.asarray(cluster_pcd.points)[inlier_indices])
        surfaces.append(surface)
        surface_id += 1

        cluster_pcd = o3d.geometry.PointCloud(
            cluster_pcd.select_by_index(inlier_indices, invert=True))
        if len(cluster_pcd.points) < min_points:
            break
        plane_normal, inlier_indices = get_plane_points(cluster_pcd,
                                                        dist=dist,
                                                        ransac_n=ransac_n,
                                                        num_iters=num_iters)

    is_container = (len(surfaces) >= min_num_planes)
    return is_container, surfaces, cluster_obb


def plot_container_surfaces(containers):
    """Plot each surface and its respective normal"""
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(1, 2, 1, projection='3d')  # pylint: disable=invalid-name
    topdown_ax = fig.add_subplot(1, 2, 2, projection='3d')

    for container_id in range(len(containers)):
        container = containers[container_id]
        color = container.color
        for surface_id in range(len(container.surfaces)):
            points = container.surfaces[surface_id].points
            normal = container.surfaces[surface_id].normal
            ax.scatter(points[:, 0],
                       points[:, 1],
                       points[:, 2],
                       alpha=0.1,
                       color=color)
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


def graph_surface_containers(containers):
    """Graph container and container surface OBBs"""
    fig = plt.figure(figsize=(10, 6))
    axs = [fig.add_subplot(1, 2, (i + 1), projection='3d') for i in range(2)]

    mins = [1e9, 1e9, 1e9]
    maxs = [-1e9, -1e9, -1e9]

    for container_id in range(len(containers)):
        container = containers[container_id]
        color = container.color
        for surface_id in range(len(container.surfaces)):
            pts = container.surfaces[surface_id].corners
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

"""
Creates path between different faces

Parameters:
raster_paths - a list of raster paths per face

Returns:
tour - a permutation of numbers between 0 and num_faces - 1
avg_pts - the average point of the raster path of each face
connections - a list of tuples with each tuple being one connection
"""
def connect_faces(raster_paths):
    # Find how to optimally connect each raster path
    avg_pts = []
    for path in raster_paths:
        path = np.array(path)
        avg_pt = np.sum(path, axis=0) / path.shape[0]
        avg_pts.append(avg_pt)
    num_pts = len(avg_pts)
    avg_pts = np.array(avg_pts)
    tour = np.random.permutation(np.arange(num_pts))
    for temp in np.logspace(5, 0, num=5000):
        [i, j] = np.random.randint(0, num_pts, 2)
        new_pts = np.copy(avg_pts)
        new_pts[[i, j], :] = avg_pts[[j, i], :]
        new_tour = np.copy(tour)
        new_tour[[i, j]] = tour[[j, i]]
        new_cost = sum([
            np.sqrt(
                np.sum(np.power((new_pts[p] - new_pts[(p + 1) % num_pts]), 2)))
            for p in range(num_pts)
        ])
        old_cost = sum([
            np.sqrt(
                np.sum(np.power((avg_pts[p] - avg_pts[(p + 1) % num_pts]), 2)))
            for p in range(num_pts)
        ])
        if np.exp((old_cost - new_cost) / temp) > np.random.random():
            tour = np.copy(new_tour)
            avg_pts = np.copy(new_pts)
    connections = make_connections(tour, avg_pts, raster_paths)
    return (tour, avg_pts, connections)


"""
Makes connections between different faces by finding the closest point to the next face

Parameters:
tour - permutation of faces
avg_pts - the avg_pt of each face
raster_paths - full raster paths (all points on that face) 

Returns:
connections - list of connections (list of tuples with (first point,second point))
"""
def make_connections(tour, avg_pts, raster_paths):
    num_pts = len(avg_pts)
    connections = []
    for i, stop in enumerate(tour):
        next_stop = tour[(i + 1) % num_pts]
        first = avg_pts[stop]
        second = avg_pts[next_stop]
        path = raster_paths[stop]
        min_dist = 10000
        min_index = -1
        for j, point in enumerate(path):
            dist = np.sqrt(np.sum(np.power((point - second), 2)))
            if dist < min_dist:
                min_index = j
                min_dist = dist
        first_point = path[min_index]
        first_index = min_index
        second_path = raster_paths[next_stop]
        min_dist = 10000
        min_index = -1
        for j, point in enumerate(second_path):
            dist = np.sqrt(np.sum(np.power((point - first_point), 2)))
            if dist < min_dist:
                min_index = j
                min_dist = dist
        first_p = (first_point, first_index)
        second_p = (second_path[min_index], min_index)
        connections.append([first_p, second_p])
    return connections

"""
Tests for intersection between a ray and any container in the scene
Parameters: o - origin of ray, numpy array
d - direction of ray, numpy array
max_t - max value of t for p = o + td
containers - a list of containers

Returns: bool - whether or not a ray intersects any container
"""
def intersects_container(o, d, max_t, containers):
    for container_id in range(len(containers)):
        container = containers[container_id]
        for surface_id in range(len(container.surfaces)):
            pts = container.surfaces[surface_id].points
            A = pts[0] - pts[1]
            B = pts[2] - pts[1]
            C = pts[3]
            N = np.cross(A, B)
            denom = np.dot(d, N)
            if abs(denom) > 0.0001:
                t = np.dot((C - o), N) / denom
                if t < max_t and t > 0:
                    return True
    return False

"""
Finds an approximate solution to a TSP problem to find a path between all connections
given an initial solution (tour)

Parameters: tour - an ordering of points (i.e. a permutation of the numbers between 0 and len(avg_pts)-1)
avg_pts - the points to connect
connections - a list of tuples laid out as (first point, second point)
cpc - list of containers
"""
def make_full_path(tour, avg_pts, raster_paths, connections, cpc):
    final_connections = []
    for index, connect in enumerate(connections):
        fp, sp = connect
        o = fp[0]
        d = sp[0] - fp[0]
        max_t = 1.0
        count = 0
        while intersects_container(o, d, max_t, cpc):
            if count > 10:
                print("failure")
                exit()
            avg_pt = avg_pts[tour[index]]
            avg_pt2 = avg_pts[tour[(index + 1) % len(tour)]]
            if count % 2 == 0:
                new_o = o + (o - avg_pt) * 0.1
                o = new_o
            else:
                new_d = sp[0] + (sp[0] - avg_pt2) * 0.1
                d = new_d - o
        first_point = (fp, o)
        second_point = (sp, o + d)
        final_connections.append([first_point, second_point])
    return final_connections


def rasterize_container_face(surface, offset=0.75, alpha=0.65):  # pylint: disable=too-many-locals
    """Constructs a raster path off a container face.

    Raster paths are constructed using "rails". Rails are two parallel edges of the container
    going in the same direction. Corresponding tickpoints will be uniformly spaced along these
    lines. A line segment normal to the rail will then be drawn through the tickpoints, and
    interior raster points will be extracted from these lines.

    Arguments:
        surface (Surface class): The surface object to rasterize.
    """
    
    # Shift corner points inwards slightly to prevent redudant edge points / surface collisions
    shifted_corner_points = alpha*surface.corners + (1-alpha)*surface.midpoint

    rails = [(shifted_corner_points[0], shifted_corner_points[1]),
             (shifted_corner_points[3], shifted_corner_points[2])]

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
        # add remainder for last tick
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

            path_segment.append(pt_r1 + running_mag_rail_normal * norm_dir + offset*surface.normal)
            running_mag_rail_normal += RASTER_TICK_SIZE

        # Reverse every other path segment to form raster path
        parity = 1 - parity
        if parity:
            path_segment = path_segment[::-1]
        raster_points.extend(path_segment)
    return raster_points

def graph_surface_containers(containers):
    """Graph container and container surface OBBs"""
    fig = plt.figure(figsize=(10, 6))
    axs = [fig.add_subplot(1, 2, (i + 1), projection='3d') for i in range(2)]

    mins = [1e9, 1e9, 1e9]
    maxs = [-1e9, -1e9, -1e9]

    for container_id in range(len(containers)):
        container = containers[container_id]
        color = container.color
        for surface_id in range(len(container.surfaces)):
            pts = container.surfaces[surface_id].corners
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


def plot_raster_paths(raster_paths, containers=None):
    """Plot surface raster paths"""
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    if containers:
        for container_id in range(len(containers)):
            container = containers[container_id]
            color = container.color
            for surface_id in range(len(container.surfaces)):
                pts = container.surfaces[surface_id].corners
                # for dim in range(3):
                #     sorted_pts = sorted(pts, key=lambda x: x[dim])  #pylint: disable=cell-var-from-loop
                #     mins[dim] = min(mins[dim], sorted_pts[0][dim])
                #     maxs[dim] = max(maxs[dim], sorted_pts[-1][dim])
                surface = Poly3DCollection([pts], facecolor=color, alpha=.25)
                ax.add_collection3d(surface)
    for i, raster_path in enumerate(raster_paths):
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
                      length=norm / 3)
    plt.show()
