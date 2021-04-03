"""Defines the Container class object"""
import statistics
import numpy as np
from scipy.spatial import Delaunay  # pylint: disable=no-name-in-module
from scipy.spatial.distance import cosine, directed_hausdorff, cdist
from sklearn.cluster import DBSCAN

import math_utils


def surface_contains_points(points, corners_2d):
    """Check if the points are contained within the 2D corners coordinates.
    
    This method splits the 4 2D corner points into two triangles and checks if the points
    are in either triangle.

    Arguments:
        points (np.ndarray): 2D points array of shape (n, 2). The points
            to be checked.
        corners_2d (np.ndarray): 4 2D points that define the corners of the
            surface.
    """
    dists = [
        (np.linalg.norm(corners_2d[3] - corners_2d[i]), i) for i in range(3)
    ]
    dists = sorted(dists, key=lambda d: d[0])
    tri1 = corners_2d[:3]
    tri2 = corners_2d[[dists[0][1], dists[1][1], 3]]
    for i in range(points.shape[0]):
        if math_utils.point_in_triangle(points[i, :],
                                        tri1) or math_utils.point_in_triangle(
                                            points[i, :], tri2):
            return True
    return False


class Container:
    """Defines a container class"""

    def __init__(self, uid=None, color=None, verbose=False):
        self.uid = uid
        self.color = color
        self.surfaces = []
        self.container_obb = None
        self.verbose = verbose

    def set_surfaces(self, surfaces):
        """Sets the surfaces of this container."""
        self.surfaces = surfaces

    def set_container_obb(self, obb):
        """Sets the container's 3D bounding box."""
        self.container_obb = obb

    def filter_surfaces(  # pylint: disable=too-many-locals
            self,
            ground_normal=None,
            ground_similarity_thresh=0.9,
            hausdorff_rel_thresh=0.2,
            cos_thresh=0.01,
            sample_size=100):
        """Remove erroneous and duplicate surfaces.

        This method removes both erroneous surfaces and duplicate surfaces.

        Erroneous surfaces are classified as those with normals parallel to the ground.
        Duplicate surfaces are removed using Hausdorff distances of the two surface points
        and cosine similarity of the normals. The Hausdorff distance is divided by
        the maximum pairwise distance of sample sets of the surface points.

        Arguments:
            ground_normal (np.ndarray or None): The ground normal.
        """
        blacklisted_indices = []
        for i in range(len(self.surfaces)):
            if ground_normal is not None and abs(
                    np.dot(self.surfaces[i].normal,
                           ground_normal)) > ground_similarity_thresh:
                blacklisted_indices.append(i)
                continue
            surf_a = self.surfaces[i]
            surf_a_len = len(surf_a.points)
            for j in range(i + 1, len(self.surfaces)):
                surf_b = self.surfaces[j]
                surf_b_len = len(surf_b.points)
                sample_a = surf_a.points[np.random.randint(surf_a_len,
                                                           size=min(
                                                               surf_a_len,
                                                               sample_size))]
                sample_b = surf_b.points[np.random.randint(surf_b_len,
                                                           size=min(
                                                               surf_b_len,
                                                               sample_size))]
                max_dist = np.max(cdist(sample_a, sample_b))
                cosine_dist = cosine(surf_a.normal, surf_b.normal)
                hausdorff_dist, _, _ = directed_hausdorff(
                    surf_a.points, surf_b.points)
                if cosine_dist < cos_thresh and hausdorff_dist / max_dist < hausdorff_rel_thresh:
                    blacklisted_indices.append(
                        i if surf_a_len < surf_b_len else j)
        self.surfaces = [
            x for (i, x) in enumerate(self.surfaces)
            if i not in blacklisted_indices
        ]

    def enforce_outward_normals(self, detailed=False, eps=1e-2, multiplier=1.2):
        """Fix surface normals such that they are pointing outwards from the surfaces."""
        if self.verbose:
            print("Container %d..." % self.uid)
        for i in range(len(self.surfaces)):
            if np.random.random() < 0.5:
                self.surfaces[i].normal *= -1
            projected_surfaces = []
            intersects = [0, 0]
            for k in range(len(self.surfaces)):
                # Skip self and perpendicular containers
                if i == k:
                    continue
                recentered_points = multiplier * self.surfaces[
                    k].corners - self.surfaces[i].midpoint
                target_coords, _, basis, _ = math_utils.project_2d(
                    recentered_points, self.surfaces[i].normal, demean=False)
                if surface_contains_points(np.array([[0, 0]]), target_coords):
                    sign = np.dot(
                        self.surfaces[k].midpoint - self.surfaces[i].midpoint,
                        self.surfaces[i].normal) < 0
                    if self.verbose:
                        print("Surface %d intersects with %d in the %d dir" %
                              (i, k, sign))
                    intersects[sign] += 1
            if self.verbose:
                print("Surface %d: %s" % (i, str(intersects)))
            if detailed:
                if intersects[0] % 2 == 1 and intersects[1] % 2 == 0:
                    self.surfaces[i].normal *= -1
                    if self.verbose:
                        print("Reversed")
                elif intersects[0] % 2 == 1 and intersects[1] % 2 == 1:
                    raise Exception(
                        "Ray-tracing on surface %d gives incomprehensible results"
                        % i)
            else:
                if intersects[0] > 0 and intersects[1] == 0:
                    self.surfaces[i].normal *= -1
                    if self.verbose:
                        print("Reversed")
                elif not (intersects[0] == 0 and
                          intersects[1] >= 0) and self.verbose:
                    print(
                        "Ray-tracing on surface %d gives incomprehensible results"
                        % i)
        if self.verbose:
            print()

    def remove_noncontiguous_surface(self, eps=0.95, min_samples=10):
        """For each surface, check if the plane is non contiguous, which may create issues
        when generating the plane corner points. Using DBSCAN to identify clusters, we
        check if surfaces are mono-clusters and if not, remove all clusters besides the
        largest one.
        """
        for i in range(len(self.surfaces)):
            clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(
                self.surfaces[i].points)
            if len(set(clustering.labels_)) > 1:
                mode = statistics.mode(clustering.labels_)
                keep = np.where(clustering.labels_ == mode)[0]
                self.surfaces[i].points = self.surfaces[i].points[keep, :]

    def remove_interior_points(self, alpha=0.4):
        """Removes surface points that are within the interior of the container's bounding
        box. These points are unreachable so must be removed. The interior is calculated as
        a weighted average of the container_obb and the midpoint
        """
        midpoint = np.mean(self.container_obb, axis=0)
        interior = (1 - alpha) * self.container_obb + alpha * midpoint
        empty_surface_indices = []
        for i in range(len(self.surfaces)):
            keep = np.where(
                Delaunay(interior).find_simplex(self.surfaces[i].points) < 0)[0]
            self.surfaces[i].points = self.surfaces[i].points[keep, :]
            if len(self.surfaces[i].points) == 0:
                empty_surface_indices.append(i)
            else:
                self.surfaces[i].compute_midpoint()
        self.surfaces = [
            self.surfaces[i]
            for i in range(len(self.surfaces))
            if i not in empty_surface_indices
        ]
