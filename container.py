"""Defines the Container class object"""
import statistics
import numpy as np
from scipy.spatial import Delaunay  # pylint: disable=no-name-in-module
from scipy.spatial.distance import cosine, directed_hausdorff, cdist
from sklearn.cluster import DBSCAN


class Container:
    """Defines a container class"""

    def __init__(self, uid=None, color=None):
        self.uid = uid
        self.color = color
        self.surfaces = []
        self.container_obb = None

    def set_surfaces(self, surfaces):
        """Sets the surfaces of this container."""
        self.surfaces = surfaces

    def set_container_obb(self, obb):
        """Sets the container's 3D bounding box."""
        self.container_obb = obb

    def filter_surfaces( # pylint: disable=too-many-locals
            self,
            hausdorff_rel_thresh=0.2,
            cos_thresh=0.01,
            sample_size=100):
        """Remove duplicate surfaces using Hausdorff distances of the two surface points
        and cosine similarity of the normals. The Hausdorff distance is divided by
        the maximum pairwise distance of sample sets of the surface points."""
        blacklisted_indices = []
        for i in range(len(self.surfaces)):
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

    def remove_noncontiguous_surface(self, eps=1, min_samples=10):
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
        self.surfaces = [
            self.surfaces[i]
            for i in range(len(self.surfaces))
            if i not in empty_surface_indices
        ]
