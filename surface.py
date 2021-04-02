"""Defines the Surface class object"""
import numpy as np
from scipy.cluster.vq import kmeans


class Surface:
    """Defines a surface class"""

    def __init__(self, uid=None):
        self.uid = uid
        self.midpoint = None
        self.points = None
        self.normal = None
        self.corners = None
        self.obb = None

    def set_normal(self, normal):
        """Set the normal of this surface."""
        self.normal = normal

    def compute_midpoint(self):
        self.midpoint = np.mean(self.points, axis=0)

    def set_points(self, points):
        """Sets the points that make up this surface."""
        self.points = points
        self.compute_midpoint()

    def set_obb(self, obb):
        """Sets the 3D bounding box of this surface."""
        self.obb = obb

    def set_corners(self, corners):
        """Sets the 4 corner points of this surface."""
        self.corners = corners