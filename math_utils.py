"""Defines math util functions."""

import itertools
import numpy as np
from scipy.spatial import ConvexHull  # pylint: disable=no-name-in-module


def point_in_triangle(point, triangle):
    """Check if a point is within a triangle."""
    v1, v2, v3 = triangle
    sign = lambda p1, p2, p3: (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[
        0]) * (p1[1] - p3[1])
    d1 = sign(point, v1, v2)
    d2 = sign(point, v2, v3)
    d3 = sign(point, v3, v1)
    has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
    has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)
    return not (has_neg and has_pos)


def project_2d(points_3d, normal, demean=True, use_basis=None):
    """Project 3d points onto 2D plane.

    Arguments:
        points_3d (np.ndarray): 3D Points array of shape (n, 3)
        normal (np.ndarray): The normal of shape (3,) of the plane to be projected onto
        demean (Bool): Boolean to demean coordinates or not
        use_basis (None or np.ndarray): If none, generate new arbitrary basis. Otherwise,
            use this parameter as the basis.
    """
    normal /= np.linalg.norm(normal)
    dists = np.dot(points_3d, normal)
    normal_proj = np.zeros_like(points_3d)
    for i, dist in enumerate(dists):
        normal_proj[i, :] = dist * normal

    proj = points_3d - normal_proj
    basis = None
    if use_basis is not None:
        basis = use_basis
    else:
        # Create arbitrary basis on plane and convert to 2D coords
        while True:
            indices = np.random.randint(len(points_3d), size=3)
            basis = [
                proj[indices[1]] - proj[indices[0]],
                proj[indices[2]] - proj[indices[0]]
            ]

            norms = [np.linalg.norm(basis[0]), np.linalg.norm(basis[1])]

            if not (np.isfinite(norms[0]) and np.isfinite(norms[1]) and
                    norms[0] > 0 and norms[1] > 0):
                continue

            basis[0] /= norms[0]
            basis[1] /= norms[1]

            if abs(np.dot(basis[0], basis[1])) < 1:
                basis[1] -= np.dot(basis[0], basis[1]) * basis[0]
                basis[1] /= np.linalg.norm(basis[1])
                basis[0] = basis[0].reshape(3, 1)
                basis[1] = basis[1].reshape(3, 1)
                basis = np.hstack(basis)
                break

    coords = np.linalg.lstsq(basis, proj.T, rcond=-1)[0].T  #(n x 2)
    mean = np.mean(coords, axis=0)
    if demean:
        coords -= mean
    return coords, normal_proj, basis, mean


def fit_quadrilateral(points, plane_normal):  # pylint: disable=too-many-locals, too-many-statements
    """Projects 3D surface points to 2D, finds the corner points of the 2D quadrilateral,
    and returns thereconstructed corner points in 3D space"""

    # Project points onto plane
    coords, normal_proj, basis, mean = project_2d(points,
                                                  plane_normal,
                                                  demean=True)

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
