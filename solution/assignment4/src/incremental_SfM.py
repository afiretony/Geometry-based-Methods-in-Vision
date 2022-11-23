"""
Implemental of Incremental Structure from Motion
Chenhao Yang 2022.11.22
"""

import numpy as np
from utils import *
from tqdm import tqdm
from visualize import *
from baseline import Baseline
import cv2 as cv
from triangulation import triangulate


class IncrementalSfM:
    def __init__(
        self, images, cameras, correspondences, init_pose=(np.eye(3), np.zeros((3, 1)))
    ) -> None:
        self.images = images  # list of images
        self.cameras = cameras  # list of camera intrinsics
        self.correspondences = correspondences  # map of correspondences
        self.projection_maps = [{} for i in range(4)]  # list of projection maps
        self.poses = [init_pose]  # list of poses
        self.PointCloud = None  # 3D point cloud

    def initialization(self):
        """
        Initialize the first two images
        """
        # get the information of the first two images
        pts1, pts2 = self.correspondences["1_2"]
        K1, K2 = self.cameras[0], self.cameras[1]
        R1, t1 = self.poses[0]
        im1, im2 = self.images[0], self.images[1]

        # compute the baseline reconstruction for the first two images
        self.baseline = Baseline(im1, im2, K1, K2, pts1, pts2, R1, t1)
        R2, t2 = self.baseline.get_pose()
        self.poses.append((R2, t2))

        # update the pts1 and pts2 (because baseline filtered some outliers)
        pts1, pts2 = self.baseline.pts1, self.baseline.pts2

        # create projection maps
        self.projection_maps[0] = self.create_projection_map(pts1, self.baseline.X)
        self.projection_maps[1] = self.create_projection_map(pts2, self.baseline.X)

        # init the point cloud
        self.PointCloud = self.baseline.X

    def create_projection_map(self, pts, P):
        """
        Create a projection map for the new image
        """
        projection_map = {}

        for i, pt in enumerate(pts):
            projection_map["{},{}".format(pt[0], pt[1])] = P[i]

        return projection_map

    def add_image(self, idx):
        """
        Add a new image to the reconstruction
        idx is 1-indexed, for total four images, idx should be 3, 4
        """
        # find the correspondences map in 3D if exists
        Points = []
        pts = []

        print("Adding image {}".format(idx))

        pts_old, pts_new = self.correspondences["{}_{}".format(idx - 1, idx)]
        for i, pt_old in enumerate(pts_old):
            old_key = "{},{}".format(pt_old[0], pt_old[1])
            if old_key in self.projection_maps[idx - 2]:
                Points.append(self.projection_maps[idx - 2][old_key])
                pts.append(pts_new[i])

        Points = np.vstack(Points).astype(np.float32)
        pts = np.vstack(pts).astype(np.float32)
        print("Number of correspondences: {}".format(len(Points)))

        # compute the pose of the new image
        newpose = self.compute_pose_PnP(
            Points, pts, self.cameras[idx - 1], self.poses[-1]
        )
        print("new pose: \n", newpose)
        self.poses.append(newpose)

        # triangulate the new points
        R1, t1 = self.poses[idx - 2]
        R2, t2 = self.poses[idx - 1]
        C1 = self.cameras[idx - 2] @ np.hstack((R1, t1))
        C2 = self.cameras[idx - 1] @ np.hstack((R2, t2))
        X, err = triangulate(C1, pts_old, C2, pts_new)

        # update the projection map
        for i, pt in enumerate(pts_new):
            self.projection_maps[idx - 1]["{},{}".format(pt[0], pt[1])] = X[i]

        # update the point cloud
        for i, pt in enumerate(pts_old):
            old_key = "{},{}".format(pt_old[0], pt_old[1])
            if old_key not in self.projection_maps[idx - 2]:
                self.PointCloud = np.vstack((self.PointCloud, X[i]))
        # self.PointCloud = np.vstack((self.PointCloud, X))
        # self.PointCloud = X

    def compute_pose_PnP(self, Points, pts, K, prev_pose):
        """
        Compute the pose of the new image
        """
        _, R, t, _ = cv.solvePnPRansac(
            Points,
            pts,
            K,
            None,
            confidence=0.6,
            reprojectionError=8.0,
            flags=cv.SOLVEPNP_DLS,
        )
        R = cv.Rodrigues(R)[0]
        # R_prev, t_prev = prev_pose
        # R = R_rel @ R_prev
        # t = R_rel @ t_prev + t_rel
        return (R, t)


if __name__ == "__main__":
    images = [
        load_image("../data/data_cow/images/{:05d}.jpg".format(i)) for i in range(1, 5)
    ]
    cameras = [np.load("../data/data_cow/cameras/cam1.npz")["K"] for i in range(4)]
    correspondences = {}
    init_pose = (
        np.load("../data/data_cow/cameras/cam1.npz")["R"],
        np.load("../data/data_cow/cameras/cam1.npz")["T"].reshape(3, 1),
    )
    keys = ["1_2", "1_3", "1_4", "2_3", "2_4", "3_4"]
    for key in keys:
        correspondences[key] = (
            np.load(
                "../data/data_cow/correspondences/pairs_{}/cam1_corresp.npy".format(key)
            ),
            np.load(
                "../data/data_cow/correspondences/pairs_{}/cam2_corresp.npy".format(key)
            ),
        )

    incremental_sfm = IncrementalSfM(images, cameras, correspondences)
    incremental_sfm.initialization()
    plot_3D(incremental_sfm.PointCloud, "r")
    incremental_sfm.add_image(3)
    plot_3D(incremental_sfm.PointCloud, "r")
    incremental_sfm.add_image(4)
    plot_3D(incremental_sfm.PointCloud, "r")
    # print(incremental_sfm.poses)
