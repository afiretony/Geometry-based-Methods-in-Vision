"""
Baseline recontruction for SfM using two views.
"""

import numpy as np
from utils import *
from tqdm import tqdm
from visualize import *
import cv2 as cv


class Baseline:
    def __init__(self, image1_path, image2_path, intrinsics_path, correspondences_path):
        # load the images and parameters
        self.im1 = load_image(image1_path)
        self.im2 = load_image(image2_path)
        self.K1, self.K2 = load_intrinsics(intrinsics_path)
        self.pts1, self.pts2 = load_correspondences(correspondences_path)
        self.X = None  # 3D points

        # set image 1 at the origin
        self.R1 = np.eye(3)
        self.t1 = np.zeros((3, 1))

        # set the parameters
        self.M = max(self.im1.shape)
        self.thresh = 0.005 * self.M  # threshold for inliners
        self.iter = 1000  # number of iterations

    def get_essential_matrix(self, F):
        """
        computes the essential matrix from the fundamental matrix
        """
        E = self.K2.T @ F @ self.K1
        return E

    def get_pose_from_essential_matrix(self, E):
        """
        Calculates 4 possible rotation and translation
        component from essential matrix
        input:
            E, essential matrix
        output:
            Rs, 4x3x3 rotation matrix
            ts, 4x3 translation matrix
        """
        Rs = np.zeros((4, 3, 3))
        ts = np.zeros((4, 3, 1))

        W = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        W_t = W.T
        u, w, vt = np.linalg.svd(E)

        R1 = u @ W @ vt
        R2 = u @ W_t @ vt
        t1 = u[:, -1].reshape((3, 1))
        t2 = -t1
        Rs[0], ts[0] = R1, t1
        Rs[1], ts[1] = R1, t2
        Rs[2], ts[2] = R2, t1
        Rs[3], ts[3] = R2, t2

        return Rs, ts

    def remove_outliers(self, inliners):
        """
        remove the outliers from the correspondences
        """
        return self.pts1[inliners], self.pts2[inliners]

    def get_pose(self):
        # get the fundamental matrix
        F, inliners = self.ransacF_8()
        print("F:", F / F[2, 2])
        F, mask = cv.findFundamentalMat(self.pts1, self.pts2, cv.FM_LMEDS)
        print("F:", F / F[2, 2])

        # remove the outliers
        self.pts1, self.pts2 = self.remove_outliers(inliners)

        # get the essential matrix
        E = self.get_essential_matrix(F)

        # get the pose
        Rs, ts = self.get_pose_from_essential_matrix(E)

        # get the correct pose
        pose = self.get_correct_pose(Rs, ts)

        return pose

    def get_correct_pose(self, Rs, ts):

        best_err = np.inf
        best_pose = None
        for i in range(4):
            R = Rs[i]
            t = ts[i]
            # get the 3D points
            X, err = self.triangulate(R, t)
            print(err)
            # check if the points are in front of the camera
            if np.all(X[:, 2] > 0):
                if err < best_err:
                    best_err = err
                    best_pose = (R, t)
                    self.X = X
        if best_pose is None:
            raise Exception("No valid pose found")
        else:
            return best_pose

    def ransacF_8(self):
        """
        Input:  pts1, Nx2 Matrix
                pts2, Nx2 Matrix
                M, a scaler parameter
        Output: F, the fundamental matrix
                inliers, Nx1 bool vector set to true for inliers
        """
        pts1, pts2 = self.pts1, self.pts2
        iter = self.iter
        thresh = self.thresh
        M = self.M

        N = pts1.shape[0]
        max_inliers = 0
        F = None
        inliners = np.ones_like(pts1[:, 0], dtype=bool)

        for i in tqdm(range(iter)):
            inds = np.random.randint(0, N, (8,))
            F8 = self.eightpoint()

            # calculate the epipolar lines
            pts1_homo = np.vstack((np.transpose(pts1), np.ones((1, N))))
            l2s = np.dot(F8, pts1_homo)
            l2s = l2s / np.sqrt(np.sum(l2s[:2, :] ** 2, axis=0))

            # calculate the deviation of pts2 away from the epiploar lines
            pts2_homo = np.vstack((pts2.T, np.ones((1, N))))
            deviate = abs(np.sum(pts2_homo * l2s, axis=0))

            # determine the inliners
            curr_inliers = np.sum(deviate < thresh)
            if curr_inliers > max_inliers:
                max_inliers = curr_inliers
                F = F8
                inliners = deviate < thresh

        ratio = max_inliers / N
        print("inliner ratio:", ratio)

        return F, inliners

    def eightpoint(self):

        M = self.M
        # normalize the coordinates
        x1, y1 = self.pts1[:, 0], self.pts1[:, 1]
        x2, y2 = self.pts2[:, 0], self.pts2[:, 1]
        x1, y1, x2, y2 = x1 / M, y1 / M, x2 / M, y2 / M
        # normalization matrix
        T = np.array([[1.0 / M, 0, 0], [0, 1.0 / M, 0], [0, 0, 1]])

        A = np.transpose(
            np.vstack(
                (x2 * x1, x2 * y1, x2, y2 * x1, y2 * y1, y2, x1, y1, np.ones(x1.shape))
            )
        )

        # get F by SVD decomposition
        u, s, vh = np.linalg.svd(A)
        f = vh[-1, :]
        F = np.reshape(f, (3, 3))

        # refine F
        # F = refineF(F, pts1 / M, pts2 / M)

        # constraint of rank 2 by setting the last singular value to 0
        F = singularize(F)

        # rescale the data
        F = np.dot(np.transpose(T), np.dot(F, T))

        return F / np.linalg.norm(F)

    def triangulate(self, R2, t2):
        """
        Triangulate a set of 2D coordinates in the image to a set of 3D points.
            Input:  R2, 3x3 rotation matrix of the second camera
                    t2, 3x1 translation vector of the second camera
            Output: w, the Nx3 matrix with the corresponding 3D points per row
                    err, the reprojection error.
        """
        x1, y1 = self.pts1[:, 0], self.pts1[:, 1]
        x2, y2 = self.pts2[:, 0], self.pts2[:, 1]
        C1 = np.hstack((self.R1, self.t1))
        C2 = np.hstack((R2, t2))

        A1 = np.vstack(
            (
                C1[0, 0] - C1[2, 0] * x1,
                C1[0, 1] - C1[2, 1] * x1,
                C1[0, 2] - C1[2, 2] * x1,
                C1[0, 3] - C1[2, 3] * x1,
            )
        ).transpose()
        A2 = np.vstack(
            (
                C1[1, 0] - C1[2, 0] * y1,
                C1[1, 1] - C1[2, 1] * y1,
                C1[1, 2] - C1[2, 2] * y1,
                C1[1, 3] - C1[2, 3] * y1,
            )
        ).transpose()
        A3 = np.vstack(
            (
                C2[0, 0] - C2[2, 0] * x2,
                C2[0, 1] - C2[2, 1] * x2,
                C2[0, 2] - C2[2, 2] * x2,
                C2[0, 3] - C2[2, 3] * x2,
            )
        ).transpose()
        A4 = np.vstack(
            (
                C2[1, 0] - C2[2, 0] * y2,
                C2[1, 1] - C2[2, 1] * y2,
                C2[1, 2] - C2[2, 2] * y2,
                C2[1, 3] - C2[2, 3] * y2,
            )
        ).transpose()

        # calculate the 3D coordinates for each point
        N = self.pts1.shape[0]
        w = np.zeros((N, 3))

        for ind in range(N):
            A = np.vstack((A1[ind, :], A2[ind, :], A3[ind, :], A4[ind, :]))
            u, s, vh = np.linalg.svd(A)
            p = vh[-1, :]
            w[ind, :] = p[:3] / p[-1]

        # project to 2D points
        W = np.hstack((w, np.ones((N, 1))))
        err = 0
        for i in range(N):
            proj1 = np.dot(C1, np.transpose(W[i, :]))
            proj2 = np.dot(C2, np.transpose(W[i, :]))
            proj1 = np.transpose(proj1[:2] / proj1[-1])
            proj2 = np.transpose(proj2[:2] / proj2[-1])
            # compute error
            err += np.sum((proj1 - self.pts1[i]) ** 2 + (proj2 - self.pts2[i]) ** 2)

        return w, err


if __name__ == "__main__":
    image1_path = "../data/monument/im1.jpg"
    image2_path = "../data/monument/im2.jpg"
    intrinsic_path = "../data/monument/intrinsics.npy"
    correspondences_path = "../data/monument/some_corresp_noisy.npz"
    baseline = Baseline(image1_path, image2_path, intrinsic_path, correspondences_path)
    pose = baseline.get_pose()
    print(pose)
    plot_3D(baseline.X, "r")
