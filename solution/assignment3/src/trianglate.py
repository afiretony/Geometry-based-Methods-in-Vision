# %%
from turtle import color
import numpy as np
import matplotlib.pyplot as plt
import matplotlib


def triangulate(C1, pts1, C2, pts2):
    """
    Triangulate a set of 2D coordinates in the image to a set of 3D points.
        Input:  C1, the 3x4 camera matrix
                pts1, the Nx2 matrix with the 2D image coordinates per row
                C2, the 3x4 camera matrix
                pts2, the Nx2 matrix with the 2D image coordinates per row
        Output: P, the Nx3 matrix with the corresponding 3D points per row
                err, the reprojection error.
    """
    x1, y1 = pts1[:, 0], pts1[:, 1]
    x2, y2 = pts2[:, 0], pts2[:, 1]
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
    N = pts1.shape[0]
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
        err += np.sum((proj1 - pts1[i]) ** 2 + (proj2 - pts2[i]) ** 2)

    return w, err


def plot_3D(P, c):
    matplotlib.use("TkAgg")
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(P[:, 0], P[:, 1], P[:, 2], color=c)
    ax.set_xlim(-0.6, 0.6)
    ax.set_ylim(-0.6, 0.6)
    ax.set_zlim(-0.6, 0.6)
    plt.show()


if __name__ == "__main__":

    C1 = np.load("../data/q3/P1.npy")
    C2 = np.load("../data/q3/P2.npy")
    pts1 = np.load("../data/q3/pts1.npy")
    pts2 = np.load("../data/q3/pts2.npy")
    im1 = plt.imread("../data/q3/img1.jpg")

    w, err = triangulate(C1, pts1, C2, pts2)

    color = im1[pts1[:, 1], pts1[:, 0], :] / 255.0
    plot_3D(w, color)

# %%
