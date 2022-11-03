# %%
import numpy as np
import scipy
from scipy import optimize
from trianglate import triangulate
import matplotlib.pyplot as plt
import matplotlib


def Residual(p1, p2, C1, x):
    """
    calculate residual
        Input:
            p1: correspondence points in image 1
            p2: correspondence points in image 2
            C1: camera matrix 1
            x:  3D points and camera matrix 2 needed to estimate
        Ooutput:
            residuals: nx4 image frame differences
    """
    n = p1.shape[0]
    P = x[: 3 * n].reshape(n, 3)

    P_homo = np.vstack([P.T, np.ones(n)])  # 4xn

    C2 = x[-12:].reshape(3, 4)

    p1_hat_homo = C1 @ P_homo  # 3xn
    p2_hat_homo = C2 @ P_homo

    p1_hat = p1_hat_homo.T  # nx3
    p2_hat = p2_hat_homo.T

    # normalize
    for i in range(p1_hat.shape[0]):
        p1_hat[i, :] = p1_hat[i, :] / p1_hat[i, -1]
        p2_hat[i, :] = p2_hat[i, :] / p2_hat[i, -1]

    p1_hat = p1_hat[:, :-1]
    p2_hat = p2_hat[:, :-1]

    residuals = np.concatenate(
        [(p1 - p1_hat).reshape([-1]), (p2 - p2_hat).reshape([-1])]
    )

    return residuals


def bundleAdjustment(p1, p2, C1, C2_init, P_init):
    """
    bundle adjustment (without decomposing camera matrix)
    Inputs:
        p1: correspondence points in image 1
        p2: correspondence points in image 2
        C1: camera matrix 1, assume to be fixed
        C2_init : camera matrix 2
        P_init: initial 3D points
    Output:
        P_new: new 3D points
    """

    x_init = np.hstack([P_init.flatten(), C2_init.flatten()])
    func = lambda x: Residual(p1, p2, C1, x)

    x_updated = scipy.optimize.least_squares(func, x_init, method="lm", verbose=2).x
    # x_updated = scipy.optimize.least_squares(func, x_init, verbose=2, max_nfev=100).x

    n = p1.shape[0]

    P_new = x_updated[: 3 * n].reshape(n, 3)

    return P_new


def plot_3D(P, c):
    matplotlib.use("TkAgg")
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(P[:, 0], P[:, 1], P[:, 2], color=c)
    ax.set_xlim(-0.6, 0.6)
    ax.set_ylim(-0.6, 0.6)
    ax.set_zlim(-0.6, 0.6)
    plt.show()


P1 = np.load("../data/q4/P1_noisy.npy")
P2 = np.load("../data/q4/P2_noisy.npy")

pts1 = np.load("../data/q4/pts1.npy")
pts2 = np.load("../data/q4/pts2.npy")

P_init, err = triangulate(P1, pts1, P2, pts2)

P_updated = bundleAdjustment(pts1, pts2, P1, P2, P_init)

# colorize
im1 = plt.imread("../data/q4/img1.jpg")
color = im1[pts1[:, 1], pts1[:, 0], :] / 255.0

# plot
plot_3D(P_updated, color)
