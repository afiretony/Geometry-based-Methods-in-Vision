# %%
import os
import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
from utils import _singularize, refineF, load_pairs, load_intrainsics
import tqdm


def eightpoint(pts1, pts2, M):
    # normalize the coordinates
    x1, y1 = pts1[:, 0], pts1[:, 1]
    x2, y2 = pts2[:, 0], pts2[:, 1]
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
    F = _singularize(F)

    # rescale the data
    F = np.dot(np.transpose(T), np.dot(F, T))

    return F / np.linalg.norm(F)


def sevenpoint(pts1, pts2, M):
    # normalize the coordinates
    x1, y1 = pts1[:, 0], pts1[:, 1]
    x2, y2 = pts2[:, 0], pts2[:, 1]
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
    f1 = vh[-1, :]
    f2 = vh[-2, :]
    F1 = np.reshape(f1, (3, 3))
    F2 = np.reshape(f2, (3, 3))

    fun = lambda alpha: np.linalg.det(alpha * F1 + (1 - alpha) * F2)
    # get the coefficients of the polynomial
    a0 = fun(0)
    a1 = 2 * (fun(1) - fun(-1)) / 3 - (fun(2) - fun(-2)) / 12
    a2 = (fun(1) + fun(-1)) / 2 - a0
    a3 = (fun(1) - fun(-1)) / 2 - a1
    # solve for alpha
    alpha = np.roots([a3, a2, a1, a0])

    Farray = [a * F1 + (1 - a) * F2 for a in alpha]
    # refine F
    # Farray = [refineF(F, pts1 / M, pts2 / M) for F in Farray]

    # denormalize F
    Farray = [np.dot(np.transpose(T), np.dot(F, T)) for F in Farray]
    Farray = [F / F[-1, -1] for F in Farray]
    return Farray


def eight_point_F(pts1, pts2, M):
    """
    estimate fundamental matrix F using 8 point algorithm
    """

    F = np.zeros((3, 3))
    N = pts1.shape[0]
    A = np.zeros((N, 9))

    # linear equation formulation
    for i in range(N):
        x, xp, y, yp = pts1[i, 0], pts2[i, 0], pts1[i, 1], pts2[i, 1]
        A[i] = np.array(([x * xp, xp * y, xp, yp * x, y * yp, yp, x, y, 1]))

    u, s, vh = np.linalg.svd(A)
    F = vh[-1, :].reshape(3, 3)

    # constraint enforcement
    u, s, vh = np.linalg.svd(F)
    s[2] = 0
    D = np.diag(s)
    F_ = u @ D @ vh

    return F_ / F_[-1, -1]


def seven_point_F(pts1, pts2):
    """
    seven point algorithm to compute F
    """

    A = np.zeros((7, 9))
    F = np.zeros((3, 3))
    Fs = []

    for i in range(7):
        x, xp, y, yp = pts1[i, 0], pts2[i, 0], pts1[i, 1], pts2[i, 1]
        A[i] = np.array(([x * xp, xp * y, xp, yp * x, y * yp, yp, x, y, 1]))

    u, s, vh = np.linalg.svd(A)
    F1 = vh[-1, :].reshape(3, 3)
    F2 = vh[-2, :].reshape(3, 3)

    lamb = sp.symbols("lamb")
    M = sp.Matrix((F1 * lamb - F2 * (1 - lamb)))
    det_M = sp.det(M)

    M_coeff = det_M.as_coefficients_dict()
    roots = np.roots(
        [M_coeff[lamb**3], M_coeff[lamb**2], M_coeff[lamb], M_coeff[1]]
    )
    real_roots = roots.real[abs(roots.imag) < 1e-5]

    for lamb in real_roots:
        F = lamb * F1 + (1 - lamb) * F2
        F = F / F[-1, -1]
        Fs.append(F)

    return Fs


def compute_epipolar_line(pts1, pts2, F):
    N = pts1.shape[0]
    homo_pts1, homo_pts2 = np.ones((3, N)), np.ones((3, N))
    homo_pts1[:-1, :] = pts1.T
    homo_pts2[:-1, :] = pts2.T
    lp = F @ homo_pts1
    l = F.T @ homo_pts2
    return l, lp


def draw_epipolar_line(scene, algo, im1_path, im2_path, l, lp, pts1, pts2, N):

    im1 = plt.imread(im1_path)
    im2 = plt.imread(im2_path)

    plt.figure(0)
    plt.imshow(im1)
    xs = np.linspace(0, im1.shape[1], 100)
    for i in range(N):
        a, b, c = l[:, i]
        ys = (-c - a * xs) / b
        plt.plot(xs, ys, "-")
    plt.plot(pts1[:N, 0], pts1[:N, 1], ".", color="black")
    plt.xlim((0, im1.shape[1]))
    plt.ylim((im1.shape[0], 0))
    plt.savefig(os.path.join("..", "figs", algo, "{}_1.jpg".format(scene)))
    plt.show()

    plt.figure(1)
    plt.imshow(im2)
    xs = np.linspace(0, im2.shape[1], 100)

    for i in range(N):
        a, b, c = lp[:, i]
        ys = (-c - a * xs) / b
        plt.plot(xs, ys, "-")
    plt.plot(pts2[:N, 0], pts2[:N, 1], ".", color="black")
    plt.xlim((0, im2.shape[1]))
    plt.ylim((im2.shape[0], 0))
    plt.savefig(os.path.join("..", "figs", algo, "{}_2.jpg".format(scene)))
    plt.show()


def compute_E(Kp, K, F):
    return Kp.T @ F @ K


def ransacF_7(pts1, pts2, M, iter, thresh):
    """
    Input:  pts1, Nx2 Matrix
            pts2, Nx2 Matrix
            M, a scaler parameter
    Output: F, the fundamental matrix
            inliers, Nx1 bool vector set to true for inliers
    """
    N = pts1.shape[0]
    max_inliers = 0
    F = None
    inliers = None

    for i in tqdm.tqdm(range(iter)):
        inds = np.random.randint(0, N, (7,))
        F7s = sevenpoint(pts1[inds, :], pts2[inds, :], M)

        for F7 in F7s:
            # calculate the epipolar lines
            pts1_homo = np.vstack((np.transpose(pts1), np.ones((1, N))))
            l2s = np.dot(F7, pts1_homo)
            l2s = l2s / np.sqrt(np.sum(l2s[:2, :] ** 2, axis=0))

            # calculate the deviation of pts2 away from the epiploar lines
            pts2_homo = np.vstack((np.transpose(pts2), np.ones((1, N))))
            deviate = abs(np.sum(pts2_homo * l2s, axis=0))

            # determine the inliners
            curr_inliers = np.sum(deviate < thresh)
            if curr_inliers > max_inliers:
                max_inliers = curr_inliers
                F = F7
        # print(deviate.shape)

    ratio = max_inliers / N
    print(ratio)

    return F, ratio


def ransacF_8(pts1, pts2, M, iter, thresh):
    """
    Input:  pts1, Nx2 Matrix
            pts2, Nx2 Matrix
            M, a scaler parameter
    Output: F, the fundamental matrix
            inliers, Nx1 bool vector set to true for inliers
    """
    N = pts1.shape[0]
    max_inliers = 0
    F = None
    inliers = None

    for i in tqdm.tqdm(range(iter)):
        inds = np.random.randint(0, N, (8,))
        F8 = eightpoint(pts1[inds, :], pts2[inds, :], M)

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
    # print(deviate.shape)

    ratio = max_inliers / N
    print(ratio)

    return F, ratio


# %%
scene = "teddy"
algo = "eight_point"

path = os.path.join("..", "data", "q1a", scene, "{}_corresp_raw.npz".format(scene))
im1_path = os.path.join("..", "data", "q1a", scene, "image_1.jpg")
im2_path = os.path.join("..", "data", "q1a", scene, "image_2.jpg")
im1 = plt.imread(im1_path)

pts1, pts2 = load_pairs(path)
M = max(im1.shape)
# F = eightpoint(pts1, pts2, M)
F, ratio = ransacF_8(pts1, pts2, M, 10000, 1.5)
l, lp = compute_epipolar_line(pts1, pts2, F)
draw_epipolar_line(scene, algo, im1_path, im2_path, l, lp, pts1, pts2, 8)

path = os.path.join(
    "..", "data", "q1a", scene, "intrinsic_matrices_{}.npz".format(scene)
)
K, Kp = load_intrainsics(path)
E = compute_E(Kp, K, F)


# %%
scene = "teddy"
path = os.path.join("..", "data", "q1b", scene, "{}_7_point_corresp.npz".format(scene))
path = os.path.join("..", "data", "q1b", scene, "{}_corresp_raw.npz".format(scene))
im1_path = os.path.join("..", "data", "q1b", scene, "image_1.jpg")
im2_path = os.path.join("..", "data", "q1b", scene, "image_2.jpg")

pts1, pts2 = load_pairs(path)

im1 = plt.imread(im1_path)
M = max(im1.shape)

Farray = sevenpoint(pts1, pts2, M)

l, lp = compute_epipolar_line(pts1, pts2, Farray[0])

draw_epipolar_line(scene, im1_path, im2_path, l, lp, pts1, pts2, 7)

# %% Test RanSAC
scene = "teddy"
algo = "eight_point"

path = os.path.join("..", "data", "q1a", scene, "{}_corresp_raw.npz".format(scene))
im1_path = os.path.join("..", "data", "q1a", scene, "image_1.jpg")
im2_path = os.path.join("..", "data", "q1a", scene, "image_2.jpg")
im1 = plt.imread(im1_path)

pts1, pts2 = load_pairs(path)
M = max(im1.shape)

iters = np.linspace(1, 10000, 10)
ratios = []
for iter in iters:
    F, ratio = ransacF_7(pts1, pts2, M, int(iter), 1.2)
    ratios.append(ratio)

# %%
ratios
# %%
plt.plot(iters, ratios)
# %%
