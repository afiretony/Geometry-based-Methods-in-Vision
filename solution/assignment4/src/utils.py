import numpy as np
import scipy
from scipy import optimize
import matplotlib.pyplot as plt


def load_correspondences(path):
    data = np.load(path)
    return data["pts1"], data["pts2"]


def load_image(path):
    im = plt.imread(path)
    return im


def load_intrinsics(path):
    data = np.load(path, allow_pickle=True).item()
    return data["K1"], data["K2"]


def singularize(F):
    U, S, V = np.linalg.svd(F)
    S[-1] = 0
    F = U.dot(np.diag(S).dot(V))
    return F


def objective_F(f, pts1, pts2):
    F = singularize(f.reshape([3, 3]))
    num_points = pts1.shape[0]
    hpts1 = np.concatenate([pts1, np.ones([num_points, 1])], axis=1)
    hpts2 = np.concatenate([pts2, np.ones([num_points, 1])], axis=1)
    Fp1 = F.dot(hpts1.T)
    FTp2 = F.T.dot(hpts2.T)

    r = 0
    for fp1, fp2, hp2 in zip(Fp1.T, FTp2.T, hpts2):
        r += (hp2.dot(fp1)) ** 2 * (
            1 / (fp1[0] ** 2 + fp1[1] ** 2) + 1 / (fp2[0] ** 2 + fp2[1] ** 2)
        )
    return r


def refineF(F, pts1, pts2):
    f = scipy.optimize.fmin_powell(
        lambda x: objective_F(x, pts1, pts2),
        F.reshape([-1]),
        maxiter=100000,
        maxfun=10000,
        disp=False,
    )
    return singularize(f.reshape([3, 3]))
