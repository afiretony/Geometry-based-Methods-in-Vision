import numpy as np
import scipy
from scipy import optimize


def load_pairs(path):
    data = np.load(path)
    return data["pts1"], data["pts2"]


def load_intrainsics(path):
    data = np.load(path)
    return data["K1"], data["K2"]


def _singularize(F):
    U, S, V = np.linalg.svd(F)
    S[-1] = 0
    F = U.dot(np.diag(S).dot(V))
    return F


def _objective_F(f, pts1, pts2):
    F = _singularize(f.reshape([3, 3]))
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
        lambda x: _objective_F(x, pts1, pts2),
        F.reshape([-1]),
        maxiter=100000,
        maxfun=10000,
        disp=False,
    )
    return _singularize(f.reshape([3, 3]))


def decompose(P):
    """
    Use RQ Decomposition to recover your camera's K, R and C matrices.
        input: camera projection matrix P
        output: K, R, C
    """
    M = P[:, 0:3]
    K, R = scipy.linalg.rq(M)
    T = np.diag(np.sign(np.diag(K)))

    # enforece camera matrix with a positive diagonal
    if scipy.linalg.det(T) < 0:
        T[1, 1] *= -1

    K = np.dot(K, T)
    R = np.dot(T, R)
    C = np.dot(scipy.linalg.inv(-M), P[:, 3])

    return K, R, C


def rodrigues(r):
    """
    Rodrigues formula.
        Input:  r, a 3x1 vector
        Output: R, a rotation matrix
    """

    theta = np.sqrt(np.sum(r**2))
    n = r / theta if theta != 0 else r
    n_cross = np.array(
        [[0, -n[2, 0], n[1, 0]], [n[2, 0], 0, -n[0, 0]], [-n[1, 0], n[0, 0], 0]]
    )
    I = np.eye(3)
    n_cross_square = np.dot(n, np.transpose(n)) - np.sum(n**2) * I
    R = I + np.sin(theta) * n_cross + (1 - np.cos(theta)) * n_cross_square

    return R


def invRodrigues(R):
    """
    Inverse Rodrigues formula.
        Input:  R, a rotation matrix
        Output: r, a 3x1 vector
    """
    A = (R - R.T) / 2
    p = np.array([[A[2, 1]], [A[0, 2]], [A[1, 0]]])
    s = np.linalg.norm(p, 2)
    c = (R[0, 0] + R[1, 1] + R[2, 2] - 1) / 2

    if s == 0.0 and c == 1.0:
        r = np.zeros((3, 1))
        return r

    elif s == 0.0 and c == -1.0:
        tmp = R + np.diag(np.array([1, 1, 1]))
        v = None
        for i in range(3):
            if np.sum(tmp[:, i]) != 0:
                v = tmp[:, i]
                break
        u = v / np.sqrt(np.sum(v**2))
        r = np.reshape(u * np.pi, (3, 1))
        if np.sqrt(np.sum(r**2)) == np.pi and (
            (r[0, 0] == 0.0 and r[1, 0] == 0.0 and r[2, 0] < 0)
            or (r[0, 0] == 0.0 and r[1, 0] < 0)
            or (r[0, 0] < 0)
        ):
            r = -r
        return r
    else:
        u = p / s
        theta = np.arctan2(s, c)
        r = u * theta
        return r
