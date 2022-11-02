# How to use RQ Decomposition to recover your camera's K, R and C matrices.
import numpy as np
import scipy.linalg

np.set_printoptions(precision=2)


def decompose(P):
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
