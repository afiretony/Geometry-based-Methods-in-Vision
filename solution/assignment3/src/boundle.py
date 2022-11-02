# %%
import numpy as np
import scipy
from scipy import optimize
from trianglate import triangulate
import matplotlib.pyplot as plt
import matplotlib


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


# def rodrigues(r):
#     """
#     Rodrigues formula.
#         Input:  r, a 3x1 vector
#         Output: R, a rotation matrix
#     """

#     theta = np.sqrt(np.sum(r**2))
#     n = r / theta if theta != 0 else r
#     n_cross = np.array(
#         [[0, -n[2, 0], n[1, 0]], [n[2, 0], 0, -n[0, 0]], [-n[1, 0], n[0, 0], 0]]
#     )
#     I = np.eye(3)
#     n_cross_square = np.dot(n, np.transpose(n)) - np.sum(n**2) * I
#     R = I + np.sin(theta) * n_cross + (1 - np.cos(theta)) * n_cross_square

#     return R


# def invRodrigues(R):
#     """
#     Inverse Rodrigues formula.
#         Input:  R, a rotation matrix
#         Output: r, a 3x1 vector
#     """
#     A = (R - R.T) / 2
#     p = np.array([[A[2, 1]], [A[0, 2]], [A[1, 0]]])
#     s = np.linalg.norm(p, 2)
#     c = (R[0, 0] + R[1, 1] + R[2, 2] - 1) / 2

#     if s == 0.0 and c == 1.0:
#         r = np.zeros((3, 1))
#         return r

#     elif s == 0.0 and c == -1.0:
#         tmp = R + np.diag(np.array([1, 1, 1]))
#         v = None
#         for i in range(3):
#             if np.sum(tmp[:, i]) != 0:
#                 v = tmp[:, i]
#                 break
#         u = v / np.sqrt(np.sum(v**2))
#         r = np.reshape(u * np.pi, (3, 1))
#         if np.sqrt(np.sum(r**2)) == np.pi and (
#             (r[0, 0] == 0.0 and r[1, 0] == 0.0 and r[2, 0] < 0)
#             or (r[0, 0] == 0.0 and r[1, 0] < 0)
#             or (r[0, 0] < 0)
#         ):
#             r = -r
#         return r
#     else:
#         u = p / s
#         theta = np.arctan2(s, c)
#         r = u * theta
#         return r


# def rodriguesResidual(K1, M1, p1, K2, p2, x):
#     """
#     Rodrigues residual.
#         Input:  K1, the intrinsics of camera 1
#                 M1, the extrinsics of camera 1
#                 p1, the 2D coordinates of points in image 1
#                 K2, the intrinsics of camera 2
#                 p2, the 2D coordinates of points in image 2
#                 x, the flattened concatenationg of P, r2, and t2.
#         Output: residuals, 4N x 1 vector, the difference between original and estimated projections
#     """

#     P, r2, t2 = x[:-6], x[-6:-3], x[-3:]

#     N = P.shape[0] // 3
#     P = np.reshape(P, (N, 3))
#     r2 = np.reshape(r2, (3, 1))
#     t2 = np.reshape(t2, (3, 1))
#     R2 = rodrigues(r2)
#     M2 = np.hstack((R2, t2))

#     P = np.vstack((np.transpose(P), np.ones((1, N))))
#     p1_hat = np.dot(np.dot(K1, M1), P)
#     p1_hat = np.transpose(p1_hat[:2, :] / p1_hat[2, :])
#     p2_hat = np.dot(np.dot(K2, M2), P)
#     p2_hat = np.transpose(p2_hat[:2, :] / p2_hat[2, :])

#     residuals = np.concatenate(
#         [(p1 - p1_hat).reshape([-1]), (p2 - p2_hat).reshape([-1])]
#     )

#     return residuals


# def bundleAdjustment(K1, M1, p1, K2, M2_init, p2, P_init):
#     """
#     Bundle adjustment.
#         Input:  K1, the intrinsics of camera 1
#                 M1, the extrinsics of camera 1
#                 p1, the 2D coordinates of points in image 1
#                 K2,  the intrinsics of camera 2
#                 M2_init, the initial extrinsics of camera 2
#                 p2, the 2D coordinates of points in image 2
#                 P_init, the initial 3D coordinates of points
#         Output: M2, the optimized extrinsics of camera 2
#                 P2, the optimized 3D coordinates of points
#     """
#     R2_init, t2_init = M2_init[:, :3], M2_init[:, 3]
#     r2_init = invRodrigues(R2_init).reshape([-1])
#     x = np.concatenate([P_init.reshape([-1]), r2_init, t2_init])

#     func = lambda x: (rodriguesResidual(K1, M1, p1, K2, p2, x) ** 2).sum()
#     x_update = scipy.optimize.minimize(func, x).x

#     P, r2, t2 = x_update[:-6], x_update[-6:-3], x_update[-3:]
#     N = P.shape[0] // 3
#     P = P.reshape(N, 3)

#     r2 = np.reshape(r2, (3, 1))
#     t2 = np.reshape(t2, (3, 1))
#     R2 = rodrigues(r2)
#     M2 = np.hstack((R2, t2))

#     return P, M2


# %%
def skew(x):
    assert len(x) == 3
    res = np.array([[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]])
    return res


"""
Q5.2: Rodrigues formula.
    Input:  r, a 3x1 vector
    Output: R, a rotation matrix
"""


def rodrigues(r):
    # Replace pass by your implementation

    theta = np.linalg.norm(r, 2)
    u = r / theta
    u = u.reshape(3, 1)
    R = (
        np.eye(3, 3) * np.cos(theta)
        + (1 - np.cos(theta)) * (u.dot(u.T))
        + skew(u) * (np.sin(theta))
    )
    return R


"""
Q5.2: Inverse Rodrigues formula.
    Input:  R, a rotation matrix
    Output: r, a 3x1 vector
"""


def invRodrigues(R):
    # Replace pass by your implementation

    A = (R - R.T) / 2
    rho = np.asarray([A[2, 1], A[0, 2], A[1, 0]]).T
    s = np.linalg.norm(rho, 2)
    c = (R[0, 0] + R[1, 1] + R[2, 2] - 1) / 2
    theta = np.arctan2(s, c)
    u = rho / s

    if s == 0 and c == 1:
        return np.asarray([0, 0, 0])

    elif s == 0 and c == -1:
        v = (R + np.eye(3)).reshape(9, 1)
        u = v / np.linalg.norm(v, 2)
        r = u * np.pi
        if np.linalg.norm(r, 2) == np.pi and (
            (r[0] == 0 and r[1] == 0 and r[2] < 0)
            or (r[0] == 0 and r[1] < 0)
            or (r[0] < 0)
        ):
            return -r
        else:
            return r

    elif np.sin(theta) != 0:
        return u * theta

    else:
        print("No condition satisfied")
        return None


"""
Q5.3: Rodrigues residual.
    Input:  K1, the intrinsics of camera 1
            M1, the extrinsics of camera 1
            p1, the 2D coordinates of points in image 1
            K2, the intrinsics of camera 2
            p2, the 2D coordinates of points in image 2
            x, the flattened concatenationg of P, r2, and t2.
    Output: residuals, 4N x 1 vector, the difference between original and estimated projections
"""


def rodriguesResidual(p1, p2, x):

    n = p1.shape[0]
    P = np.hstack((x[0:n, None], x[n : 2 * n, None], x[2 * n : 3 * n, None]))

    k1 = x[-22:-17]
    k2 = x[-17:-12]
    r1 = x[-12:-9]
    t1 = x[-9:-6, np.newaxis]
    r2 = x[-6:-3]
    t2 = x[-3:, np.newaxis]

    R1 = rodrigues(r1)
    R2 = rodrigues(r2)

    M1 = np.hstack([R1, t1])
    M2 = np.hstack([R2, t2])

    K1 = np.zeros((3, 3))
    K1[0, 0] = k1[0]
    K1[0, 1] = k1[1]
    K1[0, 2] = k1[2]
    K1[1, 1] = k1[3]
    K1[1, 2] = k1[4]
    K1[2, 2] = 1

    K2 = np.zeros((3, 3))
    K2[0, 0] = k2[0]
    K2[0, 1] = k2[1]
    K2[0, 2] = k2[2]
    K2[1, 1] = k2[3]
    K2[1, 2] = k2[4]
    K2[2, 2] = 1

    C1 = K1.dot(M1)
    C2 = K2.dot(M2)

    P_homo = np.vstack([P.T, np.ones(n)])

    p1_hat_homo = np.matmul(C1, P_homo)
    p2_hat_homo = np.matmul(C2, P_homo)

    p1_hat = p1_hat_homo.T
    p2_hat = p2_hat_homo.T

    # NORMALIZING
    for i in range(p1_hat.shape[0]):
        p1_hat[i, :] = p1_hat[i, :] / p1_hat[i, -1]
        p2_hat[i, :] = p2_hat[i, :] / p2_hat[i, -1]

    # NON - HOMOGENIZING
    p1_hat = p1_hat[:, :-1]
    p2_hat = p2_hat[:, :-1]

    residuals = np.concatenate(
        [(p1 - p1_hat).reshape([-1]), (p2 - p2_hat).reshape([-1])]
    )
    residuals = np.expand_dims(residuals, 1)
    # print('residuals shape: ', residuals.shape, ', n:', n)
    return residuals


"""
Q5.3 Bundle adjustment.
    Input:  K1, the intrinsics of camera 1
            M1, the extrinsics of camera 1
            p1, the 2D coordinates of points in image 1
            K2,  the intrinsics of camera 2
            M2_init, the initial extrinsics of camera 1
            p2, the 2D coordinates of points in image 2
            P_init, the initial 3D coordinates of points
    Output: M2, the optimized extrinsics of camera 1
            P2, the optimized 3D coordinates of points
"""


def bundleAdjustment(K1_init, M1_init, p1, K2_init, M2_init, p2, P_init):
    # Replace pass by your implementation

    R1_init = M1_init[:, 0:3]
    r1_init = invRodrigues(R1_init)
    t1_init = M1_init[:, 3].reshape([-1])
    k1_init = np.array(
        [K1_init[0, 0], K1_init[0, 1], K1_init[0, 2], K1_init[1, 1], K1_init[1, 2]]
    )

    R2_init = M2_init[:, 0:3]
    r2_init = invRodrigues(R2_init)
    t2_init = M2_init[:, 3].reshape([-1])
    k2_init = np.array(
        [K2_init[0, 0], K2_init[0, 1], K2_init[0, 2], K2_init[1, 1], K2_init[1, 2]]
    )

    x_init = np.hstack(
        [
            P_init[:, 0].reshape([-1]),
            P_init[:, 1].reshape([-1]),
            P_init[:, 2].reshape([-1]),
            k1_init,
            k2_init,
            r1_init,
            t1_init,
            r2_init,
            t2_init,
        ]
    )
    print(x_init.shape)
    print(P_init.shape)

    func = lambda x: (rodriguesResidual(p1, p2, x) ** 2).sum()
    x_updated = scipy.optimize.minimize(
        func, x_init, options={"disp": True, "maxiter": 50}
    ).x

    # r1 = x_updated[-12:-9]
    # t1 = x_updated[-9:-6, np.newaxis]
    # r2 = x_updated[-6:-3]
    # t2 = x_updated[-3:, np.newaxis]

    # R1 = rodrigues(r1)
    # R2 = rodrigues(r2)

    # M1 = np.hstack([R1, t1])
    # M2 = np.hstack([R2, t2])

    # C1 = K1.dot(M1)
    # C2 = K2.dot(M2)

    n = p1.shape[0]

    P_new = np.hstack(
        (
            x_updated[0:n, np.newaxis],
            x_updated[n : 2 * n, np.newaxis],
            x_updated[2 * n : 3 * n, np.newaxis],
        )
    )

    return P_new


# %%

P1 = np.load("../data/q4/P1_noisy.npy")
P2 = np.load("../data/q4/P2_noisy.npy")

pts1 = np.load("../data/q4/pts1.npy")
pts2 = np.load("../data/q4/pts2.npy")

K1, R1, C1 = decompose(P1)
K2, R2, C2 = decompose(P2)
M1 = np.hstack((R1, (-R1 @ C1.T).reshape(3, 1)))
M2 = np.hstack((R2, (-R2 @ C2.T).reshape(3, 1)))

# im1 = plt.imread("../data/q3/img1.jpg")

P_init, err = triangulate(P1, pts1, P2, pts2)

# %%
P = bundleAdjustment(K1, M1, pts1, K2, M2, pts2, P_init)

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.scatter(P[:, 0], P[:, 1], P[:, 2])
# ax.set_xlim(-0.6, 0.6)
# ax.set_ylim(-0.6, 0.6)
# ax.set_zlim(-0.6, 0.6)
plt.show()
# print(P)

# color = im1[pts1[:, 1], pts1[:, 0], :] / 255.0

# %%
