# %%
import numpy as np
import matplotlib.pyplot as plt
from annotations import *
import math
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon


def read_q1_data(filename):
    correspondences = []
    with open(filename) as f:
        for line in f.readlines():
            correspondence = [float(i) for i in line[:-1].split(" ")]
            correspondences.append(correspondence)
    return correspondences


def computeP(correspondences):
    num_corr = len(correspondences)
    A = np.zeros((2 * num_corr, 12))
    for i in range(num_corr):
        u, v, x, y, z = correspondences[i]
        imagex = np.cross(np.identity(3), np.array([u, v, 1]))
        world = np.zeros((3, 12))
        world[0, 0:4] = world[1, 4:8] = world[2, 8:12] = x, y, z, 1
        A_ = imagex @ world
        A[2 * i] = A_[0]
        A[2 * i + 1] = A_[1]

    u, s, vh = np.linalg.svd(A)
    P = vh[-1, :]
    # P = P / np.linalg.norm(P)
    return P.reshape(3, 4)


def test(P, correspondences):
    num_corr = len(correspondences)
    for i in range(num_corr):
        u, v, x, y, z = correspondences[i]
        im = np.array([x, y, z, 1])
        c = P @ im.T
        print(c / c[-1])
        print(u, v)


def project(P, pts):
    homo_pts = np.ones((4, pts.shape[0]))
    homo_pts[0:3] = pts.T
    image_pts = P @ homo_pts
    image_pts /= image_pts[-1, :]
    return image_pts


def drawSurfacePts(imagefile, pts):
    fig = plt.figure(figsize=(10, 8))
    img = plt.imread(imagefile)
    plt.imshow(img)
    plt.plot(pts[0, :], pts[1, :], "r.")
    plt.savefig("q1.1.jpg")
    plt.show()


def drawBoundingBox(imagefile, pts):
    fig = plt.figure(figsize=(10, 8))
    img = plt.imread(imagefile)
    plt.imshow(img)
    num_line = pts.shape[1]
    for i in range(num_line):
        plt.plot(pts[0, 2 * i : 2 * i + 2], pts[1, 2 * i : 2 * i + 2], "b-")

    plt.savefig("q1.3.jpg")
    plt.show()


def drawAnnotation(imagefile, pts):
    fig = plt.figure(figsize=(10, 8))
    img = plt.imread(imagefile)
    plt.imshow(img)

    plt.plot(pts[:, 0], pts[:, 1], "o", markersize=10)

    plt.savefig("q1b_anno.jpg")
    plt.show()


# q1a
correspondences = read_q1_data("../data/q1/bunny.txt")
P = computeP(correspondences)

image_pts = project(P, np.load("../data/q1/bunny_pts.npy"))
# drawSurfacePts("../data/q1/bunny.jpeg", image_pts)

pts = np.load("../data/q1/bunny_bd.npy").reshape(24, 3)
image_pts = project(P, pts)
# drawBoundingBox("../data/q1/bunny.jpeg", image_pts)


# q1b
correspondences = [
    [91, 152, -1, 1, 1],
    [318, 247, 1, 1, 1],
    [562, 164, 1, -1, 1],
    [99, 457, -1, 1, -1],
    [314, 569, 1, 1, -1],
    [550, 466, 1, -1, -1],
]
# edges = np.array(
#     [
#         [-1, 1, 1, 1, 1, 1],
#         [1, 1, 1, 1, -1, 1],
#         [1, -1, 1, -1, -1, 1],
#         [-1, 1, 1, -1, -1, 1],
#         [-1, 1, 1, -1, 1, -1],
#         [1, 1, 1, 1, 1, -1],
#         [1, -1, 1, 1, -1, -1],
#         [-1, -1, 1, -1, -1, -1],
#         [1, 1, -1, -1, 1, -1],
#         [1, 1, -1, 1, -1, -1],
#         [1, -1, -1, -1, -1, -1],
#         [-1, -1, -1, -1, 1, -1],
#     ]
# ).reshape(-1, 3)
# P = computeP(correspondences)
# image_pts = project(P, edges)
# drawBoundingBox("../data/q1/cube.jpeg", image_pts)
# drawAnnotation("../data/q1/cube.jpeg", np.array(correspondences))

# annotations = np.load("../data/q2/q2a.npy")/


def line_equation(pt1, pt2):
    x1, y1 = pt1
    x2, y2 = pt2
    a = y2 - y1
    b = x1 - x2
    c = y1 * (x2 - x1) - x1 * (y2 - y1)
    return a, b, c


def intersection_point(l1, l2):
    a1, b1, c1 = l1
    a2, b2, c2 = l2
    x0 = (b1 * c2 - b2 * c1) / (a1 * b2 - a2 * b1)
    y0 = (c1 * a2 - c2 * a1) / (a1 * b2 - a2 * b1)
    return x0, y0


def getVanishingPts(annotations):
    VanishingPts = []
    for pairs in annotations:
        l1 = line_equation(pairs[0, 0:2], pairs[0, 2:4])
        l2 = line_equation(pairs[1, 0:2], pairs[1, 2:4])
        VanishingPts.append(intersection_point(l1, l2))
    return VanishingPts


def vis_vanishingpts(imagefile, VanishingPts):
    fig = plt.figure(figsize=(10, 8))
    img = plt.imread(imagefile)
    plt.imshow(img)
    for pts in VanishingPts:
        plt.plot(pts[0], pts[1], "r.")
    plt.show()


def computeK(VanishingPts):
    num_pairs = 3
    A = np.zeros((num_pairs, 4))

    pt1 = [VanishingPts[0][0], VanishingPts[0][1], 1]
    pt2 = [VanishingPts[1][0], VanishingPts[1][1], 1]
    A[0] = [
        pt1[0] * pt2[0] + pt1[1] * pt2[1],
        pt1[0] * pt2[2] + pt1[2] * pt2[0],
        pt1[1] * pt2[2] + pt2[1] * pt1[2],
        pt1[2] * pt2[2],
    ]

    pt1 = [VanishingPts[0][0], VanishingPts[0][1], 1]
    pt2 = [VanishingPts[2][0], VanishingPts[2][1], 1]
    A[1] = [
        pt1[0] * pt2[0] + pt1[1] * pt2[1],
        pt1[0] * pt2[2] + pt1[2] * pt2[0],
        pt1[1] * pt2[2] + pt2[1] * pt1[2],
        pt1[2] * pt2[2],
    ]

    pt1 = [VanishingPts[2][0], VanishingPts[2][1], 1]
    pt2 = [VanishingPts[1][0], VanishingPts[1][1], 1]
    A[2] = [
        pt1[0] * pt2[0] + pt1[1] * pt2[1],
        pt1[0] * pt2[2] + pt1[2] * pt2[0],
        pt1[1] * pt2[2] + pt2[1] * pt1[2],
        pt1[2] * pt2[2],
    ]

    u, s, vh = np.linalg.svd(A)
    w = vh[-1, :]
    # print(w)
    w = np.array([[w[0], 0, w[1]], [0, w[0], w[2]], [w[1], w[2], w[3]]])
    K = np.linalg.cholesky(w)
    K = np.linalg.inv(K)
    K = K / K[-1, -1]
    return K.T


# VanishingPts = getVanishingPts(annotations)
# print(VanishingPts)
# vis_vanishingpts("../data/q2a.png", VanishingPts)
# K = computeK(VanishingPts)
# vis_annnotations_q2b()
# VanishingPts = getVanishingPts(annotations)
# print(VanishingPts)
# vis_vanishingpts("../data/q2a.png", VanishingPts)

# %%
# 2.b camera calibration from metric plane
def computeH(correspondences):
    map = np.array([[0, 1, 1, 0], [0, 0, 1, 1]])
    Hs = []
    for correspondence in correspondences:
        A = np.zeros((8, 9))
        for i in range(4):
            x, y = map[:, i]
            x_prime, y_prime = correspondence[i]
            A[2 * i] = [0, 0, 0, -x, -y, -1, y_prime * x, y_prime * y, y_prime]
            A[2 * i + 1] = [x, y, 1, 0, 0, 0, -x_prime * x, -x_prime * y, -x_prime]

            u, s, vh = np.linalg.svd(A)
            H = vh[-1, :].reshape((3, 3))
            H = H / H[-1, -1]
        Hs.append(H)

    return Hs


def computeK(Hs):
    A = np.zeros((6, 6))
    k = 0
    for H in Hs:
        h1, h2, h3 = H[:, 0], H[:, 1], H[:, 2]
        h1x, h1y, h1z = h1[0], h1[1], h1[2]
        h2x, h2y, h2z = h2[0], h2[1], h2[2]
        A[k] = [
            h1x * h2x,
            h1y * h2x + h1x * h2y,
            h1z * h2x + h1x * h2z,
            h1y * h2y,
            h1z * h2y + h1y * h2z,
            h2z * h1z,
        ]
        A[k + 1] = [
            h1x * h1x - h2x * h2x,
            2 * h1x * h1y - 2 * h2x * h2y,
            2 * h1x * h1z - 2 * h2x * h2z,
            h1y * h1y - h2y * h2y,
            2 * h1y * h1z - 2 * h2y * h2z,
            h1z * h1z - h2z * h2z,
        ]
        k += 2

    u, s, vh = np.linalg.svd(A)
    w = vh[-1, :]
    w = np.array([[w[0], w[1], w[2]], [w[1], w[3], w[4]], [w[2], w[4], w[5]]])
    K = np.linalg.cholesky(w)
    K = np.linalg.inv(K)
    K = K / K[-1, -1]
    return K.T


annotations = np.load("../data/q2/q2b.npy")
Hs = computeH(annotations)
K = computeK(Hs)


def getVanishingPts(plane_pts):
    """
    get two vanishing points for a rectangle plane
    """
    VanishingPts = []

    l1 = line_equation(plane_pts[0], plane_pts[1])
    l2 = line_equation(plane_pts[2], plane_pts[3])
    VanishingPts.append(intersection_point(l1, l2))

    l1 = line_equation(plane_pts[0], plane_pts[3])
    l2 = line_equation(plane_pts[2], plane_pts[1])
    VanishingPts.append(intersection_point(l1, l2))

    return VanishingPts


def compute_d(K, VanishingPts):
    """
    comutes d
    """
    d = []
    K_inv = np.linalg.inv(K)

    for pt in VanishingPts:
        d.append(K_inv @ np.array([pt[0], pt[1], 1]))
    return d


def compute_n(d):
    """
    compute n by cross product two d
    """
    n = np.cross(d[0], d[1])
    return -n


vp0 = getVanishingPts(annotations[0])
vp1 = getVanishingPts(annotations[1])
vp2 = getVanishingPts(annotations[2])

ds = compute_d(K, vp0)
n0 = compute_n(ds)

ds = compute_d(K, vp1)
n1 = compute_n(ds)

ds = compute_d(K, vp2)
n2 = compute_n(ds)


def angle_between(n1, n2):
    inner = np.inner(n1, n2)
    norms = np.linalg.norm(n1) * np.linalg.norm(n2)
    cos = inner / norms
    rad = np.arccos(np.clip(cos, -1.0, 1.0))
    deg = np.rad2deg(rad)

    return deg


print(angle_between(n0, n1))
print(angle_between(n0, n2))
print(angle_between(n1, n2))

# %%
# q3a


annotations = np.load("../data/q3/q3.npy")
K = -np.array(
    [
        [1.15417802e03, 0.00000000e00, 5.75066005e02],
        [-0.00000000e00, 1.15417802e03, 4.31939090e02],
        [-0.00000000e00, 0.00000000e00, 1.00000000e00],
    ]
)

K_inv = np.linalg.inv(K)


def get_ray(polygon):
    """
    polygon: nx2
    """
    x_min, x_max = np.min(polygon[:, 0]), np.max(polygon[:, 0])
    y_min, y_max = np.min(polygon[:, 1]), np.max(polygon[:, 1])
    x = np.arange(int(x_min), int(x_max), 5)
    y = np.arange(int(y_min), int(y_max), 5)
    xv, yv = np.meshgrid(x, y)
    points = np.hstack((xv.reshape(-1, 1), yv.reshape(-1, 1)))

    poly = Polygon(
        [
            (polygon[0][0], polygon[0][1]),
            (polygon[1][0], polygon[1][1]),
            (polygon[2][0], polygon[2][1]),
            (polygon[3][0], polygon[3][1]),
        ]
    )

    # create mask
    mask = np.ones((points.shape[0]), dtype=bool)

    # check if in polygon
    for i, point in enumerate(points):
        p = Point(point)
        if not poly.contains(p):
            mask[i] = False

    return points, mask


def get_color(img_path, points):
    im = plt.imread(img_path)
    color = im[points[:, 1], points[:, 0], :]
    return color


def xy2XYZ(points, n, a, K_inv):
    num = points.shape[0]
    xy_homo = np.vstack((points.T, np.ones((1, num))))
    XYZ_S = (K_inv @ xy_homo).T
    S = -a / (XYZ_S @ n.reshape(3, 1))
    XYZ = (XYZ_S.T * S.reshape(1, -1)).T
    return XYZ


def all_together(anno_index):
    VanishingPts = getVanishingPts(annotations[anno_index])
    d = compute_d(K, VanishingPts)
    n = compute_n(d)
    # print(n)
    if anno_index == 2:
        point_2 = annotations[0][2]
        XYZ_ = K_inv @ np.array([point_2[0], point_2[1], 1])
    else:
        point_1 = annotations[0][1]
        XYZ_ = K_inv @ np.array([point_1[0], point_1[1], 1])
    a = -n @ XYZ_
    points, mask = get_ray(annotations[anno_index])
    color = get_color("../data/q3.png", points)
    XYZ = xy2XYZ(points, n, a, K_inv)
    return XYZ, mask, color


def draw():
    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")

    XYZ, mask, color = all_together(0)
    ax.scatter(XYZ[:, 0][mask], XYZ[:, 1][mask], XYZ[:, 2][mask], c=color[mask])

    XYZ, mask, color = all_together(1)
    ax.scatter(XYZ[:, 0][mask], XYZ[:, 1][mask], XYZ[:, 2][mask], c=color[mask])

    XYZ, mask, color = all_together(2)
    ax.scatter(XYZ[:, 0][mask], XYZ[:, 1][mask], XYZ[:, 2][mask], c=color[mask])

    XYZ, mask, color = all_together(3)
    ax.scatter(XYZ[:, 0][mask], XYZ[:, 1][mask], XYZ[:, 2][mask], c=color[mask])

    XYZ, mask, color = all_together(4)
    ax.scatter(XYZ[:, 0][mask], XYZ[:, 1][mask], XYZ[:, 2][mask], c=color[mask])

    ax.set_xlabel("X Label")
    ax.set_ylabel("Y Label")
    ax.set_zlabel("Z Label")
    ax.axes.set_xlim3d(left=-0.4, right=0.4)
    ax.axes.set_ylim3d(bottom=-0.2, top=0.6)
    ax.axes.set_zlim3d(bottom=-0.4, top=-1.2)

    plt.show()


# %%
