# %%
from re import A
import numpy as np
import matplotlib.pyplot as plt
from skimage import io

import argparse
from utils import MyWarp, normalize, cosine

# %%
def read_img(img_path):
    im = io.imread(img_path)
    return im

def show_img(im):
    fig = plt.figure()
    plt.imshow(im)
    plt.show()

def save_img(im, filename):
    plt.figure()
    plt.imsave(filename, im)

def annotate(coord):
    x1, x2, x3, x4 = coord[0][0], coord[0][1], coord[0][2], coord[0][3]
    y1, y2, y3, y4 = coord[1][0], coord[1][1], coord[1][2], coord[1][3]
    plt.plot([x1, x2],[y1, y2], color="blue", linewidth=4)
    plt.plot([x3, x4],[y3, y4], color="blue", linewidth=4)
    plt.plot([x2, x4],[y2, y4], color="green", linewidth=4)
    plt.plot([x1, x3],[y1, y3], color="green", linewidth=4)
    

def save_annotation(coord, im, filename):
    fig = plt.figure()
    plt.imshow(im)
    annotate(coord=coord)
    plt.savefig("annotated_{}.jpg".format(filename))


def line_equation(pt1, pt2):
    x1, y1 = pt1
    x2, y2 = pt2
    a = y2 - y1
    b = x1 - x2
    c = y1 * (x2-x1) - x1 * (y2-y1)
    return a, b, c

def intersection_point(l1, l2):
    a1, b1, c1 = l1
    a2, b2, c2 = l2
    x0 = (b1*c2 - b2*c1) / (a1*b2 - a2*b1)
    y0 = (c1*a2 - c2*a1) / (a1*b2 - a2*b1)
    return x0, y0

def line_infinity(coord):
    x1, x2, x3, x4 = coord[0][0], coord[0][1], coord[0][2], coord[0][3]
    y1, y2, y3, y4 = coord[1][0], coord[1][1], coord[1][2], coord[1][3]
    
    # parallel line intersection point 1
    l1 = line_equation([x1, y1], [x2, y2])
    l2 = line_equation([x3, y3], [x4, y4])
    point_infinity1 = intersection_point(l1, l2)
    # print(point_infinity1)

    # parallel line intersection point 2
    l1 = line_equation([x1, y1], [x3, y3])
    l2 = line_equation([x2, y2], [x4, y4])
    point_infinity2 = intersection_point(l1, l2)
    # print(point_infinity2)

    # line at infinity
    line = line_equation(point_infinity1, point_infinity2)
    return normalize(line)

def compute_H(vanishing_line):
    a, b, c = vanishing_line
    H_inv_T = np.identity(3)
    H_inv_T[:, 2] = -a/c, -b/c, 1/c
    H_inv = H_inv_T.T
    H = np.linalg.inv(H_inv)
    H /= H[2,2]
    return H

def compute_cosine(coord):
    x1, x2, x3, x4 = coord[0][0], coord[0][1], coord[0][2], coord[0][3]
    y1, y2, y3, y4 = coord[1][0], coord[1][1], coord[1][2], coord[1][3]

    u = (x1-x2, y1-y2)
    v = (x3-x4, y3-y4)
    angle = cosine(u, v)
    print("Angle: {}".format(angle))

    u = (x4-x2, y4-y2)
    v = (x3-x1, y3-y1)
    angle = cosine(u, v)
    print("Angle: {}".format(angle))

# %%


# read_img("data/q1/book1.jpg")
# %%
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate affinely correct warps for images that are captured through perspective cameras.')

    parser.add_argument('--filename', type=str, default="checker1")
    args = parser.parse_args()
    im = read_img("data/q1/{}.jpg".format(args.filename))

    coord_map = {
        "checker1" : np.array([[347, 639, 74, 402], [146, 249, 260, 453], [1, 1, 1, 1]]),
        "book1"    : np.array([[1349, 2450, 1357, 2446],[519, 372, 1222, 1401], [1, 1, 1, 1]]),
        "chess1"   : np.array([[209, 375, 52, 231],[38, 111, 106, 228], [1, 1, 1, 1]]),
        "pnc"      : np.array([[925, 1169, 815, 1150], [784, 812, 939, 989], [1, 1, 1, 1 ]]),
        "cmu"      : np.array([[533, 643, 539, 649], [208, 113, 478, 507], [1,1, 1, 1]]),
        "ceiling"  : np.array([[845, 1075, 220, 609], [194, 566, 510, 721], [1,1,1,1]]),
        "door"     : np.array([[852, 1154, 883, 1170],[231, 267, 1223, 1108],[1,1,1,1]])
    }

    coord = coord_map[args.filename]
    
    save_annotation(coord, im, args.filename)

    # image homography
    vanishing_line = line_infinity(coord)
    H = compute_H(vanishing_line)
    res = MyWarp(im, H)

    save_img(res, "rectified_{}.jpg".format(args.filename))
    # show_img(res)

    # line angle
    new_coord = H @ coord
    new_coord = new_coord / new_coord[2, :]
    # new_coord[0, :] = new_coord[0, :] / new_coord[2, :]
    # new_coord[1, :] = new_coord[1, :] / new_coord[2, :]
    # print(new_coord, coord)
    compute_cosine(coord)
    compute_cosine(new_coord)
