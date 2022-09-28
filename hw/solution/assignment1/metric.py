# %%
import numpy as np
import matplotlib.pyplot as plt
from skimage import io
from scipy import linalg

import argparse
from utils import MyWarp, normalize, cosine
from affine_rectification import *

def annotate(coord):
    x1, x2, x3, x4 = coord[0][0], coord[0][1], coord[0][2], coord[0][3]
    y1, y2, y3, y4 = coord[1][0], coord[1][1], coord[1][2], coord[1][3]
    x5, x6, x7, x8 = coord[0][4], coord[0][5], coord[0][6], coord[0][7]
    y5, y6, y7, y8 = coord[1][4], coord[1][5], coord[1][6], coord[1][7]

    plt.plot([x1, x2],[y1, y2], color="blue", linewidth=4)
    plt.plot([x3, x4],[y3, y4], color="green", linewidth=4)
    plt.plot([x5, x6],[y5, y6], color="blue", linewidth=4)
    plt.plot([x7, x8],[y7, y8], color="green", linewidth=4)

# read image
im = read_img("data/q1/{}.jpg".format("checker1"))

# affine rectification
h = im.shape[0]
affine_coord = np.array([[347, 639, 74, 402], [146, 249, 260, 453], [1, 1, 1, 1]])
vanishing_line = line_infinity(affine_coord)
H = compute_H(vanishing_line)
affine_rectified = MyWarp(im, H)

# metric rectification
# coord = np.array([
#     [179, 309, 309, 443, 371, 428, 166, 613],
#     [324, 254, 254, 317, 283, 592, 410, 394]
# ]
coord = np.array([
    [563, 390, 563, 730, 737, 850, 857, 569],
    [56.7, 85.4, 56.7, 110.6, 128, 40, 79.3, 75.8]
])

fig = plt.figure()
plt.imshow(affine_rectified)
# plt.show()
annotate(coord)
plt.show()
# save_annotation(coord, "metric_annotated")
l1 = line_equation(coord[:,0], coord[:,1])
m1 = line_equation(coord[:,2], coord[:,3])
l2 = line_equation(coord[:,4], coord[:,5])
m2 = line_equation(coord[:,6], coord[:,7])
# %%

# solve for C
A = np.array(
    [
        [l1[0]*m1[0], l1[0]*m1[1] + l1[1]*m1[0]],
        [l2[0]*m2[0], l2[0]*m2[1] + l2[1]*m2[0]]
        ]
               )
b = np.array([-l1[1]*m1[1], -l2[1]*m2[1]])
x = linalg.solve(A, b)
C_prime = np.zeros((3,3))
C_prime[0, 0] = x[0]
C_prime[0][1], C_prime[1][0] = x[1], x[1]
C_prime[1][1] = 1

u, s, h = np.linalg.svd(C_prime)
H = np.identity(3)
H[0,0] = np.sqrt(s[0]**-1)
H[1,1] = np.sqrt(s[1]**-1)
H = H @ u.T
# apply homography transformation onto image
res = MyWarp(affine_rectified, H)




# %%
plt.imshow(res)
# %%
