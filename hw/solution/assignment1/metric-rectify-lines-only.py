# %%
import numpy as np
import matplotlib.pyplot as plt
from skimage import io
from scipy import linalg

import argparse
from utils import MyWarp, normalize, cosine
from affine_rectification import *

def annotate(coord, color):
    for i in range(coord.shape[1] // 2 +1):
        plt.plot(coord[0, i*2:i*2+2], coord[1, i*2:i*2+2], color=color, linewidth=3)

# read image
im = read_img("data/q1/{}.jpg".format("tiles5"))

# %%

fig = plt.figure()
plt.imshow(im)
# plt.show()
# %% 


# metric rectification
ls = np.array([
    [127.2, 93, 281.6, 338.7, 311.4, 369.4, 283.3, 338.7, 340.4, 548.5, 127, 93.1, 395, 586.8],
    [361.3, 160.9, 158.4, 104.6, 132.8, 187.3, 160.9, 212.9, 363, 363.9, 361.3, 160.9, 160, 160]
]
)

ms = np.array([
    [93.1, 281.6, 338.7, 395, 310.5, 367.7, 338.7, 395, 548.5, 586.8, 127.2, 338.4, 586.8, 551],
    [160.9, 158.4, 104.6, 159.2, 186.5, 131.9, 212.9, 160.1, 363.9, 164.3, 361.3, 362.5, 160, 360.6]
])

annotate(ls, "blue")
annotate(ms, "green")
plt.show()

# %%
def line_equation(pt1, pt2):
    x1, y1 = pt1
    x2, y2 = pt2
    a = y2 - y1
    b = x1 - x2
    c = y1 * (x2-x1) - x1 * (y2-y1)
    return a, b, c


A = np.empty((0, 6))
for i in range(ls.shape[1]//2):
    
    pt1 = [ls[0, 2*i], ls[1, 2*i]]
    pt2 = [ls[0, 2*i+1], ls[1, 2*i+1]]
    l = line_equation(pt1, pt2)

    pt1 = [ms[0, 2*i], ms[1, 2*i]]
    pt2 = [ms[0, 2*i+1], ms[1, 2*i+1]]
    m = line_equation(pt1, pt2)
    
    A = np.vstack((A,[l[0]*m[0], (l[0]*m[1]+l[1]*m[0])/2, l[1]*m[1], (l[0]*m[2]+ l[2]*m[0])/2, (l[1]*m[2]+l[2]*m[1])/2, l[2]*m[2]]))

# %%
u, s, vh = np.linalg.svd(A)
ans = vh[-1, :6]
# %%
a, b, c, d, e, f = ans[0], ans[1], ans[2], ans[3], ans[4], ans[5]
C = np.array([[a, b/2, d/2],[b/2, c, e/2], [d/2, e/2, f]])


u, s, h = np.linalg.svd(C)
H = np.identity(3)
H[0,0] = np.sqrt(s[0]**-1)
H[1,1] = np.sqrt(s[1]**-1)
H = H @ u.T
# %%
import cv2
# res = cv2.warpPerspective(im, H, (im.shape[1], im.shape[0]), dst=im,
#                                         flags=cv2.INTER_LINEAR,borderMode=cv2.BORDER_TRANSPARENT)
res = MyWarp(im,H)
plt.imshow(res)

# %%
