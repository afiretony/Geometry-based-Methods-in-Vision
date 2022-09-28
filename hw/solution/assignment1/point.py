# %%
from tkinter import X
import numpy as np
import matplotlib.pyplot as plt
from skimage import io
from scipy import linalg

import argparse
from utils import MyWarp, normalize, cosine
from affine_rectification import *

source_im = plt.imread("data/q3/desk-normal.jpg")
target_im = plt.imread("data/q3/desk-perspective.jpg")

# plt.figure()
# plt.imshow(target_im)
# plt.show()
# exit()
"""
    Compute the homography matrix from point correspondences.
    
    INPUTS:
        p1 and p2 - Each are size (2 x N) matrices of corresponding (x, y)'  
                 coordinates between two images
    OUTPUTS:
     H2to1 - a 3 x 3 matrix encoding the homography that best matches the linear 
            equation
"""

h, w = source_im.shape[0], source_im.shape[1]
p1 = np.array([[0,w,0,w], [0,0,h,h]])
p2 = np.array([[529, 872, 395, 815],[230, 271, 733, 790]])

A = np.empty((0,9))
for i in range(p1.shape[1]):
    A = np.vstack((A, np.array([0, 0, 0, p1[0, i], p1[1, i], 1, -p1[0, i]*p2[1, i], -p1[1, i]*p2[1, i], -p2[1, i]])))
    A = np.vstack((A, np.array([p1[0, i], p1[1, i], 1, 0, 0, 0, -p1[0, i]*p2[0, i], -p2[0, i]*p1[1, i], -p2[0, i]])))
u, s, vh = np.linalg.svd(A)
H = vh[-1, :].reshape((3, 3))
import cv2
composite_img = cv2.warpPerspective(source_im, H, (target_im.shape[1], target_im.shape[0]), dst=target_im,
                                        flags=cv2.INTER_LINEAR,borderMode=cv2.BORDER_TRANSPARENT)

# import cv2
# res = cv2.perspectiveTransform(source_im,H, target_im)
x_min = int(-np.max(p2, 1)[1])
y_min = int(np.min(p2, 1)[0]) # max because of negative coorrdinate

# res = MyWarp(source_im, H)
plt.imshow(composite_img)
#%%
res_h, res_w = res.shape[0], res.shape[1]
target_im[x_min:x_min+res_h, y_min:y_min+res_w, :] = res
# print(res.shape)
plt.imshow(target_im)


# %%
