import matplotlib
from matplotlib import pyplot as plt
import os
import imageio

# png_dir = '../animation/png'
# images = []
# for file_name in sorted(os.listdir(png_dir)):
#     if file_name.endswith('.png'):
#         file_path = os.path.join(png_dir, file_name)
#         images.append(imageio.imread(file_path))

# # Make it pause at the end so that the viewers can ponder
# for _ in range(10):
#     images.append(imageio.imread(file_path))

# imageio.mimsave('../animation/gif/movie.gif', images)

def movie_3D(P):
    pass

def plot_3D(P, c):
    matplotlib.use("TkAgg")
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(P[:, 0], P[:, 1], P[:, 2], color=c, s=1)
    ax.grid(False)
    # ax.set_xticks([])
    # ax.set_yticks([])
    # ax.set_zticks([])
    ax.set_facecolor([1,1,1])
    # ax.set_xlim(-8, -4)
    # ax.set_ylim(-1, 3)
    # ax.set_zlim(5, 9)
    plt.show()
    

def plot_correspondences(img1, img2, pts1, pts2):
    """Plot the corresponding points on two images side by side."""
    fig = plt.figure()
    ax1 = fig.add_subplot(121)
    ax2 = fig.add_subplot(122)
    ax1.imshow(img1)
    ax1.plot(pts1[:, 0], pts1[:, 1], "r.")
    ax2.imshow(img2)
    ax2.plot(pts2[:, 0], pts2[:, 1], "r.")
    plt.show()
    