# HW3: 3D reconstruction

## Overview

In this assignment you will begin by implementing the methods to estimate the fundamental matrix from corresponding points in two images. Next, given the fundamental matrix and calibrated intrinsics (which will be provided) you will compute the essential matrix. Next, you will implement RANSAC to improve your algorithm and use this to compute a 3D metric reconstruction from 2D correspondences using triangulation. 


## Q1: 8-point and 7-point algorithm (40 points)

### (A1) F matrix using 8-point algorithm (15 points)

Given two images from the [Co3D dataset](https://ai.facebook.com/datasets/CO3D-dataset/), you need to implement the 8-point algorithm for estimating the fundamental matrix. 

**Data**

We provide 2 sets of two-view images along with the corresponding points in the two images as a `$object_corresp_raw.npz` file. Within each `.npz` file, the fields `pts1` and `pts2` are `N × 2` matrices corresponding to the `(x, y)` coordinates of the N points in the first and second image repectively. 

 * Run your code on the 2 sets of `2` images provided in the `data/q1a` folder for this question.

**Submission** 

 * Brief explanation of your implementation.

   * **Linear solution** A solution F is obtained from the vector f corresponding to the smallest singular value of A. Where A is defined as
     $$
     Af = 
     \begin{bmatrix}
     x_1'x_1 & x_1'y_1 & x_1' & y_1'x_1 & y_1'y_1 & y_1' & x_1 & y_1 & 1\\
     \vdots &\vdots &\vdots &\vdots &\vdots &\vdots &\vdots &\vdots &\vdots \\
     x_n'x_n & x_n'y_n & x_n' & y_n'x_n & y_n'y_n & y_n' & x_n & y_n & 1\\
     \end{bmatrix}
     $$

   * **Constraint enforcement** Replace F by F', the closest singular matrix to F under a Frobenius norm. This correction is done using the SVD.

 * Epipolar lines: Show lines from fundamental matrix over the two images. See the following example figure:

| F-matrix visualizations |  |
| -----------  | -----------  |
| ![](figs/eight_point/teddy_1.jpg) | ![](figs/eight_point/teddy_2.jpg) |
| ![](figs/eight_point/chair_1.jpg) | ![](figs/eight_point/chair_2.jpg) |


### (A2) E matrix using 8-point algorithm (5 points)

Given the estimated fundamental matrix `F` (from above) and intrinsic matrices `K1` and `K2` (that we provide as `intrinsic_matrices_$object.npz`), you need to compute the essential matrix `E`.

**Submission** 

 * Brief explanation of your implementation.

   * E can be computed with 
     $$
     E = K^{\prime T} F K
     $$
     

 * Provide your estimated `E`.

   [[ 0.39839072,  3.96651768, -0.53939499],

   [-4.08818061,  0.47782966,  2.05394271],

   [ 0.21233717, -2.38218264,  0.09259576]])

   


### (B) 7-point algorithm (20 points)

Since the fundamental matrix only has 7 degrees of freedom, it is possible to calculate `F` using only 7 point correspondences. This requires solving a polynomial equation. In this question, you will implement the 7-point algorithm.

**Data**

We provide `$object_7_point_corresp.npz` that consists of 7 precise correspondences (shown below) for you to run 7-point algorithm. 

| 7-point correspondence visualization  |
| -----------  |
| <img src="figs/q1b_7point_data.jpg" width="700"> |


 * Run your code on the 2 sets of `2` images provided in the `data/q1b` folder for this question.

**Hint**

There are probably multiple solutions from the 7-point algorithm. You need to choose the correct one manually or in whatever way you want. (E.g. Hint 1: what should the epipolar lines on these images look like? Hint 2: pick some extra correspondences to check whether the estimated `F` is correct.)


**Submission** 
 * Brief explanation of your implementation

   * Using 7 annotated/automatic correspondences, find f such that Af=0
   * Two linearly independent solutions f1 f2 - last two singular vectors of A
   * find $\lambda$ s.t.  $det(\lambda F_1+(1-\lambda) F_2 = 0)$
   * (test all for inliers if multiple real solutions)

 * Epipolar lines: Similar to the above, you need to show lines from fundamental matrix over the two images.

   | ![](figs/seven_point/toybus_1.jpg)   | ![](figs/seven_point/toybus_2.jpg)   |
   | ------------------------------------ | ------------------------------------ |
   | ![](figs/seven_point/toytrain_1.jpg) | ![](figs/seven_point/toytrain_2.jpg) |

   


## Q2: RANSAC with 7-point and 8-point algorithm (30 points)

In some real world applications, manually determining correspondences is infeasible and often there will be noisy coorespondences. Fortunately, the RANSAC method can be applied to the problem of fundamental matrix estimation.

**Data**

In this question, you will use the image sets released in `q1a` and `q1b` and calculate the `F` matrix using both 7-point and 8-point algorithm with RANSAC. The given correspondences `$object_corresp_raw.npz` consists potential inlier matches. Within each `.npz` file, the fields `pts1` and `pts2` are `N × 2` matrices corresponding to the `(x, y)` coordinates of the N points in the first and second image repectively. 

**Hint**
- There are around 50-60% of inliers in the provided data.
- Pick the number of iterations and tolerance of error carefully to get reasonable `F`.

**Submission** 

 * Brief explanation of your RANSAC implementation and criteria for considering inliers.

   * Criteria: sum of distance of pts2 from l2 less than 2
   * randomly sample 7/8 correspondences for computing F
   * reproject pts2 using F and find distance between l2, compute ratio of inliner with threshold
   * update F if ratio is higher, otherwise pass

 * Report your best solution and plot the epipolar lines -- show lines from fundamental matrix that you calculate over the inliers.

   * $$
     F = \begin{bmatrix}
     1.64938352e-07& -1.11490218e-07 & 7.91414741e-04 \\
     -3.76433141e-07& -2.01839009e-07 &-3.57281514e-03 \\
     -0.00189703  &0.00389162 & 1.        \\
     \end{bmatrix}
     $$

   - | ![](figs/ransac/teddy_1.jpg)   | ![](figs/ransac/teddy_2.jpg)   |
     | ------------------------------ | ------------------------------ |
     | ![](figs/ransac8/toybus_1.jpg) | ![](figs/ransac8/toybus_2.jpg) |
   
 * Visualization (graph plot) of % of inliers vs. # of RANSAC iterations (see the example below). You should report such plots for both, the 7-pt and 8-pt Algorithms in the inner loop of RANSAC.

   ![](figs/ransac7/test_ransac7.jpg)

   ![](figs/ransac8/test_ransac8.jpg)

   


## Q3: Triangulation (30 points)

Given 2D correspondences and 2 camera matrices, your goal is to triangulate the 3D points. 

**Data**
- We provide the 2 images: `data/q3/img1.jpg` and `data/q3/img2.jpg`. 
- We provide the 2 camera matrices in `data/q3/P1.npy` and `data/q3/P2.npy`, both of which are `3x4` matrices.
- We provide 2D correspondences in `data/q3/pts1.npy` and `data/q3/pts2.npy`, where `pts1` and `pts2` are `Nx2` matrices. Below is a visualization of the correspondences:

<img src="figs/corresp.png" width="400"> 

**Submission**
- Brief explanation of your implementation.

  - Use linear triangulation, as descripted in Hartley 12.2 P312

    ![](figs/triangulation.png)

    

- A colored point cloud as below:

  | ![](figs/triangulation/cow.png) | ![](figs/triangulation/cow2.png) | ![](figs/triangulation/cow3.png) |
  | ------------------------------- | -------------------------------- | -------------------------------- |

  

## Q4: Bonus 1 - Bundle Adjustment (10 points)

Given 2D correspondences and 2 noisy camera matrices, your goal is to reconstruct the 3D structure as well as optimize noisy camera matrices.

**Data**
- We provide the 2 images: `data/q4/img1.jpg` and `data/q4/img2.jpg`. 
- We provide the 2 camera matrices in `data/q4/P1_noisy.npy` and `data/q4/P2_noisy.npy`, both of which are `3x4` matrices.
- We provide 2D correspondences in `data/q4/pts1.npy` and `data/q4/pts2.npy`, where `pts1` and `pts2` are `Nx2` matrices. Below is a visualization of the correspondences:

**Hint**

First triangulate the 3D points, then use `scipy.optimize.least_squares` to optimize the reprojection error.

**Submission**
- Brief explanation of your implementation.

    - Define 

    - Formulate residual function and return a flattened Nx4 vector $[(x1'-x1), (y1'-y1), (x2'-x2), (y2'-y2)] $ 

    - use `scipy.optimize.least_square` to minimize the cost function

    - I also did some ablation study, both Levenberg-Marquardt algorithm and Trust Region Reflective algorithm gave good results and able to converge to solution less than 3 minutes.

        

- A colored point cloud before and after bundle adjustment:

    |2D Correspondences | Before Bundle Adjustment  | After Bundle Adjustment |
    | -----------  | ----------| ---------- |
    |<img src="figs/q4corresp.png" width="400">  | <img src="figs/noisy2.png" width="300"> | <img src="figs/bundle/res1.png" style="zoom:30%;" /> |

Trust Region Reflective algorithm

![](figs/bundle/log.png)

Levenberg-Marquardt algorithm

```
Function evaluations 13256, initial cost 2.1507e+05, final cost 7.5326e+00, first-order optimality 7.54e-04.
```



## Q5: Bonus 2 - Fundamental matrix estimation on your own images. (10 points)

Input: 2 views from CMU CERLAB UAV:

| ![](data/q4/drone1.png) | ![](data/q4/drone2.png) |
| ----------------------- | ----------------------- |
|                         |                         |

```python
# find the keypoints and descriptors with SIFT
sift = cv.SIFT_create()
kp1, des1 = sift.detectAndCompute(img1, None)
kp2, des2 = sift.detectAndCompute(img2, None)

# use FLANN based matcher and ratio test to get correspondances
FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
search_params = dict(checks=50)
flann = cv.FlannBasedMatcher(index_params, search_params)
matches = flann.knnMatch(des1, des2, k=2)
pts1 = []
pts2 = []
# ratio test as per Lowe's paper
for i, (m, n) in enumerate(matches):
    if m.distance < 0.8 * n.distance:
        pts2.append(kp2[m.trainIdx].pt)
        pts1.append(kp1[m.queryIdx].pt)
```

### Result:

![](figs/epipolarline.png)
