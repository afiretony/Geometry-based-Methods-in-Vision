import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
import argparse
from skimage import io
from utils import MyWarp, normalize, cosine


class Affine_Rectification:
    """
    Computes homography transformation upto affine transfer
    """
    def __init__(self, filename, coord) -> None:
        im = self.read_img(filename)
        self.im = im
        self.filename = filename
        self.coord = np.array(coord) # [nx2] annotated imgae coordinates of points from parallel lines 
        # e.g. [[x1, x2, x3, x4,...],[y1, y2, y3, y4,...]] where x1y1 x2y2 form a line, x3y3 x4y4 from a line etc.

    def read_img(self, img_path):
        im = io.imread(img_path)
        return im
    
    def display_img(self):
        plt.figure()
        plt.imshow(self.im)
        plt.show()

    def display_annotation(self):
        fig = plt.figure()
        plt.imshow(self.im)
        self.annotate()
        plt.show()

    def save_annotation(self):
        fig = plt.figure()
        plt.imshow(self.im)
        self.annotate()
        os.makedirs("Affine_Rectified", exist_ok=True)
        output_file = self.filename.split('/')[-1]
        plt.gca().set_axis_off()
        plt.subplots_adjust(top = 1, bottom = 0, right = 1, left = 0, 
                    hspace = 0, wspace = 0)
        plt.margins(0,0)
        plt.gca().xaxis.set_major_locator(plt.NullLocator())
        plt.gca().yaxis.set_major_locator(plt.NullLocator())

        plt.savefig("Affine_Rectified/annotated_{}".format(output_file),
                    bbox_inches = 'tight', pad_inches = 0)


    def annotate(self):
        coord = self.coord
        num_lines = self.coord.shape[1] // 2
        for i in range(2):
            plt.plot(coord[0, 2*i:2*i+2], coord[1, 2*i:2*i+2], color = "blue", linewidth=3)
        for i in range(2,4):
            plt.plot(coord[0, 2*i:2*i+2], coord[1, 2*i:2*i+2], color = "green", linewidth=3)
    
    @staticmethod
    def line_equation(pt1, pt2):
        x1, y1 = pt1
        x2, y2 = pt2
        a = y2 - y1
        b = x1 - x2
        c = y1 * (x2-x1) - x1 * (y2-y1)
        return a, b, c

    @staticmethod
    def intersection_point(l1, l2):
        a1, b1, c1 = l1
        a2, b2, c2 = l2
        x0 = (b1*c2 - b2*c1) / (a1*b2 - a2*b1)
        y0 = (c1*a2 - c2*a1) / (a1*b2 - a2*b1)
        return x0, y0


    def line_infinity(self):
        """
        computes line at inifinity by two intersection points from two sets of parallel lines
        """
        # parallel line intersection point 1
        x1, x2, x3, x4 = self.coord[0, 0:4]
        y1, y2, y3, y4 = self.coord[1, 0:4]
        l1 = self.line_equation([x1, y1], [x2, y2])
        l2 = self.line_equation([x3, y3], [x4, y4])
        point_infinity1 = self.intersection_point(l1, l2)

        # parallel line intersection point 2
        x1, x2, x3, x4 = self.coord[0, 4:8]
        y1, y2, y3, y4 = self.coord[1, 4:8]
        l1 = self.line_equation([x1, y1], [x2, y2])
        l2 = self.line_equation([x3, y3], [x4, y4])
        point_infinity2 = self.intersection_point(l1, l2)

        # line at infinity
        line = self.line_equation(point_infinity1, point_infinity2)
        return normalize(line)

    def compute_H(self, vanishing_line):
        """
        Computes homography upto affine transfer using property of vanishing line
        """
        a, b, c = vanishing_line
        H_inv_T = np.identity(3)
        H_inv_T[:, 2] = -a/c, -b/c, 1/c
        H_inv = H_inv_T.T
        H = np.linalg.inv(H_inv)
        H /= H[2,2]
        self.H = H
        return H
    
    def Rectify(self):
        """
        apply affine rectification
        """

        vanishing_line = self.line_infinity()
        H = self.compute_H(vanishing_line)
        res = MyWarp(self.im, H)
        self.rectified_im = res
    
    def display_rectified(self):
        self.Rectify()
        plt.figure()
        plt.imshow(self.rectified_im)
        plt.show()

    def save_rectified(self):
        self.Rectify()
        plt.figure()
        # plt.imshow(self.rectified_im)
        os.makedirs("Affine_Rectified", exist_ok=True)
        output_file = self.filename.split('/')[-1]
        plt.imsave("Affine_Rectified/rectified_{}".format(output_file), self.rectified_im)

    def compute_cosine(self, coord):
        """
        calculates cosine between the paralle lines
        """
        x1, x2, x3, x4 = coord[0][0:4]
        y1, y2, y3, y4 = coord[1][0:4]

        u = (x1-x2, y1-y2)
        v = (x3-x4, y3-y4)
        angle = cosine(u, v)

        return angle
    
    def report_line_angle(self):
        """
        report angles between parallel lines before and after homography transformation
        """
        
        # before
        angle_l = self.compute_cosine(self.coord[:, 0:4])
        angle_m = self.compute_cosine(self.coord[:, 4:8])
        print("Before: {:.4f}  {:.4f}".format(angle_l, angle_m))

        # after
        Homo_coord = np.vstack((self.coord, np.ones((1, 8))))
        New_coord = self.H @ Homo_coord
        New_coord /= New_coord[-1, :]
        angle_l = self.compute_cosine(New_coord[:, 0:4])
        angle_m = self.compute_cosine(New_coord[:, 4:8])
        print("After: {:.4f}  {:.4f}\n".format(angle_l, angle_m))

class Metric_Rectification:
    def __init__(self, filename, coord) -> None:
        im = self.read_img(filename)
        self.im = im
        self.filename = filename
        self.coord = np.array(coord) # [nx2] annotated image coordinates of points from orthogonal lines 
        # e.g. [[x1, x2, x3, x4,...],[y1, y2, y3, y4,...]] where x1y1 x2y2 form line l, x3y3 x4y4 from a line m
        # line l is perpendicular to line m in world frame
        self.H = None

    def read_img(self, img_path):
        im = io.imread(img_path)
        return im

    def display_img(self):
        plt.figure()
        plt.imshow(self.im)
        plt.show()
    
    @staticmethod
    def line_equation(pt1, pt2):
        x1, y1 = pt1
        x2, y2 = pt2
        a = y2 - y1
        b = x1 - x2
        c = y1 * (x2-x1) - x1 * (y2-y1)
        return a, b, c

    @staticmethod
    def intersection_point(l1, l2):
        a1, b1, c1 = l1
        a2, b2, c2 = l2
        x0 = (b1*c2 - b2*c1) / (a1*b2 - a2*b1)
        y0 = (c1*a2 - c2*a1) / (a1*b2 - a2*b1)
        return x0, y0

    def display_annotation(self):
        fig = plt.figure()
        plt.imshow(self.im)
        self.annotate()
        plt.show()

    def save_annotation(self):
        fig = plt.figure()
        plt.imshow(self.im)
        self.annotate()
        os.makedirs("Metric_Rectified", exist_ok=True)
        output_file = self.filename.split('/')[-1]
        plt.gca().set_axis_off()
        plt.subplots_adjust(top = 1, bottom = 0, right = 1, left = 0, 
                    hspace = 0, wspace = 0)
        plt.margins(0,0)
        plt.gca().xaxis.set_major_locator(plt.NullLocator())
        plt.gca().yaxis.set_major_locator(plt.NullLocator())

        plt.savefig("Metric_Rectified/annotated_{}".format(output_file),
                    bbox_inches = 'tight', pad_inches = 0)

    def annotate(self):
        coord = self.coord
        num_lines = self.coord.shape[1] // 2
        for i in range(num_lines):
            if i % 2 == 0:
                plt.plot(coord[0, 2*i:2*i+2], coord[1, 2*i:2*i+2], color = "blue", linewidth=3)
            else:
                plt.plot(coord[0, 2*i:2*i+2], coord[1, 2*i:2*i+2], color = "green", linewidth=3)
    
    def computeH(self):
        l1 = self.line_equation(self.coord[:,0], self.coord[:,1])
        m1 = self.line_equation(self.coord[:,2], self.coord[:,3])
        l2 = self.line_equation(self.coord[:,4], self.coord[:,5])
        m2 = self.line_equation(self.coord[:,6], self.coord[:,7])
        
        # solve for C
        A = np.array([
                    [l1[0]*m1[0], l1[0]*m1[1] + l1[1]*m1[0]],
                    [l2[0]*m2[0], l2[0]*m2[1] + l2[1]*m2[0]]
                ]
                    )
        b = np.array([-l1[1]*m1[1], -l2[1]*m2[1]])
        x = np.linalg.solve(A, b)
        C_prime = np.zeros((3,3))
        C_prime[0, 0] = x[0]
        C_prime[0][1], C_prime[1][0] = x[1], x[1]
        C_prime[1][1] = 1

        u, s, h = np.linalg.svd(C_prime)
        H = np.identity(3)
        H[0,0] = np.sqrt(s[0]**-1)
        H[1,1] = np.sqrt(s[1]**-1)
        H = H @ u.T
        self.H = H
        return H
    
    def Rectify(self):
        self.computeH()
        self.rectified_im = MyWarp(self.im, self.H)

    def display_rectified(self):
        self.Rectify()
        plt.figure()
        plt.imshow(self.rectified_im)
        plt.show()

    def save_rectified(self):
        self.Rectify()
        plt.figure()
        os.makedirs("Metric_Rectified", exist_ok=True)
        output_file = self.filename.split('/')[-1]
        plt.imsave("Metric_Rectified/{}".format(output_file), self.rectified_im)

    def compute_cosine(self, coord):
        """
        calculates cosine between the paralle lines
        """
        x1, x2, x3, x4 = coord[0][0:4]
        y1, y2, y3, y4 = coord[1][0:4]

        u = (x1-x2, y1-y2)
        v = (x3-x4, y3-y4)
        angle = cosine(u, v)

        return angle
    
    def report_line_angle(self):
        """
        report angles between parallel lines before and after homography transformation
        """
        
        # before
        angle_l = self.compute_cosine(self.coord[:, 0:4])
        angle_m = self.compute_cosine(self.coord[:, 4:8])
        print("Before: {:.4f}  {:.4f}".format(angle_l, angle_m))

        # after
        Homo_coord = np.vstack((self.coord, np.ones((1, 8))))
        New_coord = self.H @ Homo_coord
        New_coord /= New_coord[-1, :]
        angle_l = self.compute_cosine(New_coord[:, 0:4])
        angle_m = self.compute_cosine(New_coord[:, 4:8])
        print("After: {:.4f}  {:.4f}\n".format(angle_l, angle_m))




class Point_Correspondence:
    def __init__(self, src, tgt, coord) -> None:
        self.source_im = self.read_img(src)
        self.target_im = self.read_img(tgt)
        self.coord = np.array(coord) # [nx2] annotated image coordinates of points from orthogonal lines 
        # e.g. [[x1, x2, x3, x4,...],[y1, y2, y3, y4,...]] where x1y1 x2y2 form line l, x3y3 x4y4 from a line m
        # line l is perpendicular to line m in world frame
        self.H = None
        self.composite_img = None

    def read_img(self, img_path):
        im = io.imread(img_path)
        return im

    def display_img(self, im):
        plt.figure()
        plt.imshow(im)
        plt.show()

    def display_annotation(self):
        fig = plt.figure()
        plt.imshow(self.target_im)
        self.annotate()
        plt.show()

    def save_annotation(self):
        fig = plt.figure()
        plt.imshow(self.target_im)
        self.annotate()
        os.makedirs("Point_Correspondence", exist_ok=True)
        output_file = "desk.jpg"
        plt.gca().set_axis_off()
        plt.subplots_adjust(top = 1, bottom = 0, right = 1, left = 0, 
                    hspace = 0, wspace = 0)
        plt.margins(0,0)
        plt.gca().xaxis.set_major_locator(plt.NullLocator())
        plt.gca().yaxis.set_major_locator(plt.NullLocator())

        plt.savefig("Point_Correspondence/annotated_{}".format(output_file),
                    bbox_inches = 'tight', pad_inches = 0)

    def annotate(self):
        plt.plot(self.coord[0,:], self.coord[1,:], "r+")
    
    def computeH(self):
        
        h, w = self.source_im.shape[0], self.source_im.shape[1]
        p1 = np.array([[0,w,0,w], [0,0,h,h]])
        p2 = self.coord

        A = np.empty((0,9))
        for i in range(p1.shape[1]):
            A = np.vstack((A, np.array([0, 0, 0, p1[0, i], p1[1, i], 1, -p1[0, i]*p2[1, i], -p1[1, i]*p2[1, i], -p2[1, i]])))
            A = np.vstack((A, np.array([p1[0, i], p1[1, i], 1, 0, 0, 0, -p1[0, i]*p2[0, i], -p2[0, i]*p1[1, i], -p2[0, i]])))
        u, s, vh = np.linalg.svd(A)
        H = vh[-1, :].reshape((3, 3))
        self.H = H
        return H

    def composite_image(self):
        self.composite_img = cv2.warpPerspective(self.source_im, self.H, (self.target_im.shape[1], self.target_im.shape[0]), dst=self.target_im,
                                                flags=cv2.INTER_LINEAR,borderMode=cv2.BORDER_TRANSPARENT)
    def display_warped(self):
        self.computeH()
        self.composite_image()

        plt.figure()
        plt.imshow(self.composite_img)
        plt.show()

    def save_warped(self):
        self.computeH()
        self.composite_image()

        plt.figure()
        os.makedirs("Point_Correspondence", exist_ok=True)
        output_file = "warped.jpg"
        plt.imsave("Point_Correspondence/{}".format(output_file), self.composite_img)

class Direct_Metric_Rectification:
    """
    Implementation of direct metric rectification using at least 5 perpendicular correspondences 
    No need of pre-affinely correct rectification
    """

    def __init__(self, filename, coord) -> None:
        im = self.read_img(filename)
        self.im = im
        self.filename = filename
        self.coord = np.array(coord) # [nx2] annotated image coordinates of points from orthogonal lines 
        # e.g. [[x1, x2, x3, x4,...],[y1, y2, y3, y4,...]] where x1y1 x2y2 form line l, x3y3 x4y4 from a line m
        # line l is perpendicular to line m in world frame
        self.H = None

    def read_img(self, img_path):
        im = io.imread(img_path)
        return im

    def display_img(self):
        plt.figure()
        plt.imshow(self.im)
        plt.show()
    
    @staticmethod
    def line_equation(pt1, pt2):
        x1, y1 = pt1
        x2, y2 = pt2
        a = y2 - y1
        b = x1 - x2
        c = y1 * (x2-x1) - x1 * (y2-y1)
        return a, b, c

    @staticmethod
    def intersection_point(l1, l2):
        a1, b1, c1 = l1
        a2, b2, c2 = l2
        x0 = (b1*c2 - b2*c1) / (a1*b2 - a2*b1)
        y0 = (c1*a2 - c2*a1) / (a1*b2 - a2*b1)
        return x0, y0

    def display_annotation(self):
        fig = plt.figure()
        plt.imshow(self.im)
        self.annotate()
        plt.show()

    def save_annotation(self):
        fig = plt.figure()
        plt.imshow(self.im)
        self.annotate()
        os.makedirs("Direct_Metric_Rectified", exist_ok=True)
        output_file = self.filename.split('/')[-1]
        plt.gca().set_axis_off()
        plt.subplots_adjust(top = 1, bottom = 0, right = 1, left = 0, 
                    hspace = 0, wspace = 0)
        plt.margins(0,0)
        plt.gca().xaxis.set_major_locator(plt.NullLocator())
        plt.gca().yaxis.set_major_locator(plt.NullLocator())

        plt.savefig("Direct_Metric_Rectified/annotated_{}".format(output_file),
                    bbox_inches = 'tight', pad_inches = 0)

    def annotate(self):
        coord = self.coord
        num_lines = self.coord.shape[1] // 2
        for i in range(num_lines):
            if i % 2 == 0: 
                plt.plot(coord[0, 2*i:2*i+2], coord[1, 2*i:2*i+2], color = "blue", linewidth=3)
            else:
                plt.plot(coord[0, 2*i:2*i+2], coord[1, 2*i:2*i+2], color = "green", linewidth=3)
    
    def computeH(self):
        coord = self.coord
        A = np.empty((0, 6))
        num_correspondence = coord.shape[1] // 4
        for i in range(num_correspondence):
            
            pt1 = [coord[0, 4*i], coord[1, 4*i]]
            pt2 = [coord[0, 4*i+1], coord[1, 4*i+1]]
            l = self.line_equation(pt1, pt2)

            pt1 = [coord[0, 4*i+2], coord[1, 4*i+2]]
            pt2 = [coord[0, 4*i+3], coord[1, 4*i+3]]
            m = self.line_equation(pt1, pt2)
            
            A = np.vstack((A,[l[0]*m[0], (l[0]*m[1]+l[1]*m[0])/2, l[1]*m[1], (l[0]*m[2]+ l[2]*m[0])/2, (l[1]*m[2]+l[2]*m[1])/2, l[2]*m[2]]))

        u, s, vh = np.linalg.svd(A)
        ans = vh[-1, :6]

        a, b, c, d, e, f = ans[0], ans[1], ans[2], ans[3], ans[4], ans[5]
        C = np.array([[a, b/2, d/2],[b/2, c, e/2], [d/2, e/2, f]])

        u, s, h = np.linalg.svd(C)
        H = np.identity(3)
        H[0,0] = np.sqrt(s[0]**-1)
        H[1,1] = np.sqrt(s[1]**-1)
        H = H @ u.T
        self.H = H
        return H
    
    def Rectify(self):
        self.computeH()
        self.rectified_im = MyWarp(self.im, self.H)

    def display_rectified(self):
        self.Rectify()
        plt.figure()
        plt.imshow(self.rectified_im)
        plt.show()

    def save_rectified(self):
        self.Rectify()
        plt.figure()
        os.makedirs("Direct_Metric_Rectified", exist_ok=True)
        output_file = self.filename.split('/')[-1]
        plt.imsave("Direct_Metric_Rectified/{}".format(output_file), self.rectified_im)

    def compute_cosine(self, coord):
        """
        calculates cosine between the paralle lines
        """
        x1, x2, x3, x4 = coord[0][0:4]
        y1, y2, y3, y4 = coord[1][0:4]

        u = (x1-x2, y1-y2)
        v = (x3-x4, y3-y4)
        angle = cosine(u, v)

        return angle
    
    def report_line_angle(self):
        """
        report angles between parallel lines before and after homography transformation
        """
        
        # before
        angle_1 = self.compute_cosine(self.coord[:, 0:4])
        angle_2 = self.compute_cosine(self.coord[:, 4:8])
        angle_3 = self.compute_cosine(self.coord[:, 8:12])

        print("Before: {:.4f}  {:.4f}  {:.4f}".format(angle_1, angle_2, angle_3))

        # after
        Homo_coord = np.vstack((self.coord, np.ones((1, self.coord.shape[-1]))))
        New_coord = self.H @ Homo_coord
        New_coord /= New_coord[-1, :]
        angle_1 = self.compute_cosine(New_coord[:, 0:4])
        angle_2 = self.compute_cosine(New_coord[:, 4:8])
        angle_3 = self.compute_cosine(New_coord[:, 8:12])
        print("After: {:.4f}  {:.4f}  {:.4f}".format(angle_1, angle_2, angle_3))


class Dummy_AR:
    def __init__(self, src, tgt, coord) -> None:
        
        self.source_im = []
        for path in src:
            self.source_im.append(self.read_img(path))
        self.target_im = self.read_img(tgt)
        self.coord = np.array(coord) # [3x2x4] 
        self.H = []
        self.composite_img = None

    def read_img(self, img_path):
        im = io.imread(img_path)
        return im

    def display_img(self, im):
        plt.figure()
        plt.imshow(im)
        plt.show()

    def annotate(self):
        plt.plot(self.coord[0,:], self.coord[1,:], "r+")
    
    def computeH(self):
        for i, source_im in enumerate(self.source_im):
            h, w = source_im.shape[0], source_im.shape[1]
            p1 = np.array([[0,w,0,w], [0,0,h,h]])
            p2 = self.coord[i]

            A = np.empty((0,9))
            for i in range(p1.shape[1]):
                A = np.vstack((A, np.array([0, 0, 0, p1[0, i], p1[1, i], 1, -p1[0, i]*p2[1, i], -p1[1, i]*p2[1, i], -p2[1, i]])))
                A = np.vstack((A, np.array([p1[0, i], p1[1, i], 1, 0, 0, 0, -p1[0, i]*p2[0, i], -p2[0, i]*p1[1, i], -p2[0, i]])))
            u, s, vh = np.linalg.svd(A)
            H = vh[-1, :].reshape((3, 3))
            self.H.append(H)


    def composite_image(self):
        self.composite_img = self.target_im
        for i in range(3):
            self.composite_img = cv2.warpPerspective(self.source_im[i], self.H[i], (self.target_im.shape[1], self.target_im.shape[0]), dst=self.target_im,
                                                flags=cv2.INTER_LINEAR,borderMode=cv2.BORDER_TRANSPARENT)
    def display_warped(self):
        self.computeH()
        self.composite_image()

        plt.figure()
        plt.imshow(self.composite_img)
        plt.show()

    def save_warped(self):
        self.computeH()
        self.composite_image()

        plt.figure()
        os.makedirs("Dummy_AR", exist_ok=True)
        output_file = "warped.jpg"
        plt.imsave("Dummy_AR/{}".format(output_file), self.composite_img)

# Coordinates for affine rectification
DOOR_COORD = [[852, 1154, 883, 1170, 852, 883, 1154, 1170],[231, 267, 1223, 1108, 231, 1223, 267, 1108]]
CEILING_COORD = [[845, 1075, 220, 609, 845, 220, 1075, 609], [194, 566, 510, 721, 194, 510, 566, 721]]
CHECKER1_COORD = [[347, 639, 74, 402, 347, 74, 639, 401], [146, 249, 260, 453, 146, 260, 249, 453]]
BOOK1_COORD = [[1349, 2450, 1357, 2446, 1349, 1357, 2450, 2446],[519, 372, 1222, 1401, 519, 1222, 372, 1401]]
CHESS1_COORD = [[209, 375, 52, 231, 209, 52, 375, 231],[38, 111, 106, 228, 38, 106, 111, 228]]

# coordinates for metric rectification
BOOK1_COORD_METRIC = [[434, 559, 559, 571, 277, 300, 330, 575], [376, 270, 270, 576, 513, 1105, 1105, 905]]
CHECKER1_COORD_METRIC = [[565.1, 709.1, 709.1, 855.9, 791.9, 855.9, 791.9, 625.3], [76.5, 52, 52, 97.2, 144.3, 97.2, 144.3, 144]]
CHESS1_COORD_METRIC = [[199.4, 353.4, 353.4, 439.7, 199, 440, 351, 285], [63.1, 22.4, 22.4, 69.9, 62.6, 69.5, 21.4, 112.1]]
DOOR_COORD_METRIC = [[2700, 2633, 2633, 3372, 3564, 3684, 3570, 3691], [3139, 1634, 1634, 1767, 2640, 2848, 2836, 2753]]
CEILING_COORD_METRIC = [[8870, 10930, 10903, 12540, 10970, 11920, 9660, 13410], [7140, 5380, 5380, 6540, 5350, 9010, 7190, 7720]]

# Point correspondence
POINT_CORRESPONDENCE = [[529, 872, 395, 815],[230, 271, 733, 790]]

# Coordinates for direct metric rectification
DIRECT_METRIC_COORD = [[127.2,  93. ,  93.1, 281.6, 281.6, 338.7, 338.7, 395. , 311.4,
    369.4, 310.5, 367.7, 283.3, 338.7, 338.7, 395. , 340.4, 548.5,
    548.5, 586.8, 127. ,  93.1, 127.2, 338.4, 395. , 586.8, 586.8,
    551. ],
[361.3, 160.9, 160.9, 158.4, 158.4, 104.6, 104.6, 159.2, 132.8,
    187.3, 186.5, 131.9, 160.9, 212.9, 212.9, 160.1, 363. , 363.9,
    363.9, 164.3, 361.3, 160.9, 361.3, 362.5, 160. , 160. , 160. ,
    360.6]]
    
# coordinates for dummy AR
DUMMY_AR_COORD = [[[1426, 1633, 1419, 1638],[548, 545, 734, 731]], 
[[654, 1380, 662, 1382],[590, 576, 956, 925]], 
[[350, 614, 358, 623],[565, 556, 992, 981]]]

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Geometory methos in computer vision')

    parser.add_argument('--mode', type=str)
    args = parser.parse_args()
    assert args.mode in ["affine_rectification", "metric_rectification", "point-correspondence", "direct_metric_rectification", "dummy_AR"]

    if args.mode == "affine_rectification":
        affine = Affine_Rectification("data/q1/book1.jpg", BOOK1_COORD)
        affine.save_annotation()
        affine.save_rectified()
        affine.report_line_angle()

        affine = Affine_Rectification("data/q1/checker1.jpg", CHECKER1_COORD)
        affine.save_annotation()
        affine.save_rectified()
        affine.report_line_angle()

        affine = Affine_Rectification("data/q1/chess1.jpg", CHESS1_COORD)
        affine.save_annotation()
        affine.save_rectified()
        affine.report_line_angle()

        affine = Affine_Rectification("data/q1/ceiling.jpg", CEILING_COORD)
        affine.save_annotation()
        affine.save_rectified()
        affine.report_line_angle()

        affine = Affine_Rectification("data/q1/door.jpg", DOOR_COORD)
        affine.save_annotation()
        affine.save_rectified()
        affine.report_line_angle()

    if args.mode == "metric_rectification":
        metric = Metric_Rectification("Affine_Rectified/rectified_book1.jpg", BOOK1_COORD_METRIC)
        metric.save_annotation()
        metric.save_rectified()
        metric.report_line_angle()

        metric = Metric_Rectification("Affine_Rectified/rectified_checker1.jpg", CHECKER1_COORD_METRIC)
        metric.save_annotation()
        metric.save_rectified()
        metric.report_line_angle()

        metric = Metric_Rectification("Affine_Rectified/rectified_chess1.jpg", CHESS1_COORD_METRIC)
        metric.save_annotation()
        metric.save_rectified()
        metric.report_line_angle()


        metric = Metric_Rectification("Affine_Rectified/rectified_door.jpg", DOOR_COORD_METRIC)
        metric.save_annotation()
        metric.save_rectified()
        metric.report_line_angle()

        metric = Metric_Rectification("Affine_Rectified/rectified_ceiling.jpg", CEILING_COORD_METRIC)
        metric.save_annotation()
        metric.save_rectified()
        metric.report_line_angle()

    if args.mode == "point-correspondence":
        point = Point_Correspondence("data/q3/desk-normal.jpg", "data/q3/desk-perspective.jpg", POINT_CORRESPONDENCE)
        point.save_annotation()
        point.save_warped()
    
    if args.mode == "direct_metric_rectification":
        direct = Direct_Metric_Rectification("data/q1/tiles5.jpg", DIRECT_METRIC_COORD)
        direct.save_annotation()
        direct.save_rectified()
        direct.report_line_angle()
    
    if args.mode == "dummy_AR":
        dummy = Dummy_AR(["data/q5/tnj1.jpg", "data/q5/tnj4.jpg", "data/q5/tnj3.jpg"], "data/q5/pnc-park.jpg",DUMMY_AR_COORD)
        dummy.save_warped()

