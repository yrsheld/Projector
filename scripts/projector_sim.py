#!/usr/bin/env python
import rospy
import math
import numpy as np
from geometry_msgs.msg import Pose, Point
import cv2

class ProjectorSim:
    def __init__(self, z, orientation, proj, fov, aspect_ratio, size_factor):
        """
        load projector extrinsics, if provided
        - z (meters), orientation(rad): projector extrinsics
        - aspect_ratio, size_factor: dimension (pixels) of input image
        """
        self.h_fov, self.v_fov = fov
        self.aspect_w, self.aspect_h = aspect_ratio
        self.src_img = None
        self.src_w = self.aspect_w * size_factor
        self.src_h = self.aspect_h * size_factor
        
        # projector extrinsics
        if z is not None and orientation is not None:
            self.z = z
            self.roll, self.pitch, self.yaw = orientation
        else:
            h, low_w, upper_w = proj
            self.calibrate(proj_height=h, proj_lower_width=low_w, proj_upper_width=upper_w)

        self.initFrame()

    def calibrate(self, proj_height, proj_lower_width, proj_upper_width):
        """
        obtain projector extrinsics, based on projected pattern measured on ground
        - proj_height, proj_lower_width, proj_upper_width: dimension (meters) of projection, measured in physical env
        """
        # calculate projector extrinsics
        dist_ratio = proj_upper_width / proj_lower_width
        remain = math.pi - self.v_fov
        # sin(c) = dist_ratio * sin(b)
        # c = remain-b
        # sin(c) = sin(remain)cos(b) - cos(remain)sin(b)
        # sin(b)(dist_ratio+cos(remain)) = cos(b)(sin(remain))
        # tan(b) = (sin(remain))/(dist_ratio+cos(remain))
        b = math.atan2(math.sin(remain), dist_ratio+math.cos(remain))
        c = remain-b
        
        # proj_height/sin(self.v_fov) = dist_lower/sin(b) = dist_higher/sin(c)
        dist_higher = proj_height/math.sin(self.v_fov)*math.sin(c)
        dist_lower = proj_height/math.sin(self.v_fov)*math.sin(b)
        # obtain z (distance to ground)
        self.z = dist_higher*math.sin(b)

        # obtain orientations (w.r.t. world frame)
        theta = math.acos(self.z/dist_lower)
        self.roll = -(math.pi - theta - self.v_fov/2)
        self.pitch = 0
        self.yaw = -1.57

    def initFrame(self):
        # quad frame (would be calculated)
        self.quad_w = 0.0
        self.quad_h = 0.0
        self.quad_pixel_factor = 0.002      # meter/pixel (need to match with projector.rviz)
        self.quad_position = None
        # projection frame
        self.proj_coords = None
        # camera frame
        self.cam_coords = None

        # image corrections & transformation matrices
        self.prewarped_src_img = None
        self.mat_src2proj = None            # src->proj
        self.mat_proj2cam = None            # proj->cam
        self.mat_imgPrewarp = None          # prewarp src img

    def calcFrame(self):
        """
        Given projector's pose & fov
        Calculate projection / camera / quad frame
        """
        # center
        #center_pt = (self.z * math.tan(math.pi + self.roll), 0.0)

        # calculate x coordinates
        lower_x = self.z * math.tan(math.pi + self.roll - self.v_fov/2)
        upper_x = self.z * math.tan(math.pi + self.roll + self.v_fov/2)
        print("lower_x: ", lower_x, ", upper_x: ", upper_x)
        # calculate lengths in y
        dist_lower = self.z / math.cos(math.pi + self.roll - self.v_fov/2)
        dist_higher = self.z / math.cos(math.pi + self.roll + self.v_fov/2)
        lower_half_y = dist_lower * math.tan(self.h_fov/2)
        upper_half_y = dist_higher * math.tan(self.h_fov/2)
        print("lower_width: ", lower_half_y*2, ", upper_width: ", upper_half_y*2)
        # get quad frame (centered at self.quad_position) (w.r.t projector link)
        self.quad_position = [(lower_x+upper_x)/2, 0, -self.z]
        self.quad_h = int((upper_x - lower_x) / self.quad_pixel_factor)
        self.quad_w = int((upper_half_y * 2) / self.quad_pixel_factor)
        #print("Quad position:", self.quad_position)
        #print("Quad img size: (%s, %s)"%(self.quad_w, self.quad_h))

        # get corner coordinates of projection & camera frame
        self.proj_coords, self.cam_coords = self.getFrameCoords(lower_x, upper_x, lower_half_y, upper_half_y)
        #print("Projection frame coordinates:\n", self.proj_coords)
        self.proj_pix_coords, self.cam_pix_coords = self.getFramePixelCoords(lower_x, upper_x, lower_half_y, upper_half_y)

        # calculate img transformation src->proj & proj->cam
        self.getImageTransform()

        # calculate camera frame dimension (meters)
        camera_w = self.cam_coords[0].y*2                      # = half_y * 2
        camera_h = self.cam_coords[1].x-self.cam_coords[2].x   # = half_x * 2

        return self.quad_position, camera_w, camera_h
    
    def getFrameCoords(self, lower_x, upper_x, lower_half_y, upper_half_y):
        """
        Calculate corner coordinates (unit: meters) of frames
        parent: projector_link
        # projection frame      # camera frame  (centered at quad frame)
        # 0 --- 1               #  0-1                     x
        #  \   /                #  | |                     |
        #   3-2                 #  3-2                 y---
        """
        # projection frame
        proj_pt0 = self.makePoint(upper_x, upper_half_y)
        proj_pt1 = self.makePoint(upper_x, -upper_half_y)
        proj_pt2 = self.makePoint(lower_x, -lower_half_y)
        proj_pt3 = self.makePoint(lower_x, +lower_half_y)

        # camera frame (centered at quad)
        center_x, center_y = self.quad_position[:2]
        half_y = (lower_half_y+upper_half_y)/2
        half_x = half_y * self.aspect_h / self.aspect_w 
        cam_pt0 = self.makePoint(center_x+half_x, half_y)
        cam_pt1 = self.makePoint(center_x+half_x, -half_y)
        cam_pt2 = self.makePoint(center_x-half_x, -half_y)
        cam_pt3 = self.makePoint(center_x-half_x, half_y)

        return [proj_pt0, proj_pt1, proj_pt2, proj_pt3], [cam_pt0, cam_pt1, cam_pt2, cam_pt3]

    def getFramePixelCoords(self, lower_x, upper_x, lower_half_y, upper_half_y):
        """
        Calculate corner coordinates (unit: pixels) of frames, laying upon quad image
        # projection frame      # camera frame         
        # 0 --- 1               #  0-1         ----x
        #  \   /                #  | |         |
        #   3-2                 #  3-2         y
        """
        # projection frame
        w_diff = int((upper_half_y - lower_half_y)/self.quad_pixel_factor)
        proj_pix_coords = np.float32([[0,0], [self.quad_w-1, 0], [self.quad_w-w_diff, self.quad_h-1], [w_diff,self.quad_h-1]])
        
        # camera frame
        center_x_pix, center_y_pix = int(self.quad_w/2), int(self.quad_h/2)
        half_y = (lower_half_y+upper_half_y)/2       # meters
        half_x_pix = half_y/self.quad_pixel_factor   # pixels
        half_y_pix = half_x_pix * self.aspect_h / self.aspect_w 
        cam_pix_coords = np.float32([[center_x_pix-half_x_pix, center_y_pix-half_y_pix], [center_x_pix+half_x_pix, center_y_pix-half_y_pix], 
                                     [center_x_pix+half_x_pix, center_y_pix+half_y_pix], [center_x_pix-half_x_pix, center_y_pix+half_y_pix]])
        return proj_pix_coords, cam_pix_coords

    def makePoint(self, x, y):
        """
        Point constructor
        """
        p = Point()
        p.x = x
        p.y = y
        p.z = 0.0

        return p

    def getImageTransform(self):
        """
        get transformation matrices.
        basis: quad frame
        """
        # src -> proj (M1)
        inputs = np.float32([[0,0], [self.src_w-1, 0], [self.src_w-1, self.src_h-1], [0, self.src_h-1]])
        outputs = self.proj_pix_coords
        print(outputs)
        self.mat_src2proj = cv2.getPerspectiveTransform(inputs,outputs)
        #print("M1:\n", self.mat_src2proj)

        # proj -> cam (M2)
        inputs = outputs
        outputs = self.cam_pix_coords
        self.mat_proj2cam = cv2.getPerspectiveTransform(inputs,outputs)
        #print("M2:\n", self.mat_proj2cam)

        # calculate image prewarp transform
        # inv(M1) @ M2 @ M1
        inv_m1_m2 = np.matmul(np.linalg.inv(self.mat_src2proj), self.mat_proj2cam)
        self.mat_imgPrewarp = np.matmul(inv_m1_m2, self.mat_src2proj)
        #print("Prewarp matrix:\n", self.mat_imgPrewarp)

    def projectImage(self, img):
        """
        project source image -> projection frame, laying upon quad frame
        """
        projection = cv2.warpPerspective(img, self.mat_src2proj, (self.quad_w, self.quad_h))

        return projection.astype(np.uint8)

    def prewarpImage(self, img):
        """
        Prewarp image to get correct projection result
        """
        h, w = img.shape[:2]
        prewarped = cv2.warpPerspective(img, self.mat_imgPrewarp, (w, h))

        # save image
        # cv2.imwrite("prewarped_img.jpg", prewarped)

        return prewarped

    def getFakeImage(self):
        """
        create a cross sign image of same size as src image (for testing)
        """
        img = np.full([self.src_h, self.src_w, 3], fill_value=(255,255,255), dtype=np.float32)
        
        # create a boundary and a cross in the middle
        boundary_color = (255,30,0)   # blue
        linewidth = 20
        img[:, :linewidth] = boundary_color
        img[:, self.src_w-linewidth:] = boundary_color
        img[:linewidth, :] = boundary_color
        img[self.src_h-linewidth:, :] = boundary_color

        # create a cross sign in the middle
        half_w, half_h= int(self.src_w//2), int(self.src_h//2)
        img[:, half_w-linewidth/2:half_w+linewidth/2+1] = boundary_color
        img[half_h-linewidth/2:half_h+linewidth/2+1, :] = boundary_color

        # create a line in the 4th quad
        linepos = int(self.src_w*3/4)
        img[half_h:, linepos-linewidth/2:linepos+linewidth/2+1] = boundary_color
        # save image
        filename = "sample_%sx%s.jpg"%(self.aspect_w, self.aspect_h)
        cv2.imwrite(filename, img)

        return img


#if __name__=="__main__":
#    projector = ProjectorSim(None, None, (0.34, 0.47392, 0.66688), (0.7, 0.3), (16, 9), 50)
#    projector.calcFrame()
#    projector.getFakeImage()