from .lane_detector import LaneDetector
import numpy as np

class LaneDetectorHelper:
    def __init__(self):
        self.detector = LaneDetector()
        self.poly_left = None
        self.poly_right = None
        self.left = None
        self.right = None
    
    def detect(self, img):
        self.poly_left, self.poly_right, self.left, self.right = self.detector(img)

    def draw(self, img):
        cpy = img.copy()
        lines = self.left + self.right
        
        ## Find better way !
        cpy_r = cpy[:, :, 0]
        cpy_g = cpy[:, :, 1]
        cpy_b = cpy[:, :, 2]
        cpy_r[lines >= 0.5] = 255
        cpy_g[lines >= 0.5] = 0
        cpy_b[lines >= 0.5] = 0

        cpy[:, :, 0] = cpy_r
        cpy[:, :, 1] = cpy_g
        cpy[:, :, 2] = cpy_b

        return cpy
        #print(lines)

    def get_target(self, img, x = 0.5):
        cpy = img.copy()
        left_poly, right_poly, left, right = self.detector(img)
        l1 = self.poly_left(x)
        r1 = self.poly_right(x)
        dy = -0.5 * (l1 + r1)
        dx = x + 0.5

        return dx, dy
    
    def get_trajectory_from_lane_detector(self):
        # get lane boundaries using the lane detector
        #poly_left, poly_right, _, _ = self.detector(image)
        # trajectory to follow is the mean of left and right lane boundary
        # note that we multiply with -0.5 instead of 0.5 in the formula for y below
        # according to our lane detector x is forward and y is left, but
        # according to Carla x is forward and y is right.
        x = np.arange(-2,60,1.0)
        y = -0.5*(self.poly_left(x)+self.poly_right(x))
        # x,y is now in coordinates centered at camera, but camera is 0.5 in front of vehicle center
        # hence correct x coordinates
        x += 0.5
        traj = np.stack((x,y)).T
        return traj
