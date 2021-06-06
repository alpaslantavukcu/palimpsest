from .lane_detector import LaneDetector

class LaneDetectorHelper:
    def __init__(self):
        self.detector = LaneDetector()
    
    def detect(self, img):
        cpy = img.copy()
        left_poly, right_poly, left, right = self.detector(img)
        lines = left + right
        
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
