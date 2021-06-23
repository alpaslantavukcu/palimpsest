import random as r
import numpy as np
from .dpc.exp.dist_pc import DistPlusPerclos

class SyiHelper:
    def __init__(self, tpm = 11, PERCLOS = 0.0, sdlp = 0.0) -> None:
        self.tpm = tpm
        self.sdlp = sdlp
        self.PERCLOS = PERCLOS
        self.diff_list = []
        self.dpc = DistPlusPerclos()
        self.strongest_label, self.point = "Normal", 0
    

    def update(self, img, dif):
        self.diff_list.append(dif)
        self.sdlp = np.std(self.diff_list)
        #self.update_random()
        self.strongest_label, self.PERCLOS, self.point = self.dpc.make_prediction(img)

        return self.tpm, self.sdlp, self.strongest_label, self.PERCLOS, self.point


    def decision(self):
        #flag -> red : 4, orange : 3, yellow : 2, green 1
        flag = 1
        if self.tpm < 4:
            flag = 4
        elif self.sdlp > 0.45:
            flag = 4
        elif self.PERCLOS > 0.50:
            flag = 4
        elif self.tpm < 8 and self.PERCLOS > 0.30:
            flag = 3
        elif self.tpm < 8 and self.sdlp > 0.30:
            flag = 3
        elif self.sdlp > 0.30 and self.PERCLOS > 0.30:
            flag = 3
        elif self.PERCLOS > 0.30 or self.sdlp > 0.30:
            flag = 2
        elif self.point < 1:
            flag = 1
        elif self.point < 2.5 and self.point > 1:
            flag = 2
        elif self.point < 4. and self.point > 2.5:
            flag = 3
        elif self.point > 4:
            flag = 4

        return flag

