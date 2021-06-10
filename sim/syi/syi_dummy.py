import random as r
import numpy as np
from .dpc.exp.dist_pc import DistPlusPerclos

class SyiDummy:
    def __init__(self, tpm = 14, pc = 0.25, sdlp = 0.2) -> None:
        self.tpm = tpm
        self.pc = pc
        self.sdlp = sdlp
        self.dpc = DistPlusPerclos()
        self.strongest_label, self.PERCLOS, self.point = None, None, None
    

    def update(self, img):
        self.update_random()
        self.strongest_label, self.PERCLOS, self.point = self.dpc.make_prediction(img)

        return self.tpm, self.pc, self.sdlp, self.strongest_label, self.PERCLOS, self.point

    def update_random(self):
        i = r.randint(0, 2)
        if i == 0: #tpm
            tpm_sd = 4
            [change] = np.random.normal(0, tpm_sd / 2, 1)
            self.tpm += change
        elif i == 1:
            pc_sd = 0.10
            [change] = np.random.normal(0, pc_sd / 2, 1)
            self.pc += change     
        elif i == 2:
            sdlp_sd = 0.09
            [change] = np.random.normal(0, sdlp_sd / 2, 1)
            self.pc += change

        return self.tpm, self.pc, self.sdlp                   


    def decision(self):
        #flag -> red : 4, orange : 3, yellow : 2, green 1
        flag = 1
        if self.tpm < 4:
            flag = 4
        elif self.sdlp > 0.45:
            flag = 4
        elif self.pc > 0.50:
            flag = 4
        elif self.tpm < 8 and self.pc > 0.30:
            flag = 3
        elif self.tpm < 8 and self.sdlp > 0.30:
            flag = 3
        elif self.sdlp > 0.30 and self.pc > 0.30:
            flag = 3
        elif self.pc > 0.30 or self.sdlp > 0.30:
            flag = 2

        return flag

