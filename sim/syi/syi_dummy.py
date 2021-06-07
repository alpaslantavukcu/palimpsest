import random as r
import numpy as np

class SyiDummy:
    def __init__(self, tpm = 10, pc = 0.25, sdlp = 0.2) -> None:
        self.tpm = tpm
        self.pc = pc
        self.sdlp = sdlp
    

    def update_random(self):
        i = r.randint(0, 2)
        if i == 0: #tpm
            tpm_sd = 4
            [change] = np.random.normal(0, tpm_sd, 1)
            self.tpm += change
        elif i == 1:
            pc_sd = 0.10
            [change] = np.random.normal(0, pc_sd, 1)
            self.pc += change     
        elif i == 2:
            sdlp_sd = 0.09
            [change] = np.random.normal(0, sdlp_sd, 1)
            self.pc += change                   


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

