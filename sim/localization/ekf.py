import numpy as np
import math

class EKF:
    # varyans değerleri rastgele
    def __init__(self):
        self.Q = np.diag([0.1, 0.1, np.deg2rad(1.0), 1.0]) ** 2  # Önceki hata kovaryansı
        self.R = np.diag([1.0, 1.0]) ** 2  # x ve y için kovaryans
        self.DT = 0.1
    
    def set(self, x, y, vx, yr):
        self.x = x
        self.y = y
        self.vx = vx
        self.yr = yr
        self.xEst = np.array([[x], [y], [vx], [yr]])
        self.xTrue = np.zeros((4, 1))
        self.PEst = np.eye(4)

    def observation(self, xTrue, u):
        xTrue = self.motion_model(xTrue, u)
        z = self.observation_model(xTrue)

        return xTrue, z


    def motion_model(self, x, u):
        F = np.array([[1.0, 0, 0, 0],
                    [0, 1.0, 0, 0],
                    [0, 0, 1.0, 0],
                    [0, 0, 0, 0]])

        B = np.array([[self.DT * math.cos(x[2, 0]), 0],
                    [self.DT * math.sin(x[2, 0]), 0],
                    [0.0, self.DT],
                    [1.0, 0.0]])

        x = F @ x + B @ u  # matris çarpımı

        return x


    def observation_model(self, x):
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        z = H @ x

        return z


    def jacob_f(self, x, u):
        yaw = x[3, 0]
        v = u[0, 0]
        jF = np.array([
            [1.0, 0.0, -self.DT * v * math.sin(yaw), self.DT * math.cos(yaw)],
            [0.0, 1.0, self.DT * v * math.cos(yaw), self.DT * math.sin(yaw)],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]])

        return jF


    def jacob_h(self):
        jH = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        return jH


    def ekf_estimation(self, xEst, PEst, z, u):
        #  Tahmin kısmı
        xPred = self.motion_model(xEst, u)
        jF = self.jacob_f(xEst, u)
        PPred = jF @ PEst @ jF.T + self.Q

        #  Sensörlere göre güncelleme
        jH = self.jacob_h()
        zPred = self.observation_model(xPred)
        y = z - zPred
        S = jH @ PPred @ jH.T + self.R
        K = PPred @ jH.T @ np.linalg.inv(S) # kalman kazancı
        xEst = xPred + K @ y
        PEst = (np.eye(len(xEst)) - K @ jH) @ PPred # kalman çıktısı
        return xEst, PEst


    def run(self, x, y, vx, yr):

        # history
        hxEst = self.xEst
        hxTrue = self.xTrue
        hz = np.zeros((2, 1))

        u = np.array([[vx], [yr]])
        z = np.array([[x], [y]])
        self.xTrue, _ = self.observation(self.xTrue, u)

        self.xEst, self.PEst = self.ekf_estimation(self.xEst, self.PEst, z, u)

        # doğru çalışıp çalışmadığına bakmak için
        hxEst = np.hstack((hxEst, self.xEst))
        hxTrue = np.hstack((hxTrue, self.xTrue))
        hz = np.hstack((hz, z))
        return hxEst, hxTrue

