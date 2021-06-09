import numpy as np
import math
import csv
import tf_example as tf
import cv2


N = 10000
#TF Init
predict =tf.predict()
point = predict.point

# SDLP Init
s_mu = 0.21 # in meter
s_sd = 0.09
sdlp_l = np.random.normal(s_mu, s_sd, N)
sdlp_l = [sdlp if sdlp > 0 else s_mu  for sdlp in sdlp_l]

# PERCLOS Init
pc_mu = 0.20
pc_sd = 0.10
pc_l = np.random.normal(pc_mu, pc_sd, N)

pc_l = [pc if pc > 0 else pc_mu  for pc in pc_l]
# TPM Init
tpm_mu = 12
tpm_sd = 4
tpm_l = np.random.normal(tpm_mu, tpm_sd, N)

tpm_l = [tpm if tpm > 0 else tpm_mu  for tpm in tpm_l]

"""
def err_calc(sdlp, pc):
    e_sdlp = math.exp(abs(sdlp))
    e_pc = math.exp(abs(pc))
    return e_sdlp * e_pc

"""

def err_calc(sdlp, pc, tpm, point):
    return math.sqrt(sdlp**2 + pc**2 + (21 - tpm)**2 + point)


def decision_tree(sdlp, pc, tpm,point):
    #flag -> red : 4, orange : 3, yellow : 2, green 1
    flag = 1
    if tpm < 4:
        flag = 4
    elif sdlp > 0.45:
        flag = 4
    elif pc > 0.50:
        flag = 4
    elif tpm < 8 and pc > 0.30:
        flag = 3
    elif tpm < 8 and sdlp > 0.30:
        flag = 3
    elif sdlp > 0.30 and pc > 0.30:
        flag = 3
    elif pc > 0.30 or sdlp > 0.30:
        flag = 2
    elif point < 1:
        flag = 1
    elif point <2.5 and point > 1:
        flag =2
    elif point <4. and point > 2.5:
        flag= 3
    elif point > 4:
        flag = 4

    return flag

err_l = []
flg_l = []
cam = cv2.VideoCapture(0)
for sdlp, tpm in zip(sdlp_l, tpm_l):

    ret,frame = cam.read()
    frame = cv2.flip(frame,1)
    if ret:
        predict.mainloop(frame)
        predict.detection(frame)
        pc = predict.PERCLOS
        point = predict.point
        cv2.putText(frame, f"Prediction: {predict.strongest_label}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 0, 0), 2)
        cv2.putText(frame, "Point: {:.2f}".format(predict.point), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(frame, "PERCLOS: {:.2f}".format(predict.PERCLOS), (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0),
                    2)
        err = err_calc(sdlp, pc, tpm, point)
        flg = decision_tree(sdlp, pc, tpm, point)
        err_l.append(err)
        flg_l.append(flg)
        print("ERR Stats : ")
        print("Max : {}".format(max(err_l)))
        print("Min : {}".format(min(err_l)))
        print("---------------")
        print("FLG Stats : ")
        print("Green Flag Ratio {}".format(flg_l.count(1) / N))
        print("Yellow Flag Ratio {}".format(flg_l.count(2) / N))
        print("Orange Flag Ratio {}".format(flg_l.count(3) / N))
        print("Red Flag Ratio {}".format(flg_l.count(4) / N))
cam.release()
cv2.destroyAllWindows()




"""
while True:
    sdlp = float(input())
    hr = float(input())
    tpm = float(input())
    flg = decision_tree(sdlp, hr, tpm)
    print("Flag  : {}".format(flg))
"""

print("ERR Stats : ")
print("Max : {}".format(max(err_l)))
print("Min : {}".format(min(err_l)))
print("---------------")
print("FLG Stats : ")
print("Green Flag Ratio {}".format(flg_l.count(1) / N))
print("Yellow Flag Ratio {}".format(flg_l.count(2) / N))
print("Orange Flag Ratio {}".format(flg_l.count(3) / N))
print("Red Flag Ratio {}".format(flg_l.count(4) / N))



with open('syi_data_v11.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["SDLP", "PC", "TPM", "ERR", "FLAG"])
    for sdlp, pc, tpm, err, flg in zip(sdlp_l, pc_l, tpm_l, err_l, flg_l):
        writer.writerow([sdlp, pc, tpm, err, flg])
