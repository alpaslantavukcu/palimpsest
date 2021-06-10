#  -------------------------------------------------------------
#   Copyright (c) Microsoft Corporation.  All rights reserved.
#  -------------------------------------------------------------
"""
Skeleton code showing how to load and run the TensorFlow SavedModel export package from Lobe.
"""
import argparse
import os
import json
import tensorflow as tf
from PIL import Image
import numpy as np
import cv2
from datetime import datetime
import time
from scipy.spatial import distance as dist
from imutils import face_utils
import dlib

EXPORT_MODEL_VERSION = 1

cwd = os.getcwd()

class TFModel:
    def __init__(self, model_dir) -> None:
        # make sure our exported SavedModel folder exists
        self.model_dir = model_dir
        with open(os.path.join(model_dir, "signature.json"), "r") as f:
            self.signature = json.load(f)
        self.model_file = cwd + "\\sim\\syi\\dpc\\" + self.signature.get("filename")
        print("Model file : " + str(self.model_file))
        if not os.path.isfile(self.model_file):
            raise FileNotFoundError(f"Model file does not exist")
        self.inputs = self.signature.get("inputs")
        self.outputs = self.signature.get("outputs")
        # placeholder for the tensorflow session
        self.session = None

        # Look for the version in signature file.
        # If it's not found or the doesn't match expected, print a message
        version = self.signature.get("export_model_version")
        if version is None or version != EXPORT_MODEL_VERSION:
            print(
                f"There has been a change to the model format. Please use a model with a signature 'export_model_version' that matches {EXPORT_MODEL_VERSION}."
            )

    def load(self) -> None:
        self.cleanup()
        # create a new tensorflow session
        self.session = tf.compat.v1.Session(graph=tf.Graph())
        # load our model into the session
        tf.compat.v1.saved_model.loader.load(sess=self.session, tags=self.signature.get("tags"), export_dir=self.model_dir)

    def predict(self, image: Image.Image) -> dict:
        # load the model if we don't have a session
        if self.session is None:
            self.load()

        image = self.process_image(image, self.inputs.get("Image").get("shape"))
        # create the feed dictionary that is the input to the model
        # first, add our image to the dictionary (comes from our signature.json file)
        feed_dict = {self.inputs["Image"]["name"]: [image]}

        # list the outputs we want from the model -- these come from our signature.json file
        # since we are using dictionaries that could have different orders, make tuples of (key, name) to keep track for putting
        # the results back together in a dictionary
        fetches = [(key, output["name"]) for key, output in self.outputs.items()]

        # run the model! there will be as many outputs from session.run as you have in the fetches list
        outputs = self.session.run(fetches=[name for _, name in fetches], feed_dict=feed_dict)
        return self.process_output(fetches, outputs)

    def process_image(self, image, input_shape) -> np.ndarray:
        """
        Given a PIL Image, center square crop and resize to fit the expected model input, and convert from [0,255] to [0,1] values.
        """
        width, height = image.size
        # ensure image type is compatible with model and convert if not
        if image.mode != "RGB":
            image = image.convert("RGB")
        # center crop image (you can substitute any other method to make a square image, such as just resizing or padding edges with 0)
        if width != height:
            square_size = min(width, height)
            left = (width - square_size) / 2
            top = (height - square_size) / 2
            right = (width + square_size) / 2
            bottom = (height + square_size) / 2
            # Crop the center of the image
            image = image.crop((left, top, right, bottom))
        # now the image is square, resize it to be the right shape for the model input
        input_width, input_height = input_shape[1:3]
        if image.width != input_width or image.height != input_height:
            image = image.resize((input_width, input_height))

        # make 0-1 float instead of 0-255 int (that PIL Image loads by default)
        image = np.asarray(image) / 255.0
        # format input as model expects
        return image.astype(np.float32)

    def process_output(self, fetches, outputs) -> dict:
        # do a bit of postprocessing
        out_keys = ["label", "confidence"]
        results = {}
        # since we actually ran on a batch of size 1, index out the items from the returned numpy arrays
        for i, (key, _) in enumerate(fetches):
            val = outputs[i].tolist()[0]
            if isinstance(val, bytes):
                val = val.decode()
            results[key] = val
        confs = results["Confidences"]
        labels = self.signature.get("classes").get("Label")
        output = [dict(zip(out_keys, group)) for group in zip(labels, confs)]
        sorted_output = {"predictions": sorted(output, key=lambda k: k["confidence"], reverse=True)}
        return sorted_output

    def cleanup(self) -> None:
        # close our tensorflow session if one exists
        if self.session is not None:
            self.session.close()
            self.session = None

    def __del__(self) -> None:
        self.cleanup()


class DistPlusPerclos():
    def __init__(self):
        self.phone= 0
        self.text = 0
        self.dist = 0
        self.norm =0
        self.drink =0
        self.flag = 0
        self.frames=0
        self.point = 0
        self.model_dir = cwd + "\\sim\\syi\\dpc"
        self.model = TFModel(model_dir=self.model_dir)
        self.model.load()
        self.EYE_AR = 0
        self.EYE_AR_TRESH = 0.18
        self.EYE_AR_FRAMES_HD = 25
        self.EYE_AR_FRAMES_LD = 20
        self.EYE_AR_FRAMES_BLINK = 3
        self.ear = 0
        self.COUNTER = 0
        self.Total_Blink = 0
        self.Drowsy_Blink = 0
        self.PERCLOS = float(0)
        self.closed_eye_frame = 0
        self.load_detector = dlib.get_frontal_face_detector()
        self.predictor = dlib.shape_predictor(cwd + "\\sim\\syi\\dpc\\exp\\" + "shape_predictor_68_face_landmarks.dat")

    def eye_aspect_ration(self, eye):
        self.A = dist.euclidean(eye[1], eye[5])
        self.B = dist.euclidean(eye[2], eye[4])
        self.C = dist.euclidean(eye[0], eye[3])
        self.ear = (self.A + self.B) / (2.0 * self.C)
        return self.ear

    def final_ear(self, shape):
        (self.lStart, self.lEnd) = face_utils.FACIAL_LANDMARKS_IDXS["left_eye"]
        (self.rStart, self.rEnd) = face_utils.FACIAL_LANDMARKS_IDXS["right_eye"]
        self.lefteye = shape[self.lStart:self.lEnd]
        self.righteye = shape[self.rStart:self.rEnd]
        self.leftEAR = self.eye_aspect_ration(self.lefteye)
        self.righEAR = self.eye_aspect_ration(self.righteye)
        self.ear = (self.leftEAR + self.righEAR) / (2.0)
        return (self.ear, self.lefteye, self.righteye)

    def perclos_calculation(self):
        if self.Total_Blink != 0:
            self.PERCLOS = ((self.closed_eye_frame) / self.frames) * 100
        else:
            self.PERCLOS = float(0)

        return self.PERCLOS

    def detection(self,frame):
        frame =cv2.resize(frame,(600,450))
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        rects =self.load_detector(gray,0)
        for rect in rects:
            shape = self.predictor(gray,rect)
            shape = face_utils.shape_to_np(shape)

            eye = self.final_ear(shape)
            self.ear = eye[0]
            self.lefteye = eye[1]
            self.righteye=eye[2]
        if self.ear < self.EYE_AR_TRESH:
            self.COUNTER +=1
            if self.COUNTER >= self.EYE_AR_FRAMES_HD:
                self.Drowsy_Blink +=1
                self.closed_eye_frame = self.closed_eye_frame +25
        else:
            if self.COUNTER >= self.EYE_AR_FRAMES_BLINK:
                self.Total_Blink +=1
                self.closed_eye_frame +=1
                self.COUNTER=0
        self.PERCLOS = self.perclos_calculation()
        #print(f"PERCLOS: {self.PERCLOS}")

    def prediction(self,img):
        # Assume model is in the parent directory for this file


        outputs = self.model.predict(img)
        output_list = outputs.get('predictions')
        output_dict = output_list[0]
        self.strongest_label = output_dict.get('label')
        confidance = output_dict.get('confidence')
        return self.strongest_label, confidance

    def puan(self):
        self.point = (self.phone * 6 + self.text * 4 + self.drink * 3 + self.dist * 3.75 + self.norm * (-0.5)) / self.frames
        return self.point

    def mainloop(self,img):
        self.frames += 1
        img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        image = Image.fromarray(img)
        self.strongest_label, self.confidance = self.prediction(image)
        if self.strongest_label == 'Phone':
            self.phone += 1

        elif self.strongest_label == 'Distracted':
            self.dist += 1

        elif self.strongest_label == 'Text':
            self.text += 1

        elif self.strongest_label == 'Normal':
            self.norm += 1

        elif self.strongest_label == 'Drink':
            self.drink += 1

        self.point = self.puan()
        

        # print(f"Predicted: {outputs}")
        # print(output_list)
        #print(f"Predicted: {self.strongest_label}\n Confidence: {self.confidance} ")
        #print(f"Phone: {self.phone}\nText: {self.text}\nDrink: {self.drink}\nDistracted: {self.dist}\nNormal: {self.norm}\nframes:{self.frames}\nPoint: {self.point}")


    def make_prediction(self, img):
        self.mainloop(img)
        self.detection(img)

        return self.strongest_label, self.PERCLOS, self.point


    def main(self):
        cam = cv2.VideoCapture(0)
        start_time = time.time()
        while True:
            ret,frame = cam.read()
            frame = cv2.flip(frame,1)

            if ret:
                self.mainloop(frame)
                self.detection(frame)
                self.frames +=1


#predict=predict()
#predict.main()


