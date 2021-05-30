import cv2
class ObjectDetector:
    def __init__(self):
        self.net = cv2.dnn.readNet('D:\\yolov4.weights', 'D:\\yolov4.cfg')
        self.classes = []
        with open('D:\\coco.names', 'r') as f:
            self.classes = f.read().splitlines()

    def detect(self, img):
        height, width, _ = img.shape

        blob = cv2.dnn.blobFromImage(img, 1/255, (416,416), (0,0,0), swapRB=True, crop=False)

        self.net.setInput(blob)
        output_layer_names = self.net.getUnconnectedOutLayersNames()
        layerOutputs = self.net.forward(output_layer_names)

        boxes = []
        confidences = []
        class_ids = []

        for output in layerOutputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:
                    center_x = int(detection[0]*width)
                    center_y = int(detection[1]*height)
                    w = int(detection[2]*width)
                    h = int(detection[3]*height)

                    x = int(center_x - w/2)
                    y = int(center_y - h/2)

                    boxes.append([x,y,w,h])
                    confidences.append((float(confidence)))
                    class_ids.append(class_id)

            # print(len(boxes))


        indexes = np.array(cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4))
        font = cv2.FONT_HERSHEY_TRIPLEX
        colors = np.random.uniform(0, 255, size=(len(boxes), 2))

        for i in indexes.flatten():
            x, y, w, h = boxes[i]
            label = str(self.classes[class_ids[i]])
            confidence = str(round(confidences[i], 2))
            color = colors[i]
            cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
            cv2.putText(img, label + " " + confidence, (x, y + 20), font, 0.5, (255, 255, 255), 1)
            print(label)

        return img
