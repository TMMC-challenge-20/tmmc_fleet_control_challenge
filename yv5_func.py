import cv2
import numpy as np

def detect_stop_sign(video_path, is_cuda=False):

    def build_model(is_cuda):
        net = cv2.dnn.readNet("stopsign_simplified.onnx")
        if is_cuda:
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
        else:
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        return net

    INPUT_WIDTH = 640
    INPUT_HEIGHT = 640

    def detect(image, net):
        blob = cv2.dnn.blobFromImage(image, 1/255.0, (INPUT_WIDTH, INPUT_HEIGHT), swapRB=True, crop=False)
        net.setInput(blob)
        preds = net.forward()
        return preds

    def load_classes():
        class_list = []
        with open("stopsign_classes.txt", "r") as f:
            class_list = [cname.strip() for cname in f.readlines()]
        return class_list

    class_list = load_classes()

    def wrap_detection(input_image, output_data):
        class_ids = []
        confidences = []
        boxes = []

        rows = output_data.shape[0]

        image_width, image_height, _ = input_image.shape

        x_factor = image_width / INPUT_WIDTH
        y_factor =  image_height / INPUT_HEIGHT

        for r in range(rows):
            row = output_data[r]
            confidence = row[4]
            if confidence >= 0.4:
                classes_scores = row[5:]
                _, _, _, max_indx = cv2.minMaxLoc(classes_scores)
                class_id = max_indx[1]
                if (classes_scores[class_id] > .25):
                    confidences.append(confidence)
                    class_ids.append(class_id)
                    x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item() 
                    left = int((x - 0.5 * w) * x_factor)
                    top = int((y - 0.5 * h) * y_factor)
                    width = int(w * x_factor)
                    height = int(h * y_factor)
                    box = np.array([left, top, width, height])
                    boxes.append(box)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.25, 0.45) 

        result_class_ids = []
        result_confidences = []
        result_boxes = []

        for i in indexes:
            result_confidences.append(confidences[i])
            result_class_ids.append(class_ids[i])
            result_boxes.append(boxes[i])

        return result_class_ids, result_confidences, result_boxes

    def format_yolov5(frame):
        row, col, _ = frame.shape
        _max = max(col, row)
        result = np.zeros((_max, _max, 3), np.uint8)
        result[0:row, 0:col] = frame
        return result

    net = build_model(is_cuda)
    capture = cv2.VideoCapture(video_path)

    while True:
        _, frame = capture.read()
        if frame is None:
            break

        inputImage = format_yolov5(frame)
        outs = detect(inputImage, net)

        class_ids, confidences, boxes = wrap_detection(inputImage, outs[0])

        if len(class_ids) > 0:
            yield 1
        else:
            yield 0

stop_counter = 0
for result in detect_stop_sign(0, True):
    if result == 1:
        stop_counter += 1
    else:
        stop_counter = 0

    if stop_counter == 3:
        print('STOP')
        break