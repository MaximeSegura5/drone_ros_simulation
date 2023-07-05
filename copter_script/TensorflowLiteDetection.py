import tensorflow as tf
import numpy as np
import time
import cv2
import os


def load_labels(filename):
  with open(filename, 'r') as f:
    return [line.strip() for line in f.readlines()]

class TensorflowLiteDetectionModel:
    def __init__(self, model_path, labels_path, image_size=224):
        self.interpreter = tf.lite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.labels = load_labels(labels_path)
        self.probability_threshold = 0.5
        self.input_mean = 127.5
        self.input_std = 127.5
        self.COLORS = np.random.randint(0, 255, size=(len(self.labels), 3), dtype=np.uint8)

         # check the type of the input tensor
        self.floating_model = self.input_details[0]['dtype'] == np.float32

        # NxHxWxC, H:1, W:2
        self.height = self.input_details[0]['shape'][1]
        self.width = self.input_details[0]['shape'][2]

    def run_from_filepath(self, image_path, save_image=True):
        origin_image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
        resized = cv2.resize(origin_image, (self.width, self.height), interpolation = cv2.INTER_AREA)
        image = np.array(resized)
        # add N dim
        input_data = np.expand_dims(image, axis=0)
        
        if self.floating_model:
            input_data = (np.float32(input_data) - self.input_mean) / self.input_std

        return self.run(input_data, origin_image, save_image)

    def run_from_cvimage(self, cvimage, save_image=True):
        resized = cv2.resize(cv2.cvtColor(cvimage, cv2.COLOR_BGR2RGB), (self.width, self.height), interpolation = cv2.INTER_AREA)
        image = np.array(resized)
        # add N dim
        input_data = np.expand_dims(image, axis=0)
        
        if self.floating_model:
            input_data = (np.float32(input_data) - self.input_mean) / self.input_std

        return self.run(input_data, cvimage, save_image)

    def get_output_tensor(self, index):
        """Return the output tensor at the given index."""
        output_details = self.interpreter.get_output_details()[index]
        tensor = np.squeeze(self.interpreter.get_tensor(output_details['index']))
        return tensor

    def run(self, input_data, cvimage, save_image):
        """
        Returns dict of [Label: Probability]
        """
        self.interpreter.set_tensor(self.input_details[0]["index"], input_data)
        start_time = time.time()
        self.interpreter.invoke()
        stop_time = time.time()
        # Get all outputs from the model
        boxes = self.get_output_tensor(0)
        classes = self.get_output_tensor(1)
        scores = self.get_output_tensor(2)
        count = int(self.get_output_tensor(3))

        results = []
        for i in range(count):
            if scores[i] >= self.probability_threshold:
                result = {
                    'bounding_box': boxes[i],
                    'class_id': classes[i],
                    'score': scores[i],
                    'class': self.labels[int(classes[i])]
                }
                results.append(result)
        
        if save_image:
            self.draw(results, cvimage, classes)

        return results

    def draw(self, results, cvimage, classes):
        original_image_np = cvimage.astype(np.uint8)
        for obj in results:
            # Convert the object bounding box from relative coordinates to absolute
            # coordinates based on the original image resolution
            ymin, xmin, ymax, xmax = obj['bounding_box']
            xmin = int(xmin * original_image_np.shape[1])
            xmax = int(xmax * original_image_np.shape[1])
            ymin = int(ymin * original_image_np.shape[0])
            ymax = int(ymax * original_image_np.shape[0])

            # Find the class index of the current object
            class_id = int(obj['class_id'])

            # Draw the bounding box and label on the image

            color = [int(c) for c in self.COLORS[class_id]]
            cv2.rectangle(original_image_np, (xmin, ymin), (xmax, ymax), color, 2)
            # Make adjustments to make the label visible for all objects
            y = ymin - 15 if ymin - 15 > 15 else ymin + 15
            label = "{}: {:.0f}%".format(self.labels[class_id], obj['score'] * 100)
            cv2.putText(original_image_np, label, (xmin, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # save the final image
        original_uint8 = original_image_np.astype(np.uint8)
        os.chdir('/home/administrator/Pictures/images_detection')
        cv2.imwrite("detection "+str(time.time())+".jpg", original_uint8)

if __name__ == "__main__":
    tflite_detection_model = "coco_ssd_mobilenet_v3_large_2020_01_14"
    model = TensorflowLiteDetectionModel("/home/administrator/copter_script/tensorflowlite_models/"+tflite_detection_model+"/model.tflite", "/home/administrator/copter_script/tensorflowlite_models/"+tflite_detection_model+"/labels.txt")
    objects = model.run_from_filepath("/home/administrator/image_reco/60232-640x360-parking-london-640.jpg")
    for obj in objects:
        print(obj['class']+"  "+str(obj['class_id']))