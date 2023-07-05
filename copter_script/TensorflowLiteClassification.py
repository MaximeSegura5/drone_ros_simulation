import tensorflow as tf
import numpy as np
from PIL import Image
import time
import cv2


def load_labels(filename):
  with open(filename, 'r') as f:
    return [line.strip() for line in f.readlines()]

class TensorflowLiteClassificationModel:
    def __init__(self, model_path, labels_path, image_size=224):
        self.interpreter = tf.lite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.labels = load_labels(labels_path)
        self.input_mean = 127.5
        self.input_std = 127.5
        self.probability_threshold = 0.3

         # check the type of the input tensor
        self.floating_model = self.input_details[0]['dtype'] == np.float32

        # NxHxWxC, H:1, W:2
        self.height = self.input_details[0]['shape'][1]
        self.width = self.input_details[0]['shape'][2]

    def run_from_filepath(self, image_path):
        
        image = np.array(Image.open(image_path).resize((self.width, self.height)))
        # add N dim
        input_data = np.expand_dims(image, axis=0)
        
        if self.floating_model:
            input_data = (np.float32(input_data) - self.input_mean) / self.input_std

        return self.run(input_data)

    def run_from_cvimage(self, cvimage):
        image = np.array(Image.fromarray(cv2.cvtColor(cvimage, cv2.COLOR_BGR2RGB)).resize((self.width, self.height)))
        # add N dim
        input_data = np.expand_dims(image, axis=0)
        
        if self.floating_model:
            input_data = (np.float32(input_data) - self.input_mean) / self.input_std

        return self.run(input_data)

    def run(self, input_data):
        """
        Returns dict of [Label: Probability]
        """

        self.interpreter.set_tensor(self.input_details[0]["index"], input_data)
        start_time = time.time()
        self.interpreter.invoke()
        stop_time = time.time()
        output_data = self.interpreter.get_tensor(self.output_details[0]["index"])
        results = np.squeeze(output_data)

        top_k = results.argsort()[-5:][::-1]
        print('time: {:.3f}ms'.format((stop_time - start_time) * 1000))
        objects_found = {}
        for i in top_k:
            print(results[i])
            if self.floating_model:
                if float(results[i])>self.probability_threshold:
                    objects_found[self.labels[i]] = float(results[i])
            else:
                if float(results[i]/ 255.0)>self.probability_threshold:
                    objects_found[self.labels[i]] = float(results[i]/ 255.0)
        return(objects_found)


# if __name__ == "__main__":
#     model = TensorflowLiteClassificationModel("/home/administrator/copter_script/model.tflite", "/home/administrator/copter_script/labels.txt")
#     objects = model.run_from_filepath("/home/administrator/copter_script/grace_hopper.bmp")
#     print(objects)