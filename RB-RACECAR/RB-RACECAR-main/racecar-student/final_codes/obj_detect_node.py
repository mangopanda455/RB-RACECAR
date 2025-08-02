import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String

import numpy as np
import math

from cv_bridge import CvBridge
import cv2
import os
import time

from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference

default_path = './models' # location of model weights and labels
# Define thresholds and number of classes to output
SCORE_THRESH = 0.5
NUM_CLASSES = 3


class ObjectDetNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.model_name = 'direction_data_edgetpu.tflite'
        self.label_name = 'direction_labels.txt'

        self.model_path = default_path + "/" + self.model_name
        self.label_path = default_path + "/" + self.label_name

        self.reset_model()

        # Set up subscriber and publisher nodes
        self.subscription_model_name = self.create_subscription(String, '/model', self.model_name_callback, 10)
        self.subscription_camera = self.create_subscription(Image, '/camera', self.camera_callback, 10)
        self.publisher_object = self.create_publisher(Float32MultiArray, '/object', 10) # output as [roll, pitch, yaw] angles

        self.prev_time = self.get_clock().now() # initialize time checkpoint

        # set up attitude params
        self.image = None

    # [FUNCTION] Called when new camera data is received, attidude calc completed here as well
    def camera_callback(self, data):
        self.image = data
        object_msg =  self.get_object(self.image)
        print("msg", object_msg)
        self.publisher_object.publish(object_msg)

    def model_name_callback(self, data):
        names = data.data.split(':')
        self.model_name = names[0]
        self.label_name = names[1]
        self.model_path = default_path + "/" + self.model_name
        self.label_path = default_path + "/" + self.label_name
        self.reset_model()

    def reset_model(self):
        self.interpreter = make_interpreter(self.model_path)
        self.interpreter.allocate_tensors()
        self.labels = read_label_file(self.label_path)
        self.inference_size = input_size(self.interpreter)
        print("Model set to ", self.model_path)         

    def get_object(self, image):
        bridge = CvBridge()
        cv_image = np.frombuffer(image.data, np.uint8)
        frame = cv2.imdecode(cv_image, cv2.IMREAD_COLOR)
    

        time_cp1 = time.time()
        if frame is None:
            objs = None
            print("Frame Not Found")
        else:
        
            # STEP 4: Preprocess image to the size and shape accepted by model
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rgb_image = cv2.resize(rgb_image, self.inference_size)

            # STEP 5: Let the model do the work
            run_inference(self.interpreter, rgb_image.tobytes())
        
            # STEP 6: Get objects detected from the model
            objs = get_objects(self.interpreter, SCORE_THRESH)[:NUM_CLASSES]
            time_cp2 = time.time()

            fps = round(1/(time_cp2 - time_cp1), 2)
            print("fps: ", fps)

        max_score = 0
        real_object = None
        if objs is not None:
            for object in objs:
                if object[1] > max_score:
                    max_score = object[1]
                    real_object = object
        object = Float32MultiArray()
        if real_object is not None:
            scale_x, scale_y = 640 / self.inference_size[0], 480 / self.inference_size[1]
            bbox = real_object.bbox.scale(scale_x, scale_y)
            x0, x1 = bbox.xmin, bbox.xmax
            y0, y1 = bbox.ymin, bbox.ymax
            object_array = np.array([real_object[0], real_object[1], x0, y0, x1, y1, self.inference_size[0], self.inference_size[1]])
        else:
            object_array = np.array([0.0,0.0,0.0,0.0,0.0,0.0, 0.0, 0.0])
        object.data = object_array.tolist()
        return object


def main():
    rclpy.init(args=None)
    node = ObjectDetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()