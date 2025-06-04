#!/usr/bin/env python3

import rospy

import tensorflow as tf
from tensorflow import keras
from keras.models import load_model

import numpy as np

import cv2

from PIL import Image

from sensor_msgs.msg import Image as SensorImage
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from mixed_reality.msg import Control

from cv_bridge import CvBridge

import time
import os
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"




def parse_model_outputs(outputs):
        res = []
        for i in range(outputs.shape[1]):
            res.append(outputs[0][i])

        return res

MODEL_PATH = rospy.get_param("model_path")
FIXED_THROTTLE = rospy.get_param("fixed_throttle")
STEERING = 0
THROTTLE = 1

model = None
pub_throttle_steering = None

bridge = CvBridge()


prev_time = None
def new_image(msg):
    global prev_time
    if prev_time is None:
         prev_time = time.time()
    elif time.time()-prev_time > 0.2:
        prev_time = time.time()
    else:
         return






    global STEERING
    global THROTTLE
    global FIXED_THROTTLE
    global model
    global pub_throttle_steering
    global bridge


    image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    # cv2.imshow("Received mixed image", image)
    # cv2.waitKey(0)

    image = image[None, ...]
    outputs = model.predict(image, verbose=0)
    parsed_outputs = parse_model_outputs(outputs)
    print(parsed_outputs)
    steering = 0.
    throttle = 0.
    if len(parsed_outputs) > 0:        
        # TODO TOPIC 2.1 Strg multi
        steering = parsed_outputs[STEERING]*1.4
        throttle = parsed_outputs[THROTTLE]
    if FIXED_THROTTLE:
            throttle = 1.

    if pub_throttle_steering is None:
        pub_throttle_steering = rospy.Publisher("model/throttle_steering", Control, queue_size=10)
    msg = Control(throttle, steering, False, False, False)
    pub_throttle_steering.publish(msg)
    #print("commands sent")
    



def e2e_model():
    print("Starting model node")
    global MODEL_PATH
    global model
    
    print(f"[MODEL LOOP]____________________________________\nLoading model: {MODEL_PATH}\n____________________________________")
    





    
    model = load_model(MODEL_PATH, compile=False)
    model.compile(loss="sgd", metrics=["mse"])

    print("MODEL COMPILED")

    rospy.init_node("e2e_model", anonymous=True)

    if rospy.get_param("sim"):
        rospy.Subscriber("sim/image", SensorImage, new_image)
    elif rospy.get_param("mixed"):
        rospy.Subscriber("mixed_image", SensorImage, new_image)
    else:
        rospy.Subscriber("camera", SensorImage, new_image)



    # SIM CAMERA OPTION
    # rospy.Subscriber("sim/image", SensorImage, new_image)

    # REAL CAMERA OPTION
    # rospy.Subscriber("camera", SensorImage, new_image)

    # SEG CAMERA OPTION
    # rospy.Subscriber("sim/image_seg", SensorImage, new_image)

    # MIXED CAMERA OPTION
    # rospy.Subscriber("mixed_image", SensorImage, new_image)


    
    rospy.spin()





if __name__ == '__main__':
    try:
        e2e_model()
    except rospy.ROSInterruptException:
            pass
