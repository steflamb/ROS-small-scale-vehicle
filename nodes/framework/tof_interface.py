#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

video_source = 'http://team10.local:8890/lidar_image'

def lidar_camera_node():
    print("Starting lidar image node")
    cap = cv2.VideoCapture(video_source)
    if not cap.isOpened():
        print("____________________________________")
        print("[CAMERA CLIENT]: Could not open lidar video source.")
    print("Lidar video source successfully opened")
    

    rospy.init_node("lidar_camera_node", anonymous=True)
    pub_camera = None
    bridge = CvBridge()

    stored = False

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            print("____________________________________")
            print("[CAMERA CLIENT]: Failed to capture frame.")
            break
        real_image = frame[:, :, 0]
        if not stored:
            cv2.imwrite("dummy_lidar_photo.jpg",real_image)
            stored = True

        try:
            image_message = bridge.cv2_to_imgmsg(real_image, encoding="mono8")
            #publish camera image
            if pub_camera is None:
                pub_camera = rospy.Publisher("lidar_depth_image", Image,queue_size=10)
            pub_camera.publish(image_message)
        except CvBridgeError as e:
            print(e)

        
        # print("published image")

    print("[CAMERA CLIENT]: QUIT")







if __name__ == '__main__':
    try:
        lidar_camera_node()
    except rospy.ROSInterruptException:
        pass