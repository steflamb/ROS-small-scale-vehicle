#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image as SensorImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# Bridge and global variables
bridge = CvBridge()
sim_depth = None
real_depth = None
sim_header = None
real_header = None

# Callback for simulator depth images
def new_sim_depth(msg):
    global sim_depth, sim_header
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        sim_header = msg.header
        sim_depth = depth_image.astype(np.float32)
    except CvBridgeError as e:
        rospy.logerr(f"[depth_mixer_node] Failed to convert sim depth: {e}")

# Callback for real lidar depth images
def new_real_depth(msg):
    global real_depth, real_header
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        real_header = msg.header
        real_depth = depth_image.astype(np.float32)
    except CvBridgeError as e:
        rospy.logerr(f"[depth_mixer_node] Failed to convert real depth: {e}")

# Main mixer node
def mixer_node():
    rospy.init_node('depth_mixer_node', anonymous=True)
    rospy.loginfo('[depth_mixer_node] Starting depth mixer...')
    rospy.Subscriber('/sim/image_depth', SensorImage, new_sim_depth)
    rospy.Subscriber('/lidar_depth_image', SensorImage, new_real_depth)
    pub_mixed = rospy.Publisher('/mixed_depth_image', SensorImage, queue_size=10)
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        if sim_depth is not None and real_depth is not None:
            if sim_depth.shape != real_depth.shape:
                rospy.logwarn(f"[depth_mixer_node] Shape mismatch: sim {sim_depth.shape}, real {real_depth.shape}")
            else:
                # Blend: choose the closer (smaller) depth
                mask = (sim_depth > 0) & (sim_depth < real_depth)
                mixed = np.where(mask, sim_depth, real_depth)
                # Convert to uint8
                mixed_uint8 = np.clip(mixed, 0, 255).astype(np.uint8)
                # Manually construct ROS Image message to ensure mono8
                img_msg = SensorImage()
                img_msg.header = real_header if real_header is not None else sim_header
                img_msg.height = mixed_uint8.shape[0]
                img_msg.width = mixed_uint8.shape[1]
                img_msg.encoding = 'mono8'
                img_msg.is_bigendian = 0
                img_msg.step = mixed_uint8.shape[1]
                img_msg.data = mixed_uint8.tobytes()
                pub_mixed.publish(img_msg)
        rate.sleep()

    rospy.loginfo('[depth_mixer_node] Shutting down')

if __name__ == '__main__':
    try:
        mixer_node()
    except rospy.ROSInterruptException:
        pass
