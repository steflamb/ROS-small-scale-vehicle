#!/usr/bin/env python3
import rospy
import math

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
from mixed_reality.msg import SimPose
from mixed_reality.utils.for_conversions import For_convertion_utils


def handle_sim_pose(msg):
    # print("handling sim pose")
    #TODO: double check the transformation fromsim to real coordenates
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    sim_pose = [msg.x, msg.y, msg.z]
    sim_orientation = [0,msg.yaw,0]
    real_pose = for_conversions.sim2real_xyzypr(sim_pose, sim_orientation)   #[x,y,angle]

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "sim/car"
    t.transform.translation.x = real_pose[0]
    t.transform.translation.y = real_pose[1]
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, math.radians(real_pose[2]))
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

def handle_real_pose(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "real/car"
    t.transform.translation.x = msg.pose.position.x
    t.transform.translation.y = msg.pose.position.y
    t.transform.translation.z = msg.pose.position.z
    t.transform.rotation.x = msg.pose.orientation.x
    t.transform.rotation.y = msg.pose.orientation.y
    t.transform.rotation.z = msg.pose.orientation.z
    t.transform.rotation.w = msg.pose.orientation.w

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_broadcaster')
    TRACKING = rospy.get_param("tracking")
    MAPPING = rospy.get_param("mapping")
    SIZE_FACTOR = rospy.get_param("size_factor")
    X_MAP_SHIFT = rospy.get_param("x_map_shift")
    Y_MAP_SHIFT = rospy.get_param("y_map_shift")
    ANGLE_SHIFT = rospy.get_param("angle_shift")
    for_conversions = For_convertion_utils(SIZE_FACTOR,X_MAP_SHIFT,Y_MAP_SHIFT,ANGLE_SHIFT)
    if not MAPPING:
        rospy.Subscriber("/sim/euler", SimPose, handle_sim_pose)
    if TRACKING:
        rospy.Subscriber("/donkey/pose", geometry_msgs.msg.PoseStamped, handle_real_pose)
    rospy.spin()
