#!/usr/bin/env python3

import rospy
from sensor_msgs import point_cloud2
from std_msgs.msg import Float64MultiArray, Header
from sensor_msgs.msg import PointCloud2, PointField, Image
from geometry_msgs.msg import TransformStamped
import numpy as np
import open3d as o3d

from cv_bridge import CvBridge
bridge = CvBridge()
from tf2_ros import TransformBroadcaster
import rospy
import tf2_ros
import tf_conversions

"""
use the following command in terminal to be able to use rviz
rosrun tf static_transform_publisher 0 0 0 0 0 2 map lidar 100
rosrun tf static_transform_publisher 0 0 0 1.87 0 1.08 lidar_center lidar 100
rosrun tf static_transform_publisher 5 0 0.5 1.87 0 1.04 lidar_center lidar 100

rosrun tf static_transform_publisher 4 0 0 0 1.6 0 lidar_center lidar 100

# STEFANO RUN
rosrun tf static_transform_publisher 4 0 -0.1 0 1.58 0 lidar_center lidar 100
rosrun tf static_transform_publisher 0 0 0 0 0 0 base_link lidar_center 100


for the actually good bag:
rosrun tf static_transform_publisher 4 0 0 0 1.59 0 lidar_center lidar 100
rosrun tf static_transform_publisher 0 0 0 0 0 0 base_link lidar_center 100



for the person and chair bag:
rosrun tf static_transform_publisher 4 0 1.5 1.57 0 1.46 lidar_center lidar 100
rosrun tf static_transform_publisher 0 0 0 0 0 0 base_link lidar_center 100
"""

depth_data = None
TESTING = False
data_captured = False
# TODO: move this to param file later
READ_AS_IMAGE = True

def new_depthmap(msg):
    global depth_data
    global data_captured
    depth_data = np.array(msg.data)

    # if not data_captured:
    #     np.save("dummy_depth_data.npy", np.array(msg.data))
    #     print("captured data")
    #     data_captured = Truerostopic hz

depth_image=None
def new_image(msg):
    global depth_image
    global bridge
    depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

def generate_point_cloud(depth, intrinsic_mat):
        if READ_AS_IMAGE:
            intrinsic_mat = np.array([[668.42816162,0,360.90789795],[0,668.42816162,480.72705078],[0,0,1]])
        h, w = depth.shape
        i, j = np.meshgrid(np.arange(w), np.arange(h), indexing='xy')
        z = depth.astype(np.float32)
        x = (i - intrinsic_mat[0, 2]/4) * z / intrinsic_mat[0, 0]*4
        y = (j - intrinsic_mat[1, 2]/4) * z / intrinsic_mat[1, 1]*4
        # x=i/100
        # y=j/100
        # x = (i - 100) * z / intrinsic_mat[0, 0]
        # y = (j - 0) * z / intrinsic_mat[1, 1]

        z = z.flatten()
        x = x.astype(np.float32).flatten()
        y = y.astype(np.float32).flatten()
        intensity = np.zeros((256,192)).astype(np.float32).flatten()

        min_range = 0
        distances = np.sqrt(x**2 + y**2 + z**2)
        mask = distances >= min_range

        # Keep only the points that are >= min_range
        x = x[mask]
        y = y[mask]
        z = z[mask]
        intensity = intensity[mask]

        points = []
        # intensity = np.array
        for i in range(0,len(x)):
            points.append([x[i], y[i],z[i],z[i]])

        header = Header()
        header.frame_id = "lidar"
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                      PointField('y', 8, PointField.FLOAT32, 1),
                      PointField('z', 16, PointField.FLOAT32, 1),
                      PointField('intensity', 24, PointField.FLOAT64, 1)]
        msg = point_cloud2.create_cloud(header, fields, points)

        return msg


        # points = np.stack((x, y, z), axis=-1).reshape(-1, 3)
        # print(points.shape)
        # point_cloud = o3d.geometry.PointCloud()
        # point_cloud.points = o3d.utility.Vector3dVector(points)
        # coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0,0,0])
        # o3d.visualization.draw_geometries([point_cloud, coordinate_frame])

        # msg = PointCloud2()
        # header = Header()
        # header.frame_id = "lidar"
        # msg.header = header
        # msg.height = 256
        # msg.width = 192
        # fields = [PointField('x', 0, PointField.FLOAT32, 1),
        #               PointField('y', 8, PointField.FLOAT32, 1),
        #               PointField('z', 16, PointField.FLOAT32, 1)]
        #             #    PointField('rgb', 24, PointField.FLOAT64, 1)]
        # msg.is_bigendian = False
        # msg.point_step = len(msg.fields)*8
        # msg.row_step = msg.width*msg.point_step
        # msg.data = np.concatenate((x.flatten(),y.flatten(),z.flatten())).tobytes()
        # msg.is_dense = False



        # msg = point_cloud2.create_cloud(header, fields, points)

        return msg


def publish_base_link_to_lidar_center(tb, stamp):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = "base_link"
    t.child_frame_id = "lidar_center"
    
    # Translation: x=0, y=0, z=0
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    
    # Rotation for roll=0, pitch=0, yaw=0 → quaternion = [0, 0, 0, 1]
    q = tf_conversions.transformations.quaternion_from_euler(0, -0.05, -0.1)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    
    # Publish the transform using the given TransformBroadcaster
    tb.sendTransform(t)

def publish_base_link_sim_to_lidar_center(tb, stamp):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = "base_link_sim"
    t.child_frame_id = "lidar_center"
    
    # Translation: x=0, y=0, z=0
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0
    
    # Rotation for roll=0, pitch=0, yaw=0 → quaternion = [0, 0, 0, 1]
    q = tf_conversions.transformations.quaternion_from_euler(0, 0.0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    
    # Publish the transform using the given TransformBroadcaster
    tb.sendTransform(t)

def publish_lidar_center_to_lidar(tb,stamp):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id= "lidar_center"
    t.child_frame_id = "lidar"

    t.transform.translation.x= 0.2
    t.transform.translation.y=-0.1
    t.transform.translation.z= -0.12
    q = tf_conversions.transformations.quaternion_from_euler(0,1.7,0.1)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    tb.sendTransform(t)






def pointcloud_node():
    global depth_data
    global depth_image
    print("Starting point cloud node")

    rospy.init_node("mixed_pointcloud_generation", anonymous=True)
    if READ_AS_IMAGE:
        rospy.Subscriber("/mixed_depth_image", Image, new_image)
    else:
        rospy.Subscriber("lidar/depth_map", Float64MultiArray, new_depthmap)
    pub_cloud = rospy.Publisher("lidar/pointcloud_mixed", PointCloud2, queue_size=10)
    
    rate = rospy.Rate(1)   #pointcloud should not be published so often


    tb = TransformBroadcaster()

    while not rospy.is_shutdown():

        stamp = rospy.Time.now()
        publish_base_link_to_lidar_center(tb,stamp)
        publish_lidar_center_to_lidar(tb,stamp)
        publish_base_link_sim_to_lidar_center(tb,stamp)
        

        if TESTING:
            depth_data = np.load("dummy_depth_data.npy")
            print("loaded dummy depth data")
            print([type(depth_data), depth_data.shape])

        if READ_AS_IMAGE:
            if depth_image is None:
                continue
            msg = generate_point_cloud((depth_image/255)*5, None)
            pub_cloud.publish(msg)
            print("here")
        else:
            if depth_data is None:
                continue
            intrinsic_matrix = depth_data[:9].reshape((3,3))
            depth_map = depth_data[9:].reshape((256,192))
            msg = generate_point_cloud(depth_map, intrinsic_matrix)
            pub_cloud.publish(msg)
            
        
        

        rate.sleep()

    



if __name__ == '__main__':
    try:
        pointcloud_node()
    except rospy.ROSInterruptException:
        pass