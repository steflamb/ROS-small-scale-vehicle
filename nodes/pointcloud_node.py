#!/usr/bin/env python3

import rospy
from sensor_msgs import point_cloud2
from std_msgs.msg import Float64MultiArray, Header
from sensor_msgs.msg import PointCloud2, PointField, Image

import numpy as np
import open3d as o3d

from cv_bridge import CvBridge
bridge = CvBridge()

"""
use the following command in terminal to be able to use rviz
rosrun tf static_transform_publisher 0 0 0 0 0 2 map lidar 100
rosrun tf static_transform_publisher 0 0 0 1.87 0 1.08 lidar_center lidar 100
rosrun tf static_transform_publisher 5 0 0.5 1.87 0 1.04 lidar_center lidar 100

rosrun tf static_transform_publisher 4 0 0 0 1.6 0 lidar_center lidar 100




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
        x = (i - intrinsic_mat[0, 2]/4) * z / intrinsic_mat[0, 0]
        y = (j - intrinsic_mat[1, 2]/4) * z / intrinsic_mat[1, 1]
        # x=i/100
        # y=j/100
        # x = (i - 100) * z / intrinsic_mat[0, 0]
        # y = (j - 0) * z / intrinsic_mat[1, 1]

        scaling_factor = 16  #factor = 16 for donkeycar
        z = z.flatten()*scaling_factor/2
        x = x.astype(np.float32).flatten()*scaling_factor
        y = y.astype(np.float32).flatten()*scaling_factor
        intensity = np.zeros((256,192)).astype(np.float32).flatten()

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




def pointcloud_node():
    global depth_data
    global depth_image
    print("Starting point cloud node")

    rospy.init_node("pointcloud_node", anonymous=True)
    if READ_AS_IMAGE:
        rospy.Subscriber("/lidar_depth_image", Image, new_image)
    else:
        rospy.Subscriber("lidar/depth_map", Float64MultiArray, new_depthmap)
    pub_cloud = rospy.Publisher("lidar/pointcloud", PointCloud2, queue_size=10)
    rate = rospy.Rate(2)   #pointcloud should not be published so often

    while not rospy.is_shutdown():

        if TESTING:
            depth_data = np.load("dummy_depth_data.npy")
            print("loaded dummy depth data")
            print([type(depth_data), depth_data.shape])

        if READ_AS_IMAGE:
            if depth_image is None:
                continue
            msg = generate_point_cloud((depth_image/255)*10, None)
            pub_cloud.publish(msg)
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