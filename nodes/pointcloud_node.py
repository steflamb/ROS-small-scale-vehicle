#!/usr/bin/env python3

import rospy
from sensor_msgs import point_cloud2
from std_msgs.msg import Float64MultiArray, Header
from sensor_msgs.msg import PointCloud2, PointField

import numpy as np
import open3d as o3d

"""
use the following command in terminal to be able to use rviz
rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map mymap 0
rosrun tf static_transform_publisher 0 0 0 1.87 0 1.08 sim/car lidar 100
"""

depth_data = None
TESTING = False
data_captured = False

def new_depthmap(msg):
    global depth_data
    global data_captured
    depth_data = np.array(msg.data)

    if not data_captured:
        np.save("dummy_depth_data.npy", np.array(msg.data))
        print("captured data")
        data_captured = True

def generate_point_cloud(depth, intrinsic_mat):
        h, w = depth.shape
        i, j = np.meshgrid(np.arange(w), np.arange(h), indexing='xy')
        z = depth.astype(np.float32)
        x = (i - intrinsic_mat[0, 2]) * z / intrinsic_mat[0, 0]
        y = (j - intrinsic_mat[1, 2]) * z / intrinsic_mat[1, 1]

        z = z.flatten()
        x = x.astype(np.float32).flatten()
        y = y.astype(np.float32).flatten()
        intensity = np.zeros((192,256)).astype(np.float32).flatten()

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


        # points = np.stack((x, y, z, intensity), axis=-1).reshape(-1, 3)
        # print(points.shape)

        # point_cloud = o3d.geometry.PointCloud()
        # point_cloud.points = o3d.utility.Vector3dVector(points)
        # o3d.visualization.draw_geometries([point_cloud])

        msg = PointCloud2()
        header = Header()
        header.frame_id = "lidar"
        msg.header = header
        msg.height = 192
        msg.width = 256
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                      PointField('y', 8, PointField.FLOAT32, 1),
                      PointField('z', 16, PointField.FLOAT32, 1)]
                    #    PointField('rgb', 24, PointField.FLOAT64, 1)]
        msg.is_bigendian = False
        msg.point_step = len(msg.fields)*8
        msg.row_step = msg.width*msg.point_step
        msg.data = np.concatenate((x.flatten(),y.flatten(),z.flatten())).tobytes()
        msg.is_dense = False



        msg = point_cloud2.create_cloud(header, fields, points)

        return msg




def pointcloud_node():
    global depth_data
    print("Starting point cloud node")

    rospy.init_node("pointcloud_node", anonymous=True)
    if not TESTING:
        rospy.Subscriber("lidar/depth_map", Float64MultiArray, new_depthmap)
    pub_cloud = rospy.Publisher("lidar/pointcloud", PointCloud2, queue_size=10)
    rate = rospy.Rate(1)   #pointcloud should not be published so often

    while not rospy.is_shutdown():

        if TESTING:
            depth_data = np.load("dummy_depth_data.npy")
            print("loaded dummy depth data")
            print([type(depth_data), depth_data.shape])



        if depth_data is None:
            continue
        
        intrinsic_matrix = depth_data[:9].reshape((3,3))
        depth_map = depth_data[9:].reshape((192,256))

        msg = generate_point_cloud(depth_map, intrinsic_matrix)
        pub_cloud.publish(msg)

        rate.sleep()

    



if __name__ == '__main__':
    try:
        pointcloud_node()
    except rospy.ROSInterruptException:
        pass