#!/usr/bin/env python3

import rospy
import numpy as np
import open3d as o3d

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import struct
import random
from mixed_reality.msg import SimPose, Obstacles

from visualization_msgs.msg import Marker
import tf2_geometry_msgs
from mixed_reality.utils.for_conversions import For_convertion_utils
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
import time




SIZE_FACTOR = rospy.get_param("size_factor")
X_MAP_SHIFT = rospy.get_param("x_map_shift")
Y_MAP_SHIFT = rospy.get_param("y_map_shift")
ANGLE_SHIFT = rospy.get_param("angle_shift")
for_conversions = For_convertion_utils(SIZE_FACTOR,X_MAP_SHIFT,Y_MAP_SHIFT,ANGLE_SHIFT)


class pointcloud_processing_node:
    def __init__(self):
        global for_conversions
        # Initialize ROS node
        rospy.init_node("pointcloud_processor", anonymous=True)

        # Parameters
        self.frame_id = rospy.get_param("~frame_id", "lidar")
        
        # -- Z-Filter Param (APPLIED ONLY AFTER PLANE REMOVAL) --
        self.min_z_filter = rospy.get_param("~min_z_filter", -3)
        self.max_z_filter = rospy.get_param("~min_z_filter", 2)

        # RANSAC plane segmentation params
        self.plane_distance_thresh = rospy.get_param("~plane_distance_thresh", 0.05)
        self.ransac_n             = rospy.get_param("~ransac_n", 3)
        self.num_iterations       = rospy.get_param("~num_iterations", 700)

        # DBSCAN clustering
        self.cluster_eps           = rospy.get_param("~cluster_eps", 0.1)
        self.min_points_per_cluster= rospy.get_param("~min_points_per_cluster", 100)

        self.min_range_filter = rospy.get_param("~min_range_filter", 0.)

        # Subscriber
        self.sub = rospy.Subscriber("/sim/pointcloud", PointCloud2,
                                    self.pc_callback, queue_size=1)

        # Publishers
        self.pub_clusters  = rospy.Publisher("/cloud_clusters_sim", PointCloud2, queue_size=1)
        self.pub_centroids = rospy.Publisher("/cloud_centroids_sim", MarkerArray, queue_size=1)
        self.obstacle_pub = rospy.Publisher("/obstacles_lidar_sim", Obstacles, queue_size=1)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.prev = time.time()

    def remove_ground_planes(self, pcd,
                            distance_thresh=None,
                            ransac_n=None,
                            num_iterations=None,
                            min_inliers=None,
                            max_tilt_deg=15.0):
        """
        Iteratively segment & remove nearly-horizontal planes from `pcd`
        with debug logging so you can see why it stops.
        """
        # fallbacks
        d_thresh = distance_thresh     or self.plane_distance_thresh
        ransac_n = ransac_n            or self.ransac_n
        n_iters   = num_iterations     or self.num_iterations
        # if user didn’t override, allow even small planes
        min_pts   = min_inliers       if min_inliers is not None else max(200, len(pcd.points)//20)
        cos_tilt  = np.cos(np.deg2rad(max_tilt_deg))

        rospy.loginfo(f"[ground peel] starting with {len(pcd.points)} pts, "
                    f"d_thresh={d_thresh:.3f}, min_pts={min_pts}, max_tilt={max_tilt_deg}°")

        remaining = pcd
        iteration = 0
        while True:
            iteration += 1
            model, inliers = remaining.segment_plane(
                distance_threshold=d_thresh,
                ransac_n=ransac_n,
                num_iterations=n_iters)

            num_inliers = len(inliers)
            # a) too few inliers?
            if num_inliers < min_pts:
                rospy.loginfo(f"[ground peel] it#{iteration}: only {num_inliers} inliers (< {min_pts}) → stop")
                break

            # b) check normal tilt
            a, b, c, _ = model
            normal = np.array([a, b, c])
            normal /= np.linalg.norm(normal)
            tilt_deg = np.rad2deg(np.arccos(abs(normal[2])))

            if tilt_deg > max_tilt_deg:
                rospy.loginfo(f"[ground peel] it#{iteration}: plane tilt {tilt_deg:.1f}° > {max_tilt_deg}° → stop")
                break

            rospy.loginfo(f"[ground peel] it#{iteration}: removing plane of {num_inliers} pts, tilt {tilt_deg:.1f}°")
            remaining = remaining.select_by_index(inliers, invert=True)

        rospy.loginfo(f"[ground peel] done, {len(remaining.points)} pts remain")
        return remaining

    def remove_wedge(self, pcd, angle_min_deg=-90, angle_max_deg=-30):
        """
        Remove any point whose yaw (atan2(y,x)) lies between angle_min_deg and angle_max_deg.
        """
        pts = np.asarray(pcd.points)
        angles = np.degrees(np.arctan2(pts[:,1], pts[:,0]))
        # wrap angles into [-180,180]
        angles = (angles + 180) % 360 - 180
        mask = (angles < angle_min_deg) | (angles > angle_max_deg)
        return pcd.select_by_index(np.where(mask)[0])
    
    def remove_far_points(self, pcd, max_range):
        pts = np.asarray(pcd.points)
        d2 = np.sum(pts**2, axis=1)
        mask = d2 <= max_range**2
        return pcd.select_by_index(np.where(mask)[0])
    

    def crop_box(self, pcd,
                xlim=(-5, 5),
                ylim=(-5, 5),
                zlim=(0, 2)):
        pts = np.asarray(pcd.points)
        mask = (
            (pts[:,0] >= xlim[0]) & (pts[:,0] <= xlim[1]) &
            (pts[:,1] >= ylim[0]) & (pts[:,1] <= ylim[1]) &
            (pts[:,2] >= zlim[0]) & (pts[:,2] <= zlim[1])
        )
        return pcd.select_by_index(np.where(mask)[0])
        
    def pc_callback(self, msg):
        barrier_pub=[None,None,None,None,None]
        """Callback each time we receive a pointcloud from /lidar/pointcloud."""
        # 1) Convert ROS -> Open3D
        o3d_cloud = self.ros_to_open3d(msg)
        # o3d_cloud = self.remove_far_points(o3d_cloud, 3)
        o3d_cloud = self.crop_box(o3d_cloud, xlim=(-0.5,1), ylim=(-10,10), zlim=(0,3))
        o3d_cloud
        if len(o3d_cloud.points) == 0:
            rospy.logwarn("Received empty cloud.")
            now = time.time()
            if now-self.prev>0.02:
                # self.pub_centroids.publish(marker_array)
                self.obstacle_pub.publish([])
                self.prev = now
                return

        # # # 2) Plane segmentation (NO Z-filter here)
        # plane_model, inliers = o3d_cloud.segment_plane(
        #     distance_threshold=self.plane_distance_thresh,
        #     ransac_n=self.ransac_n,
        #     num_iterations=self.num_iterations
        # )
        # if len(inliers) == 0:
        #     rospy.logwarn("No plane found; publishing original as no-ground.")
        #     cloud_noground = o3d_cloud
        # else:
        #     # Remove inliers → ground plane
        #     cloud_noground = o3d_cloud.select_by_index(inliers, invert=True)

        



        

        if len(o3d_cloud.points) < 1:
            rospy.loginfo("No points above ground; skipping clustering.")
            obstacles=[]
            # for i in range(len(obstacles)-1,3):
            #     obstacles.append(SimPose(str(i), 100, 100, 0, 0, 0, 0))
            
            now = time.time()
            if now-self.prev>0.02:
                self.obstacle_pub.publish(obstacles)
                self.prev = now
                return

        # 3) NOW apply Z-filter only for clustering
        points_noground = np.asarray(o3d_cloud.points)
        zmask = points_noground[:, 2] >= self.min_z_filter
        filtered_points = points_noground[zmask]
        zmask = filtered_points[:, 2] <= self.max_z_filter
        filtered_points = filtered_points[zmask]

        if len(filtered_points) == 0:
            rospy.loginfo("All no-ground points are below min_z_filter; nothing to cluster.")
            return

        cluster_cloud = o3d.geometry.PointCloud()
        cluster_cloud.points = o3d.utility.Vector3dVector(filtered_points)

        # 4) Cluster the filtered no-ground cloud (DBSCAN)
        labels = np.array(cluster_cloud.cluster_dbscan(
            eps=self.cluster_eps,
            min_points=self.min_points_per_cluster,
            print_progress=False
        ))
        max_label = labels.max()
        if max_label < 0:
            rospy.loginfo("No clusters found (all outliers).")
            return

        # 5) Build a colored point cloud
        clusters_colored = []
        np_points_cluster = np.asarray(cluster_cloud.points)
        for i in range(len(np_points_cluster)):
            lbl = labels[i]
            # Outlier or cluster
            if lbl == -1:
                # color outliers dark (gray)
                r, g, b = 0.3, 0.3, 0.3
            else:
                # random color based on label
                random.seed(lbl)  # ensures consistent color for same label
                r = random.random()
                g = random.random()
                b = random.random()
            x, y, z = np_points_cluster[i]
            rgb_packed = self.pack_rgb(r, g, b)
            clusters_colored.append([x, y, z, rgb_packed])

        # Convert to a new sensor_msgs/PointCloud2
        ros_clusters = self.create_colored_pc2(
            clusters_colored, frame_id=self.frame_id
        )
        self.pub_clusters.publish(ros_clusters)

        # 6) Compute and publish centroids
        marker_array = MarkerArray()
        now = rospy.Time.now()
        
        cluster_info_list = []
        for cluster_id in range(max_label + 1):
            idx = np.where(labels == cluster_id)[0]
            cluster_pts = np_points_cluster[idx]
            cluster_size = len(cluster_pts)
            cx, cy, cz = cluster_pts.mean(axis=0)
            cluster_info_list.append((cluster_id, cluster_size, cx, cy, cz))
        # cluster_info_list.sort(key=lambda x: x[1], reverse=True)
        obstacles = []
        for i,item in enumerate(cluster_info_list):
            cluster_id, cluster_size, cx, cy, cz = item

            # Create marker (still in the original frame) for visualization
            mk = self.create_sphere_marker(cluster_id, (cx, cy, cz),
                                        frame_id=self.frame_id,
                                        stamp=now)
            marker_array.markers.append(mk)
            
            
            
            # Create your point in the lidar frame.
            point_lidar = PointStamped()
            point_lidar.header.stamp = rospy.Time.now()
            point_lidar.header.frame_id = "lidar"
            point_lidar.point.x = cx
            point_lidar.point.y = cy
            point_lidar.point.z = cz


            from geometry_msgs.msg import PoseStamped

            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "lidar"
            pose_msg.pose.position.x = cx
            pose_msg.pose.position.y = cy
            pose_msg.pose.position.z = cz
            pose_msg.pose.orientation.x = 0
            pose_msg.pose.orientation.y = 0
            pose_msg.pose.orientation.z = 0
            pose_msg.pose.orientation.w = 0

            tf_buffer = tf2_ros.Buffer()
            tf_listener = tf2_ros.TransformListener(tf_buffer)
            try:
                # Lookup a transform from "lidar" to "map". Use rospy.Time(0) for latest available transform.
                transform = tf_buffer.lookup_transform("map", "lidar", rospy.Time(0), rospy.Duration(1.0))
                # Transform pose from lidar frame to map frame.
                transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_msg, transform)
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn("TF transform error: %s", e)
                # If transform fails, you might want to skip publishing or publish the original pose
                transformed_pose = pose_msg  # fallback; not recommended in production



            if barrier_pub[i] is None:
                barrier_pub[i] = rospy.Publisher(f'barrier_lidar{i}/pose', PoseStamped, queue_size=1)
            barrier_pub[i].publish(transformed_pose)
            
            
            

            
            x, y, a = for_conversions.real2sim_xyp([transformed_pose.pose.position.x, transformed_pose.pose.position.y, 0.0])
            print(str(i),x,y)

            obstacles.append(SimPose(str(i), x,y , z, 0, 0, 0))
        # for i in range(len(obstacles)-1,3):
        #     obstacles.append(SimPose(str(i), 100, 100, 0, 0, 0, 0))
        
        now = time.time()
        if now-self.prev>0.02:
            self.pub_centroids.publish(marker_array)
            self.obstacle_pub.publish(obstacles[:1])
            self.prev = now

    # --------------------------------------------------------------------------
    # Helper Methods
    # --------------------------------------------------------------------------

    def ros_to_open3d(self, ros_cloud):
        """Convert ROS PointCloud2 to Open3D PointCloud."""
        field_names = [f.name for f in ros_cloud.fields]
        # We assume fields: x, y, z
        cloud_data = list(point_cloud2.read_points(
            ros_cloud, skip_nans=True, field_names=("x", "y", "z")))
        if len(cloud_data) == 0:
            return o3d.geometry.PointCloud()
        xyz = np.array(cloud_data, dtype=np.float32).reshape(-1,3)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        return pcd

    def open3d_to_ros(self, pcd, frame_id="lidar"):
        """Convert an Open3D PointCloud to ROS PointCloud2 (XYZ only)."""
        xyz = np.asarray(pcd.points)
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id
        
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]
        return point_cloud2.create_cloud(header, fields, xyz)

    def create_colored_pc2(self, points_with_color, frame_id="lidar"):
        """
        Given an array of [x, y, z, rgb_float],
        create a sensor_msgs/PointCloud2 with those fields.
        """
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id

        fields = [
            PointField('x',   0,  PointField.FLOAT32, 1),
            PointField('y',   4,  PointField.FLOAT32, 1),
            PointField('z',   8,  PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.FLOAT32, 1),
        ]
        return point_cloud2.create_cloud(header, fields, points_with_color)

    def pack_rgb(self, r, g, b):
        """
        Pack 3 floats (0..1) into a single 32-bit 'float' compatible with PCL.
        """
        r_i = int(r*255) & 0x0000ff
        g_i = int(g*255) & 0x0000ff
        b_i = int(b*255) & 0x0000ff
        rgb = (r_i << 16) | (g_i << 8) | b_i
        # Pack into float
        return struct.unpack('f', struct.pack('I', rgb))[0]

    def create_sphere_marker(self, marker_id, pos, frame_id="lidar", stamp=None):
        """Create a small sphere Marker at 'pos' = (x,y,z)."""
        if stamp is None:
            stamp = rospy.Time.now()

        mk = Marker()
        mk.header.frame_id = frame_id
        mk.header.stamp = stamp
        mk.ns = "cluster_centroids"
        mk.id = marker_id
        mk.type = Marker.SPHERE
        mk.action = Marker.ADD
        mk.pose.orientation.w = 1.0

        mk.pose.position.x = pos[0]
        mk.pose.position.y = pos[1]
        mk.pose.position.z = pos[2]

        mk.scale.x = 0.2
        mk.scale.y = 0.2
        mk.scale.z = 0.2

        mk.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # white
        mk.lifetime = rospy.Duration(0)  # forever
        return mk


def main():
    node = pointcloud_processing_node()
    rospy.spin()

if __name__ == "__main__":
    main()
