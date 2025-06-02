from robot_feature_detector.corners3D_utils import separate_cloud, extract_corners, numpy_to_o3d_cloud
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import open3d as o3d
import sensor_msgs_py.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from scipy.spatial.transform import Rotation as R
from robot_msgs.msg import Feature, FeatureArray

class Corner3DDetector(Node):
    def __init__(self):
        #
        super().__init__("corner_detector")
        self.get_logger().info("Initializing 3D_corner node!")
   
        #
        self.subscription = self.create_subscription(PointCloud2, "/segmented_cloud", self.point_cloud_callback,  1)

        self.timer = self.create_timer(1, self.timer_callback)

        self.corner_publisher = self.create_publisher(Marker, "/ceiling_corners", 1)

        self.corner_orientation_publisher = self.create_publisher(MarkerArray, "/corner_orientations", 1)

        self.feature_publisher = self.create_publisher(FeatureArray, "features", 1)

        #
        self.WALL_COLOR = [0, 0, 255] 
        self.CEILING_COLOR = [0, 255, 0] 
        self.FLOOR_COLOR = [255, 0, 0]    
        
        self.COLOR_TOLERANCE = 10

        #
        self.np_wall = np.zeros((10, 3), dtype = np.float32)
        self.np_ceiling = np.zeros((10, 3), dtype = np.float32)
        self.previous_num_corners = 0
    
    #
    def point_cloud_callback(self, msg):
        """Process incoming point cloud message"""
        self.np_wall, self.np_ceiling, _ = separate_cloud(msg, self.WALL_COLOR, self.CEILING_COLOR, self.FLOOR_COLOR, self.COLOR_TOLERANCE)
        #self.get_logger().info(f"{self.pcl_counter} -> Wall: {len(self.np_wall)} points, Ceiling: {len(self.np_ceiling)}")
    
    #
    def timer_callback(self):
        self.wall_pcl = numpy_to_o3d_cloud(self.np_wall); self.ceiling_pcl = numpy_to_o3d_cloud(self.np_ceiling)
        wall_planes, ceiling_planes, self.corners, self.thetas, duration = extract_corners(self.wall_pcl, self.ceiling_pcl)
        #self.get_logger().info(f"Found {len(self.corners)} corners from {len(wall_planes)} walls and {len(ceiling_planes)} ceilings in {duration} seconds!")
        #if len(self.thetas) > 0: self.get_logger().info(f"Orientation of the 1st corner: {self.thetas[0] * 180 / np.pi}")
        self.publish_3D_corners_rviz()
        self.publish_orientated_corners_rviz()
        self.publish_corner_features()
    
    #
    def publish_3D_corners_rviz(self):
        #self.get_logger().info("Publishing corners for RViz!")
        marker = Marker()
        marker.header.frame_id = "lidar3D"
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = float(1)
        marker.scale.x = float(0.3)
        marker.scale.y = float(0.3)
        marker.scale.z = float(0.3)
        marker.color.a = float(1)
        marker.color.r = float(1)
        marker.color.g = float(0)
        marker.color.b = float(0)
        marker.points = []

        for point in self.corners:
            point_ = Point()
            point_.x = float(point[0])
            point_.y = float(point[1])
            point_.z = float(point[2])
            marker.points.append(point_)
            
        self.corner_publisher.publish(marker)
    
    def publish_orientated_corners_rviz(self):
        markerArray = MarkerArray()
        markerArray.markers = []

        for i in range(0, len(self.corners)):
            #self.get_logger().info(f"Corner {i} publishing")
            marker = Marker()
            marker.header.frame_id = "lidar3D"
            marker.id = i
            marker.ns = "orientations"
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            theta = self.thetas[i]
            #self.get_logger().info(f"theta: {theta}")
            q = R.from_euler('z', theta).as_quat()
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]
            marker.pose.position.x = float(self.corners[i][0])
            marker.pose.position.y = float(self.corners[i][1])
            marker.pose.position.z = float(0)
            marker.scale.x = float(1.0)
            marker.scale.y = float(0.3)
            marker.scale.z = float(0.3)
            marker.color.a = float(1)
            marker.color.r = float(1)
            marker.color.g = float(0)
            marker.color.b = float(0)
            markerArray.markers.append(marker)

            #self.get_logger().info(f"markerArray size: {len(markerArray.markers)}")
        
        if len(self.corners) < self.previous_num_corners:
            for i in range(len(self.corners), self.previous_num_corners):
                marker = Marker()
                marker.header.frame_id = "lidar3D"
                marker.id = i
                marker.ns = "orientations"
                marker.type = Marker.ARROW
                marker.action = Marker.DELETE
                markerArray.markers.append(marker)


        self.previous_num_corners = len(self.corners)
        
        self.corner_orientation_publisher.publish(markerArray)

    def publish_corner_features(self):
        feature_array = FeatureArray()
        feature_array.features = []
        feature_array.header.stamp = self.get_clock().now().to_msg()
        for i in range(0, len(self.corners)):
            feature = Feature()
            feature.type = "corner"
            position_variance = float(0.1)
            orientation_variance = float(0.1)

            feature.x = float(self.corners[i][0])
            feature.y = float(self.corners[i][1])
            feature.theta = float(self.thetas[i])
            feature.confidence = 1.0
            feature.position_covariance = [position_variance, 0.0, 0.0, position_variance]
            feature.orientation_variance = orientation_variance
            feature_array.features.append(feature)
            
        if len(self.corners) > 0:
            self.feature_publisher.publish(feature_array)
            self.get_logger().info("Publishing the features")

        
def main(args=None):
    rclpy.init(args=args)
    detector_node = Corner3DDetector()
    rclpy.spin(detector_node)
    detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
