import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Time
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from skimage.measure import ransac, LineModelND
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from robot_msgs.msg import Feature, FeatureArray
import time



class PointCloudSubscriber(Node):

    def __init__(self):
        
        super().__init__('pointcloud_subscriber')

        self.first_pcl = True
        #print something but without print
        

        self.point_cloud_numpy = np.load("/home/joao/ros2_ws/src/robot_feature_detector/robot_feature_detector/pcl/robotpoint_cloud_0.npy")
        self.point_cloud_numpy_copy = self.point_cloud_numpy.copy()

        self.subscription = self.create_subscription(
            PointCloud2,
            '/scan/point_cloud', 
            self.listener_callback,
            1)

        self.corner_publisher = self.create_publisher(
            Marker,
            '/corners_marker',
            1
        )

        self.anchor_publisher = self.create_publisher(
            Marker,
            '/anchor_points_marker',
            1
        )

        self.corner_orientation_publisher = self.create_publisher(
            MarkerArray,
            '/orientated_corners_marker',
            1
        )

        self.feature_publisher = self.create_publisher(
            FeatureArray,
            '/features',
            1
        )

        self.previous_num_corners = 0
        self.timer = self.create_timer(1, self.extract_publish_corners)

    def plot_point_cloud_with_corners(self, intersection_points):
        x = self.point_cloud_numpy_copy[:, 0]
        y = self.point_cloud_numpy_copy[:, 1]
        plt.figure(figsize=(8, 8))
        plt.scatter(x, y, s=5, c='blue', label='LiDAR Points')
        for corner in intersection_points:
            plt.scatter(corner[0], corner[1], color='red', label='Intersection Point', s=100)
        plt.axhline(0, color='black', linewidth=0.5)
        plt.axvline(0, color='black', linewidth=0.5)
        plt.title('LiDAR Point Cloud')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.axis('equal')  # Equal scaling for X and Y
        plt.show()

    def plot_point_cloud(self):
        #self.get_logger().info(f"Plotting point cloud...{self.point_cloud_numpy.shape}")

        x = self.point_cloud_numpy[:, 0]
        y = self.point_cloud_numpy[:, 1]
        #self.get_logger().info(f"x: {x.shape}")
        #self.get_logger().info("Plotting point cloud...2")
        plt.figure(figsize=(8, 8))
        plt.scatter(x, y, s=5, c='blue', label='LiDAR Points')
        plt.axhline(0, color='black', linewidth=0.5)
        plt.axvline(0, color='black', linewidth=0.5)
        plt.title('LiDAR Point Cloud')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.axis('equal') 
        plt.show()


    def save_point_cloud(self, filename):
        np.save(f"/home/joao/ros2_ws/src/robot_feature_detector/robot_feature_detector/pcl/{filename}.npy", self.point_cloud_numpy)
    
    def listener_callback(self, point_cloud: PointCloud2):

        self.point_cloud_numpy = self.convert_point_cloud_msg_to_numpy(point_cloud)
        self.get_logger().info("Received PCL!")
        #self.extract_publish_corners()

    def extract_publish_corners(self):
        while np.isnan(self.point_cloud_numpy).any() or np.isinf(self.point_cloud_numpy).any():
            #self.get_logger().warn("Point cloud contains NaN or Inf values!")
            self.point_cloud_numpy = self.point_cloud_numpy[~np.isnan(self.point_cloud_numpy).any(axis=1)]
            self.point_cloud_numpy = self.point_cloud_numpy[~np.isinf(self.point_cloud_numpy).any(axis=1)]

            #self.get_logger().info("Point cloud is valid, running RANSAC...")

        #self.plot_point_cloud()
        
        models, models_start, models_end = self.extract_lines_from_point_cloud()
        self.intersection_points, corner_lines_start, corner_lines_end = self.find_intersection(models, models_start, models_end)
        self.corner_orientations, self.anchor_points = self.get_corners_orientations(self.intersection_points, corner_lines_start, corner_lines_end)
        self.publish_corners()
        self.publish_anchors()
        self.publish_orientated_corners()
        self.publish_features()
        self.get_logger().info("Extract_publish_done")
        #len_ = len(self.intersection_points)
        #self.get_logger().info(len_)

    def extract_first_ransac_line(self, data_points, max_distance:int):
        inliers = []
        model_robust, inliers = ransac(data_points, LineModelND, min_samples= self.min_samples,
                                    residual_threshold=max_distance, max_trials=100)
        results_inliers=[]
        results_inliers_removed=[]
        if(inliers is not None):
            for i in range(0,len(data_points)):
                if (inliers[i] == False):
                    #Not an inlier
                    results_inliers_removed.append(data_points[i])
                    continue
                x=data_points[i][0]
                y=data_points[i][1]
                results_inliers.append((x,y))

        return np.array(results_inliers), np.array(results_inliers_removed), model_robust

    debug = False
    min_samples = 3

    max_distance = 0.5
    iterations = 30

    gap_threshold = 1.5
    min_length = 0.5


    def extract_lines_from_point_cloud(self):
        models = []
        models_points_start = []
        models_points_end = []
        for index in range(0, self.iterations):

            #self.get_logger().info(f"Pcl size: {len(self.point_cloud_numpy)}")

                
            if (len(self.point_cloud_numpy) < self.min_samples):
                break
            inlier_points, inliers_removed_from_starting, model = self.extract_first_ransac_line(self.point_cloud_numpy, max_distance=self.max_distance)
            self.point_cloud_numpy=inliers_removed_from_starting
            if (len(inlier_points) < self.min_samples):
                break
            p0, direction = model.params
            projections = np.dot(inlier_points - p0, direction)
            projections_sorted = np.sort(projections)
            max_gap = np.max(np.diff(projections_sorted))
            min_proj = np.min(projections)
            max_proj = np.max(projections)
            start_point = p0 + min_proj * direction
            end_point = p0 + max_proj * direction
            length = np.linalg.norm(end_point - start_point)
            if self.debug:
                self.get_logger().info("TESTING LINE")
            if length > self.min_length:
                if max_gap < self.gap_threshold:
                    models_points_start.append(start_point)
                    models_points_end.append(end_point)
                    models.append(model)
                elif self.debug:
                    self.get_logger().info("GAP TOO BIG: ")
                    #self.get_logger().info(max_gap)
            elif self.debug:
                self.get_logger().info("NOT LONG ENOUGH: ")
                #self.get_logger().info(length)
        return models, models_points_start, models_points_end
    
    def dist(self, point_1, point_2):
        return np.linalg.norm(np.array(point_1) - np.array(point_2))

    def get_corners_orientations(self, corner_list, lines_start, lines_end):
        
        orientations = []
        anchor_points = []
        # Find the anchor points of the corner (the points on the walls)
        for i in range(0, len(corner_list)):
            if self.dist(corner_list[i], lines_start[i, 0]) > self.dist(corner_list[i], lines_end[i, 0]):
                anchor_1 = lines_start[i, 0]
            else:
                anchor_1 = lines_end[i, 0]

            if self.dist(corner_list[i], lines_start[i, 1]) > self.dist(corner_list[i], lines_end[i, 1]):
                anchor_2 = lines_start[i, 1]
            else:
                anchor_2 = lines_end[i, 1]
            
            anchor_points.append(anchor_1)
            anchor_points.append(anchor_2)

            # Determine if corner is an innie (acute facing angle) or an outie (obtuse facing angle)
            origin = np.array((0, 0))
            if self.dist(origin, anchor_1) < self.dist(origin, corner_list[i]) or self.dist(origin, anchor_2) < self.dist(origin, corner_list[i]):
                innie = True
            else:
                innie = False

            # Calculate the angle between the two lines
            vector_1 =  np.array(anchor_1) - np.array(corner_list[i])
            vector_2 =  np.array(anchor_2) - np.array(corner_list[i])

            versor_1 = vector_1 / np.linalg.norm(vector_1)
            versor_2 = vector_2 / np.linalg.norm(vector_2)

            versor_mid = (versor_1 + versor_2)

            angle_mid = np.arctan2(versor_mid[1], versor_mid[0])

            if angle_mid < - np.pi:
                angle_mid += 2 * np.pi
            elif angle_mid > np.pi:
                angle_mid -= 2 * np.pi


            orientations.append(angle_mid)
        
        return orientations, anchor_points

    endpoint_threshold = 1.8

    def is_point_close_to_endpoints(self, point, start, end):
        px, py = point
        x1, y1 = start
        x2, y2 = end

        distance_to_start = np.linalg.norm(np.array([px - x1, py - y1]))
        distance_to_end = np.linalg.norm(np.array([px - x2, py - y2]))

        return distance_to_start <= self.endpoint_threshold or distance_to_end <= self.endpoint_threshold
    
    def find_intersection(self, models, models_points_start, models_points_end):
        intersection_points = []
        lines_start = []
        lines_end = []
        for i in range(len(models)):
            for j in range(i, len(models)):
                if (i == j):
                    continue
                line1 = (models[i].params[0], models[i].params[1])
                line2 = (models[j].params[0], models[j].params[1])
                p1, d1 = line1
                p2, d2 = line2

                A = np.array([d1, -d2]).T
                b = p2 - p1
                try:
                    t = np.linalg.solve(A, b)
                    intersection = p1 + t[0] * d1
                    if self.is_point_close_to_endpoints(intersection, models_points_start[i], models_points_end[i]) and self.is_point_close_to_endpoints(intersection, models_points_start[j], models_points_end[j]):
                        intersection_points.append(intersection)
                        lines_start.append((models_points_start[i], models_points_start[j]))
                        lines_end.append((models_points_end[i], models_points_end[j]))
                    elif self.debug:
                        self.get_logger().info("INTERSECTION TOO FAR FROM ENDPOINTS")
                        #print(intersection)
                        #print(models_points_start[i])
                        #print(models_points_end[i])
                        #print(models_points_start[j])
                        #print(models_points_end[j])
                except np.linalg.LinAlgError:
                    continue
        return np.array(intersection_points), np.array(lines_start), np.array(lines_end)

    def publish_features(self):
        feature_array = FeatureArray()
        feature_array.features = []
        feature_array.header.stamp = self.get_clock().now().to_msg()
        for i in range(0, len(self.intersection_points)):
            feature = Feature()
            feature.type = "corner"
            feature.x = float(self.intersection_points[i, 0])
            feature.y = float(self.intersection_points[i, 1])
            feature.theta = float(self.corner_orientations[i])
            feature.confidence = 1.0
            position_variance = float(0.1)
            orientation_variance = float(0.1)
            feature.position_covariance = [position_variance, 0.0, 0.0, position_variance]
            feature.orientation_variance = orientation_variance
            feature_array.features.append(feature)
            
        if len(self.intersection_points) > 0:
            self.feature_publisher.publish(feature_array)
            #self.get_logger().info("Publishing the features")

    def publish_corners(self):
        marker = Marker()
        marker.header.frame_id = "lidar2D"
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = float(1)
        marker.scale.x = float(0.05)
        marker.scale.y = float(0.05)
        marker.scale.z = float(0.05)
        marker.color.a = float(1)
        marker.color.r = float(1)
        marker.color.g = float(0)
        marker.color.b = float(0)
        marker.points = []

        for point in self.intersection_points:
            point_ = Point()
            point_.x = float(point[0])
            point_.y = float(point[1])
            point_.z = float(0)
            marker.points.append(point_)
            
        self.corner_publisher.publish(marker)

    def publish_anchors(self):
        marker = Marker()
        marker.header.frame_id = "lidar2D"
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = float(1)
        marker.scale.x = float(0.03)
        marker.scale.y = float(0.03)
        marker.scale.z = float(0.03)
        marker.color.a = float(1)
        marker.color.r = float(0)
        marker.color.g = float(1)
        marker.color.b = float(0)
        marker.points = []

        for point in self.anchor_points:
            point_ = Point()
            point_.x = float(point[0])
            point_.y = float(point[1])
            point_.z = float(0)
            marker.points.append(point_)
            
        self.anchor_publisher.publish(marker)

    def publish_orientated_corners(self):
        markerArray = MarkerArray()
        markerArray.markers = []

        for i in range(0, len(self.intersection_points)):
            #self.get_logger().info(f"Corner {i} publishing")
            marker = Marker()
            marker.header.frame_id = "lidar2D"
            marker.id = i
            marker.ns = "orientations"
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            theta = self.corner_orientations[i]
            #self.get_logger().info(f"theta: {theta}")
            q = R.from_euler('z', theta).as_quat()
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]
            marker.pose.position.x = float(self.intersection_points[i, 0])
            marker.pose.position.y = float(self.intersection_points[i, 1])
            marker.pose.position.z = float(0)
            marker.scale.x = float(0.5)
            marker.scale.y = float(0.09)
            marker.scale.z = float(0.09)
            marker.color.a = float(1)
            marker.color.r = float(1)
            marker.color.g = float(0)
            marker.color.b = float(0)
            markerArray.markers.append(marker)

            #self.get_logger().info(f"markerArray size: {len(markerArray.markers)}")
        
        if len(self.intersection_points) < self.previous_num_corners:
            for i in range(len(self.intersection_points), self.previous_num_corners):
                marker = Marker()
                marker.header.frame_id = "lidar2D"
                marker.id = i
                marker.ns = "orientations"
                marker.type = Marker.ARROW
                marker.action = Marker.DELETE
                markerArray.markers.append(marker)


        self.previous_num_corners = len(self.intersection_points)
        
        self.corner_orientation_publisher.publish(markerArray)

    def convert_point_cloud_msg_to_numpy(self, msg: PointCloud2):
        """Convert PointCloud2 message to a NumPy array with only x, y coordinates."""
        # Extract x, y points from PointCloud2
        points = np.array([
            (p[0], p[1]) for p in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)
        ], dtype=np.float32)
        
        return points  # Nx2 NumPy array (x, y)

def main(args=None):
    rclpy.init(args=args)
    pointcloud_subscriber = PointCloudSubscriber()
    rclpy.spin(pointcloud_subscriber)
    pointcloud_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()