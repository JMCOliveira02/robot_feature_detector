import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from skimage.measure import ransac, LineModelND
import matplotlib.pyplot as plt
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
            '/scan2D/point_cloud', 
            self.listener_callback,
            1)

        self.marker_publisher = self.create_publisher(
            Marker,
            '/corners',
            1
        )

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
        self.get_logger().info(f"Plotting point cloud...{self.point_cloud_numpy.shape}")

        x = self.point_cloud_numpy[:, 0]
        y = self.point_cloud_numpy[:, 1]
        self.get_logger().info(f"x: {x.shape}")
        self.get_logger().info("Plotting point cloud...2")
        plt.figure(figsize=(8, 8))
        plt.scatter(x, y, s=5, c='blue', label='LiDAR Points')
        plt.axhline(0, color='black', linewidth=0.5)
        plt.axvline(0, color='black', linewidth=0.5)
        plt.title('LiDAR Point Cloud')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.axis('equal') 
        plt.show()

    min_samples = 5
    debug = False

    def extract_first_ransac_line(self, data_points, max_distance:int):
        inliers = []
        model_robust, inliers = ransac(data_points, LineModelND, min_samples= self.min_samples,
                                    residual_threshold=max_distance, max_trials=1000)
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

    max_distance = 0.01
    iterations = 15

    gap_threshold = 1.5
    min_length = 0.4

    def extract_lines_from_point_cloud(self):
        models = []
        models_points_start = []
        models_points_end = []
        for index in range(0, self.iterations):
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
                    print("TESTING LINE")
                if length > self.min_length:
                    if max_gap < self.gap_threshold:
                        models_points_start.append(start_point)
                        models_points_end.append(end_point)
                        models.append(model)
                    elif self.debug:
                        print("GAP TOO BIG: ")
                        print(max_gap)
                elif self.debug:
                    print("NOT LONG ENOUGH: ")
                    print(length)
        return models, models_points_start, models_points_end
    
    endpoint_threshold = 0.1

    def dist(point_1, point_2):
        return np.linalg.norm(np.array(point_1) - np.array(point_2))

    def get_opening_angle(self, corner, models_points_start, models_points_end):
        #distance between corner and start of line
        if self.dist(corner, models_points_start[0]) > self.dist(corner, models_points_end[0]):
            anchor_1 = models_points_start[0]
        else:
            anchor_1 = models_points_end[0]

        if self.dist(corner, models_points_start[1]) > self.dist(corner, models_points_end[1]):
            anchor_2 = models_points_start[1]
        else:
            anchor_2 = models_points_end[1]


    def is_point_close_to_endpoints(self, point, start, end):
        px, py = point
        x1, y1 = start
        x2, y2 = end

        distance_to_start = np.linalg.norm(np.array([px - x1, py - y1]))
        distance_to_end = np.linalg.norm(np.array([px - x2, py - y2]))

        return distance_to_start <= self.endpoint_threshold or distance_to_end <= self.endpoint_threshold
    
    def find_intersection(self, models, models_points_start, models_points_end):
        intersection_points = []
        intersection_lines = []
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
                        intersection_lines.append((models[i], models[j]))
                    elif self.debug:
                        print("INTERSECTION TOO FAR FROM ENDPOINTS")
                        print(intersection)
                        print(models_points_start[i])
                        print(models_points_end[i])
                        print(models_points_start[j])
                        print(models_points_end[j])
                except np.linalg.LinAlgError:
                    continue
        return np.array(intersection_points), 


    def save_point_cloud(self, filename):
        np.save(f"/home/joao/ros2_ws/src/robot_feature_detector/robot_feature_detector/pcl/{filename}.npy", self.point_cloud_numpy)
    
    
    def listener_callback(self, point_cloud: PointCloud2):

        self.point_cloud_numpy = self.convert_point_cloud_msg_to_numpy(point_cloud)
        self.extract_publish_corners()

    def publish_corners(self):
        marker = Marker()
        marker.header.frame_id = "lidar2D"
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = float(1)
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
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
            
        self.marker_publisher.publish(marker)

    def extract_publish_corners(self):
        while np.isnan(self.point_cloud_numpy).any() or np.isinf(self.point_cloud_numpy).any():
            self.get_logger().warn("Point cloud contains NaN or Inf values!")
            self.point_cloud_numpy = self.point_cloud_numpy[~np.isnan(self.point_cloud_numpy).any(axis=1)]
            self.point_cloud_numpy = self.point_cloud_numpy[~np.isinf(self.point_cloud_numpy).any(axis=1)]

            self.get_logger().info("Point cloud is valid, running RANSAC...")

        #self.plot_point_cloud()
        
        models, models_start, models_end = self.extract_lines_from_point_cloud()
        self.intersection_points = self.find_intersection(models, models_start, models_end)
        self.publish_corners()


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