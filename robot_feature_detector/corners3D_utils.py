import numpy as np
import open3d as o3d
import itertools
import time
import sensor_msgs_py.point_cloud2 as pc2
import struct


#
def numpy_to_o3d_cloud(np_array):
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(np_array)
    return cloud

#
def unpack_rgb(rgb_float):
    """Unpack RGB values from a float"""
    # Convert float back to uint32
    rgb_uint = struct.unpack('I', struct.pack('f', rgb_float))[0]
    
    # Extract BGR components (note: your packing uses BGR order)
    b = rgb_uint & 0xFF
    g = (rgb_uint >> 8) & 0xFF
    r = (rgb_uint >> 16) & 0xFF
    
    return [r, g, b]  # Return as RGB

def color_matches(color1, color2, tolerance=10):
    """Check if two colors match within tolerance"""
    return all(abs(c1 - c2) <= tolerance for c1, c2 in zip(color1, color2))

def separate_cloud(msg, wall_color, ceiling_color, floor_color, tolerance=10):
    try:
        # Convert point cloud to list of points
        points_list = list(pc2.read_points(msg, field_names=("x", "y", "z", "rgb")))
        
        wall_points = []
        ceiling_points = []
        floor_points = []
        
        for point in points_list:
            x, y, z, rgb_float = point
            
            # Unpack the RGB color
            rgb = unpack_rgb(rgb_float)
            
            # Check which segment the point belongs to
            if color_matches(rgb, wall_color, tolerance):
                wall_points.append([x, y, z])
            elif color_matches(rgb, ceiling_color, tolerance):
                ceiling_points.append([x, y, z])
            elif color_matches(rgb, floor_color, tolerance):
                floor_points.append([x, y, z])
        
        # Convert to numpy arrays
        np_wall = np.array(wall_points) if wall_points else np.empty((0, 3))
        np_ceiling = np.array(ceiling_points) if ceiling_points else np.empty((0, 3))
        np_floor = np.array(floor_points) if floor_points else np.empty((0, 3))
        
        return np_wall, np_ceiling, np_floor
        
    except Exception as e:
        print(f'Error processing point cloud: {str(e)}')
        return np.empty((0, 3)), np.empty((0, 3)), np.empty((0, 3))

def flip_normal_toward_robot(normal, point_on_plane, robot_pos):
    direction_to_robot = robot_pos - point_on_plane
    if np.dot(normal, direction_to_robot) < 0:
        return normal
    else:
        return -normal 

#
def are_planes_well_conditioned(p1, p2, p3, threshold=np.deg2rad(45)):
    def angle_between(n1, n2):
        return np.arccos(np.clip(np.dot(n1, n2) / (np.linalg.norm(n1) * np.linalg.norm(n2)), -1, 1))
    
    normals = [np.array(p[:3]) for p in [p1, p2, p3]]
    angles = [angle_between(normals[i], normals[j]) for i in range(3) for j in range(i+1, 3)]
    
    return all(threshold < angle < (np.pi - threshold) for angle in angles)

def intersect_planes(p1, p2, p3):
    A = np.array([p1[:3], p2[:3], p3[:3]])
    b = -np.array([p1[3], p2[3], p3[3]])
    try:
        point = np.linalg.solve(A, b)
        return point
    except np.linalg.LinAlgError:
        return None  # Parallel or near-degenerate

def extract_planes(pcd, max_planes=10, distance_threshold=0.1, min_inliers=700):
    planes = []
    rest = pcd
    for _ in range(max_planes):
        if len(rest.points) < min_inliers:
            break
        plane_model, inliers = rest.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=3,
            num_iterations=100
        )
        if len(inliers) < min_inliers:
            break
        planes.append((plane_model, rest.select_by_index(inliers)))
        rest = rest.select_by_index(inliers, invert=True)
    return planes

def compute_corner_orientation_from_centers(corner, wall_centers):
    directions = []
    corner_2d = np.array(corner[:2])

    for center in wall_centers:
        center_2d = np.array(center[:2])
        vec = center_2d - corner_2d
        vec /= np.linalg.norm(vec)
        directions.append(vec)

    avg_dir = np.mean(directions, axis=0)
    avg_dir /= np.linalg.norm(avg_dir)

    theta = np.arctan2(avg_dir[1], avg_dir[0])
    return theta

def extract_corners(wall_pcl: o3d.geometry.PointCloud, ceiling_pcl: o3d.geometry.PointCloud):
    """Extract corners from intersections of two walls and one ceiling"""
    start = time.perf_counter()
    
    wall_planes = extract_planes(wall_pcl)
    ceiling_planes = extract_planes(ceiling_pcl)
    
    thetas = []
    corners = []

    for (w1, w1_cloud), (w2, w2_cloud) in itertools.combinations(wall_planes, 2):
        for (c1, *_) in ceiling_planes:
            if not are_planes_well_conditioned(w1, w2, c1):
                continue
            corner_ = intersect_planes(w1, w2, c1)
            if corner_ is not None:
                corners.append(corner_)

                # Compute wall centroids
                center1 = np.asarray(w1_cloud.points).mean(axis=0)
                center2 = np.asarray(w2_cloud.points).mean(axis=0)

                # Compute orientation
                theta = compute_corner_orientation_from_centers(corner_, [center1, center2])
                
                # Store as theta
                thetas.append(theta)
                


    duration = time.perf_counter() - start
    return wall_planes, ceiling_planes, corners, thetas, duration

#
def main():
    print("corner3D_utils!")


if __name__ == "__main__":
    main()
