import numpy as  np

debug = True

min_samples = 2

def extract_first_ransac_line(data_points, max_distance:int):
    inliers = []
    model_robust, inliers = ransac(data_points, LineModelND, min_samples=min_samples,
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

max_distance = 0.03
iterations = 15

gap_threshold = 1.5
min_length = 0.4

def extract_lines_from_point_cloud(point_cloud):
    models = []
    models_points_start = []
    models_points_end = []
    for index in range(0,iterations):
            if (len(point_cloud) < min_samples):
                break
            inlier_points, inliers_removed_from_starting, model = extract_first_ransac_line(point_cloud, max_distance=max_distance)
            point_cloud=inliers_removed_from_starting
            if (len(inlier_points) < min_samples):
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
            if debug:
              print("TESTING LINE")
              if length > min_length:
                if max_gap < gap_threshold:
                  models_points_start.append(start_point)
                  models_points_end.append(end_point)
                  models.append(model)
                elif debug:
                  print("GAP TOO BIG: ")
                  print(max_gap)
              elif debug:
                print("NOT LONG ENOUGH: ")
                print(length)
    return models, models_points_start, models_points_end

endpoint_threshold = 1.1
def is_point_close_to_endpoints(point, start, end):
    px, py = point
    x1, y1 = start
    x2, y2 = end

    distance_to_start = np.linalg.norm(np.array([px - x1, py - y1]))
    distance_to_end = np.linalg.norm(np.array([px - x2, py - y2]))

    return distance_to_start <= endpoint_threshold or distance_to_end <= endpoint_threshold

def find_intersection(models, models_points_start, models_points_end):
  intersection_points = []
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
              if is_point_close_to_endpoints(intersection, models_points_start[i], models_points_end[i]) and is_point_close_to_endpoints(intersection, models_points_start[j], models_points_end[j]):
                  intersection_points.append(intersection)
              elif debug:
                  print("INTERSECTION TOO FAR FROM ENDPOINTS")
                  print(intersection)
                  print(models_points_start[i])
                  print(models_points_end[i])
                  print(models_points_start[j])
                  print(models_points_end[j])
          except np.linalg.LinAlgError:
              continue
  return np.array(intersection_points)

import matplotlib.pyplot as plt

def plot_point_cloud(point_cloud):
  # Plot the point cloud for one timestep
  x, y = point_cloud  # Choose the first timestep, for example
  plt.figure(figsize=(8, 8))
  plt.scatter(x, y, s=5, c='blue', label='LiDAR Points')
  plt.axhline(0, color='black', linewidth=0.5)
  plt.axvline(0, color='black', linewidth=0.5)
  plt.title('LiDAR Point Cloud')
  plt.xlabel('X (m)')
  plt.ylabel('Y (m)')
  plt.axis('equal')  # Equal scaling for X and Y
  plt.legend()
  plt.show()

def plot_point_cloud_with_corners(point_cloud, intersection_points):
  x, y = point_cloud
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
  plt.legend()
  plt.show()

plot_point_cloud(point_clouds[0])
