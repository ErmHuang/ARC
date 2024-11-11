import rosbag
import numpy as np
import matplotlib.pyplot as plt

def extract_path_from_bag(bag_file, topic_name):
    bag = rosbag.Bag(bag_file)
    paths = []

    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        paths.append((path, t.to_sec()))

    bag.close()
    return paths

def calculate_path_length(path):
    length = 0.0
    for i in range(1, len(path)):
        length += np.sqrt((path[i][0] - path[i-1][0])**2 + (path[i][1] - path[i-1][1])**2)
    return length

def calculate_path_length(path):
    length = 0.0
    for i in range(1, len(path)):
        length += np.sqrt((path[i][0] - path[i-1][0])**2 + (path[i][1] - path[i-1][1])**2)
    return length

def calculate_planning_time(paths):
    planning_times = [paths[i][1] - paths[i-1][1] for i in range(1, len(paths))]
    return planning_times

def calculate_curvature(path):
    curvatures = []
    for i in range(1, len(path) - 1):
        x1, y1 = path[i - 1]
        x2, y2 = path[i]
        x3, y3 = path[i + 1]

        # 向量计算
        vec1 = [x2 - x1, y2 - y1]
        vec2 = [x3 - x2, y3 - y2]
        
        # 计算角度
        norm_vec1 = np.linalg.norm(vec1)
        norm_vec2 = np.linalg.norm(vec2)
        if norm_vec1 == 0 or norm_vec2 == 0:
            continue
        
        cos_theta = np.dot(vec1, vec2) / (norm_vec1 * norm_vec2)
        curvature = np.arccos(np.clip(cos_theta, -1.0, 1.0))
        curvatures.append(curvature)

    return curvatures

def count_turn_points(curvatures, threshold=0.1):
    return sum(1 for curvature in curvatures if curvature > threshold)



# a_star_paths = extract_path_from_bag('/home/erm/Projects/herm_me5400_ws/log/astar_dwa.bag', '/move_base/TrajectoryPlannerROS/global_plan')
# rrt_star_paths = extract_path_from_bag('/home/erm/Projects/herm_me5400_ws/log/rrtstar_dwa.bag', '/move_base/TrajectoryPlannerROS/global_plan')
a_star_paths = extract_path_from_bag('/home/erm/Projects/herm_me5400_ws/log/astar_emp.bag', '/move_base/TrajectoryPlannerROS/global_plan')
rrt_star_paths = extract_path_from_bag('/home/erm/Projects/herm_me5400_ws/log/rrtstar_emp.bag', '/move_base/TrajectoryPlannerROS/global_plan')
# a_star_paths = extract_path_from_bag('/home/erm/Projects/herm_me5400_ws/log/astar_teb.bag', '/move_base/TebLocalPlannerROS/global_plan')
# rrt_star_paths = extract_path_from_bag('/home/erm/Projects/herm_me5400_ws/log/rrtstar_teb.bag', '/move_base/TebLocalPlannerROS/global_plan')
filtered_a_star_paths = [path for path in a_star_paths if len(path[0]) > 10]
filtered_rrt_star_paths = [path for path in rrt_star_paths if len(path[0]) > 10]

# length
a_star_lengths = [calculate_path_length(path[0]) for path in filtered_a_star_paths]
rrt_star_lengths = [calculate_path_length(path[0]) for path in filtered_rrt_star_paths]

# time
a_star_times = calculate_planning_time(filtered_a_star_paths)
rrt_star_times = calculate_planning_time(filtered_rrt_star_paths)

# curvature
a_star_curvatures = [calculate_curvature(path[0]) for path in filtered_a_star_paths]
rrt_star_curvatures = [calculate_curvature(path[0]) for path in filtered_rrt_star_paths]
a_star_avg_curvature = [np.mean(curvature) for curvature in a_star_curvatures]
rrt_star_avg_curvature = [np.mean(curvature) for curvature in rrt_star_curvatures]

# turn points
a_star_turn_points = [count_turn_points(curvature) for curvature in a_star_curvatures]
rrt_star_turn_points = [count_turn_points(curvature) for curvature in rrt_star_curvatures]

print("Path Lengths:")
print(f"    A* Average Path Length: {np.mean(a_star_lengths):.2f} meters")
print(f"  RRT* Average Path Length: {np.mean(rrt_star_lengths):.2f} meters")

print("Planning Time:")
print(f"    A* Average Planning Time: {np.mean(a_star_times):.2f} seconds")
print(f"  RRT* Average Planning Time: {np.mean(rrt_star_times):.2f} seconds")
print(f"    A* Max Planning Time: {np.max(a_star_times):.2f} seconds")
print(f"  RRT* Max Planning Time: {np.max(rrt_star_times):.2f} seconds")

print("Curvature:")
print(f"    A* Average Curvature: {np.mean(a_star_avg_curvature):.2f}")
print(f"  RRT* Average Curvature: {np.mean(rrt_star_avg_curvature):.2f}")

print("Turn Points:")
print(f"    A* Average Number of Turn Points: {np.mean(a_star_turn_points):.2f}")
print(f"  RRT* Average Number of Turn Points: {np.mean(rrt_star_turn_points):.2f}")


# Plot Path Length Distribution
fig, axs = plt.subplots(2, 2, figsize=(15, 10))

axs[0, 0].scatter(range(1, len(a_star_lengths) + 1), a_star_lengths, color='blue', label='A* Path Lengths')
axs[0, 0].scatter(range(1, len(rrt_star_lengths) + 1), rrt_star_lengths, color='red', label='RRT* Path Lengths')
axs[0, 0].set_xlabel('Path Index')
axs[0, 0].set_ylabel('Path Length (meters)')
axs[0, 0].set_title('Path Length Comparison for A* and RRT* Planners')
axs[0, 0].legend()
axs[0, 0].grid(True)

# Plot Planning Time
axs[0, 1].scatter(range(1, len(a_star_times) + 1), a_star_times, color='blue', label='A* Planning Time')
axs[0, 1].scatter(range(1, len(rrt_star_times) + 1), rrt_star_times, color='red', label='RRT* Planning Time')
axs[0, 1].set_xlabel('Task Number')
axs[0, 1].set_ylabel('Planning Time (seconds)')
axs[0, 1].set_title('Planning Time for A* and RRT* Planners')
axs[0, 1].legend()
axs[0, 1].grid(True)

# Plot Curvature Distribution
axs[1, 0].scatter(range(1, len(a_star_avg_curvature) + 1), a_star_avg_curvature, color='blue', label='A* Curvature')
axs[1, 0].scatter(range(1, len(rrt_star_avg_curvature) + 1), rrt_star_avg_curvature, color='red', label='RRT* Curvature')
axs[1, 0].set_xlabel('Path Index')
axs[1, 0].set_ylabel('Average Curvature (radians)')
axs[1, 0].set_title('Curvature Comparison for A* and RRT* Paths')
axs[1, 0].legend()
axs[1, 0].grid(True)

# Plot Number of Turn Points
axs[1, 1].scatter(range(1, len(a_star_turn_points) + 1), a_star_turn_points, color='blue', label='A* Turn Points')
axs[1, 1].scatter(range(1, len(rrt_star_turn_points) + 1), rrt_star_turn_points, color='red', label='RRT* Turn Points')
axs[1, 1].set_xlabel('Task Number')
axs[1, 1].set_ylabel('Number of Turn Points')
axs[1, 1].set_title('Number of Turn Points in A* and RRT* Paths')
axs[1, 1].legend()
axs[1, 1].grid(True)

plt.tight_layout()
plt.show()

