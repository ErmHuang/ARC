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

def extract_positions_from_bag(bag_file, topic_name):
    bag = rosbag.Bag(bag_file)
    positions = []

    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        positions.append(position)

    bag.close()
    return positions

def extract_goal_status_from_bag(bag_file, topic_name='/move_base/status'):
    bag = rosbag.Bag(bag_file)
    goal_status_list = []

    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        if msg.status_list:
            for status in msg.status_list:
                goal_status_list.append(status.status)

    bag.close()
    return goal_status_list


def find_nearest_path_point(path, position):
    min_dist = float('inf')
    nearest_point = None
    for point in path:
        dist = np.sqrt((point[0] - position[0])**2 + (point[1] - position[1])**2)
        if dist < min_dist:
            min_dist = dist
            nearest_point = point
    return nearest_point, min_dist

def calculate_deviation_errors(path, positions):
    deviations = []
    for pos in positions:
        _, deviation = find_nearest_path_point(path, pos)
        deviations.append(deviation)
    return deviations

def calculate_rmse(deviations):
    return np.sqrt(np.mean(np.square(deviations)))

def calculate_success_rate(goal_statuses):
    success_count = sum(1 for status in goal_statuses if status == 3)  # 状态为3表示成功
    total_goals = len(goal_statuses)
    
    if total_goals == 0:
        return 0.0
    
    success_rate = (success_count / total_goals) * 100
    return success_rate

# Error analysis
# rrt_star_path_dwa = extract_path_from_bag('/home/erm/Projects/herm_me5400_ws/log/rrtstar_dwa.bag', '/move_base/TrajectoryPlannerROS/local_plan')[0][0]
# rrt_star_path_teb = extract_path_from_bag('/home/erm/Projects/herm_me5400_ws/log/rrtstar_teb.bag', '/move_base/TebLocalPlannerROS/local_plan')[0][0]
a_star_path_dwa = extract_path_from_bag('/home/erm/Projects/herm_me5400_ws/log/astar_dwa.bag', '/move_base/TrajectoryPlannerROS/local_plan')[0][0]
a_star_path_teb = extract_path_from_bag('/home/erm/Projects/herm_me5400_ws/log/astar_teb.bag', '/move_base/TebLocalPlannerROS/local_plan')[0][0]

# dwa_positions = extract_positions_from_bag('/home/erm/Projects/herm_me5400_ws/log/rrtstar_dwa.bag', '/gazebo/ground_truth/state')
# teb_positions = extract_positions_from_bag('/home/erm/Projects/herm_me5400_ws/log/rrtstar_teb.bag', '/gazebo/ground_truth/state')  
# goal_statuses_dwa = extract_goal_status_from_bag('/home/erm/Projects/herm_me5400_ws/log/rrtstar_dwa.bag')
# goal_statuses_teb = extract_goal_status_from_bag('/home/erm/Projects/herm_me5400_ws/log/rrtstar_teb.bag')
dwa_positions = extract_positions_from_bag('/home/erm/Projects/herm_me5400_ws/log/astar_dwa.bag', '/gazebo/ground_truth/state')
teb_positions = extract_positions_from_bag('/home/erm/Projects/herm_me5400_ws/log/astar_teb.bag', '/gazebo/ground_truth/state')  
goal_statuses_dwa = extract_goal_status_from_bag('/home/erm/Projects/herm_me5400_ws/log/astar_dwa.bag')
goal_statuses_teb = extract_goal_status_from_bag('/home/erm/Projects/herm_me5400_ws/log/astar_teb.bag')

# teb_deviations = calculate_deviation_errors(rrt_star_path_teb, teb_positions)
# dwa_deviations = calculate_deviation_errors(rrt_star_path_dwa, dwa_positions)
teb_deviations = calculate_deviation_errors(a_star_path_teb, teb_positions)
dwa_deviations = calculate_deviation_errors(a_star_path_dwa, dwa_positions)

teb_rmse = calculate_rmse(teb_deviations)
dwa_rmse = calculate_rmse(dwa_deviations)

# Success rate analysis
teb_success_rate = calculate_success_rate(goal_statuses_teb)
dwa_success_rate = calculate_success_rate(goal_statuses_dwa)

# RMSE
print(f"TEB RMSE: {teb_rmse:.2f} meters")
print(f"DWA RMSE: {dwa_rmse:.2f} meters")

# Success rate
print(f"TEB success rate: {teb_success_rate:.2f}%")
print(f"DWA success rate: {dwa_success_rate:.2f}%")

# Plot 
# 1. 绘制RMSE曲线图
plt.figure(figsize=(10, 5))
plt.plot(range(len(teb_deviations)), teb_deviations, label='TEB Deviation', marker='o')
plt.plot(range(len(dwa_deviations)), dwa_deviations, label='DWA Deviation', marker='x')
plt.xlabel('Index')
plt.ylabel('Deviation (meters)')
plt.title('Deviation Error over Time')
plt.legend()
plt.grid(True)
plt.show()
