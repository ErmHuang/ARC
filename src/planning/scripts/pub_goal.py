#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import math

current_position = None

def update_current_position(msg):
    global current_position
    current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def publish_goal(goal_publisher, x, y):
    goal = PoseStamped()
    goal.header.seq = 0
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = 'map'

    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = 0.0

    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 1.0

    rospy.loginfo(f"Publishing goal: x={x}, y={y}")
    goal_publisher.publish(goal)

def main():
    global current_position

    rospy.init_node('goal_exploration_node')
    goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    # rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, update_current_position)
    rospy.Subscriber('/gazebo/ground_truth/state', Odometry, update_current_position)

    # Wait for the publisher to be ready
    rospy.sleep(1)

    # List of goals to explore
    # goals = [
    #     (0.0, -11.25),
    #     (-9.0, 10.0),
    #     (-5.0, 11.0), 
    # ]

    goals = [
        (-5.0, 6.75),
        (9.0, 6.5),
        (8.5, 0.0),
        (5.5, -5.0),
        (-2.0, -8.5),
        (7.0, -10.5),
        (-5.0, -4.0), 
    ]

    for goal in goals:
        goal_reached = False
        publish_goal(goal_publisher, goal[0], goal[1])
        rate = rospy.Rate(0.2)  # 1 Hz loop rate

        while not rospy.is_shutdown() and not goal_reached:
            if current_position is not None:
                distance = calculate_distance(current_position[0], current_position[1], goal[0], goal[1])
                rospy.loginfo(f"Current distance to goal: {distance:.2f} meters")

                # Check if the robot is within 0.5 meters of the goal
                if distance < 1.5:
                    rospy.loginfo("Goal reached!")
                    goal_reached = True
            rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
