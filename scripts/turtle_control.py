#!/usr/bin/python
import math
from numpy import linalg as LA

import rospy
# Disclaimer: curses part was taken from https://gist.github.com/claymcleod/b670285f334acd56ad1c
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# Variables:
node_name = "burger_controller"
velocity_topic = "/cmd_vel"
odom_topic = "/odom"
simple_goal_topic = "/move_base_simple/goal"

global odom


class Node:
    """
    This is a main node, which will publish velocity commands
    """
    def __init__(self):
        rospy.init_node(name=node_name)
        self.p_vel = rospy.Publisher(name=velocity_topic, data_class=Twist, queue_size=10)
        self.p_goal = rospy.Publisher(name=simple_goal_topic, data_class=PoseStamped, queue_size=10)
        rospy.Subscriber(name=odom_topic, data_class=Odometry, callback=callback_odometry)


def main():
    while True:
        print_menu()
        key = raw_input()
    
        if key == 'a' or key == 'A':
            angle = float(raw_input("Enter angle to turn (degrees): "))
            turn_by_angle(angle)
        
        if key == 'd' or key == 'D':
            dist = float(raw_input("Enter distance to drive straight: "))
            drive_straight(dist)

        if key == 'p' or key == 'P':
            x = float(raw_input("Enter coordinate X:"))
            y = float(raw_input("Enter coordinate Y:"))
            go_to(x,y)
        
        if key == 'q' or key == 'Q':
            break
    
    
def print_menu():
    print("""\n\nFor distance control press D
    For angle control press A
    For position control press P
        To exit press Q""")


def test_running_simulaton():
    pass


def callback_odometry(data):
    global odom
    odom = data.pose.pose


def turn_by_angle(angle_deg):
    global odom
    heading = 180.0*euler_from_quaternion((odom.orientation.x, odom.orientation.y, odom.orientation.z, odom.orientation.w))[2] / math.pi
    new_angle = heading + angle_deg

    diff = 1
    cmd = Twist()
    R = rospy.Rate(20)
    while abs(diff) > 0.05:
        heading = 180.0 * euler_from_quaternion(
            (odom.orientation.x, odom.orientation.y, odom.orientation.z, odom.orientation.w))[2] / math.pi
        diff = new_angle - heading
        print(diff)
        cmd.angular.z = 0.03*diff
        n.p_vel.publish(cmd)
        R.sleep()

    cmd.angular.z = 0
    n.p_vel.publish(cmd)
    print("Arrived.")

def go_to(x,y):
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = 'map'
    pose_goal.pose.position.x = x
    pose_goal.pose.position.y = y
    pose_goal.pose.orientation.w = 1
    n.p_goal.publish(pose_goal)
    print("Going to the target position")



def drive_straight(distance):
    global odom
    current_position = [odom.position.x, odom.position.y]
    heading = euler_from_quaternion((odom.orientation.x, odom.orientation.y, odom.orientation.z, odom.orientation.w))[2]
    new_position = [current_position[0] + distance*math.cos(heading), current_position[1] + distance*math.sin(heading) ]

    Kp = 1
    dist = 1
    cmd = Twist()
    R = rospy.Rate(20)
    while dist > distance*0.05:
        dist = LA.norm([odom.position.x - new_position[0], odom.position.y - new_position[1]])
        print(dist)
        cmd.linear.x = 0.1
        n.p_vel.publish(cmd)
        R.sleep()

    cmd.linear.x = 0
    n.p_vel.publish(cmd)
    print("Arrived.")


if __name__ == '__main__':
    # This executes on node run
    try:
        # Creating node
        global n
        n = Node()
        main()

    except rospy.ROSInterruptException:
        pass
