import rospy
import time
import math
from iiwa_msgs.msg import CartesianPose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
import threading
import numpy as np
import tf.transformations as tft

####### robot control #######

# get current ee pose, inculding position and orientation
def get_ee_pose_callback(msg):
    global current_ee_pose
    current_ee_pose = np.array([msg.poseStamped.pose.position.x, msg.poseStamped.pose.position.y, msg.poseStamped.pose.position.z - 0.16])

    
# set ee posistion and orientation, the orientation is fixed (corresponding to the euler orientation: x:-180, y:0, z:-180)
def set_ee_pose(x,y,z):
    goal = PoseStamped()

    goal.header.seq = 1
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "ee_link"
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = z + 0.16
    goal.pose.orientation.x= 0 
    goal.pose.orientation.y= 1 
    goal.pose.orientation.z= 0 
    goal.pose.orientation.w= 0
    
    return goal

def set_ee_pose2(x,y,z,yaw_deg_clockwise=0.0):
    q_fixed = tft.quaternion_from_euler(math.radians(-180),
                                        math.radians(0),
                                        math.radians(-180))
    # extra yaw (clockwise is negative)
    q_spin  = tft.quaternion_from_euler(0, 0,
                                        math.radians(-yaw_deg_clockwise))
    # total orientation: first q_fixed, then q_spin
    q_total = tft.quaternion_multiply(q_spin, q_fixed)
    goal = PoseStamped()

    goal.header.seq = 1
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "ee_link"
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = z + 0.16
    goal.pose.orientation.x, \
    goal.pose.orientation.y, \
    goal.pose.orientation.z, \
    goal.pose.orientation.w = q_total
    
    return goal

def set_ee_twist(v_x,v_y,v_z):
    velocity_goal = TwistStamped()

    velocity_goal.header.seq = 1
    velocity_goal.header.stamp = rospy.Time.now()
    velocity_goal.header.frame_id = 'ee_velocity'
    velocity_goal.twist.linear.x = v_x
    velocity_goal.twist.linear.y = v_y
    velocity_goal.twist.linear.z = v_z
    velocity_goal.twist.angular.x = 0
    velocity_goal.twist.angular.y = 0
    velocity_goal.twist.angular.z = 0

    return velocity_goal

# go to the initial position
def get_init_pose():
    goal = set_ee_pose(0.6,0.0,0.2)
    return goal

def reset_pos(init_ee_pose):
    global robot_pub, current_ee_pose
    rate = rospy.Rate(5)
    init_positon = init_ee_pose
    print("start reset")
    while not rospy.is_shutdown():
        diff = (init_positon  - current_ee_pose)*0.1
        print("diff:", diff)
        s_target_position = current_ee_pose + diff
        if np.linalg.norm(init_positon - current_ee_pose) > 0.01:
            goal = set_ee_pose(s_target_position[0],s_target_position[1],s_target_position[2])
            robot_pub.publish(goal)
        else:
            print("reach init position")
            break
        rate.sleep()
    return True
    
if __name__ == '__main__':
    current_ee_pose = np.zeros(3)
    rospy.init_node('test_pub', anonymous=False)
    robot_pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=5)
    rospy.Subscriber('/iiwa/state/CartesianPose', CartesianPose, get_ee_pose_callback)
    rate = rospy.Rate(10)

    
    init_ee_pose = np.array([0.6,0.0,0.37])
    max_ee_pose = np.array([0.7,0.1,0.47])
    
    # reset position
    while not rospy.is_shutdown():
        if reset_pos(init_ee_pose):
            print("reset")
            break
        
    rospy.sleep(1.0)
    
    # go to the target position
    target_position = [0.5,-0.035,0.37]
    target_position2 = max_ee_pose
    while not rospy.is_shutdown():
        diff = (target_position - current_ee_pose)*0.1
        print("diff:", diff)
        s_target_position = current_ee_pose + diff
        if np.linalg.norm(target_position - current_ee_pose) > 0.01:
            goal = set_ee_pose(s_target_position[0],s_target_position[1],s_target_position[2])
            robot_pub.publish(goal)

        else:
            print("reach")
            break
        rate.sleep()
        
    goal = set_ee_pose2(target_position[0],target_position[1],target_position[2],yaw_deg_clockwise=90)
    robot_pub.publish(goal)