import rospy
import time
import math
from iiwa_msgs.msg import CartesianPose
from geometry_msgs.msg import PoseStamped
import threading
import numpy as np
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg

####### gripper control #######
def gripper_reset():
    global gripper_pub
    command = outputMsg.Robotiq2FGripper_robot_output()
    command.rACT = 0
    gripper_pub.publish(command)
    return command

def gripper_activate():
    global gripper_pub
    command = outputMsg.Robotiq2FGripper_robot_output()
    command.rPR = 0 #0-255. open:0 close:255
    command.rACT = 1
    command.rGTO = 1
    command.rSP = 255
    command.rFR = 50
    gripper_pub.publish(command)
    return command

def gripper_init():
    count = 0
    while not rospy.is_shutdown():
        command = gripper_reset()
        rate.sleep()
        count+=1
        if count > 10:
            break
    rospy.sleep(3.0)
    command = gripper_activate()
    rospy.sleep(5.0)
    print("gripper init")
    return command

def gripper_control_command(state, command):
    global gripper_pub
    position = 0
    if state > 1.0:
        position = state
    else:
        position = math.ceil(128 - 127.5*state)
        if position > 255:
            position = 255
        elif position < 0:
            position = 0
    
    command.rPR = position #0-255 : 0-8.5cm
    gripper_pub.publish(command)
    return command

####### robot control #######

# get current ee pose, inculding position and orientation
def get_ee_pose_callback(msg):
    global current_ee_pose
    current_ee_pose = np.array([msg.poseStamped.pose.position.x, msg.poseStamped.pose.position.y, msg.poseStamped.pose.position.z - 0.16])
    #print(current_ee_pose)

# set ee posistion and orientation, the orientation is fixed (corresponding to the euler orientation: x:-180, y:0, z:-180)
def set_ee_pose(x,y,z):
    goal = PoseStamped()

    goal.header.seq = 1
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "ee_link"
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = z + 0.16
    goal.pose.orientation.x = 0
    goal.pose.orientation.y = 1
    goal.pose.orientation.z = 0
    goal.pose.orientation.w = 0
    
    return goal

# go to the initial position
def get_init_pose():
    goal = set_ee_pose(0.6,0.0,0.2)
    return goal

def get_init_position():
    return np.array([0.6,0.0,0.2])

def reset_pos():
    global robot_pub, current_ee_pose
    init_positon = get_init_position()
    while np.linalg.norm(init_positon - current_ee_pose) > 0.01:
        goal = set_ee_pose(init_positon[0],init_positon[1],init_positon[2])
        robot_pub.publish(goal)
        rate.sleep()
    print("reach init position")
    return True


# use thread to subscribe the current ee pose
def thread_job():
    rospy.spin()
    
    
    
def robot_control():
    global robot_pub, gripper_pub, current_ee_pose, rate
    
    ## gripper init
    # command = gripper_init()
    # print(command)
    
    init_ee_pose = np.array([0.6,0.0,0.37])
    max_ee_pose = np.array([0.7,0.1,0.47])
    
    # reset position
    while not rospy.is_shutdown():
        if reset_pos():
            print("reset")
            break
        
    rospy.sleep(1.0)
    
    # go to the target position
    target_position = current_ee_pose + np.array([0.0,0.0,-0.175])
    target_position2 = max_ee_pose
    print(target_position)
    while not rospy.is_shutdown():
        diff = (target_position - current_ee_pose)*0.1
        s_target_position = current_ee_pose + diff
        if np.linalg.norm(target_position - current_ee_pose) > 0.01:
            goal = set_ee_pose(s_target_position[0],s_target_position[1],s_target_position[2])
            robot_pub.publish(goal)
        else:
            print("reach")
            break
        command = gripper_control_command(0, command)
        rate.sleep()
    
    

def gripper_control():
    global gripper_pub, rate
    command = gripper_init()
    print(command)
    while not rospy.is_shutdown():
        command = gripper_control_command(0, command)
        rate.sleep()



if __name__ == '__main__':
    current_ee_pose = np.zeros(3)
    rospy.init_node('test_pub', anonymous=False)
    robot_pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=1)
    gripper_pub = rospy.Publisher("Robotiq2FGripperRobotOutput", outputMsg.Robotiq2FGripper_robot_output, queue_size=10)
    rospy.Subscriber('/iiwa/state/CartesianPose', CartesianPose, get_ee_pose_callback)
    rate = rospy.Rate(10) # 10hz

    add_thread = threading.Thread(target=thread_job)
    add_thread.start()
    
    # gripper_control()
    
    robot_control()
    
    
    
