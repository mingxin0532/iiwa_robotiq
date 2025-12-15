import rospy
import time
import math
from iiwa_msgs.msg import CartesianPose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
import threading
import numpy as np
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
import tf.transformations as tft
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
def set_ee_pose(x,y,z,yaw_deg_clockwise=0.0):
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
    velocity_goal.header.frame_id = 'ee_link'
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

def get_init_position():
    return np.array([0.6,0.0,0.2])

def reset_pos():
    global robot_pub, current_ee_pose
    init_positon = get_init_position()
    while np.linalg.norm(init_positon - current_ee_pose) > 0.01:
        goal = set_ee_pose(init_positon[0],init_positon[1],init_positon[2],yaw_deg_clockwise=0)
        robot_pub.publish(goal)
        rate.sleep()
    print("reach init position")
    return True



# use thread to subscribe the current ee pose
def thread_job():
    rospy.spin()
    
# multipy action function
def multipy_move_action(target_position):
    while np.linalg.norm(target_position - current_ee_pose) > 0.01:
        distance = target_position - current_ee_pose
        print(distance)
        diff = (target_position - current_ee_pose)*0.2
        s_target_position = current_ee_pose + diff
        goal = set_ee_pose(s_target_position[0],s_target_position[1],s_target_position[2])
        robot_pub.publish(goal)

    print("reach",target_position)

def publish_velocity(target_positon):
    max_speed = 150

    while not rospy.is_shutdown():
        distance = target_positon - current_ee_pose
        print(distance)
        # print(f'distance is {distance}')s
        if np.linalg.norm(distance) < 0.03:
            velocity_goal = set_ee_twist(0,0,0)
            robot_velocity_pub.publish(velocity_goal)
            break
        diff = distance*max_speed

        velocity_goal = set_ee_twist(diff[0],diff[1],diff[2])
        robot_velocity_pub.publish(velocity_goal)
        
        rate.sleep()
    
    velocity_goal = set_ee_twist(0,0,0)
    robot_velocity_pub.publish(velocity_goal)

def test_velocity():
    while not rospy.is_shutdown():
        velocity_goal = set_ee_twist(0,-5,0)
        print(velocity_goal)
        robot_velocity_pub.publish(velocity_goal)
        rate.sleep()


def test_gripper():
    command = gripper_init()
    print(command)
    rospy.sleep(1.0)
    command = gripper_control_command(140,command)
    print(f'next :{command}')
    rospy.sleep(1.0)
    command = gripper_control_command(-1,command)
    print(f'next :{command}')



def robot_move_control():
    global robot_pub, gripper_pub, current_ee_pose, rate
    
     #gripper init
    command = gripper_init()
    print(command)
    
    init_ee_pose = np.array([0.6,0.0,0.37])
    max_ee_pose = np.array([0.7,0.1,0.47])
    
    # reset position
    while not rospy.is_shutdown():
        if reset_pos():
            print("reset")
            break
        
    rospy.sleep(1.0)
    
    # go to the target position
    tar1 = np.array([0.6,0.0,0.37])
    tar2 = tar1 + np.array([])
    tar3 = tar2 + np.array([0.0,0.2,0.0])
    tar4 = tar3 + np.array([0.0,0.0,-0.084])
    target_position = [tar1,tar2,tar3,tar4]
    for index,target in enumerate(target_position):
        if index == 1:
            print("grab in target0")
            command = gripper_control_command(255,command)
            rate.sleep() 
        print(f"moving to target{index}:{target}")
        #publish_velocity(target)
        multipy_move_action(target)
    print("All target reached!")
    command = gripper_control_command(50,command)
    print("release in target4")
    rate.sleep() 
    rospy.sleep(1.0)
    reset_pos()



    
    

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
    robot_velocity_pub = rospy.Publisher('/iiwa/command/CartesianVelocity',TwistStamped,queue_size=1)
    gripper_pub = rospy.Publisher("Robotiq2FGripperRobotOutput", outputMsg.Robotiq2FGripper_robot_output, queue_size=10)
    rospy.Subscriber('/iiwa/state/CartesianPose', CartesianPose, get_ee_pose_callback)
    rate = rospy.Rate(20) # 10hz

    add_thread = threading.Thread(target=thread_job)
    add_thread.start()

    # gripper_control()
    # test_velocity()
    robot_move_control()
    # test_gripper()
    
