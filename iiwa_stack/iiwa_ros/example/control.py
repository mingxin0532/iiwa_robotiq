import rospy
from iiwa_msgs.msg import CartesianPose
from geometry_msgs.msg import PoseStamped
import tf
import threading
import time

def talker():
    pub = rospy.Publisher('/iiwa/command/CartesianPose', CartesianPose, queue_size=1)
    
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        goal = CartesianPose()
        goal.poseStamped
        goal.poseStamped.header.seq = 1
        goal.poseStamped.header.stamp = rospy.Time.now()
        goal.poseStamped.header.frame_id = "world" #world iiwa_link_0

        goal.poseStamped.pose.position.x = 0.4
        goal.poseStamped.pose.position.y = 0.4
        goal.poseStamped.pose.position.z = 0.74
        quaternion = tf.transformations.quaternion_from_euler(0.0, 1.5707963, 0.0)

        goal.poseStamped.pose.orientation.x = quaternion[0]
        goal.poseStamped.pose.orientation.y = quaternion[1]
        goal.poseStamped.pose.orientation.z = quaternion[2]
        goal.poseStamped.pose.orientation.w = quaternion[3]
        
        rospy.loginfo(goal)
        pub.publish(goal)
        rate.sleep()

def get_ee_pose_callback(msg):
    rospy.loginfo(msg.poseStamped.pose)
    global ee_pose
    ee_pose = msg.poseStamped
    # print(msg.poseStamped.pose)

def thread_job():
    rospy.spin()

def listener():
    rospy.Subscriber('/iiwa/state/CartesianPose', CartesianPose, get_ee_pose_callback)
    add_thread = threading.Thread(target=thread_job)
    add_thread.start()
    while(1):
        time.sleep(0.1)
        print(1)

def ee_position_pub(x,y,z,ee_pos_pub):
    print("ee_pose_pub")
    ee_pose = PoseStamped()
    ee_pose.header.seq = 1
    ee_pose.header.stamp = rospy.Time.now()
    ee_pose.pose.position.x = x
    ee_pose.pose.position.y = y
    ee_pose.pose.position.z = z
    ee_pose.pose.orientation.x = 0
    ee_pose.pose.orientation.y = 1
    ee_pose.pose.orientation.z = 0
    ee_pose.pose.orientation.w = 0.01747
    
    ee_pos_pub.publish(ee_pose)


if __name__ == '__main__':
    print("aaaa")
    ee_pose = PoseStamped()
    rospy.init_node('robot_command_pub', anonymous=False)
    # listener()
    ee_pos_pub = rospy.Publisher('eePose', PoseStamped, queue_size=10)
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        ee_position_pub(0.6,0.1,0.37,ee_pos_pub)
        rate.sleep()
    
    
    
    
    