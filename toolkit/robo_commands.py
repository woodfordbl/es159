#import rosnode
import tf
import time
import rospy
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32

from robotiq_hande_ros_driver.srv import gripper_service 

from sensor_msgs.msg import Image  # Assuming the message type is sensor_msgs/Image

#480x640 image
data = {
    'data': [1,2,3,4,5,6,7,8,9,10],
    'width': 640,
    'height': 480   
}
# Define a global variable to store joint positions
joint_positions = {}
# Initialize a tf listener
listener = None

def joint_states_callback(msg):
    # Extract joint positions from the received message
    global joint_positions
    joint_positions = dict(zip(msg.name, msg.position))

def init_robot(gripper=False):
    rospy.init_node("Node1")
    

    armCmd = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
    robotCmd = rospy.Publisher('/scaled_pos_joint_traj_controller/command',JointTrajectory,queue_size=10)
    velCmd = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)

       
    init_msg = JointTrajectory()


    p = JointTrajectoryPoint()
    p.positions = [-np.pi/2,-np.pi/2,0,-np.pi/2,0,0]
    p.velocities = [0,0,0,0,0,0]

    init_msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    init_msg.points = [p]

    p.time_from_start.secs = 10

    # Create a subscriber to listen to the joint states
    rospy.Subscriber("/joint_states", JointState, joint_states_callback)

    # Initialize the tf listener
    global listener
    listener = tf.TransformListener()

    time.sleep(1)
    armCmd.publish(init_msg)
    robotCmd.publish(init_msg)

    if gripper:
        gripperpub = rospy.Publisher('/gripper_service_es159', Float32MultiArray, queue_size=10)
        return armCmd, robotCmd, velCmd, gripperMsg

    return armCmd, robotCmd, velCmd

def get_end_effector_position():
    global listener
    try:
        # Look up the transformation from the base frame to the end effector frame
        (trans, rot) = listener.lookupTransform('/base_link', '/tool0_controller', rospy.Time(0))
        # 'trans' now contains the position as [x, y, z]
        return trans, rot
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print("Error:", e)
        return None
    
def get_current_joint_angles():
    global joint_positions
    return joint_positions

def publish_position_message(message, armCmd, robotCmd):

    time.sleep(1)
    armCmd.publish(message)
    robotCmd.publish(message)

    return

def publish_velocity_message(message, velCmd):
    time.sleep(1)
    velCmd.publish(message)
    return

def create_velocity_message(velocities):
    velMsg = Float64MultiArray()
    velMsg.data = velocities
    return velMsg

def create_position_message(positions, velocities, time=4): # Creates a command to send to the robot
    message = JointTrajectory()
    
    p = JointTrajectoryPoint()
    p.positions = positions
    p.velocities = velocities

    message.joint_names = ['shoulder_pan_joint', 
                           'shoulder_lift_joint', 
                           'elbow_joint', 
                           'wrist_1_joint', 
                           'wrist_2_joint', 
                           'wrist_3_joint']

    message.points = [p]

    p.time_from_start.secs = time

    return message

def open_gripper(gripperpub):
    gripperMsg = Float32MultiArray()
    gripperMsg.data = [0, 255, 255]

    # Publish the gripper message
    gripperpub.publish(gripperMsg)
    return

def close_gripper(gripperpub):

    gripperMsg = Float32MultiArray()
    gripperMsg.data = [255, 255, 255]

    # Publish the gripper message
    gripperpub.publish(gripperMsg)

    return

def get_image():
    rospy.init_node('image_receiver_node')  # Initialize the ROS node
    bridge = CvBridge()
    receiver = ImageReceiver()  # Create an instance of the ImageReceiver class

    rate = rospy.Rate(10)  # Adjust this rate according to your needs
    while not rospy.is_shutdown():
        if receiver.cam_msg is not None:
            image_data = bridge.imgmsg_to_cv2(receiver.cam_msg, desired_encoding="passthrough")
            byte_data = image_data.tobytes()  # Get the image data as a byte string
            int_data = list(byte_data)  # Convert byte string to a list of integers
            return ' '.join(str(x) for x in int_data)  # Convert to a space-separated string of integers
        rate.sleep()

class ImageReceiver:
    def __init__(self, topic='/usb_cam/image_raw'):
        self.cam_msg = None
        self.cam_sub = rospy.Subscriber(topic, Image, self.image_callback)

    def image_callback(self, data):
        self.cam_msg = data  # Update cam_msg with the received image data