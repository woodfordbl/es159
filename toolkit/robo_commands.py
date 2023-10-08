#import rosnode
import tf
import time
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

# Define a global variable to store joint positions
joint_positions = {}
# Initialize a tf listener
listener = None

def joint_states_callback(msg):
    # Extract joint positions from the received message
    global joint_positions
    joint_positions = dict(zip(msg.name, msg.position))

def init_robot():
    rospy.init_node("Node1")

    armCmd = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
    robotCmd = rospy.Publisher('/scaled_pos_joint_traj_controller/command',JointTrajectory,queue_size=10)

    init_msg = JointTrajectory()


    p = JointTrajectoryPoint()
    p.positions = [0,0,1, 0,0,0]
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

    return armCmd, robotCmd

def get_end_effector_position():
    global listener
    try:
        # Look up the transformation from the base frame to the end effector frame
        (trans, rot) = listener.lookupTransform('/base_link', '/wrist_3_joint', rospy.Time(0))
        
        # 'trans' now contains the position as [x, y, z]
        return trans
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("Error in getting end effector position")
        return None

def publish_message(message, armCmd, robotCmd):

    time.sleep(1)
    armCmd.publish(message)
    robotCmd.publish(message)

    return

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
