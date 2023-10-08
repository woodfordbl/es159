import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import robo_commands as rc
import numpy as np
import time

armCmd, robotCmd = rc.init_robot()

def init_robot():

    shoulder_pan_joint = 1
    shoulder_lift_joint = -1 # Has to be -1 or goes into floor
    elbow_joint = 1
    wrist_1_joint = 1
    wrist_2_joint = 1
    wrist_3_joint = 1


    positions = [shoulder_pan_joint,shoulder_lift_joint,elbow_joint,wrist_1_joint,wrist_2_joint,wrist_3_joint]

    velocities = [0,0,0,0,0,0]


    message = rc.create_message(positions=positions, velocities=velocities, time=8)

    rc.publish_message(message, armCmd, robotCmd)
    return

def prelab_a():
    shoulder_pan_joint = 0 * np.pi
    shoulder_lift_joint = -.5 * np.pi
    elbow_joint = 0 * np.pi
    wrist_1_joint = -.5 * np.pi
    wrist_2_joint = .5 * np.pi
    wrist_3_joint = .5 * np.pi


    positions = [shoulder_pan_joint,shoulder_lift_joint,elbow_joint,wrist_1_joint,wrist_2_joint,wrist_3_joint]

    velocities = [0,0,0,0,0,0]


    message = rc.create_message(positions=positions, velocities=velocities, time=8)

    rc.publish_message(message, armCmd, robotCmd)
    return

def prelab_b():
    def position_1(): # Go to start corner
        shoulder_pan_joint = 0 * np.pi
        shoulder_lift_joint = 0 * np.pi
        elbow_joint = 0 * np.pi
        wrist_1_joint = .5 * np.pi
        wrist_2_joint = .5 * np.pi
        wrist_3_joint = .5 * np.pi


        positions = [shoulder_pan_joint,shoulder_lift_joint,elbow_joint,wrist_1_joint,wrist_2_joint,wrist_3_joint]

        velocities = [0,0,0,0,0,0]


        message = rc.create_message(positions=positions, velocities=velocities, time=8)

        rc.publish_message(message, armCmd, robotCmd)
        print("Completed position 1")
        return
    def position_2():
        shoulder_pan_joint = 0 * np.pi
        shoulder_lift_joint = -.125 * np.pi
        elbow_joint = .25 * np.pi
        wrist_1_joint = .375 * np.pi
        wrist_2_joint = .5 * np.pi
        wrist_3_joint = .5 * np.pi


        positions = [shoulder_pan_joint,shoulder_lift_joint,elbow_joint,wrist_1_joint,wrist_2_joint,wrist_3_joint]

        velocities = [0,0,0,0,0,0]


        message = rc.create_message(positions=positions, velocities=velocities, time=8)

        rc.publish_message(message, armCmd, robotCmd)
        print("Completed position 2")
        return
    def position_3():
        shoulder_pan_joint = .05 * np.pi
        shoulder_lift_joint = -.125 * np.pi
        elbow_joint = .25 * np.pi
        wrist_1_joint = .375 * np.pi
        wrist_2_joint = .5 * np.pi
        wrist_3_joint = .5 * np.pi


        positions = [shoulder_pan_joint,shoulder_lift_joint,elbow_joint,wrist_1_joint,wrist_2_joint,wrist_3_joint]

        velocities = [0,0,0,0,0,0]


        message = rc.create_message(positions=positions, velocities=velocities, time=8)

        rc.publish_message(message, armCmd, robotCmd)
        print("Completed position 3")
        return
    def position_4():
        shoulder_pan_joint = .05 * np.pi
        shoulder_lift_joint = 0 * np.pi
        elbow_joint = 0 * np.pi
        wrist_1_joint = .5 * np.pi
        wrist_2_joint = .5 * np.pi
        wrist_3_joint = .5 * np.pi


        positions = [shoulder_pan_joint,shoulder_lift_joint,elbow_joint,wrist_1_joint,wrist_2_joint,wrist_3_joint]

        velocities = [0,0,0,0,0,0]


        message = rc.create_message(positions=positions, velocities=velocities, time=8)

        rc.publish_message(message, armCmd, robotCmd)
        print("Completed position 4")
        return
    
    # Go through all the positions, wait for it to finish, return back to 1
    position_1()
    time.sleep(10)
    position_2()
    time.sleep(10)
    position_3()
    time.sleep(10)
    position_4()
    time.sleep(10)
    position_1
    
    return

init_robot()
time.sleep(12)
#prelab_a
prelab_b()