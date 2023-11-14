from toolkit import robo_toolkit as rt
from toolkit import robo_commands as rc
from lab3.shapes import plot_square
import numpy as np
from time import sleep

# Set np print options to display 4 decimal places
np.set_printoptions(precision=3, suppress=True)

n = 10
points = plot_square(0, 500, 500, 100, n)

# Now we want to create the joint angles that will move the arm to the elipse
lab_robot = rt.ScrewLabRobot()

guess = [0, 0, 0, 0, 0, 0]
time = 10


armCmd, robotCmd, velCmd = rc.init_robot()


T = np.array([[0, -1, 0, 0],[0, 0, -1, 0],[1, 0, 0 ,0],[0, 0, 0, 1]])

for i, coord in enumerate(points):

    # Calculate the inverse kinematics for the given coordinates
    # Create the pose
    T[:3, 3] = coord

    current_pos = rc.get_end_effector_position()[0]
    
    print("Current position:", current_pos)

    joint_angle = lab_robot.ikine(T, guess)

   
