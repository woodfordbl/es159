from toolkit import robo_toolkit as rt
from toolkit import robo_commands as rc
from lab3.shapes import plot_square
import numpy as np

# Set np print options to display 4 decimal places
np.set_printoptions(precision=3, suppress=True)

points = plot_square(0, 500, 500, 100, 10)

# Now we want to create the joint angles that will move the arm to the elipse
lab_robot = rt.ScrewLabRobot()

guess = [0, 0, 0, 0, 0, 0]
time = 10
pose = np.eye(3)

angles, velocities =  rt.plot_path(robot=lab_robot, coords=points, guess=guess, pose=pose, time=time)

armCmd, robotCmd, velCmd = rc.init_robot()
time_step = 2

for angle in angles:
    message = rc.create_position_message(angle, velocities, time=2)
    rc.publish_position_message(message=message, armCmd=armCmd, robotCmd=robotCmd)