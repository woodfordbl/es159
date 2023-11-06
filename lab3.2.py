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
pose = np.eye(3)

angles, velocities =  rt.plot_path(robot=lab_robot, coords=points, guess=guess, pose=pose, time=time)

armCmd, robotCmd, velCmd = rc.init_robot()
time_step = time / n

for velo in velocities:
    message = rc.create_velocity_message(velo)
    rc.publish_velocity_message(message=message, velCmd=velCmd)
    sleep(time_step)
