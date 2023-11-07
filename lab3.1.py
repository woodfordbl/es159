from toolkit import robo_toolkit as rt
import matplotlib.pyplot as plt
#from toolkit import robo_commands as rc
from lab3.shapes import plot_square
import numpy as np

# Set np print options to display 4 decimal places
np.set_printoptions(precision=3, suppress=True)

points = plot_square(100, 500, 500, 100, 10)

# Now we want to create the joint angles that will move the arm to the elipse
lab_robot = rt.ScrewLabRobot()

guess = [0, 0, 0, 0, 0, 0]
time = 10
pose = np.eye(3)

angles, velocities =  rt.plot_path(robot=lab_robot, coords=points, guess=guess, pose=pose, time=time)

points = []
for angle in angles:
    pose = lab_robot.fkine(angle)
    coord = pose[0:3, 3]
    points.append(coord)

# Plot the points
points = np.array(points)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(points[:, 0], points[:, 1], points[:, 2], marker='o')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xlim([90, 110])
ax.set_ylim([450, 550])
ax.set_zlim([450, 550])
plt.show()




#armCmd, robotCmd, velCmd = rc.init_robot()
#time_step = 2


"""
for angle in angles:
    # convert angle to list
    message = rc.create_position_message(angle, velocities, time=2)
    rc.publish_position_message(message=message, armCmd=armCmd, robotCmd=robotCmd)
"""