import numpy as np
from toolkit import robo_toolkit as rt
from toolkit import robo_commands as rc

armCmd, robotCmd, velCmd = rc.init_robot()


# Read the coordinates from the file and move the robot to the coordinates
path = np.loadtxt('lab4.csv', delimiter=',')

# Initialize the robot
robot = rt.ScrewLabRobot()

# Get joint angles for start position

rest = np.array([[-1,0,0,665],
                 [0,1,0,0],
                 [0,0,-1,100],
                 [0,0,0,1]])

guess = [-np.pi/18, -np.pi/3, np.pi/2, -2*np.pi/3, -np.pi/2, 4*np.pi/9]

rest_position = robot.ikine(rest, theta=guess)

x, y, z = path[0]

start = np.array([[-1,0,0,x],
                  [0,1,0,y],
                  [0,0,-1,100],
                  [0,0,0,1]])

start_position = robot.ikine(start, theta=rest_position)

thetas = [start_position]

for coord in path:
    x, y, z = coord

    pose = np.array([[-1,0,0,x],
                    [0,1,0,y],
                    [0,0,-1,z],
                    [0,0,0,1]])
    guess = thetas[-1]

    robot_position = robot.ikine(pose, theta=guess)    
    thetas.append(robot_position)

# Add the end position to the path
end_position = robot.fkine(thetas[-1])
end_position[2,3] = 100
thetas.append(robot.ikine(end_position, theta=thetas[-1]))

# Move the robot to the rest position
rc.publish_position_message(rc.create_position_message(rest_position, [0,0,0,0,0,0], time=1), armCmd, robotCmd)

# Move the robot to the start position
rc.publish_position_message(rc.create_position_message(start_position, [0,0,0,0,0,0], time=2), armCmd, robotCmd)

# Loop through the path and move the robot to each position
for i in range(len(thetas)):
    rc.publish_position_message(rc.create_position_message(thetas[i], [0,0,0,0,0,0], time=1), armCmd, robotCmd)

