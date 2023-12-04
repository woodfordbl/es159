import json
import numpy as np

import vision
from lab4 import search

from toolkit import robo_toolkit as rt
from toolkit import robo_commands as rc


#with open('lab5.json') as f:
#    data = json.load(f)

armCmd, robotCmd, velCmd, gripper_srv = rc.init_robot()

image = rc.get_image()
width = image.width
height = image.height
data = image.data

cropx = [150,400]
cropy = [50,550]

x1_actual = 340
x2_actual = 715
y1_actual = -400
y2_actual = 425

# Load image
img = vision.image(data=data, cropx=cropx,cropy=cropy)
img.crop()
img.showcolor()

red_threshold = 200 
green_threshold = 140 
blue_threshold = 160

# Red filter to find scale of image
red_filtered = img.filter(r=[red_threshold, 255], b=[0,125], g=[0,125])
blue_filtered = img.filter(r=[0,125], b=[blue_threshold,255], g=[0,125])
green_filtered = img.filter(r=[0,125], b=[0,125], g=[green_threshold, 255])

(x1, y1), (x2, y2) = img.create_scale(red_filtered)

# Get the scale of the image
x_scale = (x2_actual-x1_actual) / (x2-x1)
y_scale = (y2_actual-y1_actual) / (y2-y1)

# Search for the destination
pickup = img.find_square(filtered_img=blue_filtered, size=12, threshold=0.75)
dropoff = img.find_square(filtered_img=green_filtered, size=35, threshold=0.9)

# Convert to real world coordinates
def convert_to_real_world_coordinates(point, x1, y1, x_scale, y_scale, x1_actual, y1_actual):
    x = (point[0] - x1) * x_scale + x1_actual
    y = (point[1] - y1) * y_scale + y1_actual
    return (x, y)
pickup = convert_to_real_world_coordinates(pickup, x1, y1, x_scale, y_scale, x1_actual, y1_actual)
dropoff = convert_to_real_world_coordinates(dropoff, x1, y1, x_scale, y_scale, x1_actual, y1_actual)

lab_robot = rt.ScrewLabRobot()

z_pickup = -130
z_move = -100

pose = np.array([[-1,0,0],[0,1,0],[0,0,-1]])


# ----------------------
# Construct commands:
# ----------------------


# Initialize the robot

def create_intermediate_points(startxy, endxy, granularity=10):
    x = np.linspace(startxy[0], endxy[0], granularity)
    y = np.linspace(startxy[1], endxy[1], granularity)
    return np.array([x,y]).T

def convert_to_angles(path, z, pose, init_angles, lab_robot):
    angles = []
    angles.append(init_angles)

    target = np.eye(4)
    target[:3,:3] = pose
    target[2,3] = z

    for i in range(len(path)):
        x = path[i][0]
        y = path[i][1]

        target[0,3] = x
        target[1,3] = y

        angles.append(lab_robot.ikine(T_d = target, theta = angles[-1]))
    return np.array(angles[1:])

# Go to start position
start_position = np.array([[-1,0,0,665],
                           [0,1,0,0],
                           [0,0,-1,100],
                           [0,0,0,1]])
startxy = (665, 0)

guess = np.array([-10, -60, 90, -120, -90, 80]) * (np.pi / 180)
start_angle = lab_robot.ikine(T_d = start_position, theta = guess)

rc.publish_position_message(rc.create_position_message(start_angle, [0,0,0,0,0,0], time=1), armCmd, robotCmd)

# Go to pick up position
pickup_path = create_intermediate_points(startxy, pickup)
pickup_path_angles = convert_to_angles(pickup_path, z_move, pose, start_angle, lab_robot)
for angle in pickup_path_angles:
    rc.publish_position_message(rc.create_position_message(angle, [0,0,0,0,0,0], time=1), armCmd, robotCmd)

# Pick up the object
pickup_pose = lab_robot.fkine(pickup_path_angles[-1])
pickup_pose[2,3] = z_pickup
pickup_angles = lab_robot.ikine(T_d = pickup_pose, theta = pickup_path_angles[-1])
rc.open_gripper(gripper_srv)
rc.publish_position_message(rc.create_position_message(pickup_angles, [0,0,0,0,0,0], time=1), armCmd, robotCmd)
rc.close_gripper(gripper_srv)

# Return to z_move 
z_return = pickup_path_angles[-1]
rc.publish_position_message(rc.create_position_message(z_return, [0,0,0,0,0,0], time=1), armCmd, robotCmd)

# Go to drop off position
dropoff_path = create_intermediate_points(pickup, dropoff)
dropoff_path_angles = convert_to_angles(dropoff_path, z_move, pose, pickup_angles, lab_robot)
for angle in dropoff_path_angles:
    rc.publish_position_message(rc.create_position_message(angle, [0,0,0,0,0,0], time=1), armCmd, robotCmd)

# Drop off the object
dropoff_pose = lab_robot.fkine(dropoff_path_angles[-1])
dropoff_pose[2,3] = z_pickup
rc.publish_position_message(rc.create_position_message(dropoff_path_angles[-1], [0,0,0,0,0,0], time=1), armCmd, robotCmd)
rc.open_gripper(gripper_srv)


# Return to normal position
end_angles = np.array([0, -np.pi/2, 0, 0, 0, 0])
rc.publish_position_message(rc.create_position_message(end_angles, [0,0,0,0,0,0], time=1), armCmd, robotCmd)