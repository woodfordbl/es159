from toolkit import robo_toolkit as rt
from toolkit import robo_commands as rc
import numpy as np
from time import sleep


# Set print options
np.set_printoptions(precision=3, suppress=True)
lab_robot = rt.LabRobot()

configurations = [
    [-3*np.pi/4, -3*np.pi/4, -np.pi/4, -np.pi/4, np.pi/4, 0],
    [-np.pi/2, 0, -13*np.pi/18, -np.pi/2, -np.pi/4, np.pi/2],
    [-np.pi/2, -3*np.pi/40, -5*np.pi/12, 0, 4*np.pi/9, np.pi],
    [-np.pi/2, -np.pi/4, -7*np.pi/18, 0, -25*np.pi/36, -np.pi/2],
    [-13*np.pi/18, -25*np.pi/36, np.pi/2, -np.pi/9, 13*np.pi/36, np.pi/4],
    [0,0,0,0,0,0]]
"""
for i, config in enumerate(configurations):
    matrices = lab_robot.fkine(config, True)
    for j, m in enumerate(matrices):
        pos_m = m[:3,3]
        print(f"Configuration {i+1} | Link {j+1} --> {pos_m}")
        print()
        
    print("--"*30)
"""

armCmd, roboCmd = rc.init_robot()
time = 4

# TESTING
position = [0,0,0,0,0,0]
velocities = [0,0,0,0,0,0]
message = rc.create_position_message(positions=position, velocities=velocities, time=time)
rc.publish_message(message, armCmd, roboCmd)
sleep(time)

positions = rc.get_end_effector_position()
positions = list(positions.values())

print(f"Current joint positions: {positions}")


"""
for i, config in enumerate(configurations):
    positions = config
    velocities = [0,0,0,0,0,0]
    message = rc.create_message(positions=positions, velocities=velocities, time=time)
    rc.publish_message(message, armCmd, roboCmd)
    sleep(time)
    print(f"Completed position {i}")

    positions = rc.get_joint_positions()
    print(f"Current joint positions: {positions}")
    """
