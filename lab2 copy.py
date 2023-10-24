import numpy as np
from time import sleep
from toolkit import robo_toolkit as rt
from toolkit import robo_commands as rc

# set np print options
np.set_printoptions(suppress=True, precision=4)

screw_lab_robot = rt.ScrewLabRobot()


T1 = np.array([[-0.2823, 0.4854, 0.8275, 19.7308],
              [-0.8220, -0.5672, 0.0522, -160.3067],
              [0.4947, -0.6654, 0.5591, 522.8790],
              [0, 0, 0, 1.0000]])

T2 = np.array([[-0.4520, 0.4788, 0.7526, 720.0903],
              [-0.8911, -0.2037, -0.4056, -293.0874],
              [-0.0409, -0.8540, 0.5187, 64.4169],
              [0, 0, 0, 1.0000]])

T3 = np.array([[0.6140, -0.6315, 0.4735, 142.6523],
              [0.6449, 0.0556, -0.7622, 536.8939],
              [0.4550, 0.7734, 0.4414, 209.8776],
              [0, 0, 0, 1.0000]])

T4 = np.array([[-0.3085, 0.9512, 0.0005, -148.3657],
              [0.0979, 0.0312, 0.9947, -163.9713],
              [0.9462, 0.3069, -0.1027, 475.3604],
              [0, 0, 0, 1.0000]])

T5 = np.array([[0.6240, -0.0565, -0.7793, -849.4136],
              [-0.7600, 0.1879, -0.6222, 397.2424],
              [0.1816, 0.9806, 0.0743, 347.8927],
              [0, 0, 0, 1.0000]])

T6 = np.array([[0.0327, 0.0041, -0.9995, -126.1509],
              [0.1635, -0.9865, 0.0013, 44.9418],
              [-0.9860, -0.1634, -0.0330, 305.7310],
              [0, 0, 0, 1.0000]])

# Create list of T matrices
T_list = [T1, T2, T3, T4, T5, T6]

# Iterate through and get each theta list and fkine
guess = [0, 0, 0, 0, 0, 0]
home = [0, -np.pi/2, 0, 0, 0, 0]

theta1 = screw_lab_robot.ikine(T2, guess, opt=False)
theta2 = screw_lab_robot.ikine(T2, guess, opt=True)
pos1 = screw_lab_robot.fkine(theta1)
pos2 = screw_lab_robot.fkine(theta2)

print(f"Position 1: {pos1}")
print(f"Position 2: {pos2}")

armCmd, roboCmd = rc.init_robot()
time = 10

message = rc.create_position_message(positions=theta2, velocities=[0, 0, 0, 0, 0, 0], time=time)
rc.publish_message(message, armCmd, roboCmd)