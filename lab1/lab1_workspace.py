from toolkit import robo_toolkit as rt
import numpy as np

lab_robot = rt.LabRobot()

thetas = [
    0,
    [-np.pi/4, 0],
    [-np.pi/2,-np.pi/6],
    [-np.pi/4, np.pi/4],
    [0, np.pi/2],
    [np.pi/4, np.pi/4],
]

lab_robot.workspace(theta_values=thetas, granularity=4)