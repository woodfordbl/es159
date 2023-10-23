import numpy as np
from toolkit import robo_toolkit as rt

# set np print options
np.set_printoptions(suppress=True, precision=4)

screw_lab_robot = rt.ScrewLabRobot()

pos = [-126.1509, 44.9418, 305.7310]
orient = np.array([[0.0327,0.0041,-0.9995],[0.1635,-0.9865,0.0013 ],[-0.9860,-0.1634 ,-0.0330]])

T = np.eye(4)

T[:3,3] = pos
T[:3,:3] = orient

print("Desired Pose: \n", T)
print()

guess = [0,0,0,0,0,0]
thetas = screw_lab_robot.ikine(T, guess)

print(thetas)
guess = [1,1,1,1,1,1]
thetas = screw_lab_robot.ikine(T, guess)
print(thetas)


est_pose = screw_lab_robot.fkine(thetas)

print("Estimated Pose: \n", est_pose)
