from toolkit import robo_toolkit as rt
from toolkit import robo_commands as rc
import numpy as np
from time import sleep
import csv


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

pos_dict = {
    "pos1": {
        "pred": [],
        "tested": {
            "1": [],
        }
    }
}

with open('measured_and_predicted_values.csv', mode='w', newline='') as file:
    # Define the CSV writer
    csv_writer = csv.writer(file)

    # Write a header row with column names
    csv_writer.writerow(["Position Index", "Predicted Position", "Actual Position"])

    for j in range(0, 5):
        for i, config in enumerate(configurations):
            positions = config
            velocities = [0, 0, 0, 0, 0, 0]
            prediction = lab_robot.fkine(config)[:3, 3]
            message = rc.create_position_message(positions=positions, velocities=velocities, time=time)
            rc.publish_message(message, armCmd, roboCmd)
            sleep(time + 2)

            pos, rot = rc.get_end_effector_position()
            # Convert from m to mm
            pos = np.array(pos) * 1000

            # Write the values to the CSV file
            csv_writer.writerow([i, prediction, pos])
            print(f"Configuration {i+1} completed")
        print(f"Test {j+1} of 5 completed")
