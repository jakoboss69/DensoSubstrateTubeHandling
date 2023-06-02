from time import strftime, sleep
import sys
sys.path.insert(1, '../RobotProgramMaster/RealSense')
from RealSense2 import getSnapShot
from LabView_Python.PythonDENSO.denso_controller import DensoController, CartesianPose, JointPose, TransPose, Interpolation
import numpy as np
import cv2 as cv2
from CameraCalibration.run_calibration_function import run_calibration


def main():
    preCalib = False
    log = False
    timestamp = strftime("%Y%m%d-%H%M%S")

    
    robotPoses2 = np.loadtxt("CameraCalibration\\LargeChessboardPosesForCalibration24.04.csv", delimiter=",")
    robotPoses2_cart = []
    for i in robotPoses2:
        robotPoses2_cart.append(CartesianPose(i[0], i[1], i[2], i[3], i[4], i[5], 5))

    # robotPoses for large chessboard
    # robotPoses2 = np.loadtxt("CameraCalibration\\LargeChessboardPosesForCalibrationJOINT31.03.csv", delimiter=",")
    # robotPoses2_joint = []
    # for i in robotPoses2:
    #     robotPoses2_joint.append(JointPose(i[0], i[1], i[2], i[3], i[4], i[5]))

    
    # Connect to LabVIEW program controlling the robot:
    denso_controller = DensoController()
    connected = denso_controller.connect("tcp://192.168.100.100", "5555")
    # connected = denso_controller.connect("tcp://localhost", "5555")
    print(f"{connected = }")


    if connected:
        # Start a DENSO robot session:
        session = denso_controller.open_session()
        print(f"{session = }")

        # Activate the RoboSlave.pac program in the robot controller:
        toolkit_status = denso_controller.set_toolkit(True)
        print(f"{toolkit_status = }")

        # denso_controller.set_tool_params(-40, 0, 130, 0, 0, -2.5, 1)

        denso_controller.set_active_tool(0)

        # Set the speed of the robot
        internal = denso_controller.set_internal_speed(100)
        print(f"{internal = }")
        external = denso_controller.set_external_speed(10)
        print(f"{external = }")

        # Turn on the motors:
        servo_status = denso_controller.set_servo(True)
        print(f"{servo_status = }")

        print()
        print("Going to home position...", end="")
        # move to home position
        home_position_cart = CartesianPose(350, 0, 450, 180, 0, 180, 5)
        denso_controller.move_to_cart(home_position_cart, interpolation=Interpolation.Move_P)
        print('done with cart home position')

        images = []
        robot_poses = []
        depth_frames = []

    

        a = 1
        for pose in robotPoses2_cart:
            # go to position 1
            denso_controller.move_to_cart(pose, interpolation=Interpolation.Move_L)

            gray, depth_frame = getSnapShot(log = log, timestamp = timestamp)

            # save image in Log\\GrayImages\\24.04\\Calibration Images
            if log:
                cv2.imwrite(f"Log\\GrayImages\\10.05\\Calibration Images\\{a}.png", gray)

            print(a)
            a += 1
            # add image to list
            images.append(gray)
            depth_frames.append(depth_frame)
            # add pose to list
            pose = denso_controller.get_position_transformation_matrix()
            # turn from numpy array to list
            # pose = pose.tolist()
            # print('pose', pose[0])
            robot_poses.append(pose[0])
            print(robot_poses[0])

        # reshape robot_poses to fit numpy save function
        # robot_poses = np.array(robot_poses)

        # robot_poses_for_save = robot_poses.reshape(len(robot_poses), 16)

        # np.savetxt("Log\\TransformationMatrices\\BigChessboardRobotPoses.csv", robot_poses_for_save, delimiter=",")

        # denso_controller.move_to_joint(robotPoses2_joint[-1], interpolation=Interpolation.Move_L)
        
        # Set the speed of the robot again
        internal = denso_controller.set_internal_speed(100)
        print(f"{internal = }")
        external = denso_controller.set_external_speed(10)
        print(f"{external = }")
            
        denso_controller.move_to_cart(home_position_cart, interpolation=Interpolation.Move_P)


        # Turn off the motors:
        servo_status = denso_controller.set_servo(False)
        print(f"{servo_status = }")

        # Deactivate the RoboSlave.pac program in the robot controller:
        toolkit_status = denso_controller.set_toolkit(False)
        print(f"{toolkit_status = }")

    # Close the DENSO robot session:
    session = denso_controller.close_session()
    print(f"{session = }")

    T_BC = run_calibration(images, robot_poses, log = log,  timestamp = timestamp, preCalib=preCalib)[0]
    # run calibration and save results in T_BC.csv
    print('T_BC', T_BC)
    

    # blir gjort i run_calibration
    # np.savetxt("Log\\TransformationMatrices\\latestT_BC.csv", T_BC, delimiter=",")

if __name__ == '__main__':
    main()
