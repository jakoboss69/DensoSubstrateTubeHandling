from time import sleep, strftime
from LabView_Python.PythonDENSO.denso_controller import DensoController, CartesianPose, JointPose, TransPose, Interpolation
import numpy as np
import Gripper.gripper_controller as gripper
from FindObjects.DeterminePosition3 import determinePosition
from CameraCalibration.calibrateTub3 import calibrateTub



def main():
    # set log to True to log most data from test
    log = False
    visualize = False
    timestamp = strftime("%Y%m%d-%H%M%S")
    griptest = True
    store_pointcloud = False
    if griptest:
        # open serial connection to arduino
        ser = gripper.openSerial(8)
        gripper.grip(ser, '1') # open gripper

    # getting the latest T_BC matrix from the file CameraCalibration\\SavedMatrices\\T_BC.csv. If camera has been moved this needs to be updated by running the calibration script
    path = "Log\\TransformationMatrices\\"
    T_BC = np.loadtxt(path+'latestT_BC.csv', delimiter=',')

    # Connect to LabVIEW program controlling the robot:
    denso_controller = DensoController()
    connected = denso_controller.connect("tcp://192.168.100.100", "5555")
    print(f"{connected = }")

    if connected:
        # Start a DENSO robot session:
        session = denso_controller.open_session()
        print(f"{session = }")


        # Activate the RoboSlave.pac program in the robot controller:
        toolkit_status = denso_controller.set_toolkit(True)
        print(f"{toolkit_status = }")


        # Set the speed of the robot, maximum 10 external when testing
        internal = denso_controller.set_internal_speed(100)
        print(f"{internal = }")
        external = denso_controller.set_external_speed(15)
        print(f"{external = }")

        # Turn on the motors:
        servo_status = denso_controller.set_servo(True)
        print(f"{servo_status = }")

        # set and move to home position. using tool 0 to ensure same every time
        set_active_tool = denso_controller.set_active_tool(0)
        print(f"{set_active_tool = }")

        home_position_cart = CartesianPose(320, 0, 650, 180, 0, 180, 5)
        denso_controller.move_to_cart(home_position_cart, interpolation=Interpolation.Move_L)
        print(f"{home_position_cart = }")

        # set tool offsets and select tool
        set_tool_1_params = denso_controller.set_tool_params(-50, 0, 80, 0, 0, -2.5, 1)
        print(f'{set_tool_1_params = }')

        set_active_tool = denso_controller.set_active_tool(1)
        print(f"{set_active_tool = }")


        # run calibration of tub to get T_A1C and T_A2C
        tries = 0
        calibration = None
        while calibration == None:
            calibration = calibrateTub(log = log, timestamp = timestamp, visualize=visualize, amount = 2)
            tries += 1
            if tries > 3 and calibration == None:
                print('Calibration failed')
                
        # get tube position
        tube_frames = determinePosition(log = log, timestamp = timestamp, visualize=visualize, store_pointcloud=store_pointcloud)
        if tube_frames == False:
            print("No objects found, trying again")
            tube_frames = determinePosition(log = log, timestamp = timestamp, visualize=visualize)
            if tube_frames == False:
                print("Sorry homie, no tube found 2nd attempt either")
                


        if tube_frames == False:
            print('No objects found')
            # Move to home position:
            external = denso_controller.set_external_speed(10)
            print(f"{external = }")



            # Set the speed of the robot
            internal = denso_controller.set_internal_speed(100)
            print(f"{internal = }")

            external = denso_controller.set_external_speed(10)
            print(f"{external = }")    

            set_active_tool = denso_controller.set_active_tool(0)
            print(f"{set_active_tool = }")

            denso_controller.move_to_cart(home_position_cart,
                                            interpolation=Interpolation.Move_L) 
            print(f"{home_position_cart = }")

            # Turn off the motors:
            servo_status = denso_controller.set_servo(False)
            print(f"{servo_status = }")

            # Deactivate the RoboSlave.pac program in the robot controller:
            toolkit_status = denso_controller.set_toolkit(False)
            print(f"{toolkit_status = }")

            if griptest:
                # Close the serial connection to the arduino
                gripper.closeSerial(ser)

            # Close the DENSO robot session:
            session = denso_controller.close_session()
            print(f"{session = }")
            return
        
        else:
            # sort tube_frames by x value
            tube_frames = sorted(tube_frames, key=lambda x: x[0,3])


            tube = tube_frames[0]
            # move to tube

            tube = T_BC @ tube

            # make z translation +200mm
            tube[2,3] = tube[2,3] + 250

            denso_controller.move_to_transformation_matrix(tube, figure = 5, interpolation=Interpolation.Move_L)

            position = denso_controller.get_position_cart()

            print(position.rx, position.ry)
            position.rx = 180
            position.ry = 0
            denso_controller.move_to_cart(position, interpolation=Interpolation.Move_L)
            sleep(2)
            if griptest:
                position.z = position.z - 270
            else:
                position.z = position.z - 180
            denso_controller.set_external_speed(5)

            denso_controller.move_to_cart(position, interpolation=Interpolation.Move_L)
            gripped = False
            tries = 0
            if griptest:
                for i in range(12):
                    if not gripper.ready_to_grip(ser):
                        position.z = position.z -3
                        denso_controller.set_external_speed(5)

                        denso_controller.move_to_cart(position, interpolation=Interpolation.Move_L)
                        tries += 1
                        if tries > 12:
                            print('gripper not gripping')
                            break
                    else:
                        gripper.grip(ser, '0')
                        gripped = True
                        break
    

                
                            
            position.z = position.z + 200 + tries*3

        denso_controller.move_to_cart(position, interpolation=Interpolation.Move_L)


        # Move to home position:
        external = denso_controller.set_external_speed(15)
        print(f"{external = }")



        # Set the speed of the robot
        internal = denso_controller.set_internal_speed(100)
        print(f"{internal = }")

        external = denso_controller.set_external_speed(10)
        print(f"{external = }")    

        set_active_tool = denso_controller.set_active_tool(0)
        print(f"{set_active_tool = }")

        denso_controller.move_to_cart(home_position_cart,
                                        interpolation=Interpolation.Move_L) 
        print(f"{home_position_cart = }")



        # Turn off the motors:
        servo_status = denso_controller.set_servo(False)
        print(f"{servo_status = }")

        # Deactivate the RoboSlave.pac program in the robot controller:
        toolkit_status = denso_controller.set_toolkit(False)
        print(f"{toolkit_status = }")

        # Close the serial connection to the arduino

        # Close the DENSO robot session:
        session = denso_controller.close_session()
        print(f"{session = }")
        if griptest:
            if gripped:
                input("You have 10 seconds to catch the tube after pressing enter")
                sleep(10)
                gripper.grip(ser, '1')

            gripper.closeSerial(ser)

        


if __name__ == '__main__':
    for i in range(1):
        main()
        input("start next run")