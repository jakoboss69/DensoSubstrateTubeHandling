from time import sleep

from LabView_Python.PythonDENSO.denso_controller import DensoController, CartesianPose, JointPose, TransPose, Interpolation


def main():
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

        # Set the speed of the robot
        internal = denso_controller.set_internal_speed(100)
        print(f"{internal = }")
        external = denso_controller.set_external_speed(10)
        print(f"{external = }")

        # Turn on the motors:
        servo_status = denso_controller.set_servo(True)
        print(f"{servo_status = }")

        position_cart = denso_controller.get_position_cart()
        position_cart.z = position_cart.z + 200
        denso_controller.move_to_cart(position_cart, interpolation=Interpolation.Move_L)
        print()
        # move to home position
        home_position_cart = CartesianPose(350, 0, 450, 180, 0, 180, 5)
        denso_controller.move_to_cart(home_position_cart, interpolation=Interpolation.Move_L)
        print('done with cart home position')

        # Move in a rectangular motion:

        

        # Turn off the motors:
        servo_status = denso_controller.set_servo(False)
        print(f"{servo_status = }")

        # Deactivate the RoboSlave.pac program in the robot controller:
        toolkit_status = denso_controller.set_toolkit(False)
        print(f"{toolkit_status = }")

    # Close the DENSO robot session:
    session = denso_controller.close_session()
    print(f"{session = }")


if __name__ == '__main__':
    main()