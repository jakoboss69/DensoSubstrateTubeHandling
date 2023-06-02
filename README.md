# RobotProgramMaster
Master's thesis python files for using a Denso VS-087 robot with LabView and a Intel RealSense camera for detecting and picking up tubes in a crate.

Setup:
    Denso VS-087 robot with controller set to auto

    Windows Computer with LabView installed and connected to the Denso VS-087

    Place incubator with long side towards the robot base and quite close

    Windows Computer running the Python scripts in this repository. 
        - If trouble with ArUco-marker library, try running this in the environment your are running. Open3d is also necessary.
            - pip3 uninstall opencv-python
            - pip3 uninstall opencv-contrib-python
            - pip3 install opencv-python
            - pip3 install opencv-contrib-python

    Incubator with an ArUco-marker mounted on camera mounted on the inside of the longest wall, and one on the inside of the shortest wall. 

    Intel RealSense D415 mounted facing down. Make sure the camera is mounted in such a way so that all ArUco markers are visible.

    Calibration hand-eye and camera intrinsics
        - Run the RunCalibrationDENSO.py with checkerboard mounted. Verify that the size of the squares on the checkerboard is the same as in chessboard_.py
        - Make sure the checkerboard is visible for the camera in all (most) positions
        - select precalibrated true or false

    AutonomousGrippingTest.py
        - If gripper connected, change this parameter accordingly
        - Log if wanted. The latest matrices for calibration is stored regardless 

    DensoGoUpAndGoHome.py
        - If collision bound, stop the robot and run this program if it is safe to move upward and back to home position

    


    