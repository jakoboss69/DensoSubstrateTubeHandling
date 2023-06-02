import sys
sys.path.insert(1, '../RobotProgramMaster/RealSense')
import RealSense2 as gss
import cv2
import cv2.aruco as aruco
import numpy as np

length_marker = 89
def calibrateTub(preShot = False, precalib = False, visualize = False, log = False, timestamp = '', amount = 2):

    # get the images
    if preShot:

        grays = []

        for i in range(amount):
            grays.append(cv2.imread("Log\\GrayImages\\20230424-105707img"+str(7+1)+".png"))

    else:
        grays = gss.getSnapShotGray(amount=amount, log = log, timestamp = timestamp)

    # get the camera matrix and distortion coefficients
    if precalib:
        cameraMatrix, distortionMatrix, _, _, _ = gss.getRealSenseMatrices()
    else:
        cameraMatrix = np.loadtxt("Log\\Intrinsics\\latestCameraMatrix.csv", delimiter=",")
        distortionMatrix = np.loadtxt("Log\\Intrinsics\\latestDistortion.csv", delimiter=",")

    rvecs0, rvecs1 = [], []
    tvecs0, tvecs1 = [], []
    # set the ArUco dictionary
    arucoDict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

    num = 0

    # iterate through the images:
    for gray in grays:
        num += 1
        corners, ids, rejected = aruco.detectMarkers(gray, arucoDict)
        if type(ids) == type(None):
            num_ids = 0
        else:
            num_ids = len(ids)
            print("Number of detected ArUco markers: ", num_ids)

            # check if the markers are detected
            if ([0] not in ids) and ([1] not in ids):
                print("ArUco marker 0 and 1 not detected")
                break

            elif ([0] not in ids):
                print("ArUco marker 0 not detected")

            elif ([1] not in ids):
                print("ArUco marker 1 not detected")
            

        # draw on markers on gray image
            if visualize:
                aruco.drawDetectedMarkers(gray, corners, ids)

            for i in range(len(ids)):
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], length_marker, cameraMatrix, distortionMatrix)
                # draw on axis for each tag
                if visualize:
                    cv2.drawFrameAxes(gray, cameraMatrix, distortionMatrix, rvec, tvec, 100)

                    
                if ids[i] == [0]:
            
                    rvecs0.append(rvec[0][0])
                    tvecs0.append(tvec[0][0])

                elif ids[i] == [1]:
                    rvecs1.append(rvec[0][0])
                    tvecs1.append(tvec[0][0])
                    print(tvec[0][0])

                rvec, tvec = 0, 0
        
            if visualize:
                cv2.imwrite("Log\\GrayImages\\ArucoPictures\\imgz"+str(num)+".png", gray)
                cv2.imshow("gray"+str(num), gray)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
        
    retval = False

    if len(tvecs0) ==0 and len(tvecs1) == 0:
        print("No markers detected")
        retval = []
        return False
    
    

    if len(tvecs0) == 0:
        print("Marker 0 not detected")
        retval = False
        # T_A1C = np.zeros((4,4))
        # T_A1C[3,3] = 1
        # T_A1C[:3,:3] = cv2.Rodrigues(np.array(rvecs1)[0])[0]
        # T_A1C[:3,3] = np.array(tvecs1[0])

        # np.savetxt("Log\\TransformationMatrices\\T_A1C.csv", T_A1C, delimiter=",")

    elif len(tvecs1) == 0:
        print("Marker 1 not detected")
        retval = False
        # T_A0C = np.zeros((4,4))
        # T_A0C[3,3] = 1
        # T_A0C[:3,:3] = cv2.Rodrigues(np.array(rvecs0)[0])[0]
        # T_A0C[:3,3] = np.array(tvecs0[0])

        # np.savetxt("Log\\TransformationMatrices\\T_A0C.csv", T_A0C, delimiter=",")
    
    else:
        print("Making both transformations")

        T_A1C = np.zeros((4,4))
        T_A1C[3,3] = 1
        T_A1C[:3,:3] = cv2.Rodrigues(np.array(rvecs1)[0])[0]
        T_A1C[:3,3] = np.array(tvecs1[0])

        T_A0C = np.zeros((4,4))
        T_A0C[3,3] = 1
        T_A0C[:3,:3] = cv2.Rodrigues(np.array(rvecs0)[0])[0]
        T_A0C[:3,3] = np.array(tvecs0[0])

        np.savetxt("Log\\TransformationMatrices\\latestNEWT_A0C.csv", T_A0C, delimiter=",")
        np.savetxt("Log\\TransformationMatrices\\latestNEWT_A1C.csv", T_A1C, delimiter=",")
        

        

        if visualize:
            print("T_A0C: \n", T_A0C)
            print("T_A1C: \n", T_A1C)

        tvecs0 = np.array(tvecs0)
        tvecs1 = np.array(tvecs1)
        if visualize:
            print("tvecs for marker 0: \n", tvecs0)
            print("tvecs for marker 1: \n", tvecs1)
        


    return retval



# calibrateTub(preShot = False, precalib = False, visualize = True, log = False, timestamp = '')





