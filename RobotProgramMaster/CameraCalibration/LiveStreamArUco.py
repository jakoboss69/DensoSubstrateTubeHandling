import cv2
import cv2.aruco as aruco
import numpy as np
import sys
sys.path.insert(1, '../RobotProgramMaster/RealSense')
import RealSense2 as RS
import pyrealsense2 as rs


# generate aruco markers
def generateArucoMarkers(ids = [0, 1], sizes = [150, 200, 250, 300, 350]):

    # set the ArUco dictionary
    arucoDict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

    for size in sizes:
        for id in ids:
            # set size of marker
            marker = np.zeros((size, size), dtype=np.uint8)
            # draw marker
            marker = aruco.generateImageMarker(arucoDict, id, size, marker, 1)
            # save marker
            cv2.imwrite("Log\\GrayImages\\ArucoPictures\\marker"+str(id)+"_"+str(size)+".png", marker)
    

generateArucoMarkers()


def main():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, RS.resolution[0], RS.resolution[1], rs.format.bgr8, 30)
    pipeline.start(config)

    

    # get the camera matrix and distortion coefficients
    cameraMatrix = np.loadtxt("Log\\Intrinsics\\latestCameraMatrix.csv", delimiter=",")
    distCoeffs = np.loadtxt("Log\\Intrinsics\\latestDistortion.csv", delimiter=",")
    intr = pipeline.get_active_profile().get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    # cameraMatrix = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
    # distCoeffs = np.array([intr.coeffs[0], intr.coeffs[1], intr.coeffs[2], intr.coeffs[3], intr.coeffs[4]])


    # set the ArUco dictionary
    arucoDict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    a = 0
    while True or a < 100:
        # show video stream
        frame = pipeline.wait_for_frames()
        frame = frame.get_color_frame()
        frame = np.asanyarray(frame.get_data())
        # check if frame is empty
        ret = frame.any()


        if not ret or a == 100:

            print("Unable to capture video")
            a+=1
        else:

            # convert to grayscale
            # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = frame

            # detect ArUco markers
            corners, ids, rejected = aruco.detectMarkers(gray, arucoDict)
            # check if any markers are detected and ids type not none
            if ids is not None:
                # draw on markers on gray image
                aruco.drawDetectedMarkers(gray, corners, ids)
                # draw rejected markers
                aruco.drawDetectedMarkers(gray, rejected, borderColor=(100, 0, 240))

                for i in range(len(ids)):

                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i],50, cameraMatrix, distCoeffs)
                    # draw on axis for each tag
                    cv2.drawFrameAxes(gray, cameraMatrix, distCoeffs, rvec, tvec, 100)
                    # print(tvec[0,0,2])
            gray = cv2.resize(gray, (1080, 720))
            cv2.imshow('frame', gray)

        # set key to exit
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    # close stream
    pipeline.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()


        



