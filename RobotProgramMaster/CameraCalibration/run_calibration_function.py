import numpy
numpy.set_printoptions(linewidth=300, suppress=True)
from scipy.linalg import expm, inv
from numpy import dot, eye, array, savetxt
import sys
sys.path.insert(1, '../RobotProgramMaster/RealSense')
import RealSense2 as gss
import time

# sys.path.insert(1, '../RobotProgramMaster/RealSense')
# from CameraCalibration.chessboard_ import*
# from CameraCalibration.park_martin import calibrate

sys.path.insert(1, '../RobotProgramMaster/CameraCalibration')

# These are for local testing
from chessboard_ import*
from park_martin import calibrate

def compute_average_transform(transforms):
    """
    Computes the average of the rotation part and the translation part of a list of homogenous
    transformation matrices.

    Args:
        transforms: numpy array of shape (n, 4, 4) representing a list of n homogenous
                    transformation matrices consisting of rotation and translation.

    Returns:
        A numpy array of shape (4, 4) representing the average homogenous transformation matrix
        with the average of the rotation part (normalized) and the average of the translation part.
    """
    n = transforms.shape[0]
    
    # Compute the average rotation matrix by taking the average of the rotation matrices
    # of all the input transformation matrices
    rotation_matrices = transforms[:, :3, :3]
    average_rotation_matrix = numpy.mean(rotation_matrices, axis=0)
    
    # Normalize the average rotation matrix by computing its SVD and setting the smallest
    # singular value to 1.0
    U, S, V = numpy.linalg.svd(average_rotation_matrix)
    normalized_rotation_matrix = numpy.dot(U, V)
    
    # Compute the average translation vector by taking the average of the translation vectors
    # of all the input transformation matrices
    translation_vectors = transforms[:, :3, 3]
    average_translation_vector = numpy.mean(translation_vectors, axis=0)
    
    # Construct the average homogenous transformation matrix
    average_transform = numpy.eye(4)
    average_transform[:3, :3] = normalized_rotation_matrix
    average_transform[:3, 3] = average_translation_vector
    
    return average_transform








# take in 20230309-135037img0.png to 20230309-135037img3.png from Log\\GrayImages and create a list
# of images
def get_image_list(timestamp):
    img_list = []
    for i in range(19):
        img_list.append(cv2.imread('Log\\GrayImages\\' + timestamp + 'img' + str(i) + '.png', 0))
        # cv2.imshow('image', img_list[i])
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
    return img_list

img_list = get_image_list("20230320-114609")


# function that takes in cart_pose_list and returns a list of robot in homogenous transformation matrices
import math as math
def pose_to_transform(pose):
    x, y, z, rx, ry, rz = pose
    
    # Convert angles from degrees to radians
    rx = math.radians(rx)
    ry = math.radians(ry)
    rz = math.radians(rz)

    # Create rotation matrices for each axis
    Rx = numpy.array([[1, 0, 0], [0, math.cos(rx), -math.sin(rx)], [0, math.sin(rx), math.cos(rx)]])
    Ry = numpy.array([[math.cos(ry), 0, math.sin(ry)], [0, 1, 0], [-math.sin(ry), 0, math.cos(ry)]])
    Rz = numpy.array([[math.cos(rz), -math.sin(rz), 0], [math.sin(rz), math.cos(rz), 0], [0, 0, 1]])

    # Compute the rotation matrix R and the translation vector t
    R = Rz.dot(Ry).dot(Rx)
    t = numpy.array([[x], [y], [z]])

    # Create the homogeneous transformation matrix T
    T = numpy.block([[R, t], [numpy.zeros((1, 3)), 1]])

    return T

# robot_pose_list = []
# robot_pose_list = numpy.loadtxt('CameraCalibration\\PosesForCalibration.csv', delimiter=',')
# for i in range(20):
#     # transform cart_pose index 6 to 9 to a 4x4 matrix. Cart_pose is a (x, y, z, rx, ry, rz, figure)
#     robot_pose_list.append(pose_to_transform(cart_pose_list[i]))

# robot_pose_list = numpy.loadtxt("Log\\TransformationMatrices\\BigChessboardRobotPoses.csv", delimiter=',')
# # reshape the robot_pose_list with 
# robot_pose_list = robot_pose_list.reshape(len(robot_pose_list), 4,4)[:-1]



def expand(A):
    # make A a list if it is a numpy array
    if type(A) == numpy.ndarray:
        A = A.tolist()
    liste = []
    for i in A:
        liste.append(i)
    # print(len(A)//2)

    for i in range(len(A)//2):
        liste.append(A[i])
        liste.append(A[(len(A)//2) + i])
    for i in range((len(A)//2)-1):
        liste.append(A[i])
        liste.append(A[(len(A)//2) + i + 1])
    return liste



# import random
# def randomize(seed, liste):
#     if type(liste) == numpy.ndarray:
#         liste = liste.tolist()
#     random.seed(seed)
#     random.shuffle(liste)
#     return liste

# print(len(robot_pose_list))
# print(len(img_list))

# print('With randomizing before permutations')
# a = random.randint(0, 100)
# robot_pose_list = randomize(a, robot_pose_list)
# img_list = randomize(a, img_list)

# print("With more permutations")
# robot_pose_list = expand(robot_pose_list)
# img_list = expand(img_list)
# print(len(robot_pose_list),"images")


# print(len(robot_pose_list))
# print(len(img_list))














def run_calibration(img_list, rob_pose_list, preCalib = True, log = False, timestamp = ""):
    """
    This function runs the calibration of the camera and returns the camera pose in the robot base frame
    using the park_martin.py and chessboard_.py files

    :param img_list: list of images
    :param rob_pose_list: list of robot poses
    :param preCalib: boolean, if true, the camera matrix and distortion coefficients are loaded from cameras intrinsics
    :param log: boolean, if true, the camera matrix and distortion coefficients are saved to a file with timestamp
    :param timestamp: string, timestamp for the log file
    :return: camera_pose in robot base frame
    """
    # print(rob_pose_list.shape)

    corner_list = []
    obj_pose_list = []

    for i, img in enumerate(img_list):
        found, corners = find_corners(img)
        corner_list.append(corners)
        if not found:
            print("Failed to find corners in img # %d" % i)
            # remove the image from the list
            img_list.pop(i)
            # remove the robot pose from the numpy array
            rob_pose_list = numpy.delete(rob_pose_list, i, 0)

    for i, img in enumerate(img_list):
        found, corners = find_corners(img)
        corner_list.append(corners)
        if not found:
            print("Failed to find corners in img # %d" % i)
            # remove the image from the list
            img_list.pop(i)
            # remove the robot pose from the numpy array
            rob_pose_list = numpy.delete(rob_pose_list, i, 0)

    if preCalib:
        # camera_matrix, dist_coeffs,_ = RS.getRealSenseMatrices()
        # camera_matrix, dist_coeffs,_,_,_ = gss.getRealSenseMatrices()

        # numpy.savetxt("Log\\Intrinsics\\PreCalibCameraMatrix.csv", camera_matrix, delimiter=',')
        # numpy.savetxt("Log\\Intrinsics\\PreCalibDistortionCoefficients.csv", dist_coeffs, delimiter=',')
        camera_matrix = numpy.loadtxt('Log\\Intrinsics\\PreCalibCameraMatrix.csv', delimiter=',')
        dist_coeffs = numpy.loadtxt('Log\\Intrinsics\\PreCalibDistortionCoefficients.csv', delimiter=',')
    else:
        camera_matrix, dist_coeffs, _ , _ = calibrate_lens(img_list)

    # if log:
    #     savetxt('Log\\Intrinsics\\' + timestamp + 'CameraMatrix.csv', camera_matrix, delimiter=',')
    #     savetxt('Log\\Intrinsics\\' + timestamp + 'DistortionCoefficients.csv', dist_coeffs, delimiter=',')

        savetxt('Log\\Intrinsics\\latestCameraMatrix.csv', camera_matrix, delimiter=',')
        savetxt('Log\\Intrinsics\\latestDistortion.csv', dist_coeffs, delimiter=',')
    
    def hat(v):
        return [[   0, -v[2],  v[1]],
                [v[2],     0, -v[0]],
                [-v[1],  v[0],    0]]

    def tf_mat(r, t):
        res = eye(4)
        res[0:3, 0:3] = expm(hat(r))
        res[0:3, -1] = t
        return res

    for i, img in enumerate(img_list):
        found, corners = find_corners(img)
        corner_list.append(corners)
        if not found:
            print("Failed to find corners in img # %d" % i)
            # remove the image from the list
            img_list.pop(i)
        rvec, tvec = get_object_pose(pattern_points, corners, camera_matrix, dist_coeffs)
        object_pose = tf_mat(rvec, tvec)
        obj_pose_list.append(object_pose)

    A, B = [], []
    for i in range(1,len(img_list)):
        p = rob_pose_list[i-1], obj_pose_list[i-1]
        n = rob_pose_list[i], obj_pose_list[i]
        A.append(dot(inv(p[0]), n[0]))
        B.append(dot(inv(p[1]), n[1]))

    # A = T_w1b * Tbw2 = T_w1w2
    # B = T_ch1c * T_cch2 = T_ch1ch2
    
    A = expand(A)
    print("length of A expanded is: ",len(A))
    B = expand(B)
    print("length of B expanded is: ", len(B))

    # Transformation to chessboard in robot gripper
    X = eye(4)
    Rx, tx = calibrate(A, B)
    X[0:3, 0:3] = Rx
    X[0:3, -1] = tx

   
    tmps = numpy.array([])

    for i in range(len(img_list)):
        if log:
            cv2.imwrite('Log\\GrayImages\\'+ timestamp + 'img%d.png' % i, img_list[i])
        rob = rob_pose_list[i]
        obj = obj_pose_list[i]
        tmp = dot(rob, dot(X, inv(obj)))
        # add tmp tp tmps
        tmps = numpy.append(tmps, tmp)
        # print(tmp)

    # Here I just pick one, but maybe some average can be used instead
    # rob = rob_pose_list[0]
    # obj = obj_pose_list[0]

    cam_pose = dot(dot(rob, X), inv(obj))

    if log:
        savetxt('Log\\TransformationMatrices\\'+ timestamp +'T_BC.csv', cam_pose, delimiter=',')
    

    tmps = tmps.reshape(len(img_list), 4,4)
    xs = []
    ys = []
    zs = []
    for tmp in tmps:
        xs.append(tmp[0,3])
        ys.append(tmp[1,3])
        zs.append(tmp[2,3])
    # print avg xs, ys, zs
    print("Mean x: ", numpy.mean(xs))
    print("Mean y: ", numpy.mean(ys))
    print("Mean z: ", numpy.mean(zs))

    print("biggest gap in x: ", numpy.max(xs) - numpy.min(xs))
    print("biggest gap in y: ", numpy.max(ys) - numpy.min(ys))
    print("biggest gap in z: ", numpy.max(zs) - numpy.min(zs))

    # find index of biggest gap in x
    print("index of biggest x: ", numpy.argmax(xs))
    print("index of smallest x: ", numpy.argmin(xs))
    print("index of biggest y: ", numpy.argmax(ys))
    print("index of smallest y: ", numpy.argmin(zs))
    print("index of biggest z: ", numpy.argmax(zs))
    print("index of smallest z: ", numpy.argmin(zs))

    # avg = compute_average_transform(tmps)
    avg = cam_pose
    savetxt('Log\\TransformationMatrices\\latestT_BC.csv', avg, delimiter=',')
    return avg, tmps

# cam_pose, tmps = run_calibration(img_list, robot_pose_list, preCalib = True, log = False)
# cam_pose, tmps = run_calibration(img_list, robot_pose_list, preCalib = False, log = False)


