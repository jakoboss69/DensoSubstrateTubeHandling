import numpy
import cv2
# # small chessboard
# pattern_size = (6,4)
# square_size = 14

# large chessboard
pattern_size = (9,6)
square_size = 30

pattern_points = numpy.zeros( (numpy.prod(pattern_size), 3), numpy.float32 )
pattern_points[:,:2] = numpy.indices(pattern_size).T.reshape(-1, 2)
pattern_points *= square_size



def find_corners(image):
    found, corners = cv2.findChessboardCorners(image, pattern_size)
    term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.01)
    if found:
        cv2.cornerSubPix(image, corners, (5, 5), (-1, -1), term)
    return found, corners

def draw_corners(image, corners):
    color_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    cv2.drawChessboardCorners(color_image, pattern_size, corners, True)
    return color_image

def get_object_pose(object_points, image_points, camera_matrix, dist_coeffs):
    ret, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
    return rvec.flatten(), tvec.flatten()

def calibrate_lens(image_list):
    img_points, obj_points = [], []
    h,w = 0, 0
    a = 1
    for img in image_list:
    
        h, w = img.shape[:2]
        found,corners = find_corners(img)
        if not found:
            raise Exception("chessboard calibrate_lens Failed to find corners in img", a)
        img_points.append(corners.reshape(-1, 2))
        obj_points.append(pattern_points)
        a+=1
    
        
    camera_matrix = numpy.zeros((3,3))
    dist_coeffs = numpy.zeros(5)
    # rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w,h))
    cv2.calibrateCamera(obj_points, img_points, (w,h), camera_matrix, dist_coeffs)

    
    return camera_matrix, dist_coeffs, img_points, obj_points

