import cv2 as cv2
import pyrealsense2 as rs
import numpy as np
import open3d as o3d

# resolution = [640,480]
# resolution = [1920,1080]
resolution = [1280,720]

def getSnapShotPointCloud2(log = False, timestamp = "", resolution = resolution):
       """
       parameters:
       log: if True, the point cloud is saved to a file
       timestamp: if log is True, the point cloud is saved to a file with the given timestamp
       returns:
       open3d point cloud 
       """
       # Create a RealSense pipeline object
       pipeline = rs.pipeline()

       # Create a configuration object for the pipeline
       config = rs.config()
       config.enable_stream(rs.stream.color, resolution[0],resolution[1], rs.format.bgr8, 30)
       config.enable_stream(rs.stream.depth, resolution[0],resolution[1], rs.format.z16, 30)
       
       # Enable depth alignment to color frames
       align_to = rs.stream.color
       align = rs.align(align_to)

       # Start the pipeline
       pipeline.start(config)

       try:
       
              # Wait for a new set of frames from the camera
              frames = pipeline.wait_for_frames()

              # Align the depth frame to the color frame
              aligned_frames = align.process(frames)
              color_frame = aligned_frames.get_color_frame()
              depth_frame = aligned_frames.get_depth_frame()

              # Get the intrinsics of the color camera
              color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

              # Convert the color and depth frames to Open3D format
              color = o3d.geometry.Image(np.array(color_frame.get_data()))
              depth = o3d.geometry.Image(np.array(depth_frame.get_data()))

              # Create an Open3D RGBD image
              rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                     color, depth, depth_scale=1/float(depth_frame.get_units()))

              # Create an Open3D point cloud
              pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
                     rgbd, o3d.camera.PinholeCameraIntrinsic(
                            width=color_intrinsics.width, 
                            height=color_intrinsics.height,
                            fx=color_intrinsics.fx, 
                            fy=color_intrinsics.fy,
                            cx=color_intrinsics.ppx, 
                            cy=color_intrinsics.ppy))
              
              if log:
                     
                     # Save the point cloud to a file
                     o3d.io.write_point_cloud("Log\\PointClouds\\pointcloud" + timestamp + ".pcd", pcd)
                     print("Point cloud saved to pointcloud" + timestamp + ".pcd")
              
              o3d.io.write_point_cloud("Log\\PointClouds\\latestPointCloud.pcd", pcd)
              print("Point cloud saved to latestPointCloud.pcd")
       
       finally:
              # Stop the pipeline when done
              pipeline.stop()
       return pcd

# find out time before and after function call
# import time
# start = time.time()

# pcd = getSnapShotPointCloud2(log = True, timestamp = "test")
# print(time.time() - start)

# o3d.visualization.draw_geometries([pcd, o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])])

def getRealSenseMatrices():
       """
       parameters: none
       returns: camera matrix, distortion, extrinsics from depth to color
       """
       # Configure depth and color streams
       pipeline = rs.pipeline()
       config = rs.config()
       config.enable_stream(rs.stream.color, resolution[0], resolution[1], rs.format.bgr8, 30)
       config.enable_stream(rs.stream.depth, resolution[0], resolution[1], rs.format.z16, 30)

       # Start streaming
       pipeline.start(config)
       try:
              # Get camera intrinsics
              frames = pipeline.wait_for_frames()
              color_frame = frames.get_color_frame()
              intr = color_frame.profile.as_video_stream_profile().intrinsics
              depth_frame = frames.get_depth_frame()
              depth_intr = depth_frame.profile.as_video_stream_profile().intrinsics
              T_color_depth = color_frame.profile.get_extrinsics_to(depth_frame.profile)
              # Print camera intrinsics
       finally:
              # Stop streaming
              pipeline.stop()

       # use intr to create camera matrix
       T_color_depth = np.array([[T_color_depth.rotation[0], T_color_depth.rotation[1], 
                                             T_color_depth.rotation[2], T_color_depth.translation[0]], 
                                             [T_color_depth.rotation[3], T_color_depth.rotation[4], 
                                             T_color_depth.rotation[5], T_color_depth.translation[1]], 
                                             [T_color_depth.rotation[6], T_color_depth.rotation[7], 
                                             T_color_depth.rotation[8], T_color_depth.translation[2]], 
                                             [0, 0, 0, 1]])
       camera_matrix = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
       distortion = np.array([intr.coeffs[0], intr.coeffs[1], intr.coeffs[2], intr.coeffs[3], intr.coeffs[4]])
       return camera_matrix, distortion, T_color_depth, intr, depth_intr

# camera_matrix, distortion, T_color_depth = getRealSenseMatrices()

# np.savetxt("Log\\Intrinsics\\latestCameraMatrix.csv", camera_matrix, delimiter=",")
# np.savetxt("Log\\Intrinsics\\latestDistortion.csv", distortion, delimiter=",")
# np.savetxt("Log\\Intrinsics\\latestT_color_depth.csv", T_color_depth, delimiter=",")



def getSnapShotPointCloud(log = False, timestamp = ""):
       pc = rs.pointcloud()
       # We want the points object to be persistent so we can display the last cloud when a frame drops
       points = rs.points()

       # Declare RealSense pipeline, encapsulating the actual device and sensors
       pipe = rs.pipeline()
       config = rs.config()
       
       # Enable depth stream
       config.enable_stream(rs.stream.depth, resolution[0], resolution[1], rs.format.z16, 30)

       # Start streaming with chosen configuration
       pipe.start(config)

       # We'll use the colorizer to generate texture for our PLY
       # (alternatively, texture can be obtained from color or infrared stream)
       colorizer = rs.colorizer()

       try:
              # Wait for the next set of frames from the camera
              frames = pipe.wait_for_frames()
              colorized = colorizer.process(frames)


              # get timestamp as a string from time dd.mm.yy-hh.mm.ss
              if log:
              # Create save_to_ply object
                     ply = rs.save_to_ply("Log\\PointClouds"+timestamp+"pointCloud.ply")

              # Set options to the desired values
              # In this example we'll generate a textual PLY with normals (mesh is already created by default)
                     ply.set_option(rs.save_to_ply.option_ply_binary, True)
                     ply.set_option(rs.save_to_ply.option_ply_normals, True)

                     print("Saving to timestamp+pointCloud.ply...")
              # Apply the processing block to the frameset which contains the depth frame and the texture
                     ply.process(colorized)
                     print("Done")

              ply2 = rs.save_to_ply("Log\\PointClouds\\latestPointCloud.ply")

              # Set options to the desired values
              # In this example we'll generate a textual PLY with normals (mesh is already created by default)
              ply2.set_option(rs.save_to_ply.option_ply_binary, True)
              ply2.set_option(rs.save_to_ply.option_ply_normals, True)

              print("Saving to timestamp+pointCloud.ply...")
              # Apply the processing block to the frameset which contains the depth frame and the texture
              ply2.process(colorized)
              print("Done")
       finally:
              pipe.stop()
       
       pcd = o3d.io.read_point_cloud("Log\\PointClouds\\latestPointCloud.ply")
       return pcd



def getSnapShotGray(log = False, timestamp = "", amount = 1):
       imgs = []
       print('Taking Snapshot...')
       pipeline = rs.pipeline()
       config = rs.config()
       config.enable_stream(rs.stream.color, resolution[0], resolution[1], rs.format.bgr8, 30)
       pipeline.start(config)
       a = 0
       for i in range(amount):
              frames = pipeline.wait_for_frames()
              color_frame = frames.get_color_frame()
              color_image = np.asanyarray(color_frame.get_data())
              # make image grayscale
              gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
              imgs.append(gray_image)
              if log:
                     print('Saving Snapshot...')
                     cv2.imwrite("Log\\GrayImages"+ timestamp+"gray",a,".jpg", gray_image)
                     print('Snapshot Saved')
              a += 1

       pipeline.stop()
       if amount == 1:
              return gray_image
       else:
              return imgs


def getSnapShot(log = False, timestamp = ""):
       print('Taking Snapshot...')
       pipeline = rs.pipeline()
       config = rs.config()
       config.enable_stream(rs.stream.color, resolution[0], resolution[1], rs.format.bgr8, 30)
       config.enable_stream(rs.stream.depth, resolution[0], resolution[1], rs.format.z16, 30)
       
       align_to = rs.stream.color
       align = rs.align(align_to)
       pipeline.start(config)
       frames = pipeline.wait_for_frames()
       aligned_frames = align.process(frames)
       aligned_depth_frame = aligned_frames.get_depth_frame()
       color_frame = aligned_frames.get_color_frame()
       color_image = np.asanyarray(color_frame.get_data())
       pipeline.stop()
       # make image grayscale
       gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

       
       
       return gray_image, aligned_depth_frame

import pyrealsense2 as rs
import numpy as np

import pyrealsense2 as rs
import numpy as np

def deproject_pixel_to_point(pixel_array, depth_frame, color_intrin):
    '''
    Deprojects a set of 2D pixel points to 3D using pyrealsense2 color and depth streams.

    Args:
    - pixel_array: a NumPy array of 2D pixel points, with shape (N, 2), where N is the number of pixel points and the second dimension contains the x and y coordinates
    - depth_frame: a depth frame obtained from the RealSense depth sensor
    - color_intrin: a color intrinsics object obtained from the RealSense depth sensor

    Returns:
    - point_array: a NumPy array of 3D points corresponding to the input pixel array, with shape (N, 3), where N is the number of valid 3D points and the second dimension contains the x, y, and z coordinates
    '''
    point_list = []

    for pixel in pixel_array:
        x, y = pixel
        depth = depth_frame.get_distance(x, y)
        if depth > 0:
            point = rs.rs2_deproject_pixel_to_point(color_intrin, [x, y], depth)
            point_list.append(point)

    point_array = np.array(point_list)

    return point_array



# def getSnapShotPointCloud(log = False, timestamp = ""):
#        """
#        Connecting to camera, taking a snapshot using 3D camera and saving it as a .ply file with timestamp
#        """

#        # Declare RealSense pipeline, encapsulating the actual device and sensors
#        pipe = rs.pipeline()
#        config = rs.config()

#        # Enable depth stream
#        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

#        # Start streaming with chosen configuration
#        pipe.start(config)

#        # We'll use the colorizer to generate texture for our PLY
#        # (alternatively, texture can be obtained from color or infrared stream)
#        colorizer = rs.colorizer()

#        try:
#               # Wait for the next set of frames from the camera
#               frames = pipe.wait_for_frames()
#               colorized = colorizer.process(frames)


#               # Create save_to_ply object
#               if log & (timestamp != ''):
#                      ply = rs.save_to_ply("Log\\PointClouds\\"+timestamp+"pointCloud.ply")

#               ply2 = rs.save_to_ply("Log\\PointClouds\\latestPointCloud.ply")


#               # Set options to the desired values
#               # In this example we'll generate a textual PLY with normals (mesh is already created by default)
#               if log & (timestamp != ''):
#                      ply.set_option(rs.save_to_ply.option_ply_binary, True)
#                      ply.set_option(rs.save_to_ply.option_ply_normals, True)

#               ply2.set_option(rs.save_to_ply.option_ply_binary, True)
#               ply2.set_option(rs.save_to_ply.option_ply_normals, True)

#               # Apply the processing block to the frameset which contains the depth frame and the texture
#               if log & (timestamp != ''):
#                      ply.process(colorized)
#               ply2.process(colorized)
#               print("Done")

              

#        finally:
#                pipe.stop()


#        # get point cloud
#        pcd = o3d.io.read_point_cloud("Log\\PointClouds\\latestPointCloud.ply")

#        return pcd
