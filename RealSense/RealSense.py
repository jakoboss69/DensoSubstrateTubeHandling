import cv2 as cv2
import pyrealsense2 as rs
import numpy as np
import open3d as o3d

def getRealSenseCameraMatrix():

       # Configure depth and color streams
       pipeline = rs.pipeline()
       config = rs.config()
       config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

       # Start streaming
       pipeline.start(config)

       # Get camera intrinsics
       frames = pipeline.wait_for_frames()
       color_frame = frames.get_color_frame()
       intr = color_frame.profile.as_video_stream_profile().intrinsics

       # Print camera intrinsics
       print("Camera intrinsics: ", intr)

       # Stop streaming
       pipeline.stop()

       # use intr to create camera matrix
       camera_matrix = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
       distortion = np.array([intr.coeffs[0], intr.coeffs[1], intr.coeffs[2], intr.coeffs[3], intr.coeffs[4]])
       return camera_matrix, distortion

def getSnapShotPointCloud(log = False, timestamp = ""):
       pc = rs.pointcloud()
       # We want the points object to be persistent so we can display the last cloud when a frame drops
       points = rs.points()

       # Declare RealSense pipeline, encapsulating the actual device and sensors
       pipe = rs.pipeline()
       config = rs.config()
       
       # Enable depth stream
       config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

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
                     ply = rs.save_to_ply("log\\PointClouds"+timestamp+"pointCloud.ply")

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

def getSnapShotGray(log = False, timestamp = None):

       print('Taking Snapshot...')
       rs_pipeline = rs.pipeline()
       rs_config = rs.config()
       rs_config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
       rs_pipeline.start(rs_config)
       rs_frames = rs_pipeline.wait_for_frames()
       rs_color_frame = rs_frames.get_color_frame()
       rs_color_image = np.asanyarray(rs_color_frame.get_data())
       rs_pipeline.stop()
       # make image grayscale
       rs_gray_image = cv2.cvtColor(rs_color_image, cv2.COLOR_BGR2GRAY)

       if log & (timestamp != ''):
              print('Saving Snapshot...')
              cv2.imwrite("Log\\GrayImages"+ timestamp+"gray.jpg", rs_gray_image)
              print('Snapshot Saved')

       return rs_gray_image



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

# o3d.visualization.draw_geometries([getSnapShotPointCloud()])
