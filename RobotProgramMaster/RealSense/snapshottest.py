import open3d as o3d
import numpy as np
import pyrealsense2 as rs

# Create a RealSense pipeline object
pipeline = rs.pipeline()

# Create a configuration object for the pipeline
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

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
    

    o3d.visualization.draw_geometries([pcd])

finally:
    # Stop the pipeline when done
    pipeline.stop()

   
