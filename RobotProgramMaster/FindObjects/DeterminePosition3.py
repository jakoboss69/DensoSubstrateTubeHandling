import random
import open3d as o3d
import os
import pyrealsense2 as rs
import numpy as np
from copy import deepcopy
import sys
sys.path.insert(1, '../RobotProgramMaster/RealSense')
import RealSense2 as gss
# sys.path.insert(2, '../RobotProgramMaster/Calibration')
# from calibrateTub3 import length_marker

width_incubator = 682
length_incubator = 982
width_incubator_with_walls = 710
length_incubator_with_walls = 1000
length_marker = 150 # assuming perfekt centring of the marker


def rotate(pcd, angle_x, angle_y, angle_z):

    """
    Rotating pointcloud or array around the origin
    """

    if isinstance(pcd, o3d.geometry.PointCloud):
        pcd.rotate(np.array([[1, 0, 0], [0, np.cos(angle_x), -np.sin(angle_x)], [0, np.sin(angle_x), np.cos(angle_x)]]), center=(0, 0, 0))
        pcd.rotate(np.array([[np.cos(angle_y), 0, np.sin(angle_y)], [0, 1, 0], [-np.sin(angle_y), 0, np.cos(angle_y)]]), center=(0, 0, 0))
        pcd.rotate(np.array([[np.cos(angle_z), -np.sin(angle_z), 0], [np.sin(angle_z), np.cos(angle_z), 0], [0, 0, 1]]), center=(0, 0, 0))
        
    if isinstance(pcd, np.ndarray):
        # take only the first 3 columns and rows of the array
        R = pcd[:3, :3]
        R = np.dot(np.array([[1, 0, 0], [0, np.cos(angle_x), -np.sin(angle_x)], [0, np.sin(angle_x), np.cos(angle_x)]]), R)
        R = np.dot(np.array([[np.cos(angle_y), 0, np.sin(angle_y)], [0, 1, 0], [-np.sin(angle_y), 0, np.cos(angle_y)]]),R)
        R = np.dot(np.array([[np.cos(angle_z), -np.sin(angle_z), 0], [np.sin(angle_z), np.cos(angle_z), 0], [0, 0, 1]]),R)
        pcd[:3, :3] = R
    return pcd


def voxelDownsample(pcd, voxel_size = 0.01):
    """
    Downsamples the point cloud to reduce the number of points
    """
    pcd = pcd.voxel_down_sample(voxel_size)
    return pcd


def cluster_dbscan(pcd, eps, min_points):
    """
    Clusters the point cloud using DBSCAN and returns a list of the clustered objects as pointclouds
    """

    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=True))
    max_label = labels.max()
    print(f"Point cloud has {max_label + 1} clusters")


    # if there are no clusters, clusters = False
    if max_label == -1:
        print("No clusters found")
        return []
    else:
        # return a list of the clustered objects as pointclouds
        return [pcd.select_by_index(np.where(labels == i)[0]) for i in range(max_label + 1)]




def crop(pcd, T_A0C, T_A1C, visualize = False):
    """
    parameters:
    pcd: point cloud
    T_A0C: transformation matrix from ArUco marker 0 to camera
    TA1C: transformation matrix from ArUco marker 1 to camera
    visualize: if True, the point cloud is visualized before and after cropping
    returns:
    pcd: cropped point cloud
    """
    if visualize:
        pcd_for_visualization = deepcopy(pcd)
        pcd_for_visualization = voxelDownsample(pcd_for_visualization, voxel_size = 20)
        # make pcd_for_visualization light green
        pcd_for_visualization.paint_uniform_color([0.5, 1, 0.5])
        


    # Transform the point cloud to the coordinate system of the ArUco marker 0 and crop the length of the incubator
    pcd.transform(np.linalg.inv(T_A0C))
    pcd = pcd.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound=[-length_incubator_with_walls, -1200, -900], max_bound=[-length_marker, 900, 150]))
    if visualize:
        pcd_for_visualization.transform(np.linalg.inv(T_A0C))
        o3d.visualization.draw_geometries([pcd_for_visualization, pcd, o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0, 0, 0])])
    

    # Transform the point cloud back to the coordinate system of the ArUco marker 1 and crop the width of the incubator
    pcd.transform(T_A0C)
    pcd.transform(np.linalg.inv(T_A1C))
    pcd = pcd.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound=[-1200, -width_incubator_with_walls, -200], max_bound=[900, -length_marker, 150]))

    if visualize:
        pcd_for_visualization.transform(T_A0C)
        pcd_for_visualization.transform(np.linalg.inv(T_A1C))
        o3d.visualization.draw_geometries([pcd_for_visualization, pcd, o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0, 0, 0])])
    
    # Find average z value of the pointcloud still in T_A1C
    pcd_points = np.asarray(pcd.points)
    z = pcd_points[:, 2]
    # z_avg = np.average(z)

    #Find 20th highest z value of the pointcloud
    z = np.sort(z)
    z_max = z[-20]
    

    mask =  (pcd_points[:, 2] > z_max-30)
    
    # Transform the point cloud back to the camera coordinate system
    pcd.transform(T_A1C)
    pcd.points = o3d.utility.Vector3dVector(pcd_points[mask])

    if visualize:
        pcd_for_visualization.transform(T_A1C)
        o3d.visualization.draw_geometries([pcd_for_visualization, pcd, o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])])

    return pcd

def findPoint(cluster, T_A1C):
    """
    Simple way of finding the top points of the point cloud by slicing the 
    point cloud along the z axis of A1 and finding the point in the slice 
    with the lowest y value
    """

    # transform to A1
    cluster1 = deepcopy(cluster)

    cluster1.transform(np.linalg.inv(T_A1C))

    pcd_points = np.asarray(cluster1.points)
    
    # find the highest z value
    z = pcd_points[:, 2]
    z_max = np.max(z)

    # make all points' z value equal to the highest value
    pcd_points[:, 2] = z_max

    # find average point of pcd_points
    tube_point = np.average(pcd_points, axis=0)

    # make y_value negative half of width of incubator
    tube_point[1] = -(width_incubator_with_walls+length_marker)/2

    # translate a frame to the center point and create T_A1_center
    T_A1tube = np.eye(4)
    T_A1tube[:3,3] = tube_point

    # rotate to match tool frame
    T_A1tube = rotate(T_A1tube, 0, np.pi, 0)

    # create T_CTube
    tube_frame_in_camera = np.matmul(T_A1C, T_A1tube)

    return tube_frame_in_camera 



def determinePosition(preShot = False, visualize = False, log = False, timestamp = '', store_pointcloud = False):
    """
    parameters:
    preShot: if True, the point cloud is read from a file, if False, the point cloud is taken from the camera
    visualize: if True, the point cloud is visualized before and after cropping
    returns:
    positions and frames of the tubes in the camera coordinate system
    """

    if preShot:
        path = "Log\\PointClouds\\latestPointCloud.pcd"
        pcd = o3d.io.read_point_cloud(path)
        
    else:
        
        pcd = gss.getSnapShotPointCloud2(log = log, timestamp = timestamp)
        
    if log:
        o3d.io.write_point_cloud("Log\\PointClouds\\"+timestamp+"pointcloud.ply", pcd)
    if store_pointcloud:
        o3d.io.write_point_cloud("Log\\PointClouds\\latestPointCloud.ply", pcd)

    pcd = voxelDownsample(pcd, voxel_size=0.008)
    
    # convert pointcloud to mm from meters
    pcd.scale(1000, center=(0, 0, 0))

    # Do a rough crop of the point cloud

    pcd = pcd.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound=[-500, -500, -1500], max_bound=[500, 500, 1500]))
    pcd_for_visualization = deepcopy(pcd)

    # Get the latest AruCo positions as transformation matrices
    T_A0C = np.loadtxt("Log\\TransformationMatrices\\latestNEWT_A0C.csv", delimiter=",")
    T_A1C = np.loadtxt("Log\\TransformationMatrices\\latestNEWT_A1C.csv", delimiter=",")

    if visualize:
        pcd_for_visualization = voxelDownsample(pcd_for_visualization, voxel_size=1)
        # visualize
        o3d.visualization.draw_geometries([pcd_for_visualization, o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0, 0, 0])])



    if visualize:
        print("T_A0C: ", T_A0C)
        print("T_A1C: ", T_A1C)
    
    # crop the point cloud to only include items within the incubator
    pcd = crop(pcd, T_A0C, T_A1C, visualize=visualize)
    
    # find clusters
    pcd_list = cluster_dbscan(pcd, eps=43, min_points=70)

    tube_frames = False

    if pcd_list!=False:
        tube_frames = []
        center_points_and_angles = []
        for cluster in pcd_list:
            # give the cluster a random color
            cluster.paint_uniform_color([random.random(), random.random(), random.random()])
            tube_frame = findPoint(cluster, T_A1C)
            
            tube_frames.append(tube_frame)
            if visualize:
              
                print("Tube frame in camera: ", tube_frame)

    if visualize:
        # visualize
        tube_frames_visualize = []
        if tube_frames!=False:
            for tube_frame in tube_frames:
                tube_frame_visualize = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=tube_frame[:3,3]).rotate(tube_frame[:3,:3], center=tube_frame[:3,3])
                tube_frames_visualize.append(tube_frame_visualize)
            o3d.visualization.draw_geometries([*tube_frames_visualize,*pcd_list,  o3d.geometry.TriangleMesh.create_coordinate_frame(300, origin=[0, 0, 0])])           
            o3d.visualization.draw_geometries([pcd_for_visualization,*tube_frames_visualize,*pcd_list,  o3d.geometry.TriangleMesh.create_coordinate_frame(300, origin=[0, 0, 0])])           
            print("Frame of tube: ",tube_frame)
    
    return tube_frames


# determinePosition(visualize = True, preShot = False, log = False)

