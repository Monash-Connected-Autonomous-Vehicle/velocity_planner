"""
This file includes functions for performing linear transformations on ROS types such as transforms and poses
"""
import numpy as np
import transforms3d as tf3d
from geometry_msgs.msg import Pose

def tf_to_homogenous_mat(t):
    translation_xyz = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    rotation_wxyz = [t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z]
    rotation_mat = tf3d.quaternions.quat2mat(rotation_wxyz)

    matrix = np.eye(4) # homogenous transformation matrix
    matrix[:3, :3] = rotation_mat
    matrix[:3, 3] = translation_xyz

    return matrix

def pose_to_homogenous_mat(pose):
    translation_xyz = [pose.position.x, pose.position.y, pose.position.z]
    rotation_wxyz = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
    rotation_mat = tf3d.quaternions.quat2mat(rotation_wxyz)
    
    matrix = np.eye(4) # homogenous transformation matrix
    matrix[:3, :3] = rotation_mat
    matrix[:3, 3] = translation_xyz

    return matrix

def homogenous_mat_to_pose(mat):
    rotation_wxyz = tf3d.quaternions.mat2quat(mat[:3, :3])
    translation_xyz = mat[:3, 3]

    pose = Pose()
    [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z] = rotation_wxyz
    [pose.position.x, pose.position.y, pose.position.z] = translation_xyz

    return pose
