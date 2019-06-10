#! /usr/bin/env python
#import grasp_teacher.main
#if __name__ == '__main__':
#    grasp_teacher.main()
import rospy
import rospkg
import message_filters
import cv2
import numpy as np
import tf.transformations

from geometry_msgs.msg import Pose, PoseStamped, PoseArray

def append_array_to_pose_msg(array):

    p = PoseStamped()
    p.pose.position.x = array[0]
    p.pose.position.y = array[1]
    p.pose.position.z = array[2]

    p.pose.orientation.x = array[3]
    p.pose.orientation.y = array[4]
    p.pose.orientation.z = array[5]
    p.pose.orientation.w = array[6]

    return p


def sheperd_rot_to_quat(R):
    z00 = np.trace(R)
    z11 = R[0,0] + R[0,0] - z00
    z22 = R[1,1] + R[1,1] - z00
    z33 = R[2,2] + R[2,2] - z00

    if z00 >= 0.5:
        w = np.sqrt(1+z00)
        w_inv = 1/w
        x =(R[2,1] - R[1,2])*w_inv
        y =(R[0,2] - R[2,0])*w_inv
        z =(R[1,0] - R[0,1])*w_inv

    elif z11 >= 0.5:
        x = np.sqrt(1 + z11)
        x_inv = 1/x
        w = (R[2,1] - R[1,2])*x_inv
        y = (R[1,0] + R[0,1])*x_inv
        z = (R[2,0] + R[0,2])*x_inv

    elif z22 >= 0.5:
        y = np.sqrt(1 + z22)
        y_inv = 1/y
        w = (R[0,2] - R[2,0])*y_inv
        x = (R[1,0] + R[0,1])*y_inv
        z = (R[2,1] + R[1,2])*y_inv
    else:
        z = np.sqrt(1 + z33)
        z_inv = 1/z
        w = (R[1,0] - R[0,1])*z_inv
        x = (R[2,0] + R[0,2])*z_inv
        y = (R[2,1] + R[1,2])*z_inv

    eta = 0.5*w
    eps = np.dot(0.5, [x, y, z])
    q = [eps[0],eps[1],eps[2], eta]
    return q

def quat_to_rot(q):
    r = tf.transformations.quaternion_matrix(q)
    return r

def transformation_matrix_to_pose_msg(t_mat):

    pose = Pose()
    R = t_mat[0:3,0:3]
    q = sheperd_rot_to_quat(R)
    t = np.asarray(t_mat[0:3,3])

    pose.position.x = t[0]
    pose.position.y = t[1]
    pose.position.z = t[2]
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose

def pose_to_transformation_matrix(pose):

    q = [pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w]

    t = [pose.position.x, pose.position.y, pose.position.z]

    r_mat = quat_to_rot(q)
    t_mat = np.array(r_mat)

    t_mat[0,3] = t[0]
    t_mat[1,3] = t[1]
    t_mat[2,3] = t[2]
    return t_mat

def q_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return x, y, z, w

def q_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)

def qv_mult(q1, v1):
    q2 = (0.0,) + v1
    return q_mult(q_mult(q1, q2), q_conjugate(q1))[1:]