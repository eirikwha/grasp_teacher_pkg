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


# TODO: Listen at gripper as well, and get command (closed, force etc)

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

class GraspTeacher(object):
    def __init__(self, filepath, part_pose_topic, robot_pose_topic):

        #self.t_mat = self.read_cv_yaml(filepath)

        # Pose Estimator node publishes PoseArray
        self.part_pose_sub = message_filters.Subscriber(part_pose_topic, PoseArray)

        # Subscribes to PoseArray
        self.robot_pose_sub = message_filters.Subscriber(robot_pose_topic, PoseStamped)

        # Message synchronization
        self.ts = message_filters.TimeSynchronizer([self.part_pose_sub, self.robot_pose_sub], 10)
        self.ts.registerCallback(GraspTeacher.grasp_calc_cb)

        # Publishers
        self.pub = rospy.Publisher('/grasp_out', PoseStamped, queue_size=10)

        self.part_pose_it = 0
        self.robot_pose_it = 0

    def read_cv_yaml(self, filepath):
        # read a matrix from yaml file (i.e. hand eye calibration)
        fs = cv2.FileStorage(filepath, cv2.FILE_STORAGE_READ)
        fn = fs.getNode("transformationMatrix")
        return np.asarray(fn.mat())

    def transformation_to_pose_msg(self):
        t_mat = self.read_cv_yaml(filepath)
        R = t_mat[0:3,0:3]
        q = sheperd_rot_to_quat(R)
        t = np.asarray(t_mat[0:3,3])
        return q, t

    def grasp_calc_cb(self, part_pose_array, robot_pose):
        global part_pose_it, robot_pose_it
        # Solve all of perception here...

        qx, tx = self.transformation_to_pose_msg()

        part_pose = Pose()
        part_pose = part_pose_array[0]
        part_pose_it =+ 1

        if (part_pose_it > robot_pose_it):
            # do something with robot pose

            qp = part_pose.pose.orientation
            qr = robot_pose.pose.orientation

            pose_grasp = PoseStamped()
            pose_grasp.pose.orientation = np.dot(np.dot(qr,qx),qp)
            pose_grasp.pose.position = part_pose.pose.position + robot_pose.pose.poseition + tx
            # TODO: transformation matrix to quat and translation
            print (pose_grasp)
            self.pub.publish(pose_grasp)


if __name__ == '__main__':

    name = 'grasp_teacher'

    rospack = rospkg.RosPack()
    path = rospack.get_path(name)
    rospy.init_node(name, anonymous=True)

    filepath = rospy.get_param('/yaml_matrix_filepath', path + '/data/extrinsics.yml')
    part_pose_topic = rospy.get_param('/part_pose_topic', '/pose_out')
    robot_pose_topic = rospy.get_param('/robot_pose_topic', '/robot_state')

    grasp_teacher = GraspTeacher(filepath, part_pose_topic, robot_pose_topic)

    rospy.spin()
