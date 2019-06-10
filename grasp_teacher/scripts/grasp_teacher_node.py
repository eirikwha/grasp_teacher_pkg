#!/usr/bin/env python

import rospy
import rospkg
import cv2
import message_filters
import tf.transformations
import numpy as np

from geometry_msgs.msg import Pose, PoseStamped, PoseArray

from grasp_teacher import conversions as conversions


# TODO: Listen at gripper as well, and get command (closed, force etc)

class GraspTeacher():
    def __init__(self, filepath, part_pose_topic, robot_pose_topic):

        #self.t_mat = self.read_cv_yaml(filepath)

        # Subscribes to PoseArray from pose estimator
        self.part_pose_sub = message_filters.Subscriber(part_pose_topic, PoseArray)

        # Subscribes to robot posestamped
        self.robot_pose_sub = message_filters.Subscriber(robot_pose_topic, PoseStamped)

        # Message synchronization
        self.ts = message_filters.TimeSynchronizer([self.part_pose_sub, self.robot_pose_sub], 10)
        self.ts.registerCallback(self.grasp_calc_cb)

        # Publishers
        self.pub = rospy.Publisher('/grasp_out', PoseStamped, queue_size=10)

        self.part_pose_it = 0
        self.robot_pose_it = 0

        rospy.spin()

    #def read_calibration_yaml(self, filepath):
    #    # read a matrix from yaml file (i.e. hand eye calibration)
    #    fs = cv2.FileStorage(filepath, cv2.FILE_STORAGE_READ)
    #    fn = fs.getNode("transformationMatrix")
    #    return np.asarray(fn.mat())

    def write_grasp_pose(self, pose_stamped):

        data = np.array([pose_stamped.pose.position.x,
                         pose_stamped.pose.position.y,
                         pose_stamped.pose.position.z,
                         pose_stamped.pose.orientation.x,
                         pose_stamped.pose.orientation.y,
                         pose_stamped.pose.orientation.z,
                         pose_stamped.pose.orientation.w])

        nb = raw_input('Name your pose:')
        filepath = rospack.get_path('grasp_teacher') + '/data/grasp_teacher_pose/' + 'pose_' + nb + '.yml'

        fs = cv2.FileStorage(filepath, cv2.FILE_STORAGE_WRITE)
        fs.write("pose", data)
        fs.release()

    #def transformation_matrix_to_pose_msg(self):
    #    t_mat = self.read_calibration_yaml(filepath)
    #    R = t_mat[0:3,0:3]
    #    q = sheperd_rot_to_quat(R)
    #    t = np.asarray(t_mat[0:3,3])
    #    return q, t

    def grasp_calc_cb(self, part_pose_array, robot_pose):
        #global part_pose_it, robot_pose_it

        print "in callback"

        #qx, tx = self.transformation_matrix_to_pose_msg() # TODO: This should be moved out of callback

        part_pose = part_pose_array.poses[0]
        self.part_pose_it =+ 1

        if (self.part_pose_it > self.robot_pose_it): # TODO: this has to be callable by service call or keypress!!
            # do something with robot pose

            #qp = [part_pose.orientation.x, part_pose.orientation.y, part_pose.orientation.z, part_pose.orientation.w]
            #qr = [robot_pose.pose.orientation.x, robot_pose.pose.orientation.y, robot_pose.pose.orientation.z, robot_pose.pose.orientation.w]

            #pose_grasp_quat = tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(qr, qx),qp)
            #pose_grasp_t = part_pose.position.x + robot_pose.pose.position.x + tx[0]

            # Convert both to tranf matrices
            T_part = conversions.pose_to_transformation_matrix(part_pose)
            T_robot = conversions.pose_to_transformation_matrix(robot_pose)

            T = np.dot(T_part, T_robot)

            print T

            grasp_pose = conversions.transformation_matrix_to_pose_msg(T)

            # Do transformation
            # Convert back to pose


            p = PoseStamped()
            p.header.frame_id = 'base_frame'  # TODO: Rosparam
            p.header.stamp = rospy.Time.now()

            p.pose = grasp_pose

            # TODO: transformation matrix to quat and translation
            print (p)

            self.write_grasp_pose(p)
            self.pub.publish(p)


if __name__ == '__main__':

    name = 'grasp_teacher'

    rospack = rospkg.RosPack()
    path = rospack.get_path(name)
    rospy.init_node(name, anonymous=False)

    filepath = rospy.get_param('/yaml_matrix_filepath', path + '/data/extrinsics.yml')
    part_pose_topic = rospy.get_param('/part_pose_topic', '/pose_out')
    robot_pose_topic = rospy.get_param('/robot_pose_topic', '/rand_pose_stamped')

    print "Started ROS node."
    grasp_teacher = GraspTeacher(filepath, part_pose_topic, robot_pose_topic)
