#!/usr/bin/env python

import rospy
import rospkg
import cv2
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from grasp_teacher_msgs.srv import GetPoseStamped, GetPoseStampedRequest, GetPoseStampedResponse
from visualization_msgs.msg import MarkerArray, Marker
from grasp_teacher import conversions
import numpy as np

# Call GetPoseStamped twice, to two different servers
# Get pose
# do computation in def computegrasp():
# convert to pose stamped by conversions.py
# publish marker to rviz
# write to file by opencv filestream


def read_handeye_x(filepath):
    fs = cv2.FileStorage(filepath, cv2.FILE_STORAGE_READ)
    fn = fs.getNode("transformationMatrix")
    return np.asarray(fn.mat())

def write_grasp_pose(pose):
    rospack = rospkg.RosPack()
    nb = raw_input('Name your pose:')
    filepath = rospy.get_param("pose_write_path", rospack.get_path('grasp_teacher')) \
               + '/data/grasp_pose/' + 'pose_' + nb + '.yml'

    data = np.array([pose.position.x,
                     pose.position.y,
                     pose.position.z,
                     pose.orientation.x,
                     pose.orientation.y,
                     pose.orientation.z,
                     pose.orientation.w])

    fs = cv2.FileStorage(filepath, cv2.FILE_STORAGE_WRITE)
    fs.write("pose", data)
    fs.release()

def transform_to_part_frame(part_pose, robot_pose, handeye_calib_pose):
    t_cp = conversions.pose_to_transformation_matrix(part_pose.pose)
    t_cb = handeye_calib_pose
    t_be = conversions.pose_to_transformation_matrix(robot_pose.pose)
    return np.dot(np.dot(np.linalg.inv(t_cp), t_cb), t_be)

def visualize_grasp(grasp_pose, part_pose):
    part_pub = rospy.Publisher("/test", PoseStamped, queue_size = 1)
    grasp_pub = rospy.Publisher("/test2", PoseStamped, queue_size = 1)

    part_pose_stamped = PoseStamped()
    part_pose_stamped.header.stamp = rospy.Time.now()
    part_pose_stamped.header.frame_id = "base_link"
    part_pose_stamped.pose = part_pose

    grasp_pose_stamped = PoseStamped()
    grasp_pose_stamped.header.stamp = rospy.Time.now()
    grasp_pose_stamped.header.frame_id = "base_link"
    grasp_pose_stamped.pose = grasp_pose

    part_pub.publish(part_pose_stamped)
    grasp_pub.publish(grasp_pose_stamped)

def grasp_teacher_client():
    handeye_filepath = "/home/eirik/catkin_ws/src/hand_eye_calibration/data/calib080419/extrinsics.yml"

    part_pose_server_name = rospy.get_param(rospy.get_name() + "/pose_server_name", "part_pose_recorder")
    robot_pose_server_name = rospy.get_param(rospy.get_name() + "/robot_server_name", "robot_pose_recorder")

    print "Waiting for services at: " + part_pose_server_name + " and: " + robot_pose_server_name
    rospy.wait_for_service(part_pose_server_name)
    rospy.wait_for_service(robot_pose_server_name)
    img = np.zeros((256, 256, 1))

    print "Ready. Press r to record, and q to quit"

    while True:
        cv2.imshow("empty", img)
        k = cv2.waitKey(33)
        if k == ord('r'):
            try:
                part_pose_recorder = rospy.ServiceProxy(part_pose_server_name, GetPoseStamped)
                robot_pose_recorder = rospy.ServiceProxy(robot_pose_server_name, GetPoseStamped)
                part_pose = part_pose_recorder().pose
                robot_pose = robot_pose_recorder().pose
                print "Success"

                t = transform_to_part_frame(part_pose, robot_pose, read_handeye_x(handeye_filepath))
                grasp_pose = conversions.transformation_matrix_to_pose_msg(t)
                visualize_grasp(grasp_pose, PoseStamped().pose)
                write_grasp_pose(grasp_pose)
                print t
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e

        elif k == ord('q'):
            print "Shutting down."
            break


if __name__ == "__main__":
    rospy.init_node("grasp_teacher_client")
    grasp_teacher_client()
