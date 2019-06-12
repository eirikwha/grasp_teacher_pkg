#!/usr/bin/env python

import rospy
import rospkg
import cv2
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
import numpy as np
import rosnode

from kuka_eki_msgs.msg import KrlPos, KrlAxis
from grasp_teacher_msgs.srv import GetPoseXYZABC, GetPoseXYZABCRequest, GetPoseXYZABCResponse
from grasp_teacher_msgs.srv import GetPoseStamped, GetPoseStampedRequest, GetPoseStampedResponse
from grasp_teacher_msgs.srv import GetKrlAxis, GetKrlAxisRequest, GetKrlAxisResponse

def write_grasp_pose(pose):
    rospack = rospkg.RosPack()
    nb = raw_input('Name your pose:')
    filepath = rospy.get_param("pose_write_path", rospack.get_path('grasp_teacher')) \
               + '/data/robot_pose/' + 'pose_' + nb + '.yml'

    ## PoseStamped
    # data = np.array([pose.pose.position.x,
    #                  pose.pose.position.y,
    #                  pose.pose.position.z,
    #                  pose.pose.orientation.x,
    #                  pose.pose.orientation.y,
    #                  pose.pose.orientation.z,
    #                  pose.pose.orientation.w])

    ## KrlPos
    # data = np.array([pose.x,
    #                 pose.y,
    #                 pose.z,
    #                 pose.a,
    #                 pose.b,
    #                 pose.c])

    ## KrlAxis
    data = np.array([pose.a1,
                    pose.a2,
                    pose.a3,
                    pose.a4,
                    pose.a5,
                    pose.a6])

    fs = cv2.FileStorage(filepath, cv2.FILE_STORAGE_WRITE)
    fs.write("pose", data)
    fs.release()

def grasp_teacher_client():
    handeye_filepath = "/home/eirik/catkin_ws/src/hand_eye_calibration/data/calib080419/extrinsics.yml"

    robot_pose_server_name = rospy.get_param(rospy.get_name() + "/robot_server_name", "robot_pose_recorder")

    print "Waiting for service at: " + robot_pose_server_name
    rospy.wait_for_service(robot_pose_server_name)
    img = np.zeros((256, 256, 1))

    print "Ready. Press r to record, and q to quit"

    while True:
        cv2.imshow("empty", img)
        k = cv2.waitKey(33)
        if k == ord('r'):
            try:
                ## PoseStamped
                # robot_pose_recorder = rospy.ServiceProxy(robot_pose_server_name, GetPoseStamped)
                # robot_pose = robot_pose_recorder().pose

                ## KrlPos
                #robot_pose_recorder = rospy.ServiceProxy(robot_pose_server_name, GetPoseXYZABC)
                #robot_pose = robot_pose_recorder().krlpos
                
                ##KrlAxis
                robot_pose_recorder = rospy.ServiceProxy(robot_pose_server_name, GetKrlAxis)
                robot_pose = robot_pose_recorder().axis

                print "Success"
                write_grasp_pose(robot_pose)

            except rospy.ServiceException, e:
                print "Service call failed: %s" % e

        elif k == ord('q'):
            print "Shutting down."
            break


if __name__ == "__main__":
    rospy.init_node("pose_recorder_client")
    grasp_teacher_client()

