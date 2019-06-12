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

# get service call
# subscribe to topic
# return pose stamped

#n_pose = 0
#filepath = None
#sub = rospy.Subscriber
#pose_stamped = PoseStamped()

def handle_pose_recorder_server(req):

    ## PoseStamped
    # pose_topic = rospy.get_param(rospy.get_name() + "/robot_pose_topic", "/robot_state")
    # sub = rospy.wait_for_message(pose_topic, PoseStamped)
    # return GetPoseStampedResponse(pose = sub)

    ## KrlPos
    #pose_topic = rospy.get_param(rospy.get_name() + "/pose_topic", "/robot_state")
    #sub = rospy.wait_for_message(pose_topic, KrlPos)
    #return GetPoseXYZABCResponse(krlpos = sub)

    ## KrlAxis
    pose_topic = rospy.get_param(rospy.get_name() + "/pose_topic", "/joint_state")
    sub = rospy.wait_for_message(pose_topic, KrlAxis)
    return GetKrlAxisResponse(axis = sub)


def pose_recorder_server():
    rospy.init_node('pose_recorder_server')
    server_name = rospy.get_param(rospy.get_name() + "/server_name", "pose_recorder")
    s = rospy.Service(server_name, GetPoseStamped, handle_pose_recorder_server)
    print "Service at " + server_name + ". Ready to record pose."
    rospy.spin()


if __name__ == "__main__":
    pose_recorder_server()
