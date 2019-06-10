#!/usr/bin/env python

import rospy
import rospkg
import cv2
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
import numpy as np

n_pose = 0
filepath = None
sub = rospy.Subscriber

def pose_callback(data):

    global n_pose, sub
    rospack = rospkg.RosPack()
    nb = raw_input('Name your pose:')
    filepath = rospy.get_param("pose_write_path", rospack.get_path('grasp_teacher')) + '/data/pose/' + 'pose_' + str(n_pose) + '_' + nb + '.yml'

    pose = np.array([data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z,
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w])

    fs = cv2.FileStorage(filepath, cv2.FILE_STORAGE_WRITE)
    fs.write("pose", pose)
    fs.release()

    n_pose += 1
    print "Success, saved to %s", filepath

    sub.unregister()


def handle_pose_recorder_server(req):

    pose_topic = rospy.get_param("pose_topic", "rand_pose_stamped")
    global sub
    sub = rospy.Subscriber(pose_topic, PoseStamped, pose_callback)
    return TriggerResponse(
        success=True,
        message="Trigger successful"
    )


def pose_recorder_server():

    server_name = rospy.get_param("server_name", "pose_recorder")
    rospy.init_node('pose_recorder_server')
    s = rospy.Service(server_name, Trigger, handle_pose_recorder_server)
    print "Ready to record pose."
    rospy.spin()

if __name__ == "__main__":

    pose_recorder_server()

