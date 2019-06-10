#!/usr/bin/env python

import rospy
import rospkg
import cv2
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
import numpy as np

def pose_recorder_client():

    server_name = rospy.get_param("server_name", "pose_recorder")

    rospy.wait_for_service(server_name)
    img = np.zeros((256, 256, 1))

    print"Press r to record, and q to quit"

    while True:
        cv2.imshow("empty", img)
        k = cv2.waitKey(33)
        if k == ord('r'):
            try:
                pose_recorder = rospy.ServiceProxy(server_name, Trigger)
                resp1 = pose_recorder()
                print resp1
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

        elif k == ord('q'):
            print "Shutting down."
            break

if __name__ == "__main__":

    pose_recorder_client()