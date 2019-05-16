#! /usr/bin/env python
import rospy
import message_filters
import yaml
import cv2
import numpy as np
import tf.transformations

from geometry_msgs.msg import PoseStamped, PoseArray

def read_cv_yaml(filepath):
    # read a matrix from yaml file (i.e. hand eye calibration)
    fs = cv2.FileStorage(filepath, cv2.FILE_STORAGE_READ)
    fn = fs.getNode("transformationMatrix")
    return(fn.mat())



if __name__ == '__main__':
    t = read_cv_yaml('/home/eirik/catkin_ws/src/hand_eye_calibration/data/calib080419/extrinsics.yml')
    t = np.asarray(t)
    np.take(t, [[[0, 1, 2], [0, 1, 2], [0, 1, 2]]])
