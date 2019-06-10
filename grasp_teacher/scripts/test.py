#! /usr/bin/env python

from grasp_teacher import conversions as c

if __name__ == '__main__':

    p = c.append_array_to_pose_msg([1, 1, 1, 1, 1, 1, 1])
    print c.pose_to_transformation_matrix(p.pose)