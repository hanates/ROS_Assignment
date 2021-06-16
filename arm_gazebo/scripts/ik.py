#!/usr/bin/env python3
from numpy import lib
from arm_lib.srv import IK, IKResponse
import rospy
import sys
import tinyik as ik

def handle_ik(req):

    arm = ik.Actuator([
        "z", [0, 0, 0.15], #base arm1
        "x", [0, 0, 2.0],  #arm2
        "x", [0, 0, 1.0],  #arm3
        "x", [0, 0, 0.5],  #arm4
        "z", [0, 0, 0.1],  #palm 
        "y", [0, 0, 0.1] #palm_jnt 
        # "z", [0, 0, 0.4]
    ])

    arm.ee = req.actuator_pose

    print(arm.angles)
   

    return IKResponse(arm.angles)


def ik_server():
    rospy.init_node('ik_server')
    s = rospy.Service('ik', IK, handle_ik)
    print("Ready to ik.")
    rospy.spin()


if __name__ == "__main__":
    ik_server()
