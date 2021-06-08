#!/usr/bin/env python3
from numpy import lib
from arm_lib.srv import IK, IKResponse
import rospy
import sys
import tinyik as ik

def handle_ik(req):

    arm = ik.Actuator([
        "z", [0, 0, 0.15],
        "x", [0, 0, 2.0],
        "x", [0, 0, 1.0],
        "x", [0, 0, 0.5],
        "x", [0, 0, 0.1], # Rotation is along x and y: how do we include it here?
        "z", [0, 0, 0.5],
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
