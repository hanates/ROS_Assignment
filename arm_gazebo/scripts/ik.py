#!/usr/bin/env python3
from numpy import lib
from arm_lib.srv import IK, IKResponse
import rospy
import sys

def handle_ik(req):
    

    return IKResponse([])


def ik_server():
    rospy.init_node('ik_server')
    s = rospy.Service('ik', IK, handle_ik)
    print("Ready to ik.")
    rospy.spin()


if name == "__main__":
    ik_server()
