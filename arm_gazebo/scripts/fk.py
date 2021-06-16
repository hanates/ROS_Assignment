#!/usr/bin/env python3
from numpy import lib
from arm_lib.srv import FK, FKResponse
from arm_lib.msg import JointPose
import rospy
import tinyik as ik
import sys
sys.path.append('/home/ferox/catkin_ws/src/arm_lib/script/')
from lib import *

pub = rospy.Publisher('chatter', JointPose, queue_size=10)

def handle_fk(req):
    M1 = Rz(req.joint_angles[0]).dot(T(0, 0, 0.15))
    M2 = Rx(req.joint_angles[1]).dot(T(0, 0, 2))
    M3 = Rx(req.joint_angles[2]).dot(T(0, 0, 1))
    M4 = Rx(req.joint_angles[3]).dot(T(0, 0, 0.5))
    M5 = Rz(req.joint_angles[4]).dot(T(0, 0, 0.1))
    M6 = Ry(req.joint_angles[5]).dot(T(0, 0, 0.1))
    M7 = Rz(req.joint_angles[6]).dot(T(0, 0, 0.4)) 

    M = (((((M1.dot(M2)).dot(M3)).dot(M4)).dot(M5)).dot(M6)).dot(M7)
    
    
    # print([M[0][3], M[1][3], M[2][3]])
    
    
    
    return FKResponse([M[0][3], M[1][3], M[2][3]])


def fk_server():
    rospy.init_node('fk_server')
    s = rospy.Service('fk', FK, handle_fk)
    print("Ready to fk.")
    rospy.spin()


if __name__ == "__main__":
    fk_server()
