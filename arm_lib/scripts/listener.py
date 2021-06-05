import rospy
from arm_lib.msg import input
from arm_lib.msg import output
import numpy as np
import math

def callback(data):
    yaw = data.gamma
    pitch = data.beta
    roll = data.alpha

    x = data.x
    y = data.y
    z = data.z

    d = data.d

    arr = rotate(yaw, pitch, roll,z , y, x, d)
    
    
    output_msg = output()
    output_msg.x_n =  arr[0]
    output_msg.y_n = arr[1]
    output_msg.z_n = arr[2]
    
    rospy.loginfo("%f %f %f" % (output_msg.x_n, output_msg.y_n, output_msg.z_n))

def rotate(yaw, pitch, roll,z , y, x, d):
    
    # change deg to rad - np.deg2rad()
    yaw = np.deg2rad(yaw)
    pitch = np.deg2rad(pitch)
    roll = np.deg2rad(roll)

    yawMatrix = np.matrix([
    [math.cos(yaw), -math.sin(yaw), 0, 0],
    [math.sin(yaw), math.cos(yaw), 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
    ])

    pitchMatrix = np.matrix([
    [math.cos(pitch), 0, math.sin(pitch), 0],
    [0, 1, 0, 0],
    [-math.sin(pitch), 0, math.cos(pitch), 0], 
    [0, 0, 0, 1]
    ])

    rollMatrix = np.matrix([
    [1, 0, 0, 0],
    [0, math.cos(roll), -math.sin(roll), 0],
    [0, math.sin(roll), math.cos(roll), 0],
    [0, 0, 0, 1]
    ])

    R = (yawMatrix.dot(pitchMatrix)).dot(rollMatrix)

    v = np.array([x, y, z, 1])
    v = np.reshape(v, (4, 1))

  
    trans_v = np.matrix([
    [1, 0, 0, d*math.cos(roll)*math.sin(yaw)],
    [0, 1, 0, d*math.cos(pitch)*math.sin(yaw)],
    [0, 0, 1, d*math.cos(yaw)],
    [0, 0, 0, 1]
    ])

    # trans_v[0:2, 0:2] = R
    res = R.dot(trans_v)

    # vector matrix multiplication
    res = res * v
    
    return  res

def listener():
    rospy.init_node('input_listener', anonymous=True)
    rospy.Subscriber("chatter", input, callback)
   
    rospy.spin()
   
if __name__ == '__main__':
    listener()
