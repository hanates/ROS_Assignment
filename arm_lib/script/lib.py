import numpy as np
def Rx(theta):
    return np.array([
        [1, 0, 0, 0], 
        [0, np.cos(theta), -np.sin(theta), 0], 
        [0, np.sin(theta),np.cos(theta), 0], 
        [ 0,  0,  0,   1]])

def Ry(theta):
    return np.array([
        [np.cos(theta), 0, np.sin(theta), 0], 
        [0, 1, 0, 0], 
        [-np.sin(theta), 0, np.cos(theta), 0],
        [0, 0, 0, 1]])
	
def Rz(theta):
    return np.array([
        [np.cos(theta), np.sin(theta), 0, 0], 
        [np.sin(theta), np.cos(theta), 0, 0], 
        [0, 0, 1 , 0],
        [0, 0, 0, 1]])
	
def T(x=0, y=0, z=0):
    return np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])


#M1 = T(0, 0, 0.1)
#M2 = Rz(np.radians(60)).dot(T(0, 0, 0.05))
#M3 = Rx(np.radians(45)).dot(T(0, 0, 2.0))
#M4 = Rx(np.radians(-30)).dot(T(0, 0, 1.0))
#M5 = Rx(np.radians(-45)).dot(T(0, 0, 0.5))

#M = (((M1.dot(M2)).dot(M3)).dot(M4)).dot(M5)

#print(M)


#import tinyik as ik

#arm = ik.Actuator([
#   "z", [0, 0, 0.15],
#    "x", [0, 0, 2.0],
#    "x", [0, 0, 1.0],
#    "x", [0, 0, .5]
#])

#"z", [0, 0, 0.15],

#print(np.degrees(arm.angles))
