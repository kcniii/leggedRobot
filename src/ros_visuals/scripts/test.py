import roslib
import rospy
import tf
import math
import pinocchio as pin
from pinocchio.utils import *
import numpy as np
from pinocchio import SE3ToXYZQUAT as se3Tquaternion
from pinocchio import XYZQUATToSE3 as quaternionTse3
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from pinocchio import Motion


v = np.array([1.0, 0, 0])
w = np.array([0.5, 0, 0])
vc = pin.Motion(v,w)
print(vc.linear)
print(vc.angular)

def ot_generate():
    transformation=np.array([(-0.5, -0.3, -0.4),
                        (0.5, -0.3, -0.4),
                        (0.5, 0.3, -0.4),
                        (-0.5, 0.3, -0.4),
                        (-0.5, -0.3, 0.4),
                        (0.5, -0.3, 0.4),
                        (0.5, 0.3, 0.4),
                        (-0.5, 0.3, 0.4)]) 
    o = []
    o.append(se3Tquaternion(pin.SE3(rot_matrix(0,0,0), transformation[0])))
    o.append(se3Tquaternion(pin.SE3(rot_matrix(0,0,90), transformation[1])))
    o.append(se3Tquaternion(pin.SE3(rot_matrix(0,0,180), transformation[2])))
    o.append(se3Tquaternion(pin.SE3(rot_matrix(0,0,-90), transformation[3])))

    o.append(se3Tquaternion(pin.SE3(rot_matrix(180,0,90), transformation[4])))
    o.append(se3Tquaternion(pin.SE3(rot_matrix(180,0,180), transformation[5])))
    o.append(se3Tquaternion(pin.SE3(rot_matrix(180,0,-90), transformation[6])))
    o.append(se3Tquaternion(pin.SE3(rot_matrix(180,0,0), transformation[7])))

    return o,transformation
def rot_matrix(ax, ay, az): # x->y->z
    ax=math.radians(ax)
    ay=math.radians(ay)
    az=math.radians(az)

    Rx = np.array([[1,0,0],[0,np.cos(ax), -np.sin(ax)],[0,np.sin(ax), np.cos(ax)]])
    Ry = np.array([[np.cos(ay), 0, np.sin(ay)],[0,1,0],[-np.sin(ay), 0, np.cos(ay)]])
    Rz = np.array([[np.cos(az), -np.sin(az), 0],[np.sin(az), np.cos(az), 0], [0,0,1]])
    re1 = np.matmul(Ry,Rx)

    return np.matmul(Rz,re1)

def twist_transformation(m21,m10,r1_0,p1_0,p2_1):

    p2_0 = p1_0 + np.dot(r1_0,p2_1)

    v2_0 = m10.linear + np.dot(r1_0,m21.linear) + np.dot(m10.angular,p2_0 - p1_0)
    w2_0 = m10.angular + np.dot(r1_0,m21.angular)

    m20=Motion(v2_0,w2_0)

    return m20


[o,transformation] = ot_generate()

m21 = Motion(np.array([1,0,0,0.5,0,0]))
m10 = Motion(np.array([0, 0, 0, 0, 0, 0]))

o0_translation = np.array([1,1,1])
o0_se3 = pin.SE3(rot_matrix(0,0,0),o0_translation)
o8_se3 = quaternionTse3(o[8-1])

# m21 = Motion(np.array([1, 0, 0, 0.5, 0, 0]))
# m20 = twist_transformation(m21, m10, o0_se3.rotation, o0_se3.translation,o8_se3.translation)
# # print(m20)
# print('***')
# # print('action',o0_se3.action)
# # print("m20",m20)
# twist_global = o8_se3.act(m21)
# print(twist_global)
# wrench_value= pin.Force(np.array([1.0, 1.0, 1.0, 2.0, 2.0, 2.0]))
# print('nunununu')
nu = pin.Motion.Random()
nu.linear=np.array([1, 0, 0])
nu.angular = np.array([0, 0, 0])
print('twist_info')
print(nu)
print('linear',nu.linear)
# o0_se3 = pin.exp6(nu).dot(o0_se3)
M=pin.exp6(nu)
print(M)
print(o0_se3*M)

print('action')
print(o8_se3)
print('pin.log',pin.log(o8_se3))
pin.exp(nu)

wrench = pin.Force.Random()
print(wrench.linear)

import sys
print(sys.path)
a=b=10

my_dict = {'key1': 'value1', 'key2': 'value2', 'key3': 'value3'}
print('\n\nmy_dict',my_dict)

str_my_dict=str(my_dict)
print("\n\nstring",str_my_dict)
pass