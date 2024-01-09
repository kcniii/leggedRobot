#!/usr/bin/env python 
import roslib
roslib.load_manifest('ros_visuals')
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

def rot_matrix(ax, ay, az): # x->y->z
    # calculate by hand in order to check
    # rotation = np.array([(0.0, 0.0, 0.0, 1.0 ),
    #                      (0.0, 0, 2**0.5/2, 2**0.5/2),
    #                      (0.0, 0.0, 1.0, 0.0),
    #                      (0,0,-2**0.5/2,2**0.5/2),
    #                      (2**0.5/2,2**0.5/2, 0.0, 0.0),
    #                      (0.0,-1.0, 0.0, 0.0),
    #                      (2**0.5/2, -2**0.5/2, 0, 0),
    #                      (1.0,0.0, 0.0, 0.0)])

    # Rx = np.array([[1, 0, 0],
    #             [0, 0, -1],
    #             [0, 1, 0]])
    # Ry = np.array([[0, 0, 1],
    #             [0, 1, 0],
    #             [-1, 0, 0]])
    # Rz = np.array([[0, 1, 0],
    #             [1, 0, 0], 
    #             [0,0,1]])
    ax=math.radians(ax)
    ay=math.radians(ay)
    az=math.radians(az)

    Rx = np.array([[1,0,0],[0,np.cos(ax), -np.sin(ax)],[0,np.sin(ax), np.cos(ax)]])
    Ry = np.array([[np.cos(ay), 0, np.sin(ay)],[0,1,0],[-np.sin(ay), 0, np.cos(ay)]])
    Rz = np.array([[np.cos(az), -np.sin(az), 0],[np.sin(az), np.cos(az), 0], [0,0,1]])
    re1 = np.matmul(Ry,Rx)

    return np.matmul(Rz,re1)

def marker_info(frame_name,id):
    marker = Marker()

    marker.header.frame_id = frame_name
    # marker.header.stamp = rospy.Time.now()

    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    marker.type = Marker.POINTS
    marker.id = id
    marker.action = Marker.ADD

    # Set the scale of the marker
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    # marker.scale.z = 0.02
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    if id == 1:
    # Set the color
        marker.color.r = 1.0
    else:
        marker.color.r = 0.0
    # Set the pose of the marker
    # marker.pose.position.x = p[0]
    # marker.pose.position.y = p[1]
    # marker.pose.position.z = p[2]
    # marker.pose.orientation.x = 0.0
    # marker.pose.orientation.y = 0.0
    # marker.pose.orientation.z = 0.0
    # marker.pose.orientation.w = 1.0

    return marker

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

def tf_broadcast(o0,o):

    br.sendTransform(tuple(o0[:3]),
            tuple(o0[3:]),
            # tf.transformations.quaternion_from_euler(0, 0, msg.theta),
            rospy.Time.now(),
            "o0",
            "world")

    for i in range(8):
        br.sendTransform(
                tuple(o[i][:3]),
                tuple(o[i][3:]),
                # tf.transformations.quaternion_from_euler(0, 0, 0),
                rospy.Time.now(),
                "o"+str(i+1),
                "o0")

def p_transform(R01,p21):
    
    R01 = quaternionTse3(R01)
    p10 = R01.translation
    r01 = R01.rotation
    p20 = np.dot(np.linalg.inv(r01),p21)+p10

    return p20

# main
if __name__ == '__main__': 

    rospy.init_node('t11')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    # marker pub
    marker_pub = rospy.Publisher("/maker_p_o8", Marker, queue_size = 2)
    marker_pub1 = rospy.Publisher("/marker_p_world", Marker, queue_size = 2)

    p_o8 = np.array([-1,0,0])
    p_world = np.array([-1,0,0])

    # velocity
    nu = pin.Motion.Random()
    nu.linear = np.array([0, 0, 0.0001])
    nu.angular = np.array([0.01, 0, 0])

    # o0 initialize
    o0_translation = np.array([0,0,1])
    o0_se3 = pin.SE3(rot_matrix(0,0,0),o0_translation)

    o0_quaternion = se3Tquaternion(o0_se3)

    [o,transformation] = ot_generate()


    while not rospy.is_shutdown():

        tf_broadcast(o0_quaternion,o)

        # p_world = Rc * (R8 * p + T8) + Tc
        p_o0 = np.dot(rot_matrix(180,0,0),p_o8)+transformation[7]
        p_world = np.dot(o0_se3.rotation, p_o0) + o0_se3.translation

        marker_info_o8 = marker_info("o8", 0)
        marker_info_o8.points=[Point(p_o8[0],p_o8[1], p_o8[2])]
        marker_pub.publish(marker_info_o8) 

        marker_info_world = marker_info("world",1)
        marker_info_world.points = [Point(p_world[0],p_world[1], p_world[2])]
        marker_pub1.publish(marker_info_world)

        o0_se3 = pin.exp6(nu)*o0_se3
        o0_quaternion = se3Tquaternion(o0_se3) 

        rate.sleep()


