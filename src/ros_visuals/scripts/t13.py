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
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import WrenchStamped
from pinocchio import Motion

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

def twist_info(frame_id,V):
    # Create a TwistStamped message and fill in its values
    # V = Motion(v)
    msg = TwistStamped()
    msg.header.frame_id = frame_id
    # msg.header.stamp = rospy.Time.now()
    msg.twist.angular.x = V.angular[0]
    msg.twist.angular.y = V.angular[1]
    msg.twist.angular.z = V.angular[2]
    msg.twist.linear.x = V.linear[0]
    msg.twist.linear.y = V.linear[1]
    msg.twist.linear.z = V.linear[2]

    return msg
# b.2
# v_o8Tworld = twist_transformation(v_o8, nu, o0_se3.rotation, o0_se3.translation, o8_se3.translation)
def twist_transformation(m21,m10,r1_0,p1_0,p2_1):

    p2_0 = p1_0 + np.dot(r1_0,p2_1)

    v2_0 = m10.linear + np.dot(r1_0,m21.linear) + np.dot(m10.angular,(p2_0 - p1_0))
    # w2_0 = m10.angular + np.dot(r1_0,m21.angular)
    w2_0 = m10.angular

    m20=Motion(v2_0,w2_0)

    return m20

def wrench_info(frame_id, W):
    msg = WrenchStamped()
    msg.header.frame_id = frame_id

    msg.wrench.force.x = W.linear[0]
    msg.wrench.force.y = W.linear[1]
    msg.wrench.force.z = W.linear[2]
    msg.wrench.torque.x = W.angular[0]
    msg.wrench.torque.y = W.angular[1]
    msg.wrench.torque.z = W.angular[2]

    return msg

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
    nu.linear = np.array([0, 0, 0.001])
    nu.angular = np.array([0.01, 0, 0])

    # o0 initialize
    o0_translation = np.array([0,0,1])
    o0_se3 = pin.SE3(rot_matrix(0,0,0),o0_translation)

    o0_quaternion = se3Tquaternion(o0_se3)

    [o,transformation] = ot_generate()
    

    # b.3 create a motion vector vc
    v_o8 = pin.Motion.Random()
    v_o8.linear = np.array([1, 0, 0])
    v_o8.angular = np.array([0.5, 0, 0])

    ## twist 0  (publish vc with respect to o8)
    twist_pub0 = rospy.Publisher('/twist_o8', TwistStamped, queue_size=10) 
    twist_msg0 = twist_info('o8', v_o8)

    # b.4
    o8_se3 = quaternionTse3(o[8-1])
    ## twist 1
    twist_pub1 = rospy.Publisher('/twist_o8_T_world', TwistStamped, queue_size=10)

    # b.5
    ## twist 2
    v_world = pin.Motion(np.array([0,1,0,0,0.5,0]))
    twist_pub2 = rospy.Publisher('/twist_world', TwistStamped, queue_size=10)

    o7_se3 = quaternionTse3(o[7])
    twist_pub3 = rospy.Publisher('/twist_world_T_o7', TwistStamped, queue_size=10)

    twist_pub4 = rospy.Publisher('/twist_o8_T_world_action', TwistStamped, queue_size=10)
    twist_pub5 = rospy.Publisher('/twist_world_T_o7_action', TwistStamped, queue_size=10)

    # c
    w_o8 = pin.Force.Random()
    w_o8.linear = np.array([1,0,0])
    w_o8.angular = np.array([0.5,0,0])

    wrench_pub0 = rospy.Publisher('/wrench_o8', WrenchStamped,queue_size=10)
    
    '''
    f   linear
    tau angualr
    '''
    wrench_msg0 = wrench_info('o8',w_o8)

    wrench_pub1 = rospy.Publisher('/wrench_o8_T_world', WrenchStamped,queue_size=10)
    
    # c.5
    w_world = pin.Force(np.array([0,1,0,0,0.5,0]))
    wrench_pub2= rospy.Publisher('/wrench_world', WrenchStamped,queue_size=10)
    wrench_msg2 = wrench_info('world',w_world)
    # wrench_pub2 =  rospy.Publisher('/wrench_world', WrenchStamped,queue_size=10)

    wrench_pub3 = rospy.Publisher('/wrench_world_T_o7', WrenchStamped,queue_size=10)

    wrench_pub4 = rospy.Publisher('/wrench_o8_T_world_action', WrenchStamped,queue_size=10)
    wrench_pub5 = rospy.Publisher('/wrench_world_T_o7_action', WrenchStamped,queue_size=10)
    while not rospy.is_shutdown():

        tf_broadcast(o0_quaternion,o)
        # wrench 0
        wrench_pub0.publish(wrench_msg0)

        # wrench 1
        w_o8Tworld = o0_se3*o8_se3*w_o8
        wrench_msg1 = wrench_info('world',w_o8Tworld )
        wrench_pub1.publish(wrench_msg1)

        # wrench 2
        wrench_pub2.publish(wrench_msg2)

        # wrench 3
        w_worldTo7 = o7_se3.inverse()*(o0_se3.inverse()*w_world)
        wrench_msg3 = wrench_info('o7',w_worldTo7 )
        wrench_pub3.publish(wrench_msg3)

        # c.6 use action
        w_o8Tworld_action = o0_se3.act(o8_se3.act(w_o8))
        # v_o8Tworld_action = o0_se3.action(o8_se3.action(v_o8))
        wrench_msg4 = wrench_info('world', w_o8Tworld_action)
        wrench_pub4.publish(wrench_msg4)

        w_worldTo7_action = o7_se3.inverse().act(o0_se3.inverse().act(w_world))
        wrench_msg5 = wrench_info('o7', w_worldTo7_action)
        wrench_pub5.publish(wrench_msg5)
# *****************************************t12*******************************************
        # t1 - t4
        # t3 - t5
        # twist 0   v o8
        twist_pub0.publish(twist_msg0)

        # twist 1   v o8 T world
        v_o8Tworld = o0_se3*o8_se3*v_o8
        # v_o8Tworld = twist_transformation(v_o8, nu, o0_se3.rotation, o0_se3.translation, o8_se3.translation)
        twist_msg1 = twist_info('world', v_o8Tworld)
        twist_pub1.publish(twist_msg1)

        # twist 2   v World
        twist_msg2 = twist_info('world', v_world)
        twist_pub2.publish(twist_msg2)

        # twist 3 v world T o7
        # world_o0_o7 = o0_se3 * o7_se3
        # v_worldTo7 = world_o0_o7.inverse() * v_world

        v_worldTo7 = o7_se3.inverse()*(o0_se3.inverse()*v_world)

        twist_msg3 = twist_info('o7', v_worldTo7)
        twist_pub3.publish(twist_msg3)


        # b.6 use action
        v_o8Tworld_action = o0_se3.act(o8_se3.act(v_o8))
        # v_o8Tworld_action = o0_se3.action(o8_se3.action(v_o8))
        twist_msg4 = twist_info('world', v_o8Tworld_action)
        twist_pub4.publish(twist_msg4)

        v_worldTo7_action = o7_se3.inverse().act(o0_se3.inverse().act(v_world))
        twist_msg5 = twist_info('o7', v_worldTo7_action)
        twist_pub5.publish(twist_msg5)

# *****************************************t12*******************************************

# *****************************************t11*******************************************
        # marker pub
        # p_world = Rc * (R8 * p + T8) + Tc
        p_o0 = np.dot(rot_matrix(180,0,0),p_o8)+transformation[7]
        p_world = np.dot(o0_se3.rotation, p_o0) + o0_se3.translation

        marker_info_o8 = marker_info("o8", 0)
        marker_info_o8.points=[Point(p_o8[0],p_o8[1], p_o8[2])]
        marker_pub.publish(marker_info_o8) 

        marker_info_world = marker_info("world",1)
        marker_info_world.points = [Point(p_world[0],p_world[1], p_world[2])]
        marker_pub1.publish(marker_info_world)
# *****************************************t11*******************************************
        # update
        o0_se3 = pin.exp6(nu)*o0_se3
        o0_quaternion = se3Tquaternion(o0_se3) 

        
        
        rate.sleep()


