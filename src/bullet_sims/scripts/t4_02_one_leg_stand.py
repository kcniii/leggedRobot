import sys
sys.path.append('src/bullet_sims/scripts/')
sys.path.append('./src/simulator/src/')
import time

import numpy as np
from numpy import nan
from numpy.linalg import norm as norm
import matplotlib.pyplot as plt

# pinocchio
import pinocchio as pin

# simulator
import pybullet as pb
from simulator.pybullet_wrapper import PybulletWrapper
from simulator.robot import Robot

# robot and controller
from t4_tsid_wrapper import TSIDWrapper
import t4_conf as conf

# ROS
import rospy
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

################################################################################
# settings
################################################################################

DO_PLOT = True

################################################################################
# Robot
################################################################################

class Talos(Robot):
    def __init__(self, simulator, urdf, model, q=None, verbose=True, useFixedBase=False):
        self.tau = 0
        # TODO call base class constructor
        super().__init__(simulator, 
                        urdf, 
                        model, 
                        [0, 0, 1.1], 
                        [0,0,0,1], 
                        q=q, 
                        useFixedBase=useFixedBase,
                        verbose=verbose)
        
        # TODO add publisher
        self.joint_pub = rospy.Publisher("/joint_state",JointState,queue_size=10)

        # TODO add tf broadcaster
        self.br = tf.TransformBroadcaster()


    def update(self):
        # TODO update base class
        super().update()
    
    def publish(self, T_b_w,tau):
        # TODO publish jointstate
        joint_msg = JointState()
        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.name = self.actuatedJointNames()
        joint_msg.position = self._q
        joint_msg.velocity = self._v
        joint_msg.effort = tau

        self.joint_pub.publish(joint_msg)

        # TODO broadcast transformation T_b_w
        self.br.sendTransform(T_b_w[0].translation,
                    tf.transformations.quaternion_from_matrix(T_b_w[0].homogeneous),
                    rospy.Time.now(),
                    "base_link",
                    "world")
        pass

################################################################################
# main
################################################################################

def main(): 
    # TODO init TSIDWrapper
    wrapper = TSIDWrapper(conf)
    # TODO init Simulator
    simulator = PybulletWrapper()
    # TODO init ROBOT
    robot = Talos(simulator=simulator,
        urdf=conf.urdf,
        model=wrapper.model,
        q=conf.q_home,
        verbose=True,
        useFixedBase=False)
    
    t_publish = 0.0
    flag1 = 0
    flag2 = 0

    a = 0.05
    f = 0.5
    while not rospy.is_shutdown():

        # elaped time
        t = simulator.simTime()

        # TODO: update the simulator and the robot
        simulator.debug()
        simulator.step()
        robot.update()
        # TODO: update TSID controller
        tau_sol,dv_sol = wrapper.update(robot._q, robot._v,t)
        # command to the robot
        robot.setActuatedJointTorques(tau_sol)

        if flag1 == 0:
            flag1 = 1
            com_p = wrapper.comState().pos()
            rf_p = wrapper.get_placement_RF().translation
            p_com = np.array([rf_p[0], rf_p[1], com_p[2]])
            wrapper.setComRefState(p_com)

        # standing on one leg
        if t > 2.0 and flag2 == 0:
            flag2 = 1
            wrapper.remove_contact_LF()
            lf_pos_SE3 = wrapper.get_placement_LF()
            lf_pos=lf_pos_SE3.translation
            
            new_lf_pos = np.array([lf_pos[0], lf_pos[1], 0.3])
            tmp = pin.SE3(np.eye(3),new_lf_pos)
            wrapper.set_LF_pose_ref(tmp)

        # publish to ros
        if t - t_publish > 1./30.:
            t_publish = t
            # get current BASE Pose
            T_b_w = wrapper.baseState()
            robot.publish(T_b_w,tau_sol)
    
if __name__ == '__main__': 
    rospy.init_node('tutorial_4_standing')
    main()