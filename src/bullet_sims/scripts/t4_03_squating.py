import sys
sys.path.append('src/bullet_sims/scripts/')
sys.path.append('./src/simulator/src/')

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
def plot_pva(trajectory):
    x_t = [point[0] for point in trajectory]

    # position
    y_p0 = [point[1][0] for point in trajectory]
    y_pb0 = [point[3][0] for point in trajectory]
    y_pr0 = [point[2][0] for point in trajectory]

    y_p1 = [point[1][1] for point in trajectory]
    y_pb1 = [point[3][1] for point in trajectory]
    y_pr1 = [point[2][1] for point in trajectory]

    y_p2 = [point[1][2] for point in trajectory]
    y_pb2 = [point[3][2] for point in trajectory]
    y_pr2 = [point[2][2] for point in trajectory]

    # velocity ***********************************************
    y_v0 = [point[4][0] for point in trajectory]
    y_vb0 = [point[6][0] for point in trajectory]
    y_vr0 = [point[5][0] for point in trajectory]

    y_v1 = [point[4][1] for point in trajectory]
    y_vb1 = [point[6][1] for point in trajectory]
    y_vr1 = [point[5][1] for point in trajectory]

    y_v2 = [point[4][2] for point in trajectory]
    y_vb2 = [point[6][2] for point in trajectory]
    y_vr2 = [point[5][2] for point in trajectory]

    # acceleration *******************************************
    y_a0 = [point[7][0] for point in trajectory]
    # y_ab0 = [point[6][0] for point in trajectory]
    y_ar0 = [point[8][0] for point in trajectory]

    y_a1 = [point[7][1] for point in trajectory]
    # y_ab1 = [point[6][1] for point in trajectory]
    y_ar1 = [point[8][1] for point in trajectory]

    y_a2 = [point[7][2] for point in trajectory]
    # y_ab2 = [point[6][2] for point in trajectory]
    y_ar2 = [point[8][2] for point in trajectory]
    # ax1 ************************************************
    fig1, ax1 = plt.subplots(3,1)
    ax1[0].plot(x_t, y_p0, 'b-')  # 绘制蓝色的轨迹线
    ax1[0].plot(x_t, y_pb0, 'g--')  # 绘制蓝色的轨迹线
    ax1[0].plot(x_t, y_pr0, 'r--')  # 绘制蓝色的轨迹线

    ax1[0].set_xlim([0, 20])
    # ax1[0].set_ylim([-0.2, 0])

    ax1[1].plot(x_t, y_p1, 'b-')  # 绘制蓝色的轨迹线
    ax1[1].plot(x_t, y_pb1, 'g--')  # 绘制蓝色的轨迹线
    ax1[1].plot(x_t, y_pr1, 'r--')  # 绘制蓝色的轨迹线

    ax1[1].set_xlim([0, 20])
    # ax1[1].set_ylim([-0.2, 0])

    ax1[2].plot(x_t, y_p2, 'b-')  # 绘制蓝色的轨迹线
    ax1[2].plot(x_t, y_pb2, 'g--')  # 绘制蓝色的轨迹线
    ax1[2].plot(x_t, y_pr2, 'r--')  # 绘制蓝色的轨迹线

    ax1[2].set_xlim([0, 20])

    # ax2 ********************************************************************
    fig2, ax2 = plt.subplots(3,1)
    ax2[0].plot(x_t, y_v0, 'b-')  # 绘制蓝色的轨迹线
    ax2[0].plot(x_t, y_vb0, 'g--')  # 绘制蓝色的轨迹线
    ax2[0].plot(x_t, y_vr0, 'r--')  # 绘制蓝色的轨迹线

    ax2[0].set_xlim([0, 20])
    # ax1[0].set_ylim([-0.2, 0])

    ax2[1].plot(x_t, y_v1, 'b-')  # 绘制蓝色的轨迹线
    ax2[1].plot(x_t, y_vb1, 'g--')  # 绘制蓝色的轨迹线
    ax2[1].plot(x_t, y_vr1, 'r--')  # 绘制蓝色的轨迹线

    ax2[1].set_xlim([0, 20])
    # ax1[1].set_ylim([-0.2, 0])

    ax2[2].plot(x_t, y_v2, 'b-')  # 绘制蓝色的轨迹线
    ax2[2].plot(x_t, y_vb2, 'g--')  # 绘制蓝色的轨迹线
    ax2[2].plot(x_t, y_vr2, 'r--')  # 绘制蓝色的轨迹线

    ax2[2].set_xlim([0, 20])

    # ax3 ********************************************************************
    fig3, ax3 = plt.subplots(3,1)
    ax3[0].plot(x_t, y_a0, 'b-')  # 绘制蓝色的轨迹线
    # ax3[0].plot(x_t, y_vb0, 'g--')  # 绘制蓝色的轨迹线
    ax3[0].plot(x_t, y_ar0, 'r--')  # 绘制蓝色的轨迹线

    ax3[0].set_xlim([0, 20])
    # ax1[0].set_ylim([-0.2, 0])

    ax3[1].plot(x_t, y_a1, 'b-')  # 绘制蓝色的轨迹线
    # ax3[1].plot(x_t, y_vb1, 'g--')  # 绘制蓝色的轨迹线
    ax3[1].plot(x_t, y_ar1, 'r--')  # 绘制蓝色的轨迹线

    ax3[1].set_xlim([0, 20])
    # ax1[1].set_ylim([-0.2, 0])

    ax3[2].plot(x_t, y_a2, 'b-')  # 绘制蓝色的轨迹线
    # ax3[2].plot(x_t, y_vb2, 'g--')  # 绘制蓝色的轨迹线
    ax3[2].plot(x_t, y_ar2, 'r--')  # 绘制蓝色的轨迹线

    ax3[2].set_xlim([0, 20])

    plt.show()

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
                        useFixedBase=False,
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
    t_plot = 0.0
    flag1 = 0
    flag2 = 0
    flag3 = 0
    flag4 = 0

    c = np.array([0.4, -0.2, 1.1]) #center
    r = 0.2 #radius
    f = 0.1 #frequency

    a = 0.05 #amplitude
    f = 0.5 #frequency

    trajectory = []
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
            lf_pos = wrapper.get_placement_LF().translation

            new_lf_pos = np.array([lf_pos[0], lf_pos[1], 0.3])
            wrapper.set_LF_pose_ref(pin.SE3(np.eye(3),new_lf_pos))

        if t > 4.0:
            com_p = wrapper.comState().pos()
            com_p[2] = com_p[2] + a*np.sin(2*np.pi*0.5*t)
            # a1 = wrapper.comState().acc()com
            # compute the derivatives and call setComRefState(p_com, v_com, a_com)
            v_com = np.array([0, 0, a*np.cos(2*np.pi*0.5*t)*2*np.pi*0.5])
            a_com = np.array([0, 0, -a*np.sin(2*np.pi*0.5*t)*2*np.pi*0.5*2*np.pi*0.5])
            wrapper.setComRefState(com_p, v_com, a_com)
        
        if t > 8.0:

            p_RH = c + np.array([0, r*np.cos(2*np.pi*f*(t-8.0)), r*np.sin(2*np.pi*f*(t-8.0))])
            v_RH = np.array([0, -r*np.sin(2*np.pi*f*(t-8.0))*2*np.pi*f, r*np.cos(2*np.pi*f*(t-8.0))*2*np.pi*f])
            a_RH = np.array([0, -r*np.cos(2*np.pi*f*(t-8.0))*2*np.pi*f*2*np.pi*f, -r*np.sin(2*np.pi*f*(t-8.0))*2*np.pi*f*2*np.pi*f])
            wrapper.set_RH_pos_ref(p_RH, v_RH, a_RH)

            if flag3 == 0:
                flag3 = 1
                wrapper.rightHandTask.setKp(
                    wrapper.conf.kp_hand* np.array([1, 1, 1, 1, 1, 3]))
                wrapper.rightHandTask.setKd(
                    2.0 * np.sqrt(wrapper.conf.kp_hand) * np.array([1, 1, 1, 1, 1, 3]))
                wrapper.formulation.addMotionTask(wrapper.rightHandTask, wrapper.conf.w_hand, 1, 0.0)
                # wrapper.formulation.addMotionTask(wrapper.leftHandTask, wrapper.conf.w_hand, 1, 0.0)
                
        if t > 16.0 and flag4 ==0:
            flag4 = 1
            plot_pva(trajectory)

        # plot
        # print("ploting: ",t)
        if t - t_plot > 1./30.:
            t_plot = t
            if t>0 and t< 16.0 and flag4 == 0:
                com1 = wrapper.comState()
                ref1_com = wrapper.comReference()
                base1_com_p = robot.baseCoMPosition()
                base1_com_v = robot.baseCoMVelocity()
                
                trajectory.append((t, com1.pos(),ref1_com.pos(), base1_com_p,  # 1 2 3
                                com1.vel(),ref1_com.vel(),base1_com_v, # 4 5 6
                                com1.acc(),ref1_com.acc())) # 7 8

        if t - t_publish > 1./10.:
            t_publish = t
            # get current BASE Pose
            T_b_w = wrapper.baseState()
            robot.publish(T_b_w,tau_sol)
    
if __name__ == '__main__': 
    rospy.init_node('tutorial_4_standing')
    main()