import copy
import numpy as np
import numpy.linalg as la
from simulator.body import Body

# simulator
from simulator.pybullet_wrapper import PybulletWrapper
from simulator.robot import Robot

# robot modeling
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

# from simulator.robot import Robot

# import ndcurves as curves
from enum import Enum

# ROS
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

# todo 
import pybullet as pb
# TODO
urdf = "/home/kaicheng/ros/workspaces/t3_ws/src/talos/talos_description/robots/talos_reduced.urdf"
path_meshes = "/home/kaicheng/ros/workspaces/t3_ws/src/talos/talos_description/meshes/../.."
################################################################################
# utility functions
################################################################################
class State(Enum):
    JOINT_SPLINE = 0,
    CART_SPLINE = 1
################################################################################
# Robot
################################################################################
class Talos(Robot):
    def __init__(self, simulator, q=None, verbose=True, useFixedBase=True):
        #TODO: Call base class constructor, make publisher
        self.wrapper = pin.RobotWrapper.BuildFromURDF(urdf,             # Model description
                                path_meshes,                 # Model geometry descriptors 
                                None,                        #pin.JointModelFreeFlyer(),   # Floating base model. Use "None" if fixed
                                True,                        # Printout model details
                                None)                        # Load meshes different from the descripor
        print('\n\n\nself.model.nq',self.wrapper.model.nq)
        self.data = self.wrapper.model.createData()
        self.model = self.wrapper.model
        super().__init__(simulator, 
                        urdf, 
                        self.model, 
                        [0, 0, 1.15], 
                        [0,0,0,1], 
                        q=q, 
                        useFixedBase=True,
                        verbose=verbose)
        
        # self.qr= q
        self.jointstate_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    def update(self):
        # TODO: update base class, update pinocchio robot wrapper kinematics
        super().update()
        self.wrapper.forwardKinematics(self._q)
        
    def wrapper(self):          
        return self._wrapper

    def data(self):
        return self._wrapper.data
    
    def publish(self,tau): #
        # TODO: publish robot state to a
        joint_state_msg = JointState()

        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = self.actuatedJointNames()
        joint_state_msg.position = self._q
        joint_state_msg.velocity = self._v
        joint_state_msg.effort = tau

        self.jointstate_pub.publish(joint_state_msg)
################################################################################
# Controllers
################################################################################

class JointSpaceController:
    """JointSpaceController
    Tracking controller in jointspace
    """
    def __init__(self, robot, Kp, Kd):
        # save gains, robot
        self.robot = robot
        self.Kp = Kp
        self.Kd = Kd
    
    def update(self, q_r, q_r_dot, q_r_ddot): # q_r  -> q_spline
        # Compute jointspace torque, return torque
        # \tau = M(q)[q_r_ddot-Kd(qdot-q_r_dot)-Kp(q-q_r)] + h(q,qdot)
        error_q = self.robot._q - q_r
        error_q_dot = self.robot._v - q_r_dot
        # h = pin.rnea(self.robot.wrapper.model, 
        #             self.robot.wrapper.data, 
        #             self.robot._q, 
        #             self.robot._v, 
        #             self.robot._a)
        # h = pin.rnea(self.robot.model, self.robot.data, self.robot._q, self.robot._v,0)
        h= 0
        M = pin.crba(self.robot.model, self.robot.data, self.robot._q)  
        tau_new = M.dot(q_r_ddot - self.Kd.dot(error_q_dot) - self.Kp.dot(error_q))+h

        return tau_new
    
class CartesianSpaceController:
    """CartesianSpaceController
    Tracking controller in cartspace
    """
    def __init__(self, robot, joint_name, Kp, Kd):
        # save gains, robot
        self.robot = robot
        self.joint_name = joint_name
        self.Kp = Kp
        self.Kd = Kd


    # τ = M J^# (x_ddot_d - J_dot*q) + h
    def update(self, X_r, X_dot_r, X_ddot_r): # here X_r is a SE3 object
            # 1 compute cartesian control torque, return torque
        id = self.robot.model.getJointId(self.joint_name) # First, get the joint id from the joint name via
            # 2 Compute the Joint Jacobian as show in the linked example.
        J = pin.computeJointJacobian(self.robot.model, self.robot.data, self.robot._q, id)
            # 3 Get the current Cartesian pose and velocity of your controlled frame with
        X = self.robot.data.oMi[id]
        # print('\n\n\n\n\n\n\n X ***************************',X)
        X_dot = J.dot(self.robot._v)
        # print('\n\n\n',X_dot)
            # 4 Compute the Cartesian desired acc X_ddot_d using the logarithmic Map: pin.log()
            #   4x4 or se3 -> log6
            # 3x3 ->  log3   ?????
            # X_ddot_d = X_ddot_r-Kd(X_dot-X_dot_r)-Kp(X-X_r)
            # TypeError: unsupported operand type(s) for *: 'SE3' and 'int'

        # a1 = self.Kd.dot(X_dot-X_dot_r)
        # b1 = pin.log(X.inverse()*X_r)
        # a2 = self.Kp.dot(b1.vector)
        # X_ddot_d = X_ddot_r - a1 - a2
        X_ddot_d = X_ddot_r - self.Kd.dot(X_dot-X_dot_r) - self.Kp.dot(pin.log(X.inverse()*X_r).vector)

            # 5 Compute the term J_dot*q_dot. In pinocchio this available through the function: pin.getClassicalAcceleration(...)
        J_dot = pin.computeJointJacobiansTimeVariation(self.robot.model, self.robot.data, self.robot._q, self.robot._v)
        # q_dot = pin.getClassicalAcceleration(self.robot.model, self.robot.data, id, pin.ReferenceFrame.WORLD)
        # J_dot_q_dot = J_dot*q_dot
        #TODO 
        J_dot_q = J_dot.dot(self.robot._v)

            # 6 Map the desired acc to joint space using the pseudo inverse of the Jacobian: pin.computeJointJacobianInverse(...)
        # J_inv = pin.computeJointJacobianInverse(self.robot.model, self.robot.data, self.robot._q)
        J_pseudo_inv = np.linalg.pinv(J)
        # compute the damped Jacobian
        # map x_ddot_d to q_ddot_d

        
        # finnaly compute the torque
        M = pin.crba(self.robot.model, self.robot.data, self.robot._q)
        # print(M.shape)
        #TODO
        # h = pin.rnea(self.robot.model, self.robot.data, self.robot._q, self.robot._v,self.robot.data.a)
        h = 0

        tau_cart = M.dot(np.dot(J_pseudo_inv,(X_ddot_d - J_dot_q)))+h
        # print(tau_cart.shape)

        # N = np.eye(self.robot.model.nq) - np.dot(J.T, J_pseudo_inv.T)
        # #tau_posture = -Kp(q-q_r) - Kd*qdot
        # #TODO
        # tau_posture = -self.Kp.dot(self.robot._q - self.robot.q()) - self.Kd.dot(self.robot._v)
        # tau = tau_cart + N*tau_posture

        return tau_cart
################################################################################
# Application
################################################################################
class Environment:
    def __init__(self):        
        # state
        self.cur_state = 0
        # self.cur_state = State.CART_SPLINE
        # create simulation
        self.simulator = PybulletWrapper()
        
        ########################################################################
        # spawn the robot
        ########################################################################
        self.q_home = np.zeros(32)
        self.q_home[14:22] = np.array([0, +0.45, 0, -1, 0, 0, 0, 0 ])
        self.q_home[22:30] = np.array([0, -0.45, 0, -1, 0, 0, 0, 0 ])
        
        self.q_init = np.zeros(32)

        # TODO: spawn robot
        self.robot = Talos(self.simulator, q=self.q_init)
        ########################################################################
        # joint space spline: init -> home
        ########################################################################

        # TODO: create a joint spline 
        # self.q_spline = pin.interpolate(self.robot.wrapper.model, self.q_init, self.q_home, 0)
        # self.q_spline = self.q_init
        self.q_spline_t = self.q_init
        self.q_spline_dot_t = np.zeros(32)
        self.q_spline_ddot = np.zeros(32)

        # TODO: create a joint controller
        
        self.Kp = np.eye(32)
        # leg
        self.Kp[0:12,0:12] = self.Kp[0:12,0:12]*2200
        # arm
        self.Kp[14:30,14:30] = self.Kp[14:30,14:30]*200
        # torso
        self.Kp[12:14,12:14] = self.Kp[12:14,12:14] *500
        # head
        self.Kp[30:32,30:32]= self.Kp[30:32,30:32]*300

        self.Kd = np.eye(32)
        # leg
        self.Kd[0:12,0:12] = self.Kd[0:12,0:12]*4

        self.Kp_c = np.eye(6)*1
        # self.Kd_c = np.eye(6)*550
        self.Kd_c = np.eye(6)*900
        self.flag = 0

        self.X_goal = pin.SE3.Random()
        # self.X_goal.translation = np.array([0, 0, 0])
        # self.X_goal.rotation = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        ########################################################################
        # cart space: hand motion
        ########################################################################

        # TODO: create a cartesian controller
        # if self.cur_state == 0:
        #     self.controller = JointSpaceController(self.robot, self.Kp, self.Kd)
        # elif self.cur_state == 1:
        #     self.controller = CartesianSpaceController(self.robot, 'arm_left_7_joint', self.Kp_c, self.Kd_c)
        #     self.X_goal = self.controller.robot.data.oMi[self.controller.robot.model.getJointId('arm_left_7_joint')]
        
        ########################################################################
        # logging
        ########################################################################
        
        # TODO: publish robot state every 0.01 s to ROS
        self.t_publish = 0.0
        self.publish_period = 0.01

        # rospy.wait_for_message('/marker_topic', PoseStamped)
        # self.marer_sub =  rospy.Subscriber('/marker_pose', PoseStamped, self.pose_callback)
    
    def pose_callback(self, msg): # extract the pose from the message
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        # Set the desired pose as the new goal
        self.X_goal = pin.XYZQUATToSE3([x, y, z, qx, qy, qz, qw])

    def update(self, t, dt):
        # TODO: update the robot and model
        self.robot.update()
        tau = np.zeros(self.robot.model.nq)

        # update the controller
        # TODO: Do jointspace and cartesianspace control
        # if t<=5:
        if t<=5:
            self.controller = JointSpaceController(self.robot, self.Kp, self.Kd)
            # self.cur_state = 0
            if t == 0:
                q_spline_t1 = self.q_init
                print("Joint control>>>>>>>>>>")
            else:   
                q_spline_t1 = pin.interpolate(self.robot.model, self.q_init, self.q_home, (t-dt)/5)

            self.q_spline_t = pin.interpolate(self.robot.model, self.q_init, self.q_home, t/5)
            q_spline_t2 = pin.interpolate(self.robot.model, self.q_init, self.q_home, (t+dt)/5)

            self.q_spline_dot_t = (self.q_spline_t - q_spline_t1)/dt
            q_spline_dot_t1 = (q_spline_t2 - self.q_spline_t)/dt

            self.q_spline_ddot = (q_spline_dot_t1 - self.q_spline_dot_t)/dt
            # self.cur_state = 1
            tau_new = self.controller.update(self.q_spline_t, self.q_spline_dot_t, self.q_spline_ddot)
        
        else:
            self.controller = CartesianSpaceController(self.robot, 'arm_left_7_joint', self.Kp_c, self.Kd_c)
            if self.flag == 0:
                self.X_goal = self.controller.robot.data.oMi[self.controller.robot.model.getJointId('arm_left_7_joint')]
                # print(self.X_goal)
                print("Cartesian control>>>>>>>>>>")
                self.flag = 1
            # X_r is equal to the initial of self.X_goal
            # X_goal = pin.SE3.Random()
            # X_goal.translation = np.array([0, 0, 0])

            rospy.Subscriber('/marker_pose', PoseStamped, self.pose_callback)

        # Set the desired pose as the new goal
            X_r = self.X_goal
            X_dot_r = np.zeros(6)
            X_ddot_r = np.zeros(6)
            tau_new=self.controller.update(X_r, X_dot_r, X_ddot_r)
        
        # switch to Cartesian control

        # if t<= 5:
        #     tau_new = self.controller.update(self.q_spline_t, self.q_spline_dot_t, self.q_spline_ddot)
        # else:
        #     print("\n\n\n\n\nCatesian control\n\n\n\n\n")
        #     X_r = self.X_goal
        #     X_dot_r = np.zeros(6)
        #     X_ddot_r = np.zeros(6)
        #     tau_new=self.controller.update(X_r, X_dot_r, X_ddot_r)
        
        
        self.robot.setActuatedJointTorques(tau_new)

        # TODO: publish
        self.robot.publish(tau_new)

def main():
    env = Environment()

    while not rospy.is_shutdown():
        t = env.simulator.simTime()
        dt = env.simulator.stepTime()

        env.update(t, dt)
        
        env.simulator.debug()
        env.simulator.step()
        # rate.sleep()

if __name__ == '__main__': 
    rospy.init_node('tutorial_3_robot_sim')
    rate = rospy.Rate(10)
    main()