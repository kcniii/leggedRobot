# import sys
# sys.path.append('./src/simple_walking/wbc/')
# sys.path.append('./src/simple_walking/robots/')
# sys.path.append('./src/simple_walking/modules/')
import numpy as np
import pinocchio as pin
# simulator
#>>>>TODO: Fix include
from simple_walking.robots.pybullet_wrapper import PybulletWrapper
from pybullet_wrapper.simulator.robot import Robot
from simple_walking.wbc.tsid_wrapper import TSIDWrapper
import simple_walking.robots.talos_conf as conf
from simple_walking.modules.footstep_planner import Side, FootStepPlanner, FootStep

# from simulator.pybullet_wrapper import PybulletWrapper
# from simulator.robot import Robot
# from simulator.tsid_wrapper import TSIDWrapper
# import simulator.talos_conf as conf
# from simulator.footstep_planner import Side
# from simulator.footstep_planner import FootStepPlanner, FootStep

import pybullet as pb

# whole-body controller
#>>>>TODO: Fix include
# from simple_walking.wbc.tsid_wrapper import TSIDWrapper

# # robot configs
# #>>>>TODO: Fix include
# import simple_walking.robots.talos_conf as conf

# #>>>>TODO: Fix include
# from simple_walking.modules.footstep_planner import Side

# ROS visualizations
import rospy
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from visualization_msgs.msg import MarkerArray, Marker

class Talos:
    """Talos robot
    combines wbc with pybullet, functions to read and set
    sensor values.
    """
    def __init__(self, simulator):
        
        self.conf = conf
        self.sim = simulator # pybullet simulator
        
        #>>>>TODO: Like allways create the tsid wrapper
        self.stack = TSIDWrapper(conf)
        
        # spawn robot in simulation
        #>>>>TODO: Create the pybullet robot for the simulation
        self.robot = Robot(simulator,
                        conf.urdf, # talos_no_hands.urdf
                        self.stack.model, 
                        basePosition=np.array([0,0,1.1]), 
                        baseQuationerion=np.array([0,0,0,1]), 
                        q=conf.q0, 
                        useFixedBase=False, 
                        verbose=True)
        
        ########################################################################
        # state
        ########################################################################
        self.support_foot = Side.RIGHT
        self.swing_foot = Side.LEFT
        
        ########################################################################
        # estimators
        ########################################################################
        self.zmp = None
        self.dcm = None
        
        ########################################################################
        # sensors
        ########################################################################
        # ft sensors
        #>>>>TODO: Turn on the force torque sensor in the robots feet
        pb.enableJointForceTorqueSensor(self.robot.id(), self.robot.jointNameIndexMap()['leg_right_6_joint'], True)
        pb.enableJointForceTorqueSensor(self.robot.id(), self.robot.jointNameIndexMap()['leg_left_6_joint'], True)
        ########################################################################
        # visualizations
        ########################################################################
        
        #>>>> TODO: joint state publisher
        self.js_pub = rospy.Publisher("/joint_state", JointState, queue_size=10)
        
        #>>>> TODO: floating base broadcaster
        self.br = tf.TransformBroadcaster()
    
        #>>>> TODO: zmp and dcm point publisher 
        #>>>> Hint: Use visualization_msgs::MarkerArray, SPHERE to visualize 
        self.zmp_pub = rospy.Publisher("/zmp", MarkerArray, queue_size=10)
        self.dcm_pub = rospy.Publisher("/dcm", MarkerArray, queue_size=10)

        #>>>> TODO: wrench publisher for left and right foot
        #>>>> Hint: use geometry_msgs::Wrench
        self.wrench_pub_left = rospy.Publisher("/wrench_left", WrenchStamped, queue_size=10)
        self.wrench_pub_right = rospy.Publisher("/wrench_right", WrenchStamped, queue_size=10)

    def update(self):
        """updates the robot
        """
        t = self.sim.simTime()
        dt = self.sim.stepTime()

        #>>>> TODO: update the pybullet robot
        self.sim.debug()
        self.sim.step()
        # update the estimators
        self._update_zmp_estimate()
        self._update_dcm_estimate()
        
        # update wbc and send back to pybullet
        self._solve(t, dt)
        
    def setSupportFoot(self, side):
        """sets the the support foot of the robot on given side
        """
        
        # The support foot is in rigid contact with the ground and should 
        # hold the weight of the robot
        self.support_foot = side
        
        #>>>> TODO: Activate the foot contact on the support foot
        if self.support_foot == Side.LEFT:
            self.stack.contact_LF_active = True
            self.stack.motion_LF_active  =  False

        elif self.support_foot == Side.RIGHT:
            self.stack.contact_RF_active = True
            self.stack.motion_RF_active  =  False

        #>>>> TODO: At the same time deactivate the motion task on the support foot
    
    def setSwingFoot(self, side):
        """sets the swing foot of the robot on given side
        """
        
        # The swing foot is not in contact and can move
        self.swing_foot = side
        
        #>>>> TODO: Deactivate the foot contact on the swing foot
        #>>>> TODO: At the same time turn on the motion task on the swing foot
        if self.swing_foot == Side.LEFT:
            self.stack.contact_LF_active = False
            self.stack.motion_LF_active  =  True

        elif self.swing_foot == Side.RIGHT:
            self.stack.contact_RF_active = False
            self.stack.motion_RF_active  =  True
        
    def updateSwingFootRef(self, T_swing_w, V_swing_w, A_swing_w):
        """updates the swing foot motion reference
        """
        
        #>>>> TODO: set the pose, velocity and acceleration on the swing foots
        if self.swing_foot == Side.RIGHT:
            self.stack.set_RF_pos_ref(T_swing_w, V_swing_w, A_swing_w)
        elif self.swing_foot == Side.LEFT :
            self.stack.set_LF_pos_ref(T_swing_w, V_swing_w, A_swing_w)

        # motion task

    def swingFootPose(self):
        """return the pose of the current swing foot
        """
        #>>>>TODO: return correct foot pose
        if self.swing_foot == Side.RIGHT:
            return self.stack.get_LF_3d_pos_vel_acc()[0]
        elif self.swing_foot == Side.LEFT :
            return self.stack.get_RF_3d_pos_vel_acc()[0]
    
    def supportFootPose(self):
        """return the pose of the current support foot
        """
        #>>>>TODO: return correct foot pose
        if self.support_foot == Side.RIGHT:
            return self.stack.get_RF_3d_pos_vel_acc()[0]
        elif self.support_foot == Side.LEFT :
            return self.stack.get_LF_3d_pos_vel_acc()[0]
    
    def js_info(self):
        js_msg = JointState()
        js_msg.header.stamp = rospy.Time.now()
        js_msg.name = self.robot.actuatedJointNames()
        q = self.robot.q()
        v = self.robot.v()
        js_msg.position = q
        js_msg.velocity = v
        js_msg.effort =None #tbc
        return js_msg
    
    def br_sendTrans(self):
        T_b_w = self.stack.baseState()
        self.br.sendTransform(T_b_w[0].translation,
                    tf.transformations.quaternion_from_matrix(T_b_w[0].homogeneous),
                    rospy.Time.now(),
                    "base_link",
                    "world")

    def wrench_info(self,wr,wr_id):
        wr_msg = WrenchStamped()
        wr_msg.header.frame_id = wr_id
        wr_msg.wrench.force.x = wr.linear[0]
        wr_msg.wrench.force.y = wr.linear[1]
        wr_msg.wrench.force.z = wr.linear[2]
        wr_msg.wrench.torque.x = wr.angular[0]
        wr_msg.wrench.torque.y = wr.angular[1]
        wr_msg.wrench.torque.z = wr.angular[2]
        return wr_msg

    def marker_info(self,id,position):
        marker = Marker()
        marker.header.frame_id = id
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]

        # TODO: add marker to array

        return marker
    
    def publish(self):
        t = rospy.Time.now()
        
        #>>>> TODO: publish the jointstate #########################################
        js_msg = self.js_info()
        self.js_pub.publish(js_msg)

        #>>>> TODO: broadcast odometry #############################################
        self.br_sendT()
        
        #>>>> TODO: publish feet wrenches ###########################################
        wren = pb.getJointState(self.robot.id(), self.robot.jointNameIndexMap()['leg_right_6_joint'])[2]
        wnp = np.array([-wren[0], -wren[1], -wren[2], -wren[3], -wren[4], -wren[5]])
        wr_r_ankel = pin.Force(wnp)

        wren = pb.getJointState(self.robot.id(), self.robot.jointNameIndexMap()['leg_left_6_joint'])[2]
        wnp = np.array([-wren[0], -wren[1], -wren[2], -wren[3], -wren[4], -wren[5]])
        wr_l_ankel = pin.Force(wnp)

        data = self.robot._model.createData()
        pin.framesForwardKinematics(self.robot._model, data, self.robot._q)
        # pin.framesForwardKinematics(self.robot._model, data, q_tsid)

        H_w_lsole = data.oMf[self.robot._model.getFrameId("left_sole_link")]
        H_w_rsole = data.oMf[self.robot._model.getFrameId("right_sole_link")]    
        H_w_lankle = data.oMf[self.robot._model.getFrameId("leg_left_6_joint")]
        H_w_rankle = data.oMf[self.robot._model.getFrameId("leg_right_6_joint")]
        
        wr_r_world = H_w_rankle*wr_r_ankel
        wr_l_world = H_w_lankle*wr_l_ankel

        wr_r_sole = np.inv(H_w_rsole)*wr_r_world
        wr_l_sole = np.inv(H_w_lsole)*wr_l_world

        wr_msg_left = self.wrench_info(wr_r_sole, "leg_left_6_joint")
        wr_msg_right = self.wrench_info(wr_l_sole, "leg_right_6_joint")

        self.wrench_pub_left.publish(wr_msg_left)
        self.wrench_pub_right.publish(wr_msg_right)

        #>>>> TODO: publish dcm and zmp marker #####################################
        zmp_msg = self.marker_info("world", self.zmp)
        dcm_msg = self.marker_info("world", self.dcm)

        self.zmp_pub.publish(zmp_msg)
        self.dcm_pub.publish(dcm_msg)

    ############################################################################
    # private funcitons
    ############################################################################

    def _solve(self, t, dt):
        # get the current state
        q = self.robot.q()
        v = self.robot.v()
        dt = self.sim.stepTime()
        # solve the whole body qp
        #>>>> TODO: sovle the wbc and command the torque to pybullet robot
        hqp = self.stack.formulation.computeProblemData(t, q, v)
        sol = self.stack.solver.solve(hqp)
        if(sol.status!=0):
            print("QP problem could not be solved! Error code:", sol.status)
            pass
        dv_sol = self.stack.formulation.getAccelerations(sol)
        n_update = int(1./(200*dt)) #t
        q, v = self.stack.integrate_dv(q, v, dv_sol, n_update*dt)
        self.robot.setActuatedJointPositions(q, v) #tbc
    
    def _update_zmp_estimate(self):
        """update the estimated zmp position
        """
        #>>>> TODO: compute the zmp based on force torque sensor readings
        self.zmp = None
        
    def _update_dcm_estimate(self):
        """update the estimated dcm position
        """
        #>>>> TODO: compute the com based on current center of mass state
        self.dcm = None

def plot_all():
    step_planner = FootStepPlanner(conf)
    T0 = pin.SE3(np.eye(3), np.array([0, -0.096, 0])) 

    side = Side.RIGHT
    steps_line = step_planner.planLine(T0, side, 8)
    foot_steps = FootStep(steps_line, 0, side)
    foot_steps.plot(PybulletWrapper())

def main(): 
    plot_all()
    talos = Talos(simulator=PybulletWrapper())

    while not rospy.is_shutdown():    
        talos.update()  

if __name__ == '__main__': 
    rospy.init_node('tutorial_4_standing')
    main()