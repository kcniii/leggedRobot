"""robot
Robot base class (connection pybullet & pinocchio)
Author: Simon Armleder
"""

import numpy as np
import pybullet as pb
import pinocchio as pin

from pybullet_wrapper.simulator.body import Body
import pybullet_wrapper.simulator.utilities as sim_utils

class ControlMode:
    positionControl = 'positionControl'
    torqueControl = 'torqueControl'

class Robot(Body):
    '''
    The robot base class: merges pinocchio and pybullet body
    Note: if the body is a floating base rbdyn joint vector has a different dimension
    than the pybullet joint vector. As it contains the base transformation 
    X_b_w in the first six elements.

    Note: pinocchio is not using pybullet base frame as its base!
    This is important when converting pinocchio frames to pb and vice versa
    '''
    def __init__(self, 
                simulator,
                filename,
                model, 
                basePosition=np.array([0,0,0]), 
                baseQuationerion=np.array([0,0,0,1]), 
                q=None, 
                useFixedBase=True, 
                verbose=True):

        super().__init__(
                simulator,
                filename=filename,
                position=basePosition,
                orientation=baseQuationerion,
                use_fixed_base=useFixedBase,
                flags=pb.URDF_USE_INERTIA_FROM_FILE) # +pb.URDF_MERGE_FIXED_LINKS <-- changes the dynamics quite a lot!

        # print information
        if verbose:
            print('*' * 100 + '\nPyBullet Robot Info ' + '\u2193 '*20 + '\n' + '*' * 100)
            print('robot ID:              \n', self.id())
            print('robot name:            ', self.name())
            print('robot total mass:      ', self.mass())
            print('base link name:        ', self.baseName())
            print('num of joints:         ', self.numJoints())
            print('num of actuated joints:', self.numActuatedJoints())
            print('joint names:           ', len(self.jointNames()), self.jointNames())
            print('joint indexes:         ', len(self.jointIndexes()), self.jointIndexes())
            print('actuated joint names:  ', len(self.actuatedJointNames()), self.actuatedJointNames())
            print('actuated joint indexes:', len(self.actuatedJointIndexes()), self.actuatedJointIndexes())
            print('link names:            ', len(self.linkNames()), self.linkNames())
            print('link indexes:          ', len(self.linkIndexes()), self.linkIndexes())
            print('joint dampings:        ', self.jointDampings())
            print('joint frictions:       ', self.jointFrictions())
            print('*' * 100 + '\nPyBullet Robot Info ' + '\u2191 '*20 + '\n' + '*' * 100)

        # self.enableTorqueControl()
        self.enablePositionControl()

        # save the model
        self._model = model

        # offsets in pos and vel vector for floating base
        self._pos_idx_offset = 0
        self._vel_idx_offset = 0
        if self.isFloatingBase():
            self._pos_idx_offset += 7 # 3 pos + 4 quaternion
            self._vel_idx_offset += 6 # 3 lin vel + 3 omega

        # build the mappings
        self._tau_map_pin_pb, self._q_map_pb_pin, self._q_map_pin_pb, self._v_map_pb_pin, self._v_map_pin_pb = self.computeBulletPinoccioMaps(self._model)

        # state of the robot and commands
        self._q = np.zeros(self.numActuatedJoints()+self._pos_idx_offset)
        self._v = np.zeros(self.numActuatedJoints()+self._vel_idx_offset)
        self._q_cmd = np.zeros(self.numActuatedJoints()+self._pos_idx_offset)      
        self._v_cmd = np.zeros(self.numActuatedJoints()+self._vel_idx_offset)
        self._tau_cmd = np.zeros(self.numActuatedJoints())

        if verbose:
            print("************************************************************")
            print('q home', q)

        # set the intial jointstate, use zero if not specified
        if q is None:
            q = np.zeros(self.numActuatedJoints()+self._pos_idx_offset)
            if self.isFloatingBase():
                q[:3] = basePosition
                q[3:7] = baseQuationerion

        if verbose:
            print("************************************************************")
            print('q', q)

        # visualizations
        self._com_marker = False

        if verbose:
            print("************************************************************")
            print('q_br', self._q)

        # set the robots jointstate
        self.resetJointState(q)

        if verbose:
            print("************************************************************")
            print('q_ar', self._q)

        # run robot a little to set inital jointstate
        for i in range(10):
            self._sim.step()

        # save the inital jointstate and inital base frame
        self._q_init = q
        self._x_root_w, self._Q_root_w = self.computeRootPose()
        self._T_root_w = pin.SE3(sim_utils.quaternionToMatrix(self._Q_root_w), self._x_root_w)

        # update the robot state once
        self.updateState()

        if verbose:
            print("************************************************************")
            print('q_end', self._q)

    def model(self):
        return self._model

    #---------------------------------------------------------------------------
    # update
    #---------------------------------------------------------------------------

    def update(self):
        '''
        call this in every iteration
        '''

        # update the robot model
        self.updateState()

        # update visuals
        if self._com_marker:
            self._sim.resetBasePose(self._com_marker, self.baseCoMPosition(), [1, 0, 0, 0])

    #---------------------------------------------------------------------------
    # settings
    #---------------------------------------------------------------------------

    def resetJointState(self, q=None, v=None):
        '''
        Note: joint position needs to respect joint limits
        Note: q and v are assumend to be in pinocchio format and also contain floating base
        '''

        # transform to pybullet
        self._q_cmd[self._q_map_pin_pb] = q
        self._v_cmd[self._v_map_pin_pb] = v

        if q is None:
            self._q_cmd[self._q_map_pin_pb] = self._q_init

        # reset states and reactivate position control to hold state
        if v is not None:
            self._sim.resetJointState(self.id(), self.actuatedJointIndexes(), self._q_cmd[self._pos_idx_offset:], self._v_cmd[self._vel_idx_offset:])
            self.setActuatedJointPositions(self._q_cmd, self._v_cmd)
        else:
            self._sim.resetJointState(self.id(), self.actuatedJointIndexes(), self._q_cmd[self._pos_idx_offset:])
            self.setActuatedJointPositions(self._q_cmd)

        # update the robot state once
        self.updateState()

    def resetJointDampings(self, jointDamping=0.0):
        self._sim.jointDamping(self.id(), self.jointIndexes(), jointDamping)

    def enablePositionControl(self):
        self._sim.enableJointPositionControlMode(self.id(), self.jointIndexes())
        self._controlMode = ControlMode.positionControl
        print(self._controlMode, 'enabled!')

    def enableTorqueControl(self):
        self._sim.enableJointTorqueControlMode(self.id(), self.jointIndexes(), 0)
        self._controlMode = ControlMode.torqueControl
        print(self._controlMode, 'enabled!')

    #---------------------------------------------------------------------------
    # joint state
    #---------------------------------------------------------------------------

    def jointPosition(self):
        '''
        Full joint positions (also non actuated)
        '''
        return self._sim.jointPosition(self.id(), self.jointIndexes())

    def jointVelocity(self):
        return self._sim.jointVelocity(self.id(), self.jointIndexes())

    def actuatedJointPosition(self):
        '''
        Actuated joint positions
        '''
        return self._sim.jointPosition(self.id(), self.actuatedJointIndexes())

    def actuatedJointVelocity(self):
        return self._sim.jointVelocity(self.id(), self.actuatedJointIndexes())

    def getRobotStates(self):
        return {'baseStates':self.baseStates(),
                'actuatedJointPositions':self.actuatedJointPosition(),
                'actuatedJointVelocities':self.actuatedJointVelocity()}

    #---------------------------------------------------------------------------
    # control
    #---------------------------------------------------------------------------

    def setActuatedJointPositions(self, q, v=None):
        '''
        Note: joint position needs to respect joint limits
        Note: q and v are assumend to be in pinocchio format and also contain floating base
        '''
        # set the control method
        if self._controlMode is not ControlMode.positionControl:
            self.enablePositionControl()
        
        # transform to pybullet
        self._q_cmd[self._q_map_pin_pb] = q
        self._v_cmd[self._v_map_pin_pb] = v

        # set the command
        if v is not None:
            self._sim.jointPositionControl(self.id(), self.actuatedJointIndexes(), self._q_cmd[self._pos_idx_offset:], self._v_cmd[self._vel_idx_offset:])
        else:
            self._sim.jointPositionControl(self.id(), self.actuatedJointIndexes(), self._q_cmd[self._pos_idx_offset:])

    def setActuatedJointTorques(self, tau):
        '''
        Note: tau is assumend to be in pinocchio format
        '''
        # set the control mode
        if self._controlMode is not ControlMode.torqueControl:
            self.enableTorqueControl()
        
        # transform the pybullet
        self._tau_cmd[self._tau_map_pin_pb] = tau

        # set the torque
        self._sim.jointTorqueControl(self.id(), self.actuatedJointIndexes(), self._tau_cmd)

    def computeInverseKinematic(self, linkName, position, orientation=None):
        return self._sim.computeInverseKinematic(self.id(), linkName, position, orientation)

    #---------------------------------------------------------------------------
    # com state
    #---------------------------------------------------------------------------

    def baseWorldAcceleration(self):
        return np.concatenate(self._base_linacc, self._base_angacc)

    def baseCoMPosition(self):
        return self.baseWorldPosition()

    def baseCoMVelocity(self):
        return self.baseWorldLinearVeloctiy()

    def baseCoMOrientation(self):
        return self.baseWorldOrientation()

    def baseCoMPose(self):
        return self.baseWorldPose()

    def addCoMMarker(self):
        self._com_marker = self._sim.addSphereMarker()

    #---------------------------------------------------------------------------
    # conversion of state (q, v, tau) to pinocchio
    #---------------------------------------------------------------------------

    def computeRootPose(self):
        '''
        compute the base pose that is used by pinocchio wrt to world (0 frame)
        Important: this can be different than pybullet base frame
        '''
        # get the com frame wrt world
        x_com_w, Q_com_w = self.baseCoMPose()

        # floating base q = [x_b_w, Q_b_w, q_0,...,q_n]
        # note: to get the base frame position we need do take the offset
        # between the base com and the base_link into account
        # Simon: really strange...
        base_state = pb.getDynamicsInfo(self._id, -1)
        x_b_com, Q_b_com = pb.invertTransform(base_state[3], base_state[4])
        x_b_w, Q_b_w = pb.multiplyTransforms(x_com_w, Q_com_w, x_b_com, Q_b_com)  
        return np.array(x_b_w), np.array(Q_b_w)

    def updateState(self):
        '''
        compute the state of the robot in a form that is suitable for pinoochio
        '''

        # jointstates
        q = self.actuatedJointPosition()
        v = self.actuatedJointVelocity()

        if self._use_floating_base:
            # compute the root pose at which pinoochio is anchored in the pybullet world
            x_b_w, Q_b_w = self.computeRootPose()

            # floating base q = [x_b_w, Q_b_w, q_0,...,q_n]
            q = np.concatenate((x_b_w, Q_b_w, q))

            # Note: the floating base velocity is expressed in the base frame, not in world!
            R_b_w = np.array(pb.getMatrixFromQuaternion(Q_b_w)).reshape((3, 3))

            # floating base qP = [xP, omega, qP_0, ..., qP_n]
            XP_com_w = self.baseWorldVelocity()
            v = np.concatenate((R_b_w.dot(XP_com_w[:3]), R_b_w.dot(XP_com_w[3:]), v))
        
        # save internally in pinocchio format
        self._q[self._q_map_pb_pin] = q
        self._v[self._v_map_pb_pin] = v

    def q(self):
        '''
        current position for pinocchio: q = [floating base, joint position]
        '''
        return self._q

    def v(self):
        '''
        current velocites for pinocchio:  v = [floating base vel, joint vel]
        '''
        return self._v
    
    def computeBulletPinoccioMaps(self, model):
        '''
        find the state mappings between pybullet and pinoochio
        such that pybullet jointstate matches pinoochio jointstate
        '''

        # actuated torques
        tau_map_pin_pb = np.zeros(self.numActuatedJoints(), dtype=int)

        # position [x,Q,q]
        q_map_pb_pin = np.hstack((np.arange(self._pos_idx_offset), np.zeros(self.numActuatedJoints(), dtype=int)))
        q_map_pin_pb = np.hstack((np.arange(self._pos_idx_offset), np.zeros(self.numActuatedJoints(), dtype=int)))
        # velocity [v,omega,qdot]
        v_map_pb_pin = np.hstack((np.arange(self._vel_idx_offset), np.zeros(self.numActuatedJoints(), dtype=int)))
        v_map_pin_pb = np.hstack((np.arange(self._vel_idx_offset), np.zeros(self.numActuatedJoints(), dtype=int)))

        for q_idx_pb, name in enumerate(self.actuatedJointNames()):
            joint_id_pin = model.getJointId(name)
            q_idx_pin = model.joints[joint_id_pin].idx_q 
            v_idx_pin = model.joints[joint_id_pin].idx_v

            # actuated (-7) torque map: pybullet to pinoccio
            # use: tau_pb[tau_map_pin_pb] = tau_pin
            tau_map_pin_pb[v_idx_pin - self._vel_idx_offset] = q_idx_pb

            # postion map pybullet to pinoccio
            # use: q_pin[q_map_pb_pin] = q_pb
            q_map_pb_pin[q_idx_pb + self._pos_idx_offset] = q_idx_pin
            # postion map pinoccio to pybullet
            # use: q_pb[q_map_pin_pb] = q_pin
            q_map_pin_pb[q_idx_pin] = q_idx_pb + self._pos_idx_offset

            # velocity map pybullet to pinoccio
            # use: v_pin[v_map_pb_pin] = v_pb
            v_map_pb_pin[q_idx_pb + self._vel_idx_offset] = v_idx_pin
            # velocity map pinoccio to pybullet
            # use: v_pb[v_map_pin_pb] = v_pin
            v_map_pin_pb[v_idx_pin] = q_idx_pb + self._vel_idx_offset

        return tau_map_pin_pb, q_map_pb_pin, q_map_pin_pb, v_map_pb_pin, v_map_pin_pb

    def transformWorldToRoot(self, x_w, Q_w):
        '''
        transforms a pybullet pose (x_w, Q_w) in the world frame into 
        a pinocchio pose SE3 in the kinematic root frame as SE3 transformation
        '''
        return self._T_root_w.actInv(pin.SE3(sim_utils.quaternionToMatrix(Q_w), x_w))

    def transformRootToWorld(self, T_root):
        '''
        transform a pinocchio pose SE3 in the kinematic root frame into a 
        pybullet pose (x_w, Q_w) in the world frame 
        '''
        T_w = self._T_root_w*T_root
        return T_w.translation, sim_utils.matrixToQuaternion(T_w.rotation)