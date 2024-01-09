"""body
Pybullet body base class
Author: Simon Armleder
"""

import pybullet as pb
import numpy as np
import os

class Color:
    red = [1,0,0]
    green = [0,1,0]
    blue = [0,0,1]

class Body:
    '''
    Base Class for a pybullet body in the simulation
    '''
    def __init__(self,
                simulator,
                filename=None,
                position=[0,0,0],
                orientation=[0,0,0,1],
                use_fixed_base=True,
                flags=None,
                 *args, 
                 **kwargs):
        
        # save inital pose for resets
        self._sim = simulator
        self._init_pos = position
        self._init_orint = orientation
        
        # remember if floating of fixed
        self._use_fixed_base = use_fixed_base
        self._use_floating_base = not use_fixed_base

        # try loading body, if non given _body remains uninitalized
        # and should be created before calling any function
        
        print("body")
        print(filename)
        print(position)
        print(orientation)
        
        self._id = None
        if filename:
            extension = os.path.splitext(filename)[1]
            if extension == '.urdf':
                self._id = self._sim.createBodyFromUrdf(filename, position, orientation, use_fixed_base, flags, *args, **kwargs)
            elif extension == '.obj':
                self._id = self._sim.createBodyFromMesh(filename, position, orientation, *args, **kwargs)

    #---------------------------------------------------------------------------
    # property functions
    #---------------------------------------------------------------------------

    def id(self):
        return self._id

    def numJoints(self):
        return pb.getNumJoints(self._id)

    def numActuatedJoints(self):
        return len(self._actuated_joint_indexes)

    def isFixedBase(self):
        return self._use_fixed_base

    def isFloatingBase(self):
        return self._use_floating_base

    def mass(self):
        totalMass = 0
        for linkId in range(-1, self.numJoints()):
            totalMass += self.linkMass(linkId)
        return totalMass

    def baseName(self):
        return pb.getBodyInfo(self._id)[0].decode()

    def name(self):
        return pb.getBodyInfo(self._id)[1].decode()

    def jointIndex(self, jointName):
        return self.jointNameIndexMap()[jointName]

    def jointIndexes(self):
        return list(range(self.numJoints()))

    def jointName(self, jointIndex):
        return pb.getJointInfo(self._id, jointIndex)[1].decode()

    def jointNames(self):
        try:
            return self._joint_names
        except AttributeError:
            self._joint_names = []
            for i in range(self.numJoints()):
                joint_name = self.jointName(i)
                self._joint_names.append(joint_name)
            return self._joint_names

    def linkIndex(self, linkName):
        return self.linkNameIndexMap()[linkName]

    def linkIndexes(self):
        return list(range(self.numJoints()))

    def linkName(self, linkIndex):
        return pb.getJointInfo(self._id, linkIndex)[12].decode()

    def linkNames(self):
        try:
            return self._link_names
        except AttributeError:
            self._link_names = []
            for i in range(self.numJoints()):
                link_name = self.linkName(i)
                self._link_names.append(link_name)
            return self._link_names

    def jointType(self, jointIndex):
        return pb.getJointInfo(self._id, jointIndex)[2]

    def jointDamping(self, jointIndex):
        return pb.getJointInfo(self._id, jointIndex)[6]

    def jointDampings(self):
        joint_dampings = []
        for i in range(self.numJoints()):
            joint_dampings.append(self.jointDamping(i))
        return joint_dampings

    def jointFriction(self, jointIndex):
        return pb.getJointInfo(self._id, jointIndex)[7]

    def jointFrictions(self):
        joint_frictions = []
        for i in range(self.numJoints()):
            joint_frictions.append(self.jointFriction(i))
        return joint_frictions

    def jointLowerLimit(self, jointIndex):
        return pb.getJointInfo(self._id, jointIndex)[8]

    def actuatedJointLowerLimits(self):
        try:
            return self._actuated_joint_lower_limits
        except AttributeError:
            self._actuated_joint_lower_limits = [self.jointLowerLimit(j) for j in self.actuatedJointIndexes()]
            return self._actuated_joint_lower_limits

    def jointUpperLimit(self, jointIndex):
        return pb.getJointInfo(self._id, jointIndex)[9]

    def actuatedJointUpperLimits(self):
        try:
            return self._actuated_joint_upper_limits
        except AttributeError:
            self._actuated_joint_upper_limits = [self.jointUpperLimit(j) for j in self.actuatedJointIndexes()]
            return self._actuated_joint_upper_limits

    def jointMaxForce(self, jointIndex):
        return pb.getJointInfo(self._id, jointIndex)[10]

    def jointMaxVelocity(self, jointIndex):
        return pb.getJointInfo(self._id, jointIndex)[11]

    def jointIndexNameMap(self):
        try:
            return self._joint_index_name_map
        except AttributeError:
            joint_indexes = self.jointIndexes()
            joint_names = self.jointNames()
            self._joint_index_name_map = dict(zip(joint_indexes, joint_names))
            return self._joint_index_name_map

    def linkIndexNameMap(self):
        try:
            return self._link_index_name_map
        except AttributeError:
            linkIndexes = self.linkIndexes()
            linkNames = self.linkNames()
            self._link_index_name_map = dict(zip(linkIndexes, linkNames))
            return self._link_index_name_map

    def linkNameIndexMap(self):
        try:
            return self._link_name_index_map
        except AttributeError:
            linkIndexes = self.linkIndexes()
            linkNames = self.linkNames()
            self._link_name_index_map = dict(zip(linkNames, linkIndexes))
            return self._link_name_index_map

    def jointNameIndexMap(self):
        try:
            return self._joint_name_index_map
        except AttributeError:
            joint_indexes = self.jointIndexes()
            joint_names = self.jointNames()
            self._joint_name_index_map = dict(zip(joint_names, joint_indexes))
        return self._joint_name_index_map

    def numActuatedJoints(self):
        n = 0
        for i in range(self.numJoints()):
            if self.jointType(i) is not pb.JOINT_FIXED:
                n += 1
        return n

    def actuatedJointNames(self):
        try:
            return self._actuated_joint_names
        except AttributeError:
            self._actuated_joint_names = []
            for i in range(self.numJoints()):
                if self.jointType(i) is not pb.JOINT_FIXED:
                    self._actuated_joint_names.append(self.jointName(i))
            return self._actuated_joint_names

    def actuatedJointIndexes(self):
        try:
            return self._actuated_joint_indexes
        except AttributeError:
            self._actuated_joint_indexes = []
            for joint_name in self.actuatedJointNames():
                self._actuated_joint_indexes.append(self.jointIndex(joint_name))
            return self._actuated_joint_indexes

    def linkMass(self, link_id):
        return pb.getDynamicsInfo(self._id, link_id)[0]

    def linkLocalInertialTransform(self, link_id):
        return pb.getDynamicsInfo(self._id, link_id)[3:5]

    def linkLocalInertialPosition(self, link_id):
        return np.array(pb.getDynamicsInfo(self._id, link_id)[3])

    def linkLocalInertiaQuaternion(self, link_id):
        return np.array(pb.getDynamicsInfo(self._id, link_id)[4])

    #---------------------------------------------------------------------------
    # get link states
    #---------------------------------------------------------------------------
    def linkWorldPosition(self, link_ids):
        return self._sim.linkWorldPositions(self._id, link_ids)

    def linkWorldOrientation(self, link_ids):
        return self._sim.linkWorldOrientation(self._id, link_ids)
    
    def linkWorldLinearVelocity(self, link_ids):
        return self._sim.linkWorldLinearVelocity(self._id, link_ids)

    def linkWorldAngularVelocity(self, link_ids):
        return self._sim.linkWorldAngularVelocity(self._id, link_ids)

    def linkWorldVelocity(self, link_ids):
        return self._sim.linkWorldVelocity(self._id, link_ids)

    #---------------------------------------------------------------------------
    # get base state
    #---------------------------------------------------------------------------

    def baseWorldPosition(self):
        return self._sim.baseWorldPosition(self._id)

    def baseWorldOrientation(self):
        return self._sim.baseWorldOrientation(self._id)

    def baseWorldLinearVeloctiy(self):
        return self._sim.baseWorldLinearVelocity(self._id)

    def baseWorldAngulaVeloctiy(self):
        return self._sim.baseWorldAngularVelocity(self._id)

    def baseWorldPose(self):
        return self._sim.baseWorldPose(self._id)

    def baseWorldVelocity(self):
        return self._sim.baseWorldVelocity(self._id)

    def baseWorldAcceleration(self):
        raise NotImplementedError

    def baseStates(self):
        return {'position':self.baseWorldPosition(),
                'quaternion':self.baseWorldOrientation(),
                'velocityLinear':self.baseWorldLinearVeloctiy(),
                'velocityAngular':self.baseWorldAngulaVeloctiy()}

    def getBaseLocalInertiaTransform(self):
        return self.linkLocalInertialTransform(-1)

    def getBaseLocalInertiaPosition(self):
        return self.linkLocalInertialPosition(-1)

    def getBaseLocalInertiaQuaternion(self):
        return self.linkLocalInertiaQuaternion(-1)

    #---------------------------------------------------------------------------
    # reset
    #---------------------------------------------------------------------------

    def resetBasePose(self, position, orientation=[0,0,0,1]):
        self._sim.resetBasePose(self._id, position, orientation)

    def resetBaseVelocity(self, linear=np.zeros(3), angular=np.zeros(3)):
        self._sim.resetBaseVelocity(self._id, linear, angular)

    def resetBaseStates(self, position=[0,0,0], orientation=[0,0,0,1], linear=[0,0,0], angular=[0,0,0]):
        self.resetBasePose(position, orientation)
        self.resetBaseVelocity(linear, angular)

    #---------------------------------------------------------------------------
    # debug functions
    #---------------------------------------------------------------------------

    def applyForce(self, f_w, p_w=[0, 0, 0], frame=pb.LINK_FRAME):
        f = np.linalg.norm(np.array(f_w))
        if(f > 0):
            pb.applyExternalForce(self._id, -1, f_w, p_w, frame)

    def addDebugLinkFrames(self, axisLength=0.1, axisWidth=1):
        for linkId in range(-1, self.numJoints()):
            self.addDebugLinkFrame(linkId, axisLength, axisWidth)

    def addDebugLinkInertiaFrames(self, axisLength=0.05, axisWidth=5):
        for linkId in range(-1, self.numJoints()):
            self.addDebugLinkInertiaFrame(linkId, axisLength=axisLength, axisWidth=axisWidth)

    def addDebugLinkFrame(self, linkId, axisLength=0.2, axisWidth=1):
        localTransCom = self.linkLocalInertialTransform(linkId)
        comPosLocal, comQuatLocal = pb.invertTransform(localTransCom[0], localTransCom[1])
        comRotLocal = np.array(pb.getMatrixFromQuaternion(comQuatLocal)).reshape((3,3))
        pb.addUserDebugLine(comPosLocal, comPosLocal+comRotLocal[:,0] * axisLength, Color.red, lineWidth=axisWidth, parentObjectUniqueId=self._id, parentLinkIndex=linkId)
        pb.addUserDebugLine(comPosLocal, comPosLocal+comRotLocal[:,1] * axisLength, Color.green, lineWidth=axisWidth, parentObjectUniqueId=self._id, parentLinkIndex=linkId)
        pb.addUserDebugLine(comPosLocal, comPosLocal+comRotLocal[:,2] * axisLength, Color.blue, lineWidth=axisWidth, parentObjectUniqueId=self._id, parentLinkIndex=linkId)

    def addDebugLinkInertiaFrame(self, linkId, axisLength=0.2, axisWidth=1):
        position, quaternion = [0, 0, 0], [0, 0, 0, 1]
        rotation = np.array(pb.getMatrixFromQuaternion(quaternion)).reshape((3, 3))
        pb.addUserDebugLine(position, position+rotation[:,0] * axisLength, Color.red, lineWidth=axisWidth, parentObjectUniqueId=self._id, parentLinkIndex=linkId)
        pb.addUserDebugLine(position, position+rotation[:,1] * axisLength, Color.green, lineWidth=axisWidth, parentObjectUniqueId=self._id, parentLinkIndex=linkId)
        pb.addUserDebugLine(position, position+rotation[:,2] * axisLength, Color.blue, lineWidth=axisWidth, parentObjectUniqueId=self._id, parentLinkIndex=linkId)

    def addDebugFrame(self, position, quaternion, axisLength=0.2, axisWidth=1):
        position, quaternion = [0, 0, 0], [0, 0, 0, 1]
        rotation = np.array(pb.getMatrixFromQuaternion(quaternion)).reshape((3, 3))
        pb.addUserDebugLine(position, position + rotation[:, 0] * axisLength, Color.red, lineWidth=axisWidth)
        pb.addUserDebugLine(position, position + rotation[:, 1] * axisLength, Color.green, lineWidth=axisWidth)
        pb.addUserDebugLine(position, position + rotation[:, 2] * axisLength, Color.blue, lineWidth=axisWidth)