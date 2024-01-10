"""pybullet_wrapper
Pybullet simulation wrapper
Author: Simon Armleder
"""

import pybullet as pb
from pybullet_utils.bullet_client import BulletClient
import pybullet_data
import numpy as np
import time


class Floor:
    def __init__(self, basePosition=[0, 0, 0], baseRPY=[0, 0, 0]):
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.createCollisionShape(pb.GEOM_PLANE)
        self._id = pb.createMultiBody(
            0, 0, basePosition=basePosition, baseOrientation=pb.getQuaternionFromEuler(baseRPY))

    def id(self):
        return self._id

    def changeFriction(self, lateralFriction=1.0, spinningFriction=1.0):
        print("Before: Floor friction lateralFriction:",
              pb.getDynamicsInfo(self._id, -1)[1])
        print("Before: Floor friction spinningFriction:",
              pb.getDynamicsInfo(self._id, -1)[7])
        pb.changeDynamics(bodyUniqueId=self._id, linkIndex=-1,
                          lateralFriction=lateralFriction, spinningFriction=spinningFriction)
        print("After: Floor friction lateralFriction:",
              pb.getDynamicsInfo(self._id, -1)[1])
        print("After: Floor friction spinningFriction:",
              pb.getDynamicsInfo(self._id, -1)[7])

    def changeImpedance(self, stiffness, damping):
        print("Before: Floor stiffness:", pb.getDynamicsInfo(self._id, -1)[9])
        print("Before: Floor damping:", pb.getDynamicsInfo(self._id, -1)[8])
        pb.changeDynamics(bodyUniqueId=self._id, linkIndex=-1,
                          contactStiffness=stiffness, contactDamping=damping)
        print("After: Floor stiffness:", pb.getDynamicsInfo(self._id, -1)[9])
        print("After: Floor damping:", pb.getDynamicsInfo(self._id, -1)[8])


class PybulletWrapper:
    '''
    Simple wrapper for easy pybullet access
    Note: This is incomplete...
    '''

    def __init__(
            self,
            render=True,
            sim_rate=1000,
            gravity=np.array([0, 0, -9.81]),
            num_solver_iterations=None,
            num_sub_steps=None):

        if render:
            self._connection_mode = pb.GUI
        else:
            self._connection_mode = pb.DIRECT
        self.connect(render)

        self._sim_rate = sim_rate
        self._sim_time_step = 1.0 / self._sim_rate
        self._gravity = gravity

        self._running = True
        self._sim_count = 0
        self._sim_time = 0.0
        self._start_time = time.time()

        pb.setGravity(gravity[0], gravity[1], gravity[2])
        pb.setTimeStep(self._sim_time_step)
        if num_solver_iterations is not None and num_sub_steps is not None:
            pb.setPhysicsEngineParameter(
                fixedTimeStep=self._sim_time_step,
                numSolverIterations=num_solver_iterations,
                numSubSteps=num_sub_steps)

        self._floor = Floor()

        self.configVisuals(COV_ENABLE_GUI=False,
                           COV_ENABLE_RGB_BUFFER_PREVIEW=False,
                           COV_ENABLE_DEPTH_BUFFER_PREVIEW=False,
                           COV_ENABLE_SEGMENTATION_MARK_PREVIEW=False)

    def setRealTime(self):
        '''
        set into real time mode:
        node: call this right before simulation loop
        node: in realtime mode step and debug functions are no longer valid
        '''
        pb.setTimeStep(1./240.0)
        pb.setRealTimeSimulation(1)

    def isRunning(self):
        '''
        Return ture if running
        '''
        return self._running

    def isRendering(self):
        '''
        Is running in GUI mode
        '''
        return self._connection_mode == pb.GUI

    def configVisuals(self, COV_ENABLE_GUI=False, COV_ENABLE_RGB_BUFFER_PREVIEW=False, COV_ENABLE_DEPTH_BUFFER_PREVIEW=False, COV_ENABLE_SEGMENTATION_MARK_PREVIEW=False):
        '''
        config visualizer elements
        '''
        pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, COV_ENABLE_GUI)
        pb.configureDebugVisualizer(
            pb.COV_ENABLE_RGB_BUFFER_PREVIEW, COV_ENABLE_RGB_BUFFER_PREVIEW)
        pb.configureDebugVisualizer(
            pb.COV_ENABLE_DEPTH_BUFFER_PREVIEW, COV_ENABLE_DEPTH_BUFFER_PREVIEW)
        pb.configureDebugVisualizer(
            pb.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, COV_ENABLE_SEGMENTATION_MARK_PREVIEW)

    def stepTime(self):
        return self._sim_time_step

    def stepFreq(self):
        return self._sim_rate

    def simTime(self):
        return self._sim_time

    def simCount(self):
        return self._sim_count

    def step(self):
        '''
        move one step forward
        '''
        pb.stepSimulation()
        if self._connection_mode == pb.GUI:
            factor = pb.readUserDebugParameter(self.time_warping_)
            time.sleep(self._sim_time_step*factor)
        self._sim_count += 1
        self._sim_time = self._sim_count * self._sim_time_step

    def debug(self):
        '''
        add key debugging
        '''
        # pause simulation by space key event
        keys = pb.getKeyboardEvents()
        space_key = ord(' ')
        quit_key = ord('q')
        if space_key in keys and keys[space_key] & pb.KEY_WAS_TRIGGERED:
            pause_time = time.time()
            real_time = pause_time - self._start_time
            print("*" * 10 + "Simulation Paused! Press 'Space' to resume!" + "*" * 10)
            print("Sim Time=        {:1}".format(self._sim_time))
            print("Real Time=       {:1}".format(real_time))
            # TODO this measure isn't correct, because of rendering...
            print("Real Time Factor={:1}".format(self._sim_time/real_time))
            while True:
                keys = pb.getKeyboardEvents()
                if space_key in keys and keys[space_key] & pb.KEY_WAS_TRIGGERED:
                    self._start_time += (time.time() - pause_time)
                    break
        elif quit_key in keys and keys[quit_key] & pb.KEY_WAS_TRIGGERED:
            print("Quiting")
            self._running = False

    def disconnect(self):
        '''
        disconnects from physics server (or shuts server down in in pb.GUI mode)
        '''
        pb.disconnect()

    def resetSimulation(self):
        '''
        clears all objects, contrains, world is empty afterwards, then
        add the foor bag in
        '''
        pb.resetSimulation()

    def connect(self, render=True):
        '''
        mode switing not worlking, need to rebuild world manually...
        '''
        if render:
            if self._connection_mode == pb.GUI:
                # render in gui mode
                if not pb.isConnected():
                    pb.connect(self._connection_mode)
            else:
                # switch connection mode
                # state_id = self.save()
                # change mode
                self.disconnect()
                self._connection_mode = pb.GUI
                pb.connect(self._connection_mode)
                # self.load(state_id) # error
        else:
            if self._connection_mode == pb.DIRECT:
                if not pb.isConnected():
                    pb.connect(self._connection_mode)
            else:
                # switch connection mode
                # state_id = self.save()
                # change mode
                self._connection_mode = pb.DIRECT
                self.disconnect()
                pb.connect(self._connection_mode)
                # self.load(state_id) # error

        # setup the gui
        self.setup()
        self._floor = Floor()

    def setup(self):
        # create a user interface
        self.time_warping_ = pb.addUserDebugParameter(
            "time_warping", 0, 100, 1)

    def gravity(self):
        return self._gravity

    ############################################################################
    # common stuff
    ############################################################################

    def numJoints(self, body_id):
        return pb.getNumJoints(body_id)

    def numActuatedJoints(self, body_id):
        n = 0
        for i in range(self.numJoints(body_id)):
            if self.getJointType(i) is not pb.JOINT_FIXED:
                n += 1
        return n

    ############################################################################
    # cartesian stuff
    ############################################################################

    def resetBasePose(self, body_id, pos, orient=[0, 0, 0, 1]):
        if isinstance(pos, np.ndarray):
            pos = pos.ravel().tolist()
        if isinstance(orient, np.ndarray):
            orient = orient.ravel().tolist()
        pb.resetBasePositionAndOrientation(body_id, pos, orient)

    def resetBaseVelocity(self, body_id, linear, angular):
        if isinstance(linear, np.ndarray):
            linear = linear.ravel().tolist()
        if isinstance(angular, np.ndarray):
            angular = angular.ravel().tolist()
        pb.resetBaseVelocity(body_id, linear, angular)

    def computeTotalMass(self, body_id, link_ids=None):
        if link_ids == None:
            link_ids = list(range(self.numJoints(body_id)))
            link_ids.append(-1)
        return np.sum(self.linkMass(body_id, link_ids))

    def computeComPosition(self, body_id, link_ids=None):
        if link_ids == None:
            link_ids = list(range(self.numJoints(body_id)))
            link_ids.append(-1)

        mass = self.linkMass(body_id, link_ids)
        pos = self.linkWorldPositions(body_id, link_ids)
        return np.sum(pos.transpose()*mass, axis=1)/np.sum(mass)

    def computeComVelocity(self, body_id, link_ids=None):
        if link_ids == None:
            link_ids = list(range(self.numJoints(body_id)))
            link_ids.append(-1)

        mass = self.linkMass(body_id, link_ids)
        vel = self.linkWorldLinearVelocity(body_id, link_ids)
        return np.sum(vel.transpose()*mass, axis=1)/np.sum(mass)

    def baseWorldPosition(self, body_id):
        return np.array(pb.getBasePositionAndOrientation(body_id)[0])

    def baseWorldOrientation(self, body_id):
        return np.array(pb.getBasePositionAndOrientation(body_id)[1])

    def baseWorldPose(self, body_id):
        pos, orient = pb.getBasePositionAndOrientation(body_id)
        return np.array(pos), np.array(orient)

    def baseWorldLinearVelocity(self, body_id):
        return np.array(pb.getBaseVelocity(body_id)[0])

    def baseWorldAngularVelocity(self, body_id):
        return np.array(pb.getBaseVelocity(body_id)[1])

    def baseWorldVelocity(self, body_id):
        lin_vel, ang_vel = pb.getBaseVelocity(body_id)
        return np.concatenate([np.array(lin_vel), np.array(ang_vel)])

    def baseMass(self, body_id):
        return self.linkMass(body_id, -1)

    def linkWorldPositions(self, body_id, link_ids):
        # single link
        if isinstance(link_ids, int):
            if link_ids == -1:
                return self.baseWorldPosition(body_id)
            return np.array(pb.getLinkState(body_id, link_ids)[0])

        # multiple links
        positions = []
        for link_id in link_ids:
            if link_id == -1:
                positions.append(self.baseWorldPosition(body_id))
            else:
                positions.append(
                    np.array(pb.getLinkState(body_id, link_id)[0]))
        return np.asarray(positions)

    def linkWorldOrientation(self, body_id, link_ids):
        # single link
        if isinstance(link_ids, int):
            if link_ids == -1:
                return self.baseWorldOrientation(body_id)
            return np.array(pb.getLinkState(body_id, link_ids)[1])

        # multiple links
        orientations = []
        for link_id in link_ids:
            if link_id == -1:
                orientations.append(self.baseWorldOrientation(body_id))
            else:
                orientations.append(
                    np.array(pb.getLinkState(body_id, link_id)[1]))
        return np.asarray(orientations)

    def linkWorldLinearVelocity(self, body_id, link_ids):
        # single link
        if isinstance(link_ids, int):
            if link_ids == -1:
                return self.baseWorldLinearVelocity(body_id)
            return np.array(pb.getLinkState(body_id, link_ids, computeLinkVelocity=True)[6])

        # multiple links
        velocities = []
        for link_id in link_ids:
            if link_id == -1:
                velocities.append(self.baseWorldLinearVelocity(body_id))
            else:
                velocities.append(pb.getLinkState(
                    body_id, link_id, computeLinkVelocity=True)[6])
        return np.asarray(velocities)

    def linkWorldAngularVelocity(self, body_id, link_ids):
        # single link
        if isinstance(link_ids, int):
            if link_ids == -1:
                return self.baseWorldAngularVelocity(body_id)
            return np.array(pb.getLinkState(body_id, link_ids, computeLinkVelocity=True)[7])

        # multiple links
        velocities = []
        for link_id in link_ids:
            if link_id == -1:
                velocities.append(self.baseWorldAngularVelocity(body_id))
            else:
                velocities.append(pb.getLinkState(
                    body_id, link_id, computeLinkVelocity=True)[7])
        return np.asarray(velocities)

    def linkWorldVelocity(self, body_id, link_ids):
        # single link
        if isinstance(link_ids, int):
            if link_ids == -1:
                return self.baseWorldVelocity(body_id)
            lin_vel, ang_vel = np.array(pb.getLinkState(
                body_id, link_ids, computeLinkVelocity=True)[6:8])
            return np.concatenate((lin_vel, ang_vel))

        # multiple links
        velocities = []
        for link_id in link_ids:
            if link_id == -1:
                return self.baseWorldVelocity(body_id)
            else:
                lin_vel, ang_vel = np.array(pb.getLinkState(
                    body_id, link_id, computeLinkVelocity=True)[6:8])
            velocities.append(np.concatenate(lin_vel, ang_vel))
        return np.asarray(velocities)

    def linkMass(self, body_id, link_ids):
        if isinstance(link_ids, int):
            return pb.getDynamicsInfo(body_id, link_ids)[0]
        return np.array([pb.getDynamicsInfo(body_id, link_id)[0] for link_id in link_ids])

    ############################################################################
    # joint stuff
    ############################################################################

    def jointNames(self, body_id, joint_ids):
        return [pb.getJointInfo(body_id, joint_id)[1] for joint_id in joint_ids]

    def jointPosition(self, body_id, joint_ids=None):
        if joint_ids == None:
            joint_ids = range(self.numJoints(body_id))
        if isinstance(joint_ids, int):
            return pb.getJointStates(body_id, joint_ids)[0]
        return np.array([state[0] for state in pb.getJointStates(body_id, joint_ids)])

    def jointVelocity(self, body_id, joint_ids=None):
        if joint_ids == None:
            joint_ids = range(self.numJoints(body_id))
        if isinstance(joint_ids, int):
            return pb.getJointStates(body_id, joint_ids)[1]
        return np.array([state[1] for state in pb.getJointStates(body_id, joint_ids)])

    def jointDamping(self, body_id, joint_ids=None, joint_damping=0):
        if joint_ids == None:
            joint_ids = range(self.numJoints(body_id))
        if isinstance(joint_ids, int):
            pb.changeDynamics(body_id, linkIndex=joint_ids,
                              jointDamping=joint_damping)
        else:
            for i in range(joint_ids):
                pb.changeDynamics(body_id, linkIndex=i,
                                  jointDamping=joint_damping)

    def jointAxis(self, body_id, joint_ids=None):
        if joint_ids == None:
            joint_ids = range(self.numJoints(body_id))
        return np.asarray([pb.getJointInfo(body_id, joint_id)[-4] for joint_id in joint_ids])

    def resetJointState(self, body_id, joint_ids=None, positions=None, velocites=None):
        if isinstance(joint_ids, int):
            pb.resetJointState(body_id, joint_ids, positions, velocites)
        else:
            if joint_ids is None:
                joint_ids = range(self.numActuatedJoints(body_id))
            if positions is None:
                positions = [0]*len(joint_ids)
            if velocites is None:
                velocites = [0]*len(joint_ids)
            for j in range(len(joint_ids)):
                pb.resetJointState(
                    body_id, joint_ids[j], positions[j], velocites[j])

    ############################################################################
    # kinematic stuff
    ############################################################################

    def enableJointPositionControlMode(self, body_id, joint_ids=None):
        '''
        Set joint to joint position mode
        '''
        if isinstance(joint_ids, int):
            pb.setJointMotorControl2(
                body_id, joint_ids, pb.POSITION_CONTROL, targetPositions=0.0)
        else:
            if joint_ids is None:
                joint_ids = range(self.numJoints(body_id))
            pb.setJointMotorControlArray(body_id,
                                         joint_ids,
                                         pb.POSITION_CONTROL,
                                         targetPositions=[0.0]*len(joint_ids))

    def jointPositionControl(self, body_id, joint_ids, q_target, qP_target=None, max_torque=10000):
        if isinstance(joint_ids, int):
            pb.setJointMotorControl2(
                body_id,
                joint_ids,
                pb.POSITION_CONTROL,
                targetPosition=q_target,
                targetVelocity=qP_target,
                force=max_torque)
        else:
            if isinstance(q_target, np.ndarray):
                q_target = q_target.ravel().tolist()
            if isinstance(qP_target, np.ndarray):
                qP_target = qP_target.ravel().tolist()
            if isinstance(max_torque, np.ndarray):
                max_torque = max_torque.ravel().tolist()
            elif isinstance(max_torque, int):
                max_torque = [max_torque]*len(q_target)

            if qP_target:
                pb.setJointMotorControlArray(
                    body_id,
                    joint_ids,
                    pb.POSITION_CONTROL,
                    targetPositions=q_target,
                    targetVelocities=qP_target,
                    forces=max_torque)
            else:
                pb.setJointMotorControlArray(
                    body_id,
                    joint_ids,
                    pb.POSITION_CONTROL,
                    targetPositions=q_target,
                    forces=max_torque)

    def computeInverseKinematic(self, body_id, link_id, pos, orn=None):
        if isinstance(pos, np.ndarray):
            pos = pos.ravel().tolist()
        if isinstance(orn, np.ndarray):
            orn = orn.ravel().tolist()

        if orn is None:
            q = pb.calculateInverseKinematics(
                body_id,
                link_id,
                pos,
                maxNumIterations=20)
        else:
            q = pb.calculateInverseKinematics(
                body_id,
                link_id,
                pos,
                orn,
                maxNumIterations=20)

        return np.asarray(q)

    ############################################################################
    # dynamics stuff
    ############################################################################

    def enableJointTorqueControlMode(self, body_id, joint_ids=None, friction=0):
        if isinstance(joint_ids, int):
            pb.changeDynamics(body_id, joint_ids,
                              linearDamping=0, angularDamping=0)
            pb.setJointMotorControl2(
                body_id, joint_ids, pb.VELOCITY_CONTROL, force=friction)
        else:
            if joint_ids is None:
                joint_ids = range(self.numJoints(body_id))
            for j in joint_ids:
                pb.changeDynamics(
                    body_id, j, linearDamping=0, angularDamping=0)
                pb.setJointMotorControl2(
                    body_id, j, pb.VELOCITY_CONTROL, force=friction)

    def jointTorqueControl(self, body_id, joint_ids, torques):
        if isinstance(joint_ids, int):
            pb.setJointMotorControl(
                body_id, joint_ids, pb.TORQUE_CONTROL, force=torques)
        else:
            if isinstance(torques, np.ndarray):
                torques = torques.ravel().tolist()
            pb.setJointMotorControlArray(
                body_id, joint_ids, pb.TORQUE_CONTROL, forces=torques)

    def computeInverseDynamics(self, body_id, q, qP, qPP_d):
        if isinstance(q, np.ndarray):
            q = q.ravel().tolist()
        if isinstance(qP, np.ndarray):
            qP = qP.ravel().tolist()
        if isinstance(qPP_d, np.ndarray):
            qPP_d = qPP_d.ravel().tolist()
        return np.asarray(pb.calculateInverseDynamics(body_id, q, qP, qPP_d))

    def computeMassMatrix(self, body_id, q):
        if isinstance(q, np.ndarray):
            q = q.ravel().tolist()
        return np.asarray(pb.calculateMassMatrix(body_id, q))

    def computeForwardDynamics(self, body_id, q, qP, torques):
        M_inv = np.linalg.inv(self.computeMassMatrix(body_id, q))
        N = self.computeInverseDynamics(body_id, q, qP, np.zeros(len(q)))
        qPP = M_inv.dot(torques - N)
        return qPP

    ############################################################################

    def contactPointsForce(self, body_id_a, body_id_b, link_id_a, link_id_b=None):
        '''
        compute the contact positions and contact forces for all contact points 
        between body_a link_a and body_b link_b
        '''
        if link_id_b:
            contact_list = pb.getContactPoints(
                body_id_a, body_id_b, link_id_a, link_id_b)
        else:
            contact_list = pb.getContactPoints(body_id_a, body_id_b, link_id_a)

        # contact locations
        x_c_w = [contact[5] for contact in contact_list]

        # normals force
        e_n = np.asarray([contact[7] for contact in contact_list])
        f_n = np.asarray([contact[9] for contact in contact_list])

        # lateral force 1
        f_l1 = np.asarray([contact[10] for contact in contact_list])
        e_l1 = np.asarray([contact[11] for contact in contact_list])

        # lateral force 2
        f_l2 = np.asarray([contact[12] for contact in contact_list])
        e_l2 = np.asarray([contact[13] for contact in contact_list])

        # force vector
        f_c_w = e_n*f_n[:, None] + e_l1*f_l1[:, None] + e_l2*f_l2[:, None]

        return np.asarray(x_c_w), f_c_w

    def transformToBodyLink(self, points, body_id, link_id):
        '''
        Transform world points to body link
        '''
        x_l_w = self.linkWorldPositions(body_id, link_id)
        Q_l_w = self.linkWorldOrientation(body_id, link_id)
        x_w_l, Q_w_l = pb.invertTransform(x_l_w, Q_l_w)

        if len(points.shape) == 1:
            x_l, _ = pb.multiplyTransforms(points, [0, 0, 0, 1], x_w_l, Q_w_l)
        else:
            x_l = []
            for x_w in points:
                x, _, = pb.multiplyTransforms(x_w, [0, 0, 0, 1], x_w_l, Q_w_l)
                x_l.append(x)
        return np.asarray(x_l)

    ############################################################################
    # bodies
    ############################################################################

    def floor(self):
        '''
        get the floor plane.
        '''
        return self._floor

    def createPrimitiveBody(self, shape_type, position, orientation=[0., 0., 0., 1.], mass=1., inertial_position=[0., 0., 0.], color=None,
                            with_collision=True, *args, **kwargs):
        '''
        Create a primitves. Such as Spheres, Cubes, cylinderes, ...
        See: pb.GEOM_...
        '''
        if isinstance(position, np.ndarray):
            position = position.ravel().tolist()
        if isinstance(orientation, np.ndarray):
            orientation = orientation.ravel().tolist()
        if isinstance(inertial_position, np.ndarray):
            inertial_position = inertial_position.ravel().tolist()

        collision_shape = None
        if with_collision:
            collision_shape = pb.createCollisionShape(
                shapeType=shape_type, **kwargs)

        if color is not None:
            kwargs['rgbaColor'] = color
        visual_shape = pb.createVisualShape(shapeType=shape_type, **kwargs)

        if with_collision:
            body = pb.createMultiBody(baseMass=mass,
                                      baseCollisionShapeIndex=collision_shape,
                                      baseVisualShapeIndex=visual_shape,
                                      basePosition=position,
                                      baseOrientation=orientation,
                                      baseInertialFramePosition=inertial_position)
        else:
            body = pb.createMultiBody(baseMass=mass,
                                      baseVisualShapeIndex=visual_shape,
                                      basePosition=position,
                                      baseOrientation=orientation,
                                      baseInertialFramePosition=inertial_position)
        return body

    def createBodyFromUrdf(self, filename, position, orientation=[0., 0., 0., 1.], use_fixed_base=True, flags=None, *args, **kwargs):
        '''
        Create a new body from urdf file
        '''
        if isinstance(position, np.ndarray):
            position = position.ravel().tolist()
        if isinstance(orientation, np.ndarray):
            orientation = orientation.ravel().tolist()
        if flags is None:
            return pb.loadURDF(
                fileName=filename,
                basePosition=position,
                baseOrientation=orientation,
                #useMaximalCoordinates=6,
                useFixedBase=use_fixed_base,
                **kwargs)
        else:
            print("wraper")
            print(filename)
            print(position)
            print(orientation)
            return pb.loadURDF(
                fileName=filename,
                basePosition=position,
                baseOrientation=orientation,
                #useMaximalCoordinates=6,
                useFixedBase=use_fixed_base,
                flags=flags,
                **kwargs)

    def createBodyFromMesh(self, filename, position, orientation=[0., 0., 0., 1.], mass=1., scale=(1., 1., 1.),
                           inertial_position=[0., 0., 0.], color=None, with_collision=True, *args, **kwargs):
        '''
        Create a new body from mesh (.obj) in filename
        '''
        if isinstance(position, np.ndarray):
            position = position.ravel().tolist()
        if isinstance(orientation, np.ndarray):
            orientation = orientation.ravel().tolist()
        if isinstance(inertial_position, np.ndarray):
            inertial_position = inertial_position.ravel().tolist()

        collision_shape = None
        if with_collision:
            collision_shape = pb.createCollisionShape(
                pb.GEOM_MESH, fileName=filename, meshScale=scale, **kwargs)

        if color is not None:
            kwargs['rgbaColor'] = color
        visual_shape = pb.createVisualShape(
            pb.GEOM_MESH, fileName=filename, meshScale=scale, **kwargs)

        if with_collision:
            body = pb.createMultiBody(baseMass=mass,
                                      baseCollisionShapeIndex=collision_shape,
                                      baseVisualShapeIndex=visual_shape,
                                      basePosition=position,
                                      baseOrientation=orientation,
                                      baseInertialFramePosition=inertial_position)
        else:
            body = pb.createMultiBody(baseMass=mass,
                                      baseVisualShapeIndex=visual_shape,
                                      basePosition=position,
                                      baseOrientation=orientation,
                                      baseInertialFramePosition=inertial_position)
        return body

    ############################################################################
    # debugging stuff
    ############################################################################

    def addUserDebugText(self, parent_id, link_id, text, x, color=[0, 0, 0]):
        '''
        Add some text
        '''
        pb.addUserDebugText(text, x, parentObjectUniqueId=parent_id,
                            parentLinkIndex=link_id, textColorRGB=color)

    def addLinkDebugFrame(self, parent_id, link_id, x=[0, 0, 0], Q=[0, 0, 0, 1], length=0.1, width=6, lifeTime=0):
        '''
        Add a coordinate frame at link_id with the relative transformation x,Q to the link.
        '''
        if isinstance(x, np.ndarray):
            x = x.ravel().tolist()
        if isinstance(Q, np.ndarray):
            Q = Q.ravel().tolist()

        x_e, _ = pb.multiplyTransforms(x, Q, [length, 0, 0], Q)
        y_e, _ = pb.multiplyTransforms(x, Q, [0, length, 0], Q)
        z_e, _ = pb.multiplyTransforms(x, Q, [0, 0, length], Q)

        ids = []
        ids.append(pb.addUserDebugLine(x, x_e, [1, 0, 0],
                                       parentObjectUniqueId=parent_id, parentLinkIndex=link_id, lineWidth=width, lifeTime=lifeTime))
        ids.append(pb.addUserDebugLine(x, y_e, [0, 1, 0],
                                       parentObjectUniqueId=parent_id, parentLinkIndex=link_id, lineWidth=width, lifeTime=lifeTime))
        ids.append(pb.addUserDebugLine(x, z_e, [0, 0, 1],
                                       parentObjectUniqueId=parent_id, parentLinkIndex=link_id, lineWidth=width, lifeTime=lifeTime))
        return ids

    def addGlobalDebugLine(self, p1=[0, 0, 0], p2=[0, 0, 1], line_id=-1, color=[0, 0, 0], lineWidth=6, lifeTime=0):
        '''
        Add a line in the global frame, starting at x in direction d  with length length.
        '''
        if line_id < 0:
            id = pb.addUserDebugLine(
                p1, p2, color, lineWidth=lineWidth, lifeTime=lifeTime)
        else:
            id = pb.addUserDebugLine(
                p1, p2, color, lineWidth=lineWidth, replaceItemUniqueId=line_id, lifeTime=lifeTime)
        return id

    def addGlobalDebugRectancle(self, x, Q=[0, 0, 0, 1], length=0.2, width=0.1, line_ids=None, color=[0, 0, 0], lineWidth=1, lifeTime=0):
        '''
        Add a rectancle in the global frame, with center x and orientation Q.
        '''
        if isinstance(x, np.ndarray):
            x = x.ravel().tolist()
        if isinstance(Q, np.ndarray):
            Q = Q.ravel().tolist()

        point1, _ = pb.multiplyTransforms(
            x, Q, [+length / 2, +width / 2, 0.01], [0, 0, 0, 1])
        point2, _ = pb.multiplyTransforms(
            x, Q, [-length / 2, +width / 2, 0.01], [0, 0, 0, 1])
        point3, _ = pb.multiplyTransforms(
            x, Q, [-length / 2, -width / 2, 0.01], [0, 0, 0, 1])
        point4, _ = pb.multiplyTransforms(
            x, Q, [+length / 2, -width / 2, 0.01], [0, 0, 0, 1])

        if line_ids is None:
            line_ids = [-1]*4
        line_ids[0] = self.addGlobalDebugLine(
            point1, point2, color=color, line_id=line_ids[0], lineWidth=lineWidth, lifeTime=lifeTime)
        line_ids[1] = self.addGlobalDebugLine(
            point2, point3, color=color, line_id=line_ids[1], lineWidth=lineWidth, lifeTime=lifeTime)
        line_ids[2] = self.addGlobalDebugLine(
            point3, point4, color=color, line_id=line_ids[2], lineWidth=lineWidth, lifeTime=lifeTime)
        line_ids[3] = self.addGlobalDebugLine(
            point4, point1, color=color, line_id=line_ids[3], lineWidth=lineWidth, lifeTime=lifeTime)
        return line_ids

    def addGlobalDebugTrajectory(self, X, Y, Z, color=[0, 0, 0], line_ids=[], lineWidth=6, lifeTime=0):
        '''
        Create a trajectory in global frame.
        Lines are updated if lines_ids is provied, otherwise new line elements are created.
        '''
        i = 0

        # update existing lines
        for id in line_ids:
            point1 = [X[i], Y[i], Z[i]]
            point2 = [X[i+1], Y[i+1], Z[i+1]]
            line_ids[i] = self.addGlobalDebugLine(
                point1, point2, line_id=id, color=color, lineWidth=lineWidth, lifeTime=lifeTime)
            i += 1

        # add new lines to the list
        while i < len(X) - 1:
            point1 = [X[i], Y[i], Z[i]]
            point2 = [X[i+1], Y[i+1], Z[i+1]]
            line_ids.append(self.addGlobalDebugLine(
                point1, point2, color=color, lineWidth=lineWidth, lifeTime=lifeTime))
            i += 1

        return line_ids

    def removeDebugItem(self, id):
        '''
        Remove item
        '''
        pb.removeUserDebugItem(id)

    def addSphereMarker(self, position=[0, 0, 0], orientation=[0, 0, 0, 1], radius=0.02, color=[1, 0, 0, 1]):
        '''
        create a new shape and return its id.
        Note: we set collision shape to zero to avoid collsion checks
        '''
        shape = pb.createVisualShape(shapeType=pb.GEOM_SPHERE,
                                     rgbaColor=color,
                                     radius=radius)
        if isinstance(position, np.ndarray):
            position = position.ravel().tolist()
        if isinstance(orientation, np.ndarray):
            orientation = orientation.ravel().tolist()
        return pb.createMultiBody(baseMass=0,
                                  basePosition=position,
                                  baseOrientation=orientation,
                                  baseCollisionShapeIndex=-1,
                                  baseVisualShapeIndex=shape,
                                  useMaximalCoordinates=True)

    ############################################################################
    # utility suff
    ############################################################################

    def save(self, filename=None):
        if isinstance(filename, str):
            pb.saveBullet(bulletFileName=filename)
            return filename
        else:
            return pb.saveState()

    def load(self, state):
        if isinstance(state, str):
            pb.restoreState(fileName=state)
        else:
            pb.restoreState(stateId=state)


if __name__ == '__main__':
    sim = PybulletWrapper()

    while True:
        sim.step()
        sim.debug()
