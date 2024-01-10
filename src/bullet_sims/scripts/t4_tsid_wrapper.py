import os
import subprocess
import time

import numpy as np
import pinocchio as pin
import tsid

################################################################################
# utiltity functions
################################################################################

def create_sample(pos, vel=None, acc=None):
    if isinstance(pos, pin.SE3):
        sample = tsid.TrajectorySample(12, 6)
        sample.value(pos)
    elif isinstance(pos, np.ndarray):
        sample = tsid.TrajectorySample(pos.shape[0], pos.shape[0])
        sample.value(pos)
    else:
        raise NotImplemented
    if vel is not None:
        sample.derivative(vel)
    if acc is not None:
        sample.second_derivative(acc)
    return sample

def vectorToSE3(vec):
    return pin.SE3(vec[3:].reshape(3, 3), vec[:3])

def se3ToVector(s3e):
    return np.concatenate([s3e.translation, s3e.rotation.reshape(3*3)])

def update_sample(sample, pos, vel=None, acc=None):
    if isinstance(pos, pin.SE3):
        sample.value(pos)
    elif isinstance(pos, np.ndarray):
        sample.value(pos)
    else:
        raise NotImplemented
    if vel is not None:
        sample.derivative(vel)
    if acc is not None:
        sample.second_derivative(acc)
    return sample

################################################################################
# TSID Wrapper
################################################################################

class TSIDWrapper:
    ''' Standard TSID formulation for a humanoid robot standing on its rectangular feet.
        - Center of mass task (CoM)
        - Angular Momentum task (AM)
        - Postural task (All dofs)
        - 6d rigid contact constraint for both feet (6d rigid contact)
        - Motion task (position and orientation) for both feed 
        - Upper body torso task (keep the upper body horizontal)
    '''

    def __init__(self, conf):

        self.conf = conf

        ########################################################################
        # robot
        ########################################################################

        self.robot = tsid.RobotWrapper(
            conf.urdf, [conf.path], pin.JointModelFreeFlyer(), False)
        robot = self.robot
        self.model = robot.model()

        '''
        Check all links and print them
        '''
        link_names = [link.name for link in list(self.model.frames)]
        actuated_link_names = [link.name for link in list(
            self.model.frames) if link.type is pin.FrameType.JOINT][1:]
        actuated_link_ids = [self.model.getFrameId(
            name) for name in actuated_link_names]
        print('Actuated_link_names=', actuated_link_names)
        print('actuated_link_ids=', actuated_link_ids)

        assert self.model.existFrame(conf.rf_frame_name)
        assert self.model.existFrame(conf.lf_frame_name)

        # set inital state
        q = conf.q_home
        v = np.zeros(robot.nv)

        ########################################################################
        # formuatlion
        ########################################################################

        '''
        Formulation as hirachical Quadratic program. Decision variables are 
        accelerations (base+joints) and contact forces. Torques are then computed
        with the equation of motion.

        Set the inital configuration with q and v
        '''
        formulation = tsid.InverseDynamicsFormulationAccForce(
            "tsid", robot, False)
        formulation.computeProblemData(0.0, q, v)
        data = formulation.data()

        ########################################################################
        # project robot to the ground
        ########################################################################

        rf_id = self.model.getFrameId(conf.rf_frame_name)
        T_rf_ref = self.robot.framePosition(data, rf_id)
        q[2] -= T_rf_ref.translation[2]
        formulation.computeProblemData(0.0, q, v)
        data = formulation.data()

        ########################################################################
        # setup the end-effector contacts
        ########################################################################

        '''
        Surface Contacts are added on both feet.
        We need to prove the contact shape (footprint), friction coeffs, normal
        vecotr etc.
        TSID will then compute contact wrenches
        '''

        # setup the contact vertices of the feet polygon
        feet_contact_vertices = np.ones((3, 4)) * (-conf.lz)
        feet_contact_vertices[0, :] = [-conf.lfxn, -
                                       conf.lfxn, conf.lfxp, conf.lfxp]
        feet_contact_vertices[1, :] = [-conf.lfyn,
                                       conf.lfyp, -conf.lfyn, conf.lfyp]

        '''
        create the right foot contact task
        
        creates a 6d contact task (tsid.Contact6d). Constrains motion in any direction
        with: J*qPP = -JP*qP
        Set the soleshape of the contact and friction. By defining the four
        vertices of the contact.
        '''
        # create the 6d plane to plane contact, set gains Kp and Kd
        contactRF = tsid.Contact6d("contact_rfoot", robot, conf.rf_frame_name, feet_contact_vertices,
                                   conf.contactNormal, conf.f_mu, conf.f_fMin, conf.f_fMax)
        contactRF.setKp(conf.kp_contact * np.ones(6))
        contactRF.setKd(2.0 * np.sqrt(contactRF.Kp))

        # set the reference of the contact at the current position
        self.RF = robot.model().getFrameId(conf.rf_frame_name)
        H_rf_ref = robot.framePosition(data, self.RF)
        contactRF.setReference(H_rf_ref)
        if conf.w_feet_contact >= 0.0:
            formulation.addRigidContact(
                contactRF, conf.w_force_reg, conf.w_feet_contact, 1)
        else:
            formulation.addRigidContact(contactRF, conf.w_force_reg)

        '''
        create the left foot contact task
        '''
        contactLF = tsid.Contact6d("contact_lfoot", robot, conf.lf_frame_name, feet_contact_vertices,
                                   conf.contactNormal, conf.f_mu, conf.f_fMin, conf.f_fMax)
        contactLF.setKp(conf.kp_contact * np.ones(6))
        contactLF.setKd(2.0 * np.sqrt(conf.kp_contact) * np.ones(6))

        # set the reference of the contact at the current position
        self.LF = robot.model().getFrameId(conf.lf_frame_name)
        H_lf_ref = robot.framePosition(data, self.LF)
        contactLF.setReference(H_lf_ref)
        if conf.w_feet_contact >= 0.0:
            formulation.addRigidContact(
                contactLF, conf.w_force_reg, conf.w_feet_contact, 1)
        else:
            formulation.addRigidContact(contactLF, conf.w_force_reg)

        ########################################################################
        # com poition and momentum tasks
        ########################################################################

        '''
        Center of mass taks.
        Can control the center of mass (COM) position
        '''
        comTask = tsid.TaskComEquality("task-com", robot)
        comTask.setKp(conf.kp_com * np.ones(3))
        comTask.setKd(2.0 * np.sqrt(conf.kp_com) * np.ones(3))
        formulation.addMotionTask(comTask, conf.w_com, 1, 0.0)

        # com reference current pos
        com_ref = create_sample(robot.com(data))
        comTask.setReference(com_ref)

        '''
        Angular momentum task.
        Adds further stability by regulating angular momentum (AM) to zero 
        '''
        amTask = tsid.TaskAMEquality("task-am", robot)
        amTask.setKp(conf.kp_am * np.array([1., 1., 10.]))
        amTask.setKd(2.0 * np.sqrt(conf.kp_am * np.array([1., 1., 10.])))
        formulation.addMotionTask(amTask, conf.w_am, 1, 0.)

        # am reference is zero
        amRef = create_sample(np.zeros(3))
        amTask.setReference(amRef)

        ########################################################################
        # posture task
        ########################################################################

        '''
        Add a posture task.
        Will make the solution of the QP unique and acts as regualization
        '''
        postureTask = tsid.TaskJointPosture("task-posture", robot)
        postureTask.setKp(conf.kp_posture)
        postureTask.setKd(2.0 * np.sqrt(conf.kp_posture))
        postureTask.setMask(conf.masks_posture)
        formulation.addMotionTask(postureTask, conf.w_posture, 1, 0.0)

        # posture reference (set to current)
        # Note: need to remove floating base!
        q_ref = conf.q_posture
        posture_ref = create_sample(q_ref)
        postureTask.setReference(posture_ref)
        ########################################################################
        # posture task
        ########################################################################

        '''
        SE3 task for left position
        '''
        self.leftFootTask = tsid.TaskSE3Equality(
            "task-left-foot", self.robot, self.conf.lf_frame_name)
        self.leftFootTask.setKp(
            self.conf.kp_foot * np.array([1, 1, 1, 1, 1, 3]))
        self.leftFootTask.setKd(
            2.0 * np.sqrt(self.conf.kp_foot) * np.array([1, 1, 1, 1, 1, 3]))
        self.trajLF = tsid.TrajectorySE3Constant("traj-left-foot", H_lf_ref)
        formulation.addMotionTask(self.leftFootTask, self.conf.w_foot, 1, 0.0)

        # left foot reference
        T_lf_w = self.robot.framePosition(data, self.LF)
        self.lf_ref = create_sample(T_lf_w)
        self.leftFootTask.setReference(self.lf_ref)

        '''
        SE3 task for right foot position
        '''
        self.rightFootTask = tsid.TaskSE3Equality(
            "task-right-foot", self.robot, self.conf.rf_frame_name)
        self.rightFootTask.setKp(
            self.conf.kp_foot * np.array([1, 1, 1, 1, 1, 3]))
        self.rightFootTask.setKd(
            2.0 * np.sqrt(self.conf.kp_foot) * np.array([1, 1, 1, 1, 1, 3]))
        self.trajRF = tsid.TrajectorySE3Constant("traj-right-foot", H_rf_ref)
        formulation.addMotionTask(self.rightFootTask, self.conf.w_foot, 1, 0.0)

        # right foot reference
        T_rf_w = self.robot.framePosition(data, self.RF)
        self.rf_ref = create_sample(T_rf_w)
        self.rightFootTask.setReference(self.rf_ref)
        

        '''
        SE3 task for left hand pose
        '''
        # TODO: ADD a tsid.TaskSE3Equality for the left hand
        # self.LH = # the frame id   
        self.LH = robot.model().getFrameId(conf.lh_frame_name)
        # self.leftHandTask = # the motion task
        self.leftHandTask = tsid.TaskSE3Equality(
            "task-left-hand" , self.robot, self.conf.lh_frame_name)
        self.leftHandTask.setKp(
            100*np.array([1, 1, 1, 0, 0, 0]))
        self.leftHandTask.setKd(
            2.0 * np.sqrt(100)* np.array([1, 1, 1, 0, 0, 0]))
        # self.lh_ref = # the TrajectorySample (see: create_sample(...))
        T_lh_w = self.robot.framePosition(data, self.LH)
        self.lh_ref = create_sample(T_lh_w)


        '''
        SE3 task for right hand pose
        '''
        # TODO: ADD a tsid.TaskSE3Equality for the right hand
        # self.RH = # the frame id
        self.RH = robot.model().getFrameId(conf.rh_frame_name)

        # self.rightHandTask = # the motion task
        self.rightHandTask = tsid.TaskSE3Equality(
            "task-right-hand" , self.robot, self.conf.rh_frame_name)
        self.rightHandTask.setKp(
            100*np.array([1, 1, 1, 0, 0, 0]))
        self.rightHandTask.setKd(
            2.0 * np.sqrt(100)* np.array([1, 1, 1, 0, 0, 0]))
        # self.rh_ref = # the TrajectorySample (see: create_sample(...))
        T_rh_w = self.robot.framePosition(data, self.RH)
        self.rh_ref = create_sample(T_rh_w)

        ########################################################################
        # torso task
        ########################################################################

        '''
        Keep the torso orientation upright                          
        '''
        self.torsoTask = tsid.TaskSE3Equality(
            "task-torso", self.robot, self.conf.torso_frame_name)
        self.torsoTask.setKp(self.conf.kp_torso * np.array([0, 0, 0, 1, 1, 1]))
        self.torsoTask.setKd(2.0 * np.sqrt(self.conf.kp_torso)
                             * np.array([0, 0, 0, 1, 1, 1]))
        formulation.addMotionTask(self.torsoTask, self.conf.w_torso, 1, 0.0)

        # torso reference is current
        torso_id = self.model.getFrameId(conf.torso_frame_name)
        H_torso_ref = robot.framePosition(data, torso_id)
        self.torso_ref = create_sample(H_torso_ref)
        self.torsoTask.setReference(self.torso_ref)
        
        assert self.model.existFrame(conf.base_frame_name)
        self.base_id = self.model.getFrameId(conf.base_frame_name)

        ########################################################################
        # bounds on torque and velocity
        ########################################################################

        '''
        add bounds on the actuator torques
        '''
        self.tau_max = conf.tau_max_scaling * \
            robot.model().effortLimit[-robot.na:]
        self.tau_min = -self.tau_max
        actuationBoundsTask = tsid.TaskActuationBounds(
            "task-actuation-bounds", robot)
        actuationBoundsTask.setBounds(self.tau_min, self.tau_max)
        if conf.w_torque_bounds > 0.0:
            formulation.addActuationTask(
                actuationBoundsTask, conf.w_torque_bounds, 0, 0.0)

        '''
        add bound on the actuator velocites
        '''

        jointVelBoundsTask = tsid.TaskJointBounds(
            "task-joint-vel-bounds", robot, conf.dt)
        self.v_max = conf.v_max_scaling * \
            robot.model().velocityLimit[-robot.na:]
        self.v_min = -self.v_max
        jointVelBoundsTask.setVelocityBounds(self.v_min, self.v_max)
        if conf.w_joint_bounds > 0.0:
            formulation.addMotionTask(
                jointVelBoundsTask, conf.w_joint_bounds, 0, 0.0)

        ########################################################################
        # solver
        ########################################################################

        print('Adding Solver')
        print('n_var=\t', formulation.nVar)
        print('n_eq=\t', formulation.nEq)
        print('n_in=\t', formulation.nIn)

        self.solver = tsid.SolverHQuadProgFast("qp solver")
        self.solver.resize(formulation.nVar, formulation.nEq, formulation.nIn)

        ########################################################################
        # store everything
        ########################################################################

        self.com_ref = com_ref
        self.posture_ref = posture_ref

        self.comTask = comTask
        self.amTask = amTask
        self.postureTask = postureTask
        self.actuationBoundsTask = actuationBoundsTask
        self.jointVelBoundsTask = jointVelBoundsTask
        self.formulation = formulation

        self.q = q
        self.v = v

        self.contactRF = contactRF
        self.contactLF = contactLF

        self.contact_LF_active = True
        self.contact_RF_active = True
        self.motion_RH_active = False
        self.motion_LH_active = False
        
        self.sol = None
        self.tau = np.zeros(self.robot.na)
        self.acc = np.zeros(self.robot.nv)

    ############################################################################
    # functions
    ############################################################################

    def update(self, q, v, t, do_sove=True):
        hqp_data = self.formulation.computeProblemData(t, q, v)
        
        if do_sove:
            sol = self.solver.solve(hqp_data)
            if(sol.status!=0):
                print("QP problem could not be solved! Error code:", sol.status)
            self.sol = sol
            self.tau_sol = self.formulation.getActuatorForces(sol)
            self.dv_sol = self.formulation.getAccelerations(sol)
        
        return self.tau_sol, self.dv_sol

    def integrate_dv(self, q, v, dv, dt):
        v_mean = v + 0.5 * dt * dv
        v += dt * dv
        q = pin.integrate(self.model, q, dt * v_mean)
        return q, v

    ############################################################################
    # getter
    ############################################################################

    def get_joint_idx_v(self, name):
        joint_id_pin = self.model.getJointId(name)
        return self.model.joints[joint_id_pin].idx_v

    def get_joint_idx_q(self, name):
        joint_id_pin = self.model.getJointId(name)
        return self.model.joints[joint_id_pin].idx_q

    ############################################################################
    # com
    ############################################################################

    def setComRefState(self, pos, vel=None, acc=None):
        update_sample(self.com_ref, pos, vel, acc)
        self.comTask.setReference(self.com_ref)

    def comState(self):
        data = self.formulation.data()
        pos = self.robot.com(data)
        vel = self.robot.com_vel(data)
        acc = self.robot.com_acc(data)
        return create_sample(pos, vel, acc)

    def comReference(self):
        return self.com_ref

    ############################################################################
    # torso task
    ############################################################################

    def setTorsoRefState(self, pos, vel=None, acc=None):
        update_sample(self.torso_ref, pos, vel, acc)
        self.torsoTask.setReference(self.torso_ref)

    def torsoState(self, dv=None):
        data = self.formulation.data()
        T_frame_w = self.robot.framePosition(data, self.torso_id)
        v_frame_w = self.robot.frameVelocity(data, self.torso_id)
        if dv is not None:
            a_frame_w = self.torsoTask.getAcceleration(dv)
            return T_frame_w, v_frame_w, a_frame_w
        return T_frame_w, v_frame_w

    def torsoReference(self):
        return self.torso_ref
    
    def baseState(self, dv=None):
        data = self.formulation.data()
        T_frame_w = self.robot.framePosition(data, self.base_id)
        v_frame_w = self.robot.frameVelocity(data, self.base_id)
        if dv is not None:
            a_frame_w = self.torso_task.getAcceleration(dv)
            return T_frame_w, v_frame_w, a_frame_w
        return T_frame_w, v_frame_w

    ############################################################################
    # posture task
    ############################################################################

    def setPostureRef(self, q):
        update_sample(self.posture_ref, q)
        self.posture_task.setReference(self.posture_ref)

    ############################################################################
    # set endeffector motion references
    ############################################################################

    def set_RF_pose_ref(self, pose, vel=None, acc=None):
        update_sample(self.rf_ref, pose, vel, acc)
        self.rightFootTask.setReference(self.rf_ref)

    def set_RF_pos_ref(self, pos, vel=None, acc=None):
        X = self.rf_ref.pos(); X[:3] = pos
        V = self.rf_ref.vel(); V[:3] = vel
        A = self.rf_ref.acc(); A[:3] = acc
        update_sample(self.rf_ref, X, V, A)
        self.rightFootTask.setReference(self.rf_ref)

    def set_LF_pose_ref(self, pose, vel=None, acc=None):
        update_sample(self.lf_ref, pose, vel, acc)
        self.leftFootTask.setReference(self.lf_ref)

    def set_LF_pos_ref(self, pos, vel=None, acc=None):
        X = self.lf_ref.pos(); X[:3] = pos
        V = self.lf_ref.vel(); V[:3] = vel
        A = self.lf_ref.acc(); A[:3] = acc
        update_sample(self.lf_ref, X, V, A)
        self.leftFootTask.setReference(self.lf_ref)

    def set_RH_pose_ref(self, pose, vel=None, acc=None):
        update_sample(self.rh_ref, pose, vel, acc)
        self.rightHandTask.setReference(self.rh_ref)

    def set_RH_pos_ref(self, pos, vel, acc):
        X = self.rh_ref.pos(); X[:3] = pos
        V = self.rh_ref.vel(); V[:3] = vel
        A = self.rh_ref.acc(); A[:3] = acc
        update_sample(self.rh_ref, X, V, A)
        self.rightHandTask.setReference(self.rh_ref)

    def set_LH_pose_ref(self, pose, vel=np.zeros(6), acc=np.zeros(6)):
        update_sample(self.lh_ref, pose, vel, acc)
        self.leftHandTask.setReference(self.lh_ref)

    def set_LH_pos_ref(self, pos, vel, acc):
        X = self.lh_ref.pos(); X[:3] = pos
        V = self.lh_ref.vel(); V[:3] = vel
        A = self.lh_ref.acc(); A[:3] = acc
        update_sample(self.lh_ref, X, V, A)
        self.leftHandTask.setReference(self.lh_ref)

    ############################################################################
    # get endeffector states
    ############################################################################

    def get_placement_LF(self):
        return self.robot.framePosition(self.formulation.data(), self.LF)

    def get_placement_RF(self):
        return self.robot.framePosition(self.formulation.data(), self.RF)

    def get_pose_LH(self):
        return self.robot.framePosition(self.formulation.data(), self.LH)

    def get_pose_RH(self):
        return self.robot.framePosition(self.formulation.data(), self.RH)

    def get_LF_3d_pos_vel_acc(self, dv=None):
        data = self.formulation.data()
        H = self.robot.framePosition(data, self.LF)
        v = self.robot.frameVelocity(data, self.LF)
        if dv is not None:
            a = self.leftFootTask.getAcceleration(dv)
            return H.translation, v.linear, a[:3]
        return H.translation, v.linear

    def get_RF_3d_pos_vel_acc(self, dv=None):
        data = self.formulation.data()
        H = self.robot.framePosition(data, self.RF)
        v = self.robot.frameVelocity(data, self.RF)
        if dv is not None:
            a = self.rightFootTask.getAcceleration(dv)
            return H.translation, v.linear, a[:3]
        return H.translation, v.linear

    def get_LH_3d_pos_vel_acc(self, dv=None):
        data = self.formulation.data()
        T_h_w = self.robot.framePosition(data, self.LH)
        v_h_b = self.robot.frameVelocity(data, self.LH)
        if dv is not None:
            a_h_b = self.leftHandTask.getAcceleration(dv)
            return T_h_w.translation, v_h_b.linear, a_h_b[:3]
        return T_h_w.translation, v_h_b.linear

    def get_RH_3d_pos_vel_acc(self, dv=None):
        data = self.formulation.data()
        T_h_w = self.robot.framePosition(data, self.RH)
        v_h_b = self.robot.frameVelocity(data, self.RH)
        if dv is not None:
            a_h_b = self.rightHandTask.getAcceleration(dv)
            return T_h_w.translation, v_h_b.linear, a_h_b[:3]
        return T_h_w.translation, v_h_b.linear

    def get_LH_3d_pose_vel_acc(self, dv=None):
        data = self.formulation.data()
        T_h_w = self.robot.framePosition(data, self.LH)
        v_h_b = self.robot.frameVelocity(data, self.LH)
        if dv is not None:
            a_h_b = self.leftHandTask.getAcceleration(dv)
            return T_h_w, v_h_b, a_h_b
        return T_h_w, v_h_b

    def get_RH_3d_pose_vel_acc(self, dv=None):
        data = self.formulation.data()
        T_h_w = self.robot.framePosition(data, self.RH)
        v_h_b = self.robot.frameVelocity(data, self.RH)
        if dv is not None:
            a_h_b = self.rightHandTask.getAcceleration(dv)
            return T_h_w, v_h_b, a_h_b
        return T_h_w, v_h_b

    ############################################################################
    # remove and add contact
    ############################################################################
    
    def remove_contact_RF(self, transition_time=0.0):
        if self.contact_RF_active:
            # set ref to current pose
            H_rf_ref = self.robot.framePosition(
                self.formulation.data(), self.RF)
            update_sample(self.rf_ref, H_rf_ref)
            self.rightFootTask.setReference(self.rf_ref)
            # remove contact
            self.formulation.removeRigidContact(
                self.contactRF.name, transition_time)
            self.contact_RF_active = False

    def remove_contact_LF(self, transition_time=0.0):
        if self.contact_LF_active:
            # set ref to current pose
            H_lf_ref = self.robot.framePosition(
                self.formulation.data(), self.LF)
            update_sample(self.lf_ref, H_lf_ref)
            self.leftFootTask.setReference(self.lf_ref)
            # remove contact
            self.formulation.removeRigidContact(
                self.contactLF.name, transition_time)
            self.contact_LF_active = False

    def add_contact_RF(self, transition_time=0.0):
        # add task to stack, set reference to current
        if not self.contact_RF_active:
            H_rf_ref = self.robot.framePosition(
                self.formulation.data(), self.RF)
            self.contactRF.setReference(H_rf_ref)
            if self.conf.w_feet_contact >= 0.0:
                self.formulation.addRigidContact(
                    self.contactRF, self.conf.w_force_reg, self.conf.w_feet_contact, 1)
            else:
                self.formulation.addRigidContact(
                    self.contactRF, self.conf.w_force_reg)
            self.contact_RF_active = True

    def add_contact_LF(self, transition_time=0.0):
        # add task to stack, set reference to current
        if not self.contact_LF_active:
            H_lf_ref = self.robot.framePosition(
                self.formulation.data(), self.LF)
            self.contactLF.setReference(H_lf_ref)
            if self.conf.w_feet_contact >= 0.0:
                self.formulation.addRigidContact(
                    self.contactLF, self.conf.w_force_reg, self.conf.w_feet_contact, 1)
            else:
                self.formulation.addRigidContact(
                    self.contactLF, self.conf.w_force_reg)
            self.contact_LF_active = True

    ############################################################################
    # remove and add motions
    ############################################################################

    def remove_motion_LH(self, transition_time=0.0):
        # remove task from stack
        if self.motion_LH_active:
            self.formulation.removeTask("task-left-hand", transition_time)
            self.motion_LH_active = False

    def remove_motion_RH(self, transition_time=0.0):
        # remove task from stack
        if self.motion_RH_active:
            self.formulation.removeTask("task-right-hand", transition_time)
            self.motion_RH_active = False

    def add_motion_LH(self, transition_time=0.0):
        if not self.motion_LH_active:
            H_lh_ref = self.robot.framePosition(self.formulation.data(), self.LH)
            update_sample(self.lh_ref, H_lh_ref)
            self.leftHandTask.setReference(self.lh_ref)
            self.formulation.addMotionTask(
                self.leftHandTask, self.conf.w_hand, 1, transition_time)

            self.motion_LH_active = True

    def add_motion_RH(self, transition_time=0.0):
        if not self.motion_RH_active:
            H_rh_ref = self.robot.framePosition(self.formulation.data(), self.RH)
            update_sample(self.rh_ref, H_rh_ref)
            self.rightHandTask.setReference(self.rh_ref)
            self.formulation.addMotionTask(
                self.rightHandTask, self.conf.w_hand, 1, transition_time)

            self.motion_RH_active = True

    ############################################################################
    # get forces
    ############################################################################

    def get_RF_wrench(self, sol):
        if self.formulation.checkContact(self.contactRF.name, sol):
            return self.formulation.getContactForce(self.contactRF.name, sol)
        else:
            return np.zeros(6)

    def get_RF_normal_force(self, sol):
        return self.contactRF.getNormalForce(self.get_RF_wrench(sol))

    def get_LF_wrench(self, sol):
        if self.formulation.checkContact(self.contactLF.name, sol):
            return self.formulation.getContactForce(self.contactLF.name, sol)
        else:
            return np.zeros(6)

    def get_LF_normal_force(self, sol):
        return self.contactLF.getNormalForce(self.get_LF_wrench(sol))

    def get_RH_normal_force(self, sol):
        if self.formulation.checkContact(self.contactRH.name, sol):
            f = self.formulation.getContactForce(self.contactRH.name, sol)
            return self.contactRH.getNormalForce(f)
        else:
            return 0.0

    def get_LH_normal_force(self, sol):
        if self.formulation.checkContact(self.contactLH.name, sol):
            f = self.formulation.getContactForce(self.contactLH.name, sol)
            return self.contactLH.getNormalForce(f)
        else:
            return 0.0