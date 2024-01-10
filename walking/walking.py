"""
talos walking simulation
"""

import rospy
import tf
import numpy as np 
import pinocchio as pin

# from pybullet_wrapper.simulator.robot import Robot

from simple_walking.robots.pybullet_wrapper import PybulletWrapper
from simple_walking.wbc.t4_tsid_wrapper import TSIDWrapper
import simple_walking.robots.talos_conf as conf
from simple_walking.robots.talos import Talos
from simple_walking.modules.footstep_planner import Side, FootStepPlanner, FootStep
# from simple_walking.modules.foot_trajectory import SwingFootTrajectory
from simple_walking.modules.foot_trajectory import SwingFootTrajectory
from simple_walking.modules.lip_mpc import LIPMPC, generate_zmp_reference, LIPInterpolator
################################################################################
# main
################################################################################  
    
def main(): 
    
    simulator = PybulletWrapper()
    robot =  Talos(simulator)
    stack = TSIDWrapper(conf)

    no_steps = 20
    planner = FootStepPlanner(conf) 

    lf_initial = stack.get_placement_LF().translation
    rf_initial = stack.get_placement_RF().translation
    print('lf_initial', lf_initial)
    print('rf_initial', rf_initial)

    T0 = pin.SE3(np.eye(3), np.array([lf_initial[0], lf_initial[1], 0])) 
    plan = planner.planLine(T0, Side.RIGHT, no_steps) # [0, -0.096, 0]

    plan = np.vstack([plan, plan[-1], plan[-1]])
    print("plan:\n", plan.shape, "\n",plan)

    foot_steps = []
    for i in range(0, len(plan)): 
        foot_steps.append(np.array([plan[i][0],0,plan[i][1],0])) # foot_steps = [x,vx,y,vy]

    print("foot_steps:")
    for i in range(0, len(foot_steps)):
        print(foot_steps[i])


    T_swing_w = foot_steps[1]
    #>>>>TODO: set intial support foot pose to right foot
    T_support_w = foot_steps[0] 
    # generate reference
    #>>>>TODO: Generate the mpc reference
    NO_MPC_SAMPLES_PER_STEP = 8 #tbc
    ZMP_ref = generate_zmp_reference(foot_steps,NO_MPC_SAMPLES_PER_STEP)
    print("ZMP_ref", ZMP_ref)

    plot_steps = FootStep(plan, 0, T_support_w)
    plot_steps.plot(simulator) # plot the foot_steps in pybullet

    mpc = LIPMPC(conf) # buildSolveOCP()

    x0 = np.array(foot_steps[0]) #tbc
    interpolator = LIPInterpolator(x0, conf) # integrate()->x_next comsate

    c, c_dot, c_ddot = interpolator.comState() # set the right foot as support foot
    


    pre_dur = 3.0   # Time to wait befor walking should start
    N_pre = 3000  #>>>>TODO: number of sim steps before walking starts 
    # 8*4 = 32
    N_mpc = no_steps*8   #>>>>TODO: total number of mpc steps during 
    # 32*100 = 3200
    N_sim =  N_mpc*100 #>>>>TODO: total number of sim steps during walking

    TIME_VEC = np.nan*np.empty(N_sim)
    # STATE_VEC = np.nan*np.empty([N_sim, 4])
    # ZMP_REF_VEC = np.nan*np.ones([N_sim, 2])
    # ZMP_VEC = np.nan*np.ones([N_sim, 2])
    com_pin_p = np.nan*np.ones([N_sim, 3])
    com_pin_v = np.nan*np.ones([N_sim, 3])
    com_pin_a = np.nan*np.ones([N_sim, 3])

    com_pb_p = np.nan*np.ones([N_sim, 3])
    com_pb_v = np.nan*np.ones([N_sim, 3])

    com_ref_p = np.nan*np.ones([N_sim, 3])
    com_ref_v = np.nan*np.ones([N_sim, 3])
    com_ref_a = np.nan*np.ones([N_sim, 3])

    
    k = 0                                               # current MPC index                          
    plan_idx = 0                                       # current index of the step within foot step plan
    t_step_elapsed = 0.0                                # elapsed time within current step (use to evaluate spline)
    t_publish = 0.0                                     # last publish time (last time we published something)
    
    u_k = np.array([0, 0.096])                         # current control input
    NO_MPC_SAMPLES = no_steps*NO_MPC_SAMPLES_PER_STEP   # total number of mpc samples
    x_k = x0                                            # current state

    # sp_side = Side.RIGHT
    sw_side = Side.RIGHT
    flag = 0
    for i in range(-4000, N_sim): # -3000, 6400

        t = simulator.simTime()
        dt = simulator.stepTime()

        tau_sol,dv_sol = stack.update(robot.robot._q, robot.robot._v,t)
        robot.robot.setActuatedJointTorques(tau_sol)

        if i>-2000 and i < 0 and flag == 0:
            flag = 1 
            stack.setComRefState(c)

        if i >= 0 and i % 100 == 0: # 0.001s
            x_k = interpolator.x
            ZMP_ref_k = ZMP_ref[k:k+16]
            idx_terminal_k = NO_MPC_SAMPLES - k # 64-k
            u_k = mpc.buildSolveOCP(x_k, ZMP_ref_k, idx_terminal_k)
            k += 1

        # if i >= 0:
        #     x_k = interpolator.integrate(u_k)

        #     c, c_dot, c_ddot = interpolator.comState() # set the right foot as support foot
        #     stack.setComRefState(c,c_dot,c_ddot)
            

        if i >= 0 and i%800 ==0: #>>>>TODO: when to update spline 0.8s
            t_step_elapsed = 0.0
            
            # print('plan_idx', plan_idx)
            plan_idx+=1

            if sw_side == Side.RIGHT:
                swing_pos = stack.get_placement_RF().translation
                stack.remove_contact_RF()
                stack.add_contact_LF()
                # swing_pos[2] = 0
                next_swing_step = plan[plan_idx]
                # next_swing_step[2] = 0
                if plan_idx == 1:
                    traj = SwingFootTrajectory(swing_pos, next_swing_step, 0.8)
                else:
                    traj = SwingFootTrajectory(swing_pos, next_swing_step+np.array([0.08,0.042,0]), 0.8) #  left-t1

            elif sw_side == Side.LEFT:
                swing_pos = stack.get_placement_LF().translation
                stack.remove_contact_LF()
                stack.add_contact_RF()
                # swing_pos[2] = 0
                next_swing_step = plan[plan_idx]
                # next_swing_step[2] = 0
                traj = SwingFootTrajectory(swing_pos, next_swing_step+np.array([0.08,-0.04,0]), 0.8)

            # print('plan',plan[plan_idx-1],'sw', swing_pos, 'next', next_swing_step)
            # stack.setComRefState(getCurrCom)
            if sw_side == Side.RIGHT:
                sw_side = Side.LEFT
            elif sw_side == Side.LEFT:
                sw_side = Side.RIGHT
            

        if i>=0:
            # print('t_step_elapsed', t_step_elapsed)
            # pose, vel, acc= traj.evaluate(t_step_elapsed)
            pose, vel, acc= traj.evaluate(t_step_elapsed/0.8)
            pose = pin.SE3(np.eye(3), pose)


            # since it swaped, so the pose should be the opposite
            if sw_side == Side.RIGHT:
                stack.set_LF_pose_ref(pose,vel,acc)
            elif sw_side == Side.LEFT:
                stack.set_RF_pose_ref(pose,vel,acc)


        if i >= 0:
            x_k = interpolator.integrate(u_k)

            c, c_dot, c_ddot = interpolator.comState() # set the right foot as support foot
            # c = pin.SE3(np.eye(3), c)
            stack.setComRefState(c,c_dot,c_ddot)
            t_step_elapsed += 0.001


        simulator.debug()
        simulator.step()
        robot.robot.update()

        # publish to ros
        # if i - t_publish > 1./30.:
        #     t_publish = i

        #     T_b_w = stack.baseState()
        #     tf.TransformBroadcaster().sendTransform(T_b_w[0].translation,
        #                 tf.transformations.quaternion_from_matrix(T_b_w[0].homogeneous),
        #                 rospy.Time.now(),
        #                 "base_link",
        #                 "world")
            

        # store for visualizations

        # pin stack.comState()
        # pb  robot.robot.baseCoMPosition() & robot.robot.baseCoMVelocity()
        # ref stack.comReference()
        if i >= 0:
            pass
            TIME_VEC[k] = i

            # pin
            com_pin_p[k] = stack.comState().value()
            com_pin_v[k] = stack.comState().derivative()
            com_pin_a[k] = stack.comState().second_derivative()

            # pb
            com_pb_p[k] = robot.robot.baseCoMPosition()
            com_pb_v[k] = robot.robot.baseCoMVelocity()

            # ref   stack.comReference()
            com_ref_p[k] = c
            com_ref_v[k] = c_dot
            com_ref_a[k] = c_ddot


            # STATE_VEC[k] = x_k
            # ZMP_VEC[k] = u_k
            # ZMP_REF_VEC[k] = mpc.ZMP_ref_k[0]
    
    import matplotlib.pyplot as plt
    plt.style.use('seaborn-dark')
    
    # >>>>TODO: plot everything
    fig, ax = plt.subplots(3,1)
    fig.suptitle('CoM Position X_Y_Z')

    ax[0].plot(TIME_VEC, com_pin_p[:,0], 'r-',label='CoM PIN X')
    ax[0].plot(TIME_VEC, com_pb_p[:,0], 'b-',label='CoM PB  X')
    ax[0].plot(TIME_VEC, com_ref_p[:,0], 'g-.', label='CoM Ref X')
    ax[0].legend(loc='upper left', shadow=True)

    ax[1].plot(TIME_VEC, com_pin_p[:,1], 'r-',label='CoM PIN Y')
    ax[1].plot(TIME_VEC, com_pb_p[:,1], 'b-',label='CoM PB  Y')
    ax[1].plot(TIME_VEC, com_ref_p[:,1], 'g-.', label='CoM Ref Y')
    ax[1].legend(loc='upper left', shadow=True)

    ax[2].plot(TIME_VEC, com_pin_p[:,2], 'r-',label='CoM PIN Z')
    ax[2].plot(TIME_VEC, com_pb_p[:,2], 'b-',label='CoM PB  Z')
    ax[2].plot(TIME_VEC, com_ref_p[:,2], 'g-.', label='CoM Ref Z')
    ax[2].legend(loc='upper left', shadow=True)
    
    fig1, ax1 = plt.subplots(3,1)
    fig1.suptitle('CoM Velocity X_Y_Z')
    ax1[0].plot(TIME_VEC, com_pin_v[:,0], 'r-',label='CoM PIN V_X')
    ax1[0].plot(TIME_VEC, com_pb_v[:,0], 'b-',label='CoM PB  V_X')
    ax1[0].plot(TIME_VEC, com_ref_v[:,0], 'g-.', label='CoM Ref V_X')
    ax1[0].legend(loc='upper left', shadow=True)

    ax1[1].plot(TIME_VEC, com_pin_v[:,1], 'r-',label='CoM PIN V_Y')
    ax1[1].plot(TIME_VEC, com_pb_v[:,1], 'b-',label='CoM PB  V_Y')
    ax1[1].plot(TIME_VEC, com_ref_v[:,1], 'g-.', label='CoM Ref V_Y')
    ax1[1].legend(loc='upper left', shadow=True)

    ax1[2].plot(TIME_VEC, com_pin_v[:,2], 'r-',label='CoM PIN V_Z')
    ax1[2].plot(TIME_VEC, com_pb_v[:,2], 'b-',label='CoM PB  V_Z')
    ax1[2].plot(TIME_VEC, com_ref_v[:,2], 'g-.', label='CoM Ref V_Z')
    ax1[2].legend(loc='upper left', shadow=True)

    fig2, ax2 = plt.subplots(2,1)
    fig2.suptitle('CoM Acceleration X_Y_Z')
    ax2[0].plot(TIME_VEC, com_pin_a[:,0], 'r-',label='CoM PIN Acc_X')
    ax2[0].plot(TIME_VEC, com_ref_a[:,0], 'g-.', label='CoM Ref Acc_X')
    ax2[0].legend(loc='upper left', shadow=True)

    ax2[1].plot(TIME_VEC, com_pin_a[:,1], 'r-',label='CoM PIN Acc_Y')
    ax2[1].plot(TIME_VEC, com_ref_a[:,1], 'g-.', label='CoM Ref Acc_Y')
    ax2[1].legend(loc='upper left', shadow=True)
    plt.show()

    # footprint = np.array([0.2, 0.13])
    # ZMP_LB_VEC = ZMP_REF_VEC - footprint[None,:]
    # ZMP_UB_VEC = ZMP_REF_VEC + footprint[None,:]
    # fig, ax = plt.subplots(3,1)
    # ax[0].plot(TIME_VEC, STATE_VEC[:,0], 'b-',label='CoM_Pos_X')
    # ax[0].plot(TIME_VEC, ZMP_VEC[:,0], 'y-',label='ZMP_X')
    # ax[0].plot(TIME_VEC, ZMP_REF_VEC[:,0], 'g-.', label='ZMP_Ref_X')
    # ax[0].plot(TIME_VEC, ZMP_LB_VEC[:,0]-footprint[1]/2,  'r-.',label='ZMP_LB_X')
    # ax[0].plot(TIME_VEC, ZMP_UB_VEC[:,0]+footprint[1]/2,  'm-.',label='zmp_UB_X')
    # ax[0].legend(loc='upper left', shadow=True)

    # ax[1].plot(TIME_VEC, STATE_VEC[:,1], 'b-',label='CoM_Vel_X')
    # ax[1].legend(loc='upper left', shadow=True)

    # acc_x =  conf.g/conf.h*(STATE_VEC[:,0]-ZMP_VEC[:,0])
    # ax[2].plot(TIME_VEC, acc_x, 'b-',label='CoM_Acc_X')
    # ax[2].legend(loc='upper left', shadow=True)



    # fig1, ax1 = plt.subplots(3,1)
    # ax1[0].plot(TIME_VEC, STATE_VEC[:,2], 'b-',label='CoM_Pos_Y')
    # ax1[0].plot(TIME_VEC, ZMP_VEC[:,1], 'y-',label='ZMP_Y')
    # ax1[0].plot(TIME_VEC, ZMP_REF_VEC[:,1], 'g-.', label='ZMP_Ref_Y')
    # ax1[0].plot(TIME_VEC, ZMP_LB_VEC[:,1]-footprint[1]/2,  'r-.',label='ZMP_LB_Y')
    # ax1[0].plot(TIME_VEC, ZMP_UB_VEC[:,1]+footprint[1]/2,  'm-.',label='zmp_UB_Y')
    # ax1[0].legend(loc='upper left', shadow=True)

    # ax1[1].plot(TIME_VEC, STATE_VEC[:,3], 'b-',label='CoM_Vel_Y')
    # ax1[1].legend(loc='upper left', shadow=True)

    # acc_y =  conf.g/conf.h*(STATE_VEC[:,2]-ZMP_VEC[:,1])
    # ax1[2].plot(TIME_VEC, acc_y, 'b-',label='CoM_Acc_Y')
    # ax1[2].legend(loc='upper left', shadow=True)

    # plt.show()
    # fig, ax = plt.subplots(2,1)
    # ax[0].plot(STATE_VEC[:,0], STATE_VEC[:,2], 'b-',label='CoM_Pos_Y')
    # # ax[0].plot(TIME_VEC, STATE_VEC[:,2], 'b-',label='CoM_Pos_Y')
    # # ax[1].plot(TIME_VEC, STATE_VEC[:,0], 'b-',label='CoM_Pos_X')
    # plt.show()


if __name__ == '__main__': 
    # rospy.init_node('walking')
    main()