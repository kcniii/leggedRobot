"""Task2: Linear inverted pendulum MPC

The goal of this file is to formulate the optimal control problem (OCP)
in equation 12 but this time as a model predictive controller (MPC).

In this case we will solve the trajectory planning multiple times over 
a shorter horizon of just 2 steps (receding horizon).
Time between two MPC updates is called T_MPC.

In between MPC updates we simulate the Linear inverted pendulum at a smaller
step time T_SIM, with the lates MPC control ouput u.

Our state & control is the same as before
x = [cx, vx, cy, vy]
u = [px, py]

You will need to fill in the TODO to solve the task.
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
# # sys.path.append("src/simple_walking/modules/env")
sys.path.append("src/")
from pydrake.all import MathematicalProgram, Solve

import matplotlib.animation as animation
import simple_walking.robots.talos_conf as conf
from simple_walking.modules.lip_mpc import LIPMPC


################################################################################
# settings
################################################################################

NO_STEPS                = 8         # total number of foot steps
STEP_TIME               = 0.8       # time needed for every step

# Robot Parameters:
# --------------
h                       = 0.80      # fixed CoM height (assuming walking on a flat terrain)
g                       = 9.81      # norm of the gravity vector
foot_length             = 0.10      # foot size in the x-direction
foot_width              = 0.06      # foot size in the y-direciton


# MPC Parameters:
# --------------
T_MPC                   = 0.1                                               # sampling time interval of the MPC
# 8
NO_MPC_SAMPLES_PER_STEP = int(round(STEP_TIME/T_MPC))                       # number of mpc updates per step
NO_STEPS_PER_HORIZON  = 2                                                   # how many steps in the horizon
# 1.6
T_HORIZON = NO_STEPS_PER_HORIZON*STEP_TIME                                  # duration of future horizon
# 2*0.8/0.1 = 16
NO_MPC_SAMPLES_HORIZON = int(round(NO_STEPS_PER_HORIZON*STEP_TIME/T_MPC))   # number of mpc updates per horizon

# Cost Parameters:
# ---------------
alpha       = 10**(-1)                                  # ZMP error squared cost weight (= tracking cost)
gamma       = 10**(-3)                                  # CoM velocity error squared cost weight (= smoothing cost)

# Simulation Parameters:
# --------------
T_SIM                   = 0.005                         # 200 Hz simulation time
# 0.1/0.005 = 20
NO_SIM_SAMPLES_PER_MPC = int(round(T_MPC/T_SIM))        # NO SIM samples between MPC updates
# 8*0.8/0.1 = 64
NO_MPC_SAMPLES = int(round(NO_STEPS*STEP_TIME/T_MPC))   # Total number of MPC samples
# 8*0.8/0.005 = 1280
NO_SIM_SAMPLES = int(round(NO_STEPS*STEP_TIME/T_SIM))   # Total number of Simulator samples

################################################################################
# Helper fnc
################################################################################

def generate_foot_steps(foot_step_0, step_size_x, no_steps):
    """Write a function that generates footstep of stepsize=step_size_x in the 
    x direction starting from foot_step_0 located at (x0, y0).

    Args:
        foot_step_0 (_type_): _description_
        step_size_x (_type_): _description_
        no_steps (_type_): _description_
    """

    #>>>>TODO: copy from previous Task 2
    foot_steps = []
    foot_steps.append(foot_step_0)

    for i in range(no_steps-1):
        xk = np.array([foot_step_0[0] + (i+1)*step_size_x,0])

        if i%2==0:
            yk = np.array([-foot_step_0[2],0])
        else:
            yk = np.array([foot_step_0[2] ,0])

        new_fs = np.concatenate([xk,yk])
        foot_steps.append(new_fs)

    return foot_steps

def plot_foot_steps(foot_steps, XY_foot_print, ax):
    """Write a function that plots footsteps in the xy plane using the given
    footprint (length, width)
    You can use the function ax.fill() to gerneate a rectable.
    Color left and right steps differt and check if the step sequence makes sense.

    Args:
        foot_steps (_type_): _description_
    """
    #>>>>TODO: copy from previous Task 2
    ax.plot(foot_steps[:,0],foot_steps[:,2],'y') # plotf
    xl = XY_foot_print[0]/2  # 0.05
    yl = XY_foot_print[1]/2   # 0.03

    for i in foot_steps:
        if i[2] < 0:
            color = 'r'
        else:
            color = 'g'
        ax.fill(i[0]+np.array([-xl,xl,xl,-xl]),i[2]+np.array([-yl,-yl,yl,yl]),color,alpha=1)
        plt.axis('equal')
        plt.grid('true')
        plt.show()
    # plt.axis('equal')
    # plt.grid('true')

def generate_zmp_reference(foot_steps, no_samples_per_step):
    """generate a function that computes a referecne trajecotry for the zmp.
    Our goal is to keep the ZMP at the footstep center within each step

    Args:
        foot_steps (_type_): _description_
        no_samples_per_step (_type_): _description_
    """
    #>>>>TODO: copy from previous Task 2
    zmp_ref = []
    for i in foot_steps: #tbc
        zmp_ref.append([i[0],i[2]]*no_samples_per_step) #Q
    zmp_ref = np.array(zmp_ref)
    zmp_ref = zmp_ref.reshape(-1,2)
    return zmp_ref

################################################################################
# Dynamics of the simplified walking model
################################################################################

def continious_LIP_dynamics():
    """returns the static matrices A,B of the continious LIP dynamics

    Args:
        g (_type_): gravity
        h (_type_): height
    Returns:
        np.array: A, B
    """
    # Generate A, B for the continous linear inverted pendulum #tbd
    #>>>>Hint: Look at Eq. 4 and rewrite as a system first order diff. eq.
    # x_ddot = g/h*(x-u)
    # x_t+1 = A*x_t + B*u_t
    w = np.sqrt(g/h)
    A =np.array([[0,1],[w,0]])
    B =np.array([[0],[-w]])

    
    #>>>>TODO: copy from previous Task 2
    return A, B

def discrete_LIP_dynamics(dt):
    """returns the matrices static Ad,Bd of the discretized LIP dynamics

    Args:
        dt (_type_): discretization steps
        g (_type_): gravity
        h (_type_): height

    Returns:
        _type_: _description_
    """
    #>>>>TODO: copy from previous Task 2
    w = np.sqrt(g/h)
    t = dt
    Ad = np.array([[np.cosh(w*t),1/w*np.sinh(w*t)],[w*np.sinh(w*t),np.cosh(w*t)]])
    Bd = np.array([[1-np.cosh(w*t)],[-w*(np.sinh(w*t))]])
    return Ad, Bd

################################################################################
# Simulation
################################################################################

class Simulator:
    """Simulates the Linear inverted pendulum continous dynamics
    Uses simple euler integration to simulate LIP at sample time dt
    """
    def __init__(self, x_inital, dt):
        self.dt = dt
        self.x = x_inital
        
        self.A, self.B = continious_LIP_dynamics()
        self.D = np.array([[0, 0], [1, 0], [0, 0], [0, 1]])
        
    def simulate(self, u, d=np.zeros(2)):
        """updates the LIP state x using based on command u
        
        Optionally: Takes a disturbance acceleration d to simulate effect
        of external pushes on the LIP.
        """

        #>>>>TODO: Compute x_dot and use euler integration to approximate
        # the state at t+dt
        x = self.x[0]
        y = self.x[2]
        vx = self.x[1]
        vy = self.x[3]

        w = g/h
        ax = w*(x-u[0])+d[0]
        ay = w*(y-u[1])+d[1]
        
        x_new = x+vx*self.dt
        y_new = y+vy*self.dt
        vx_new = vx+ax*self.dt
        vy_new = vy+ay*self.dt

        self.x[0] = x_new
        self.x[1] = vx_new
        self.x[2] = y_new
        self.x[3] = vy_new
        # self.x = np.array([[x+vx*self.dt],[vx+ax*self.dt],[y+vy*self.dt],[vy+ay*self.dt]])

        # x_reshape = self.x.reshape(2,2).T
        # w = np.sqrt(g/h)
        # x_dot= np.array([[0,1],[w,0]])@x_reshape + np.array([[0],[-w]])*u
        # x_next = x_reshape+ x_dot*self.dt

        # self.x = x_next.T.reshape(1,4)
        #>>>>TODO: The disturbance is added in x_dot as self.D@d
        
        return self.x    

################################################################################
# MPC
################################################################################

class MPC:
    """MPC for the Linear inverted pendulum
    """
    def __init__(self, dt, T_horizon): #0.1, 1.6
        self.dt = dt                                        # mpc dt
        self.T_horizon = T_horizon                          # time of horizon
        # 1.6/0.1 = 16
        self.no_samples = int(round(T_horizon/self.dt))     # mpc samples in horizon (nodes)

        self.Ad, self.Bd = discrete_LIP_dynamics(dt)
        
        self.X_k = None                                     # state over current horizon
        self.U_k = None                                     # control over current horizon
        self.ZMP_ref_k = None                               # ZMP reference over current horizon
        
    def buildSolveOCP(self, x_k, ZMP_ref_k, terminal_idx):
        """ build the MathematicalProgram that solves the mpc problem and 
        returns the first command of U_k

        Args:
            x_k (_type_): the current state of the lip when starting the mpc
            ZMP_ref_k (_type_): the reference over the current horizon, shape=(no_samples, 2)
            terminal_idx (_type_): index of the terminal constraint within horizon (or bigger than horizon if no constraint)
            
        """
        
        # variables
        nx = 4 #>>>>TODO: State dimension = ?
        nu = 2 #>>>>TODO: control dimension = ?
        prog = MathematicalProgram()
        
        state = prog.NewContinuousVariables(self.no_samples, nx, 'state') # 16*4
        control = prog.NewContinuousVariables(self.no_samples, nu, 'control') # 16*2
        # 1. intial constraint
        #>>>>TODO: Add inital state constraint, Hint: x_k
        # prog.AddConstraint(state[0,0] == x_k[0]) #tbc: stae[0,i] ?
        # prog.AddConstraint(state[0,1] == x_k[1])
        # prog.AddConstraint(state[0,2] == x_k[2])
        # prog.AddConstraint(state[0,3] == x_k[3])

        for i in range(nx):
            prog.AddConstraint(state[0,i] == x_k[i])
        # 2. at each time step: respect the LIP descretized dynamics
        #>>>>TODO: Enforce the dynamics at every time step

        for i in range(self.no_samples-1): #tbc
            
            a = self.Ad@(state[i].reshape(2,2).T)
            b = self.Bd@np.array([control[i]])
            x_next = (a+b).T.reshape(4)
            
            for j in range(nx):
                prog.AddConstraint(state[i+1,j] == x_next[j])

        # 3. at each time step: keep the ZMP within the foot sole (use the footprint and planned step position)
        #>>>>TODO: Add ZMP upper and lower bound to keep the control (ZMP) within each footprints
        #Hint: first compute upper and lower bound based on zmp_ref then add constraints.
        #Hint: Add constraints at every time step
        
        # k = 64 - terminal_idx # terminal_idx = 64 - k
        for i in range(self.no_samples): # 16
            # n = i+k
            # n1 = n//8
            # xmin = foot_steps[n1][0]-footprint[0]/2
            # xmax = foot_steps[n1][0]+footprint[0]/2
            # ymin = foot_steps[n1][2]-footprint[1]/2
            # ymax = foot_steps[n1][2]+footprint[1]/2
            xmin = ZMP_ref_k[i,0]-footprint[0]/2
            xmax = ZMP_ref_k[i,0]+footprint[0]/2
            ymin = ZMP_ref_k[i,1]-footprint[1]/2
            ymax = ZMP_ref_k[i,1]+footprint[1]/2
            prog.AddBoundingBoxConstraint(xmin,xmax,control[i][0])
            prog.AddBoundingBoxConstraint(ymin,ymax,control[i][1])

        # 4. if terminal_idx < self.no_samples than we have the terminal state within
        # the current horizon. In this case create the terminal state (foot step pos + zero vel)
        # and apply the state constraint to all states >= terminal_idx within the horizon
        #>>>>TODO: Add the terminal constraint if requires
        #Hint: If you are unsure, you can start testing without this first!
        # idx_terminal_k = NO_MPC_SAMPLES - k # 64-k
        # k=60 -> idx_terminal_k = 4
        if terminal_idx < self.no_samples: # when mpc samples less than horizon 16
            for i in range(terminal_idx,self.no_samples):
                # for j in range(nx):
                #     prog.AddConstraint(state[i,j] == foot_steps[-1][j])
                prog.AddConstraint(state[i,0] == ZMP_ref_k[-1][0])
                prog.AddConstraint(state[i,1] == 0)
                prog.AddConstraint(state[i,2] == ZMP_ref_k[-1][1])
                prog.AddConstraint(state[i,3] == 0)


        # setup our cost: minimize zmp error (tracking), minimize CoM velocity (smoothing)
        #>>>>TODO: add the cost at each timestep, hint: prog.AddCost
        for i in range(self.no_samples-1):
            prog.AddCost(alpha * ((control[i,0]-ZMP_ref_k[i][0])**2 +(control[i,1]-ZMP_ref_k[i][1])**2) + 
                        gamma * (state[i][1]**2 + state[i][3]**2))
        # solve
        result = Solve(prog)
        if not result.is_success:
            print("result failure")
            
        self.X_k = result.GetSolution(state)
        self.U_k = result.GetSolution(control)
        # print("X_k",self.X_k)
        # print("U_k",self.U_k)
        if np.isnan(self.X_k).any():
            print("isnan failure")
        
        self.ZMP_ref_k = ZMP_ref_k
        return self.U_k[0]
    
################################################################################
# run the simulation
################################################################################

# inital state in x0 = [px0, vx0]
x_0 = np.array([0.0, 0.0])
# inital state in y0 = [py0, vy0]
y_0 = np.array([-0.09, 0.0])

# footprint
footprint = np.array([foot_length, foot_width])

# generate the footsteps
step_size = 0.2
#>>>>TODO: 1. generate the foot step plan using generate_foot_steps
foot_steps=generate_foot_steps(np.concatenate([x_0,y_0]),step_size,NO_STEPS)

# reapeat the last two foot steps (so the mpc horizon never exceeds the plan!)
foot_steps = np.vstack([
    foot_steps, foot_steps[-1], foot_steps[-1]])
# plt.show()
pass


# zmp reference trajecotry
#>>>>TODO: 2. generate the complete ZMP reference using generate_zmp_reference
zmp_ref = generate_zmp_reference(foot_steps,NO_MPC_SAMPLES_PER_STEP) # 10 steps, 9th&10th repeat 8th
# generate mpc
# mpc = MPC(T_MPC, T_HORIZON) # 0.1, 1.6
mpc = LIPMPC(conf) # buildSolveOCP()

# generate the pendulum simulator
state_0 = np.concatenate([x_0, y_0])
sim = Simulator(state_0, T_SIM)

# setup some vectors for plotting stuff
TIME_VEC = np.nan*np.ones(NO_SIM_SAMPLES)
STATE_VEC = np.nan*np.ones([NO_SIM_SAMPLES, 4])
ZMP_REF_VEC = np.nan*np.ones([NO_SIM_SAMPLES, 2])
ZMP_VEC = np.nan*np.ones([NO_SIM_SAMPLES, 2])

# time to add some disturbance
t_push = 3.2
u_k = np.array([0, -0.09])
x_k = state_0
# execution loop

k = 0   # the number of mpc update
for i in range(NO_SIM_SAMPLES): # 0-1280
    
    # simulation time
    t = i*T_SIM # i*0.005 # get the current time at each mpc step
        
    if i % NO_SIM_SAMPLES_PER_MPC == 0: # 0.1/0.005 = 20 -> every 0.1 sec
        # time to update the mpc
        
        # current state
        #>>>>TODO: get current state from the simulator
        # u_k = zmp_ref[k] #tbc
        # x_k = sim.simulate(u_k)
        # x_k = sim.simulate(u_k) #tbc: u_k?
        #>>>>TODO: extract the current horizon from the complete reference trajecotry ZMP_ref
        ZMP_ref_k = zmp_ref[k:k+NO_MPC_SAMPLES_HORIZON] # k:k+16
    
        # check if we have terminal constraint
        idx_terminal_k = NO_MPC_SAMPLES - k # 64-k
        #>>>>TODO: Update the mpc, get new command
        u_k = mpc.buildSolveOCP(x_k, ZMP_ref_k, idx_terminal_k)
        # u_k = None
        
        k += 1
    
    # simulate a push for 0.05 sec with 1.0 m/s^2 acceleration 
    x_ddot_ext = np.array([0, 0])
    
    #>>>>TODO: when you got everything working try adding a small disturbance
    if i > int(t_push/T_SIM) and i < int((t_push + 0.05)/T_SIM):
        x_ddot_ext = np.array([0, 1.0])
    
    #>>>>TODO: Update the simulation using the current command
    x_k = sim.simulate(u_k,x_ddot_ext)

    # save some shit
    TIME_VEC[k] = t
    STATE_VEC[k] = x_k
    ZMP_VEC[k] = u_k
    ZMP_REF_VEC[k] = mpc.ZMP_ref_k[0]

ZMP_LB_VEC = ZMP_REF_VEC - footprint[None,:]
ZMP_UB_VEC = ZMP_REF_VEC + footprint[None,:]

#>>>>TODO: Use the recodings in STATE_VEC and ZMP_VEC to compute the 
# LIP acceleration
#>>>>Hint: Use the continious dynamic matrices
STATE_DOT_VEC = None

################################################################################
# plot something
#>>>>TODO: plot everything in x-axis
#>>>>TODO: plot everything in y-axis
#>>>>TODO: plot everything in xy-plane
fig, ax = plt.subplots(3,1)
ax[0].plot(TIME_VEC, STATE_VEC[:,0], 'b-',label='CoM_Pos_X')
ax[0].plot(TIME_VEC, ZMP_VEC[:,0], 'y-',label='ZMP_X')
ax[0].plot(TIME_VEC, ZMP_REF_VEC[:,0], 'g-.', label='ZMP_Ref_X')
ax[0].plot(TIME_VEC, ZMP_LB_VEC[:,0]-footprint[1]/2,  'r-.',label='ZMP_LB_X')
ax[0].plot(TIME_VEC, ZMP_UB_VEC[:,0]+footprint[1]/2,  'm-.',label='zmp_UB_X')
ax[0].legend(loc='upper left', shadow=True)

ax[1].plot(TIME_VEC, STATE_VEC[:,1], 'b-',label='CoM_Vel_X')
ax[1].legend(loc='upper left', shadow=True)

acc_x =  g/h*(STATE_VEC[:,0]-ZMP_VEC[:,0])
ax[2].plot(TIME_VEC, acc_x, 'b-',label='CoM_Acc_X')
ax[2].legend(loc='upper left', shadow=True)



fig1, ax1 = plt.subplots(3,1)
ax1[0].plot(TIME_VEC, STATE_VEC[:,2], 'b-',label='CoM_Pos_Y')
ax1[0].plot(TIME_VEC, ZMP_VEC[:,1], 'y-',label='ZMP_Y')
ax1[0].plot(TIME_VEC, ZMP_REF_VEC[:,1], 'g-.', label='ZMP_Ref_Y')
ax1[0].plot(TIME_VEC, ZMP_LB_VEC[:,1]-footprint[1]/2,  'r-.',label='ZMP_LB_Y')
ax1[0].plot(TIME_VEC, ZMP_UB_VEC[:,1]+footprint[1]/2,  'm-.',label='zmp_UB_Y')
ax1[0].legend(loc='upper left', shadow=True)

ax1[1].plot(TIME_VEC, STATE_VEC[:,3], 'b-',label='CoM_Vel_Y')
ax1[1].legend(loc='upper left', shadow=True)

acc_y =  g/h*(STATE_VEC[:,2]-ZMP_VEC[:,1])
ax1[2].plot(TIME_VEC, acc_y, 'b-',label='CoM_Acc_Y')
ax1[2].legend(loc='upper left', shadow=True)

plt.show()


