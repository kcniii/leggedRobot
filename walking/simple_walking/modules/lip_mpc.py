import numpy as np

# import sys
# sys.path.append("/home/kaicheng/env/lib/python3.8/site-packages/")

# import pydrake.all
# from all import MathematicalProgram, Solve
from pydrake.all import MathematicalProgram, Solve



################################################################################
# Helper fnc
################################################################################

def continious_LIP_dynamics(g, h):
    """returns the static matrices A,B of the continious LIP dynamics
    """
    #>>>>TODO: Compute
    A = np.array([[0, 1], [g/h, 0]])
    B = np.array([[0], [-g/h]])
    return A, B

def discrete_LIP_dynamics(g, h, dt):
    """returns the matrices static Ad,Bd of the discretized LIP dynamics
    """
    #>>>>TODO: Compute
    w = np.sqrt(g/h)
    t = 0.1
    Ad = np.array([[np.cosh(w*t),(1/w)*np.sinh(w*t)],[w*np.sinh(w*t),np.cosh(w*t)]])
    Bd = np.array([[1-np.cosh(w*t)],[-w*(np.sinh(w*t))]])
    return Ad, Bd

################################################################################
# LIPInterpolator
################################################################################

class LIPInterpolator:
    """Integrates the linear inverted pendulum model using the 
    continous dynamics.
    """
    # self.x
    # x vx y vy

    # self.x.reshape(2,2)
    # x vx
    # y vy

    # self.x.reshape(2.2).T
    # x y
    # vx vy

    def __init__(self, x_inital, conf):
        self.conf = conf
        self.dt = conf.dt
        self.x = x_inital
        self.x_dot = np.array([[0,0],[0,0]])
        #>>>>TODO: Finish
        self.g = conf.g
        self.h = conf.h

        self.A, self.B = continious_LIP_dynamics(conf.g, conf.h)
        self.a = None

    def integrate(self, u):
        #>>>>TODO: integrate with dt
        #>>>>Note: use continious dynamics
        # x = self.x.reshape(2,2).T

        self.x_dot = self.A@ self.x.reshape(2,2).T+ self.B*u
        x_next = self.x.reshape(2,2).T + self.x_dot*self.dt
        self.x = x_next.T.reshape(4)
        ## x vx y vy

        # x = self.x[0]
        # y = self.x[2]
        # vx = self.x[1]
        # vy = self.x[3]

        # w = 9.81/0.85
        # ax = w*(x-u[0])
        # ay = w*(y-u[1])
        
        # x_new = x+vx*self.dt
        # y_new = y+vy*self.dt
        # vx_new = vx+ax*self.dt
        # vy_new = vy+ay*self.dt

        # self.x[0] = x_new
        # self.x[1] = vx_new
        # self.x[2] = y_new
        # self.x[3] = vy_new

        return self.x
    
    def comState(self):
        #>>>>TODO: return the center of mass state
        # that is position \in R3, velocity \in R3, acceleration \in R3
        c = np.array([self.x[0], self.x[2], 0.85])
        c_dot = np.array([self.x_dot[0,0], self.x_dot[0,1], 0])
        # c_ddot = np.array([self.x_dot[1],self.x_dot[3],0])
        c_ddot =np.array([self.x_dot[1,0],self.x_dot[1,1], 0])
        return c, c_dot, c_ddot
    
    def dcm(self):
        #>>>>TODO: return the computed dcm
        w = self.g/self.h
        x = self.comState()[0]
        x_dot = self.comState()[1]
        dcm = x + x_dot/w
        return dcm
    
    def zmp(self):
        #>>>>TODO: return the zmp
        x = self.comState()[0]
        zmp = x- self.a*self.h/self.g
        return zmp
        
    
################################################################################
# LIPMPC
################################################################################

class LIPMPC:
    def __init__(self, conf):
        self.conf = conf
        self.dt = conf.dt_mpc # 0.1
        self.no_samples = conf.no_mpc_samples_per_horizon # 16
        
        # solution and references over the horizon
        self.X_k = None
        self.U_k = None
        self.ZMP_ref_k = None

        self.Ad, self.Bd = discrete_LIP_dynamics(conf.g, conf.h, conf.dt_mpc)  
        
    def buildSolveOCP(self, x_k, ZMP_ref_k, terminal_idx):
        """build and solve ocp

        Args:
            x_k (_type_): inital mpc state
            ZMP_ref_k (_type_): zmp reference over horizon
            terminal_idx (_type_): index within horizon to apply terminal constraint

        Returns:
            _type_: control
        """
        
        #>>>>TODO: build and solve the ocp
        #>>>>Note: start without terminal constraints
        nx = 4 
        nu = 2 
        prog = MathematicalProgram()
        
        state = prog.NewContinuousVariables(self.no_samples, nx, 'state') # 16*4
        control = prog.NewContinuousVariables(self.no_samples, nu, 'control') # 16*2
        # 1. intial constraint
        for i in range(nx):
            prog.AddConstraint(state[0,i] == x_k[i])
    
        # 2. at each time step: respect the LIP descretized dynamics
        for i in range(self.no_samples-1): #tbc
            
            a = self.Ad@(state[i].reshape(2,2).T)
            b = self.Bd@np.array([control[i]])
            x_next = (a+b).T.reshape(4)
            # 
            for j in range(nx):
                prog.AddConstraint(state[i+1,j] == x_next[j])

        # 3. at each time step: keep the ZMP within the foot sole (use the footprint and planned step position)
        #>>>>TODO: Add ZMP upper and lower bound to keep the control (ZMP) within each footprints
        #Hint: first compute upper and lower bound based on zmp_ref then add constraints.
        #Hint: Add constraints at every time step
        
        for i in range(self.no_samples): # 10 # only 8 foot steps
            xmin = ZMP_ref_k[i][0]-self.conf.lfxn
            xmax = ZMP_ref_k[i][0]+self.conf.lfxp
            ymin = ZMP_ref_k[i][1]-self.conf.lfyn
            ymax = ZMP_ref_k[i][1]+self.conf.lfyp

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
                #     prog.AddConstraint(state[i,j] == ZMP_ref_k[-1][j])
                prog.AddConstraint(state[i,0] == ZMP_ref_k[-1][0])
                prog.AddConstraint(state[i,1] == 0)
                prog.AddConstraint(state[i,2] == ZMP_ref_k[-1][1])
                prog.AddConstraint(state[i,3] == 0)

        # setup our cost: minimize zmp error (tracking), minimize CoM velocity (smoothing)
        #>>>>TODO: add the cost at each timestep, hint: prog.AddCost
        for i in range(self.no_samples-1):
            prog.AddCost(self.conf.alpha * ((control[i,0]-ZMP_ref_k[i][0])**2 +(control[i,1]-ZMP_ref_k[i][1])**2) + 
                        self.conf.gamma * (state[i][1]**2 + state[i][3]**2))
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
    

def generate_zmp_reference(foot_steps, no_samples_per_step):
    """generate the zmp reference given a sequence of footsteps
    """

    #>>>>TODO: use the previously footstep type to build the reference 
    # trajectory for the zmp
    zmp_ref = []
    for i in foot_steps: #tbc
        zmp_ref.append([i[0],i[2]]*no_samples_per_step) #Q
    zmp_ref = np.array(zmp_ref)
    # reshape to [~,2]
    zmp_ref = zmp_ref.reshape(-1,2)

    return zmp_ref