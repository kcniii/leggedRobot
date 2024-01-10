import numpy as np
from ndcurves import  exact_cubic, curve_constraints,polynomial
import pinocchio as pin
from matplotlib import pyplot as plt

class SwingFootTrajectory:
    """SwingFootTrajectory
    Interpolate Foot trajectory between SE3 T0 and T1
    """
    def __init__(self, T0, T1, duration, height=0.05):
        """initialize SwingFootTrajectory

        Args:
            T0 (pin.SE3): Inital foot pose
            T1 (pin.SE3): Final foot pose
            duration (float): step duration
            height (float, optional): setp height. Defaults to 0.05.
        """
        self._height = height
        self._t_elapsed = 0.0
        self._duration = duration
        self.spline = 0
        self.reset(T0, T1)
        

    def reset(self, T0, T1):
        '''reset back to zero, update poses
        '''
        #>>>>TODO: plan the spline
        self._t_elapsed = 0.0
        self.T0 = T0
        self.T1 = T1

        T = 0.8
        half_T = T/2

        # Build the system of equations
        A_z = np.array([
            [0, 0, 0, 0, 0, 0, 1],
            [T**6, T**5, T**4, T**3, T**2, T, 1],
            [half_T**6, half_T**5, half_T**4, half_T**3, half_T**2, half_T, 1],
            [0, 0, 0, 0, 0, 1, 0],
            [6*T**5, 5*T**4, 4*T**3, 3*T**2, 2*T, 1, 0],
            [0, 0, 0, 0, 2, 0, 0],
            [30*T**4, 20*T**3, 12*T**2, 6*T, 2, 0, 0]
        ])
        B_z = np.array([self.T0[2], self.T1[2], self._height, 0, 0, 0, 0])
        
        # Solve the system of equations
        self.coeff_z = np.linalg.solve(A_z, B_z)

        A_x = np.array([
            [0, 0, 0, 0, 0, 1],
            [T**5, T**4, T**3, T**2, T, 1],
            [0, 0, 0, 0, 1, 0],
            [5*T**4, 4*T**3, 3*T**2, 2*T, 1, 0],
            [0, 0, 0, 2, 0, 0],
            [20*T**3, 12*T**2, 6*T, 2, 0, 0]
        ])

        B_x = np.array([self.T0[0], self.T1[0], 0, 0, 0, 0])

        self.coeff_x = np.linalg.solve(A_x, B_x)

        A_y = np.array([
            [0, 0, 0, 0, 0, 1],
            [T**5, T**4, T**3, T**2, T, 1],
            [0, 0, 0, 0, 1, 0],
            [5*T**4, 4*T**3, 3*T**2, 2*T, 1, 0],
            [0, 0, 0, 2, 0, 0],
            [20*T**3, 12*T**2, 6*T, 2, 0, 0]
        ])

        B_y = np.array([self.T0[1], self.T1[1], 0, 0, 0, 0])

        self.coeff_y = np.linalg.solve(A_y, B_y)

    def isDone(self):
        return self._t_elapsed >= self._duration 
    
    def evaluate(self, t):
        """evaluate at time t
        """
        #>>>>TODO: evaluate the spline at time t, return pose, velocity, acceleration
        t = np.clip(t, 0, 0.8)
        # Find position, velocity and acceleration at time t
        pos_z = np.polyval(self.coeff_z, t)
        vel_z = np.polyval(np.polyder(self.coeff_z, 1), t)
        acc_z = np.polyval(np.polyder(self.coeff_z, 2), t)

        pos_x = np.polyval(self.coeff_x, t)
        vel_x = np.polyval(np.polyder(self.coeff_x, 1), t)
        acc_x = np.polyval(np.polyder(self.coeff_x, 2), t)

        pos_y = np.polyval(self.coeff_y, t)
        vel_y = np.polyval(np.polyder(self.coeff_y, 1), t)
        acc_y = np.polyval(np.polyder(self.coeff_y, 2), t)

        pos = np.array([pos_x, pos_y, pos_z])
        vel = np.array([vel_x, vel_y, vel_z])
        acc = np.array([acc_x, acc_y, acc_z])
        # Update elapsed time
        return pos, vel, acc

if __name__=="__main__":
    T0 = pin.SE3(np.eye(3), np.array([0, 0.096, 0]))
    T1 = pin.SE3(np.eye(3), np.array([0.5, 0.096, 0]))


    #>>>>TODO: plot to make sure everything is correct
    t0 = T0.translation
    t1 = T1.translation
    traj = SwingFootTrajectory(t0, t1, 1.0)

    # plot
    p =[]
    v =[]
    a =[]

    for i in range(100):
        p.append(traj.evaluate(i/100)[0])
        v.append(traj.evaluate(i/100)[1])
        a.append(traj.evaluate(i/100)[2])

    plt.figure(1)
    plt.title('Position')
    plt.plot(p)

    # plt.figure(2)
    # plt.title('Velocity')
    # plt.plot(v)

    # plt.figure(3)
    # plt.title('Acceleration')
    # plt.plot(a)

    plt.show()