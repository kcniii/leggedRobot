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
        # T0 = pin.SE3(np.eye(3), np.array([0, 0, 0]))
        # T1 = pin.SE3(np.eye(3), np.array([0.2, 0, 0]))

        # p0 = T0.translation # 0 0 0
        # p2 = T1.translation # 0.2 0 0
        p0 = T0
        p2 = T1
        # p1 = np.array([(p0[0]+p2[0])/2, (p0[1]+p2[1])/2, 0.05]) # 0.1 0 0.05
        p1 = p0 + 0.5*(p2-p0)
        p1[2] = self._height
        
        t2 = self._duration # 1.0

        p = np.matrix([p0,p1,p2]).T 
        t = np.matrix([0,0.5,1]).T

        # constraints: vel, acc
        c = curve_constraints()
        c.init_vel = np.matrix([0,0,0]).T
        c.end_vel = np.matrix([0,0,0]).T
        c.init_acc = np.matrix([0,0,0]).T
        c.end_acc = np.matrix([0,0,0]).T

        ec = exact_cubic(p,t,c)
        # coeffs = np.matrix([p0,p1,p2]).T
        # ec = polynomial(coeffs)
        self.spline = ec

    def isDone(self):
        return self._t_elapsed >= self._duration 
    
    def evaluate(self, t):
        """evaluate at time t
        """
        #>>>>TODO: evaluate the spline at time t, return pose, velocity, acceleration
        p = self.spline(t)
        v = self.spline.derivate(t,1)
        a = self.spline.derivate(t,2)

        return p, v, a

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