import numpy as np
import pinocchio as pin
from enum import Enum
# import talos_conf as conf
# import simulator.talos_conf as conf
import simple_walking.robots.talos_conf as conf

import pybullet as pb
# from simulator.pybullet_wrapper import PybulletWrapper
from simple_walking.robots.pybullet_wrapper import PybulletWrapper


class Side(Enum):
    """Side
    Describes which foot to use
    """
    LEFT=0
    RIGHT=1

def other_foot_id(id):
    if id == Side.LEFT:
        return Side.RIGHT
    else:
        return Side.LEFT
        
class FootStep:
    """FootStep
    Holds all information describing a single footstep
    """
    def __init__(self, pose, footprint, side=Side.LEFT):
        """inti FootStep

        Args:
            pose (pin.SE3): the pose of the footstep
            footprint (np.array): 3 by n matrix of foot vertices
            side (_type_, optional): Foot identifier. Defaults to Side.LEFT.
        """
        self.pose = pose
        self.footprint = footprint
        self.side = side
        
    def poseInWorld(self):
        return self.pose
        
    def plot(self, simulation):
        
        #>>>>TODO: plot in pybullet footprint, addGlobalDebugRectancle(...)
        #>>>>TODO: display the side of the step, addUserDebugText(...)
        #>>>>TODO: plot step target position addSphereMarker(...)

        for i in self.pose:
            simulation.addSphereMarker(i,radius=0.02)

            simulation.addGlobalDebugRectancle(i - np.array([0.02,0,0]),length=0.2, width=0.13)
            if i[1] >0 :
                simulation.addUserDebugText(parent_id=-1, link_id=-1, text = 'LEFT', x = i + np.array([0,0.05,0]))
            else:
                simulation.addUserDebugText(parent_id=-1, link_id=-1, text = 'RIGHT', x = i + np.array([0,0.05,0]))


        return None

class FootStepPlanner:
    """FootStepPlanner
    Creates footstep plans (list of right and left steps)
    """
    
    def __init__(self, conf):
        self.conf = conf
        self.steps = []
        
    def planLine(self, T_0_w, side, no_steps):
        """plan a sequence of steps in a strait line

        Args:
            T_0_w (pin.SE3): The inital starting position of the plan
            side (Side): The intial foot for starting the plan
            no_steps (_type_): The number of steps to take

        Returns:
            list: sequence of steps
        """
        
        # the displacement between steps in x and y direction
        dx = self.conf.step_size_x # 0.25
        dy = 2*self.conf.step_size_y # 0.096
        # dy = 0.16899
        
        # the footprint of the robot
        lfxp, lfxn = self.conf.lfxp, self.conf.lfxn # 0.12, 0.08 #tbc
        lfyp, lfyn = self.conf.lfyp, self.conf.lfyn # 0.065, 0.065
        
        #>>>>TODO: Plan a sequence of steps with T_0_w being the first step pose.
        #>>>>Note: Plan the second step parallel to the first step (robot starts standing on both feet)
        #>>>>Note: Plan the final step parallel to the last-1 step (robot stops standing on both feet)

        t0 = T_0_w.translation
        steps=[]
        steps.append(t0)

        t1 = np.array([t0[0], t0[1]-dy, 0])
        # steps.append(t1)
        for i in range(no_steps-1):
            if i % 2 == 0:
                steps.append(np.array([t1[0]+(i+1)*dx, t1[1], 0]))
            else:
                steps.append(np.array([t1[0]+(i+1)*dx, t0[1], 0]))

        # if side == Side.RIGHT: 
        #     t1 = np.array([t0[0], t0[1]+dy, 0])
        #     # steps.append(t1)
        #     for i in range(no_steps):
        #         if i % 2 == 0:
        #             steps.append(np.array([t1[0]+i*dx, t0[1], 0]))
        #         else:
        #             steps.append(np.array([t1[0]+i*dx, t1[1], 0]))

        # elif side == Side.LEFT:
        #     t1 = np.array([t0[0], t0[1]+dy, 0])
        #     # steps.append(t1)
        #     for i in range(no_steps):
        #         if i % 2 == 0:
        #             steps.append(np.array([t1[0]+i*dx, t1[1], 0]))
        #         else:
        #             steps.append(np.array([t1[0]+i*dx, t0[1], 0]))
    
        # # final step
        # if steps[-1][1] == t1[1]:
        #     final_step = np.array([steps[-1][0],t0[1], 0])
        # if steps[-1][1] == t0[1]:
        #     final_step = np.array([steps[-1][0], t1[1], 0])
        # steps.append(final_step)

        steps1 =[]
        for j in steps:
            steps1.append(pin.SE3(T_0_w.rotation, j))
            # steps1.append(pin.SE3(np.eye(3), j))
        self.steps = steps1


        return steps

    
    def plot(self, simulation):
        for step in self.steps:
            step.plot(simulation)

            
if __name__=='__main__':
    """ Test footstep planner
    """
    
    #>>>>TODO: Generate a plan and plot it in pybullet.
    step_planner = FootStepPlanner(conf)
    T0 = pin.SE3(np.eye(3), np.array([0, 0.096, 0])) 

    # plan
    side = Side.LEFT
    steps_line = step_planner.planLine(T0, side, 8)

    for i in steps_line:
        print(i)
    foot_steps = FootStep(steps_line, 0, side)

    #>>>>TODO: Check that the plan looks as expected
    # plot in pybullet

    # line_id = -1
    # simulator = PybulletWrapper()
    # while True:
    #     foot_steps.plot(simulator)