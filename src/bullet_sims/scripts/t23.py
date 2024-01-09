import pybullet as pb
import numpy as np
from simulator.pybullet_wrapper import PybulletWrapper
from simulator.robot import Robot
import pinocchio as pin
import rospy
from sensor_msgs.msg import JointState

# For REEM-C robot
#urdf = "src/reemc/reemc_description/robots/reemc.urdf"
#path_meshes = "src/reemc/reemc_description/meshes/../.."

# For Talos robot
urdf = "src/talos/talos_description/robots/talos_reduced.urdf"
path_meshes = "src/talos/talos_description/meshes/../.."

'''
Talos
0, 1, 2, 3, 4, 5, 			    # left leg
6, 7, 8, 9, 10, 11, 			# right leg
12, 13,                         # torso
14, 15, 16, 17, 18, 19, 20, 21  # left arm
22, 23, 24, 25, 26, 27, 28, 29  # right arm
30, 31                          # head

REEMC
0, 1, 2, 3, 4, 5, 			    # left leg
6, 7, 8, 9, 10, 11, 			# right leg
12, 13,                         # torso
14, 15, 16, 17, 18, 19, 20,     # left arm
21, 22, 23, 24, 25, 26, 27,     # right arm
28, 29                          # head
'''

def jointState_info(q,qdot,tau):
    joint_state_msg = JointState()

    # Set the header for the JointState message
    joint_state_msg.header.stamp = rospy.Time.now()

    # Set the joint names for the JointState message
    joint_state_msg.name = robot.actuatedJointNames()

    # Set the joint positions, velocities, and efforts for the JointState message
    joint_state_msg.position = q
    joint_state_msg.velocity = qdot
    joint_state_msg.effort = tau
    return joint_state_msg

# Initial condition for the simulator an model
z_init = 1.15
q_actuated_home = np.zeros(32)
q_actuated_home[:6] = np.array([0, 0, 0, 0, 0, 0])
q_actuated_home[6:12] = np.array([0, 0, 0, 0, 0, 0])
q_actuated_home[14:22] = np.array([0, 0, 0, 0, 0, 0, 0, 0 ])
q_actuated_home[22:30] = np.array([0, 0, 0, 0, 0, 0, 0, 0 ])

# Initialization position including floating base
q_home = np.hstack([np.array([0, 0, z_init, 0, 0, 0, 1]), q_actuated_home])

# setup the task stack
modelWrap = pin.RobotWrapper.BuildFromURDF(urdf,                        # Model description
                                        path_meshes,                 # Model geometry descriptors 
                                        pin.JointModelFreeFlyer(),   # Floating base model. Use "None" if fixed
                                        True,                        # Printout model details
                                        None)                        # Load meshes different from the descripor
# Get model from wrapper
model = modelWrap.model

# setup the simulator
simulator = PybulletWrapper(sim_rate=1000)

#Create Pybullet-Pinocchio map
robot = Robot(simulator,            # The Pybullet wrapper
            urdf,                 # Robot descriptor
            model,                # Pinocchio model
            [0, 0, z_init],       # Floating base initial position
            [0,0,0,1],            # Floating base initial orientation [x,y,z,w]
            q=q_home,             # Initial state
            useFixedBase=False,   # Fixed base or not
            verbose=True)         # Printout details

# Create Pinocchio data structure of the model
data = robot._model.createData()

#Needed for compatibility
simulator.addLinkDebugFrame(-1,-1)

# Setup pybullet camera
pb.resetDebugVisualizerCamera(
    cameraDistance=1.2,
    cameraYaw=90,
    cameraPitch=-20,
    cameraTargetPosition=[0.0, 0.0, 0.8])

# Joint command vector
tau = q_actuated_home*0

M = pin.crba(robot._model, data,robot._q)
nle = pin.nonLinearEffects(robot._model, data, robot._q, robot._v)
print("Complete inertia matrix M:\n", M)
print("Non-linear effects vector nle:\n", nle)

qd=np.zeros(39)

Kp = np.eye(39)
# leg
Kp[-32:19,-32:19] = Kp[-32:19,-32:19]*2200
# arm
Kp[-18:-1,-18:-1] = Kp[-18:-1,-18:-1]*150
# torso
Kp[19:21,19:21] = Kp[19:21,19:21] *500
# head
Kp[-2:,-2:]= Kp[-2:,-2:]*2

Kd = np.eye(39)
# leg
Kd[-32:19,-32:19] = Kd[-32:19,-32:19]*4
# arm
# Kd[-18:-1,-18:-1] = Kd[-18:-1,-18:-1]*1
# # torso
# Kd[19:21,19:21] = Kd[19:21,19:21] *2
# # head
# Kd[-2:,-2:]= Kd[-2:,-2:]*2


leg_q = np.array([0, 0, -0.44, 0.9, -0.45, 0])
arm_q = np.array([0, -0.24, 0, -1, 0, 0, 0, 0])

q_actuated_home_set = q_actuated_home
q_actuated_home_set[0:6]=q_actuated_home_set[6:12] = leg_q
q_actuated_home_set[14:22]=q_actuated_home_set[22:30] = arm_q
q_home_set = np.hstack([np.array([0, 0, z_init, 0, 0, 0, 1]), q_actuated_home_set])

i=0

rospy.init_node('joint_state_publisher')
rate = rospy.Rate(10)

joint_state_publisher = rospy.Publisher('/joint_states', JointState,queue_size=10)

done = False

while not done:
    # update the simulator and the robot
    simulator.step()    
    simulator.debug()
    robot.update()

    # command to the robot
    robot.setActuatedJointTorques(tau)

    #Implement a joint space PD controller of the form:
    if i<= 1: # q_hoem-> initial q_home_set-> end
        q_spline = pin.interpolate(model, q_home, q_home_set, i)
    else:
        q_spline = q_home_set
    tau = np.dot(Kp[-32:,-32:],(q_spline[-32:]-robot._q[-32:])) - np.dot(Kd[-32:,-32:],robot._v[-32:])

    i = i+0.001

    q = robot._q[-32:]
    qdot = robot._v[-32:]
    tau_use = tau[-32:]
    joint_state_msg = jointState_info(q,qdot,tau_use)
    joint_state_publisher.publish(joint_state_msg)
    # rate.sleep()
