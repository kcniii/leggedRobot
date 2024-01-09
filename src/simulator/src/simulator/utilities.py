"""simulator
Robot base class (connection pybullet & pinocchio)
Author: Simon Armleder
"""
import numpy as np
import pybullet as pb

import eigenpy as ei
import pinocchio as pin

from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms

################################################################################
# spatial geometry for pybullet
################################################################################

def quaternionToMatrix(Q):
    '''
    Quaternion [x,y,z,w] numpy array to rotation matrix
    '''
    return np.array(pb.getMatrixFromQuaternion(Q)).reshape((3, 3))

def matrixToQuaternion(R):
    '''
    rotation matrix to Quaternion [x,y,z,w]
    '''
    Q = ei.Quaterniond(ei.Matrix3d(R))
    return np.array([Q.x(), Q.y(), Q.z(), Q.w()])

def quaternionToEuler(Q):
    '''
    Quaternion to euler numpy array [x,y,z]
    '''
    return np.array(pb.getEulerFromQuaternion(Q))

def eulerToQuaternion(eul):
    '''
    Euler [x,y,z] to quaternion numpy array
    '''
    return np.array(pb.getQuaternionFromEuler(eul))

def inverseTransform(t_A_B, Q_A_B):
    '''
    get the inverse transformation
    '''
    t_B_A, Q_B_A = pb.invertTransform(t_A_B, Q_A_B)
    return np.array(t_B_A), np.array(Q_B_A)

def transformPoint(t_A_B, Q_A_B, p_A):
    '''
    Transforms point frame A into frame B
    '''    
    p_B, Q = pb.multiplyTransforms(t_A_B, Q_A_B, p_A, [0,0,0,1])
    return np.array(p_B)

def transformFrame(t_A_B, Q_A_B, p_A, Q_A):
    '''
    Transforms frame A into frame B
    '''    
    p_B, Q_B = pb.multiplyTransforms(t_A_B, Q_A_B, p_A, Q_A)
    return np.array(p_B), np.array(Q_B)

def motionVecRotation(R):
    '''
    stack rotation matrices on main diagonal twice
    '''
    return np.kron(np.eye(2), R)

################################################################################
# pinocchio
################################################################################

def errorSE3(X, X_des):
    """Compute motion vector between two transformations, 
    note: result expressed in their parent frames
    """
    error = pin.log(X_des.inverse () * X)
    return error

def interpolateSE3(X_init, X_goal, s):
    """Interpolate  
    """
    X_inter = X_init * pin.exp6( s * pin.log6(X_init.inverse()*X_goal))
    return X_inter

def polyActivation(x, x0, x1, y0, y1):
    """polynomial activation function
    """
    t = (x - x0)/(x1 - x0)
    s = min(1, max(0, -2*t*t*t + 3*t*t))
    return (y1 - y0)*s + y0

################################################################################
# plotting
################################################################################

def plot(ax, x, y, xAxisLabel=None, yAxisLabel=None, **kwargs):
    line = ax.plot(x, y, **kwargs)
    if xAxisLabel is not None: 
        ax.set_xlabel(xAxisLabel)
    if yAxisLabel is not None:
        ax.set_ylabel(yAxisLabel)
    ax.grid(True)
    if 'label' in kwargs:
        legend = ax.legend()
        legend.get_frame().set_alpha(0.5)
    return line

def scatter(ax, x, y, xAxisLabel=None, yAxisLabel=None, **kwargs):
    line = ax.scatter(x, y, **kwargs)
    if xAxisLabel is not None: 
        ax.set_xlabel(xAxisLabel)
    if yAxisLabel is not None:
        ax.set_ylabel(yAxisLabel)
    ax.grid(True)
    if 'label' in kwargs:
        legend = ax.legend()
        legend.get_frame().set_alpha(0.5)
    return line

def savePlot(fig, path, figsize=[12,9], dpi=300, **kwargs):
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(path, figsize=figsize, dpi=dpi, **kwargs)

def confidence_ellipse_2d(x, y, ax, n_std=[3.0], facecolor='none', **kwargs):
    if x.size != y.size:
        raise ValueError("x and y must be the same size")

    cov = np.cov(x, y)
    pearson = cov[0, 1]/np.sqrt(cov[0, 0] * cov[1, 1])
    ell_radius_x = np.sqrt(1 + pearson)
    ell_radius_y = np.sqrt(1 - pearson)

    mean_x = np.mean(x)
    mean_y = np.mean(y)

    for s in n_std:
        ellipse = Ellipse((0, 0), width=ell_radius_x * 2, height=ell_radius_y * 2,
                        facecolor=facecolor, **kwargs)

        scale_x = np.sqrt(cov[0, 0]) * s
        scale_y = np.sqrt(cov[1, 1]) * s

        transf = transforms.Affine2D() \
            .rotate_deg(45) \
            .scale(scale_x, scale_y) \
            .translate(mean_x, mean_y)

        ellipse.set_transform(transf + ax.transData)
        ax.add_patch(ellipse)