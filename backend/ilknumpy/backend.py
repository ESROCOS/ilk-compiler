import math
import numpy as np
from collections import namedtuple

def pose():
    return np.identity(4)


class ik_pos_dbg:
    def __init__(self):
        self.iter_count = 0
        self.actual_pos = np.zeros((3,1))
        self.actual_or  = np.identity(3)


ik_pos_cfg_t = namedtuple('ik_pos_cfg',
    ['dt', 'eps_pos_err_norm', 'eps_or_err_norm', 'max_iter',
     'ls_damping' # damped-least-squares parameter
    ])

def ik_pos_cfg(**kwargs):
    kwargs.setdefault('dt', 0.004)
    kwargs.setdefault('eps_pos_err_norm', 1e-6)
    kwargs.setdefault('eps_or_err_norm' , 1e-6)
    kwargs.setdefault('max_iter', 100)
    kwargs.setdefault('ls_damping', 0.05)
    return ik_pos_cfg_t(
        dt = kwargs['dt'],
        eps_pos_err_norm = kwargs['eps_pos_err_norm'],
        eps_or_err_norm  = kwargs['eps_or_err_norm'],
        max_iter = kwargs['max_iter'],
        ls_damping = kwargs['ls_damping']
    )



def setPosition(pose, x,y,z):
    pose[0,3] = x
    pose[1,3] = y
    pose[2,3] = z

def setRotation(pose, xx, xy, xz, yx, yy, yz, zx, zy, zz):
    pose[0,0] = xx; pose[0,1] = xy; pose[0,2] = xz;
    pose[1,0] = yx; pose[1,1] = yy; pose[1,2] = yz;
    pose[2,0] = zx; pose[2,1] = zy; pose[2,2] = zz;


def positionView(pose):
    return pose[0:3,3]

def rotationView(pose):
    return pose[0:3,0:3]

def zaxisView(pose):
    return pose[0:3,2]


def rot_z__a_x_b(angle):
    mx = np.identity(4)
    s = math.sin(angle);
    c = math.cos(angle);

    mx[0,0] = c;
    mx[0,1] = -s;
    mx[1,0] = s;
    mx[1,1] = c;
    return mx

def rot_z__b_x_a(angle):
    mx = np.identity(4)
    s = math.sin(angle);
    c = math.cos(angle);

    mx[0,0] = c;
    mx[0,1] = s;
    mx[1,0] = -s;
    mx[1,1] = c;
    return mx

def tr_z__a_x_b(length):
    mx = np.identity(4)
    mx[2,3] = length
    return mx

def tr_z__b_x_a(length):
    mx = np.identity(4)
    mx[2,3] = -length
    return mx


class AxisAngle :
    def __init__(self, **kwargs):
        kwargs.setdefault('axis' , np.array([1.0, 0.0, 0.0]))
        kwargs.setdefault('angle', 0.0)
        self.axis  = kwargs['axis'] / np.linalg.norm(kwargs['axis'])
        self.angle = kwargs['angle']

    def omega() :
        return axis*angle

'''
 The difference between two rotation matrices, as an axis-angle.
 @return the rotation required to go from the second to the first argument,
         that is, the first "minus" the second.
'''
def orientationDistance(_R_desired, _R_actual) :
    thresh = 1e-6   # TODO #magic-number

    # TODO checks on the size 3x3
    R = _R_actual.transpose() @ _R_desired   #  this is 'actual_R_desired'

    x = R[2,1] - R[1,2]
    y = R[0,2] - R[2,0]
    z = R[1,0] - R[0,1]
    norm = math.sqrt(x*x + y*y + z*z)
    if norm < thresh :
        return AxisAngle( axis=np.array([0.0, 0.0, 1.0]), angle=0.0) # arbitrary choice of (0,0,1) axis

    theta = math.atan2( norm, R.trace()-1 )
    return AxisAngle(axis=np.array([x/norm, y/norm, z/norm]), angle=theta)
    # TODO check corner cases, theta close to 0/PI, bad numerical


def linearCoords(_6dvector):
    return _6dvector[3:6]

def angularCoords(_6dvector):
    return _6dvector[0:3]

def geometricJacobianColumn_revolute(poi, jointOrigin, jointAxis, column):
    angularCoords(column)[:] = jointAxis
    linearCoords (column)[:] = np.cross(jointAxis, poi - jointOrigin )

def geometricJacobianColumn_prismatic(jointAxis, column):
    angularCoords(column)[:] = np.zeros((3,1))
    linearCoords (column)[:] = jointAxis


def leastSquaresSolve(A, b):
    U, Sigma, VT = np.linalg.svd(A, full_matrices=False)
    return (VT.transpose() * (1/Sigma[..., None, :]) ) @ U.transpose() @ b



def ct_twist(newframe_H_current, twist_current, twist_newframe) :
    ## We need to perform a spatial motion vector coordinate transform, given
    ## the homogeneous coordinate transform for the same frames.
    ## Check e.g. chapter 2 of Roy's RBDA, for the relation between the two
    ## transforms, to understand the last line of this function

    R = rotationView( newframe_H_current )      # 3x3 rotation matrix
    w = angularCoords( twist_current )          # omega in the current coordinates
    angularCoords( twist_newframe )[:] = R @ w  # omega in the new coordinates
    linearCoords ( twist_newframe )[:] = R @ linearCoords( twist_current ) +\
                           np.cross( positionView( newframe_H_current),
                                  angularCoords( twist_newframe )    )

