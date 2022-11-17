import math
import numpy as np
import numpy.linalg as nplin
from collections import namedtuple

from ilknumpy import backend as backend

def ModelConstants():
    fr_elbow__fr_upper_arm = backend.pose()
    backend.setPosition(fr_elbow__fr_upper_arm,0.425,0.0,-0.1197)
    backend.setRotation(fr_elbow__fr_upper_arm,1.0,-0.0,0.0,
                                 0.0,1.0,-0.0,
                                 0.0,0.0,1.0)

    fr_shoulder_lift__fr_shoulder = backend.pose()
    backend.setPosition(fr_shoulder_lift__fr_shoulder,0.13585,0.0,0.0)
    backend.setRotation(fr_shoulder_lift__fr_shoulder,0.0,-0.0,1.0,
                                 -0.0,-1.0,0.0,
                                 1.0,-0.0,-0.0)

    fr_shoulder_pan__fr_base = backend.pose()
    backend.setPosition(fr_shoulder_pan__fr_base,0.0,0.0,0.089159)
    backend.setRotation(fr_shoulder_pan__fr_base,1.0,-0.0,0.0,
                                 0.0,1.0,-0.0,
                                 0.0,0.0,1.0)

    fr_wr1__fr_forearm = backend.pose()
    backend.setPosition(fr_wr1__fr_forearm,0.39225,0.0,0.09315)
    backend.setRotation(fr_wr1__fr_forearm,1.0,-0.0,0.0,
                                 0.0,1.0,-0.0,
                                 0.0,0.0,1.0)

    fr_wr2__fr_wrist_1 = backend.pose()
    backend.setPosition(fr_wr2__fr_wrist_1,0.09475,0.0,0.0)
    backend.setRotation(fr_wr2__fr_wrist_1,0.0,-0.0,1.0,
                                 -0.0,-1.0,0.0,
                                 1.0,-0.0,-0.0)

    fr_wr3__fr_wrist_2 = backend.pose()
    backend.setPosition(fr_wr3__fr_wrist_2,0.0825,0.0,0.0)
    backend.setRotation(fr_wr3__fr_wrist_2,-0.0,0.0,1.0,
                                 0.0,1.0,0.0,
                                 -1.0,0.0,-0.0)


    mc_config = namedtuple('mc', [
    'fr_elbow__fr_upper_arm',
    'fr_shoulder_lift__fr_shoulder',
    'fr_shoulder_pan__fr_base',
    'fr_wr1__fr_forearm',
    'fr_wr2__fr_wrist_1',
    'fr_wr3__fr_wrist_2',
    ])

    mc = mc_config(
    fr_elbow__fr_upper_arm = fr_elbow__fr_upper_arm,
    fr_shoulder_lift__fr_shoulder = fr_shoulder_lift__fr_shoulder,
    fr_shoulder_pan__fr_base = fr_shoulder_pan__fr_base,
    fr_wr1__fr_forearm = fr_wr1__fr_forearm,
    fr_wr2__fr_wrist_1 = fr_wr2__fr_wrist_1,
    fr_wr3__fr_wrist_2 = fr_wr3__fr_wrist_2,
    )
    return mc

def fk1(mc, q):

    fr_forearm__fr_elbow = backend.rot_z__a_x_b(q[2])
    fr_shoulder__fr_shoulder_pan = backend.rot_z__a_x_b(q[0])
    fr_upper_arm__fr_shoulder_lift = backend.rot_z__a_x_b(q[1])
    fr_wrist_1__fr_wr1 = backend.rot_z__a_x_b(q[3])
    fr_wrist_2__fr_wr2 = backend.rot_z__a_x_b(q[4])
    fr_wrist_3__fr_wr3 = backend.rot_z__a_x_b(q[5])

    fr_wrist_3__fr_wrist_2 = mc.fr_wr3__fr_wrist_2 @ fr_wrist_3__fr_wr3
    fr_wrist_3__fr_wr2 = fr_wrist_2__fr_wr2 @ fr_wrist_3__fr_wrist_2
    fr_wrist_3__fr_wrist_1 = mc.fr_wr2__fr_wrist_1 @ fr_wrist_3__fr_wr2
    fr_wrist_3__fr_wr1 = fr_wrist_1__fr_wr1 @ fr_wrist_3__fr_wrist_1
    fr_wrist_3__fr_forearm = mc.fr_wr1__fr_forearm @ fr_wrist_3__fr_wr1
    fr_wrist_3__fr_elbow = fr_forearm__fr_elbow @ fr_wrist_3__fr_forearm
    fr_wrist_3__fr_upper_arm = mc.fr_elbow__fr_upper_arm @ fr_wrist_3__fr_elbow
    fr_wrist_3__fr_shoulder_lift = fr_upper_arm__fr_shoulder_lift @ fr_wrist_3__fr_upper_arm
    fr_wrist_3__fr_shoulder = mc.fr_shoulder_lift__fr_shoulder @ fr_wrist_3__fr_shoulder_lift
    fr_wrist_3__fr_shoulder_pan = fr_shoulder__fr_shoulder_pan @ fr_wrist_3__fr_shoulder
    fr_wrist_3__fr_base = mc.fr_shoulder_pan__fr_base @ fr_wrist_3__fr_shoulder_pan



    return fr_wrist_3__fr_base


def fk2(mc, q):

    fr_forearm__fr_elbow = backend.rot_z__a_x_b(q[2])
    fr_shoulder__fr_shoulder_pan = backend.rot_z__a_x_b(q[0])
    fr_upper_arm__fr_shoulder_lift = backend.rot_z__a_x_b(q[1])

    fr_forearm__fr_upper_arm = mc.fr_elbow__fr_upper_arm @ fr_forearm__fr_elbow
    fr_forearm__fr_shoulder_lift = fr_upper_arm__fr_shoulder_lift @ fr_forearm__fr_upper_arm
    fr_forearm__fr_shoulder = mc.fr_shoulder_lift__fr_shoulder @ fr_forearm__fr_shoulder_lift
    fr_forearm__fr_shoulder_pan = fr_shoulder__fr_shoulder_pan @ fr_forearm__fr_shoulder
    fr_forearm__fr_base = mc.fr_shoulder_pan__fr_base @ fr_forearm__fr_shoulder_pan



    return fr_forearm__fr_base


def fk__ik1(mc, q):
    fr_forearm__fr_elbow = backend.rot_z__a_x_b(q[2])
    fr_shoulder__fr_shoulder_pan = backend.rot_z__a_x_b(q[0])
    fr_upper_arm__fr_shoulder_lift = backend.rot_z__a_x_b(q[1])
    fr_wrist_1__fr_wr1 = backend.rot_z__a_x_b(q[3])
    fr_wrist_2__fr_wr2 = backend.rot_z__a_x_b(q[4])
    fr_wrist_3__fr_wr3 = backend.rot_z__a_x_b(q[5])

    fr_shoulder_lift__fr_shoulder_pan = fr_shoulder__fr_shoulder_pan @ mc.fr_shoulder_lift__fr_shoulder
    fr_shoulder_lift__fr_base = mc.fr_shoulder_pan__fr_base @ fr_shoulder_lift__fr_shoulder_pan
    fr_elbow__fr_shoulder_lift = fr_upper_arm__fr_shoulder_lift @ mc.fr_elbow__fr_upper_arm
    fr_elbow__fr_base = fr_shoulder_lift__fr_base @ fr_elbow__fr_shoulder_lift
    fr_wr1__fr_elbow = fr_forearm__fr_elbow @ mc.fr_wr1__fr_forearm
    fr_wr1__fr_base = fr_elbow__fr_base @ fr_wr1__fr_elbow
    fr_wr2__fr_wr1 = fr_wrist_1__fr_wr1 @ mc.fr_wr2__fr_wrist_1
    fr_wr2__fr_base = fr_wr1__fr_base @ fr_wr2__fr_wr1
    fr_wr3__fr_wr2 = fr_wrist_2__fr_wr2 @ mc.fr_wr3__fr_wrist_2
    fr_wr3__fr_base = fr_wr2__fr_base @ fr_wr3__fr_wr2
    fr_wrist_3__fr_base = fr_wr3__fr_base @ fr_wrist_3__fr_wr3

    J_fr_wrist_3_fr_base = backend.np.zeros([6,6])
    poi_J_fr_wrist_3_fr_base = backend.positionView(fr_wrist_3__fr_base)

    backend.geometricJacobianColumn_revolute(
      poi_J_fr_wrist_3_fr_base,
      backend.positionView(mc.fr_shoulder_pan__fr_base),
      backend.zaxisView(mc.fr_shoulder_pan__fr_base),
      J_fr_wrist_3_fr_base[:,0] )
    backend.geometricJacobianColumn_revolute(
      poi_J_fr_wrist_3_fr_base,
      backend.positionView(fr_shoulder_lift__fr_base),
      backend.zaxisView(fr_shoulder_lift__fr_base),
      J_fr_wrist_3_fr_base[:,1] )
    backend.geometricJacobianColumn_revolute(
      poi_J_fr_wrist_3_fr_base,
      backend.positionView(fr_elbow__fr_base),
      backend.zaxisView(fr_elbow__fr_base),
      J_fr_wrist_3_fr_base[:,2] )
    backend.geometricJacobianColumn_revolute(
      poi_J_fr_wrist_3_fr_base,
      backend.positionView(fr_wr1__fr_base),
      backend.zaxisView(fr_wr1__fr_base),
      J_fr_wrist_3_fr_base[:,3] )
    backend.geometricJacobianColumn_revolute(
      poi_J_fr_wrist_3_fr_base,
      backend.positionView(fr_wr2__fr_base),
      backend.zaxisView(fr_wr2__fr_base),
      J_fr_wrist_3_fr_base[:,4] )
    backend.geometricJacobianColumn_revolute(
      poi_J_fr_wrist_3_fr_base,
      backend.positionView(fr_wr3__fr_base),
      backend.zaxisView(fr_wr3__fr_base),
      J_fr_wrist_3_fr_base[:,5] )

    return fr_wrist_3__fr_base, J_fr_wrist_3_fr_base


def ik1(mc, q, v):
    _, J = fk__ik1(mc, q)
    aux = J[3:3+3, 0:6]
    q_ik = backend.leastSquaresSolve(aux, v)
    return q_ik

def ik2(mc, cfg, desp, deso, q_guess):
    ep = cfg.eps_pos_err_norm*10
    eo = cfg.eps_or_err_norm*10

    q_ik = q_guess

    dbg = backend.ik_pos_dbg()
    R = np.zeros([3,3])
    ik_twist = np.zeros(6)
    angview  = backend.angularCoords( ik_twist )
    linview  = backend.linearCoords( ik_twist )
    while( ((ep > cfg.eps_pos_err_norm) or (eo > cfg.eps_or_err_norm)) and dbg.iter_count < cfg.max_iter):
        pose, Jacobian = fk__ik1(mc, q_ik)
        R = backend.rotationView(pose)
        ee_err_or  = backend.orientationDistance(deso, R)
        angview[:] = R @ (ee_err_or.axis * math.sin(ee_err_or.angle)/cfg.dt)
        ee_err_pos = desp - backend.positionView(pose)
        linview[:] = ee_err_pos / cfg.dt
        qd = backend.leastSquaresSolve(Jacobian, ik_twist)
        q_ik += qd * cfg.dt
        ep = np.linalg.norm(ee_err_pos)
        eo = abs(ee_err_or.angle)
        dbg.iter_count = dbg.iter_count + 1

    dbg.actual_pos = backend.positionView(pose)
    dbg.actual_or  = backend.rotationView(pose)
    return q_ik, dbg

