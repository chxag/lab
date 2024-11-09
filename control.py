#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:32:51 2023

@author: stonneau
"""

import numpy as np
import quadprog
from bezier import Bezier
    
# in my solution these gains were good enough for all joints but you might want to tune this.
Kp = 300.               # proportional gain (P of PD)
Kv = 2 * np.sqrt(Kp)   # derivative gain (D of PD)

def controllaw(sim, robot, trajs, tcurrent, cube):
    q, vq = sim.getpybulletstate()
    #TODO 
    torques = [0.0 for _ in sim.bulletCtrlJointsInPinOrder]
    sim.step(torques)

def to_quadratic_form(C, c):
    return np.dot(C.T, C), -np.dot(C.T, c)

def quadprog_solve_qp(H, q, G=None, h=None, C=None, d=None, verbose=False):
    """
    min (1/2)x' Px + q'x
    s.t. G x <= h
         C x  = d
    """

    qp_G = 0.5 * (H + H.T)
    qp_a = -q
    qp_C = None
    qp_b = None
    meq = 0
    if C is not None:
        if G is not None:
            qp_C = -np.vstack([C, G]).T
            qp_b = -np.hstack([d, h])
        else:
            qp_C = -C.transpose()
            qp_b = -d
        meq = C.shape[0]
    elif G is not None:
        qp_C = -G.T
        qp_b = -h
    res = quadprog.solve_qp(qp_G, qp_a, qp_C, qp_b, meq)
    if verbose: 
        return res
    return res[0]

def pi(p0, p1, ti):
    ti2 = ti * ti
    ti3 = ti2*ti
    di = ti3 - 2* ti2 + ti
    ei = ti3 - ti2
    bi = (2*ti3 - 3*ti2 + 1) * p0 + (3*ti2 - 2*ti3) * p1
    Ai = np.diag(np.array[di,di,di,ei,ei,ei])
    return (Ai, bi)

def jointlimitconstrant(p0, p1, ti):    
    q_min = robot.model.upperPositionLimit
    q_max = robot.model.lowerPositionLimit
    (Ai, bi) = pi(p0, p1, ti)
    bmin = bi - q_min
    Amin = -Ai
    bmax = q_max - bi
    Amax = Ai.copy()
    A = np.vstack([Amin, Amax])
    b = np.concatenate([bmin, bmax])
    return (A,b)

def stack_joint_limit_constraints(p0, p1,T):
    A = np.zeros((0,9))
    b = np.zeros(0)

    for i in range(1, T):
        ti = 1. / (T * i)
        Ai, bi = jointlimitconstrant(p0, p1, ti)
        A = np.vstack([A, Ai])
        b = np.concatenate([b, bi])
    return A,b

    
#TODO this is just an example, you are free to do as you please.
#In any case this trajectory does not follow the path 
#0 init and end velocities
def maketraj(q0,q1,T): #TODO compute a real trajectory !
    n_cols = 9
    l = np.zeros(n_cols)

    cs = [q0] + [np.zeros(3) for _ in range(2)] + [q1]
    Cs = []

    C = np.zeros((3, n_cols))
    C[0,0] = 1
    C[1,1] = 1
    C[2,2] = 1
    Cs.append(C)

    for i in range(2):
        C = np.zeros((3, n_cols)) 
        C[0, (i+1)*3] = 1
        C[1, (i+1)*3+1] = 1
        C[2, (i+1)*3+2] = 1
        C[0, (i)*3] = -1
        C[1, (i)*3+1] = -1
        C[2, (i)*3+2] = -1
        Cs.append(C)
    
    C = np.zeros((3, n_cols))
    C[0,0] = 1
    C[1,1] = 1
    C[2,2] = 1

    H = np.zeros((n_cols, n_cols))
    hessians = [to_quadratic_form(C, c) for (C, c) in zip(Cs, cs)]
    Hl = [sum(x) for x in zip(*hessians)]
    H = Hl[0], l = Hl[1]

    A,b = stack_joint_limit_constraints(q0, q1, T)

    x = quadprog_solve_qp(H, l, A, b)

    control_points = [x[0:3], x[3:6], x[6:9]]
    q_of_t = Bezier(control_points,t_max=T)
    vq_of_t = q_of_t.derivative(1)
    vvq_of_t = vq_of_t.derivative(1)
    return q_of_t, vq_of_t, vvq_of_t


if __name__ == "__main__":
        
    from tools import setupwithpybullet, setupwithpybulletandmeshcat, rununtil
    from config import DT
    
    robot, sim, cube = setupwithpybullet()
    
    
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET    
    from inverse_geometry import computeqgrasppose
    from path import computepath
    
    q0,successinit = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT, None)
    qe,successend = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET,  None)
    path = computepath(robot,cube,None,q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)

    
    #setting initial configuration
    sim.setqsim(q0)
    
    
    #TODO this is just a random trajectory, you need to do this yourself
    total_time=4.
    trajs = maketraj(q0, qe, total_time)   
    
    tcur = 0.
    
    
    while tcur < total_time:
        rununtil(controllaw, DT, sim, robot, trajs, tcur, cube)
        tcur += DT
    
    
    