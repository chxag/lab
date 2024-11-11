#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:32:51 2023

@author: stonneau
"""

import numpy as np
import pinocchio as pin

from bezier import Bezier
from config import LEFT_HAND, RIGHT_HAND

from path import computepath
    
# in my solution these gains were good enough for all joints but you might want to tune this.
Kp = 20000          # proportional gain (P of PD)
Kv = 2 * np.sqrt(Kp)   # derivative gain (D of PD)


def controllaw(sim, robot, trajs, tcurrent, cube):
    q, vq = sim.getpybulletstate()
    q_t, vq_t, aq_t = trajs
    
    aq = aq_t(tcurrent) + Kp * (q_t(tcurrent) - q) + Kv * (vq_t(tcurrent) - vq)
    
    fc_value = 55 #think its too week it goes further than the obstacle (no go over obstacle)
    
    fc = np.array([0, fc_value, 0, 0, 0, 0])
    
    pin.computeJointJacobians(robot.model, robot.data, q)
    
    left_hand = robot.model.getFrameId(LEFT_HAND)
    right_hand = robot.model.getFrameId(RIGHT_HAND)
    
    Jleft = pin.computeFrameJacobian(robot.model, robot.data, q, left_hand)
    print(Jleft)
    Jright = pin.computeFrameJacobian(robot.model, robot.data, q, right_hand)
        
    fcJleft = np.dot(Jleft.T, fc)
    fcJright = np.dot(Jright.T, fc)
    
    fcJ = fcJleft + fcJright
    
    #TODO 
    torques = pin.rnea(robot.model, robot.data, q, vq, aq) + fcJ
    sim.step(torques)
    
    return torques
    

    

if __name__ == "__main__":
        
    from tools import setupwithpybullet, setupwithpybulletandmeshcat, rununtil
    from config import DT
    
    robot, sim, cube = setupwithpybullet()
    
    
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET    
    from inverse_geometry import computeqgrasppose
    from path import computepath
    
    q0,successinit = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT, None)
    qe,successend = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET,  None)
    path = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)
    
#     original path
#     path = np.array([[-4.88294048e-01,  3.92096586e-17,  1.95172965e-17, -7.16814094e-01,
#         7.19954859e-02, -2.97292784e-01,  7.47902815e-18,  2.25297298e-01,
#        -2.88658422e-01, -2.97808375e-02, -3.02837503e-01,  1.10090761e-01,
#         3.80500469e-18,  1.92746742e-01,  2.01334901e+00], [ 1.68545355e-01, -2.03731804e-13, -2.60716346e-13, -9.53545905e-01,
#        -2.62740450e-01, -1.08665691e-01,  1.11209026e-02,  3.49415748e-01,
#        -7.89525653e-01, -5.03413243e-01, -1.04480256e-01, -2.90349384e-01,
#         7.85567093e-03,  3.72762519e-01,  1.90410369e+00], [ 2.51624388e+00, -6.95598328e-15,  4.11091675e-14, -1.57079630e+00,
#        -1.89429829e+00,  1.57079630e+00,  5.85392303e-01, -7.07977916e-01,
#        -1.35670924e+00, -1.57079600e+00, -1.27779000e+00,  1.47995699e+00,
#        -2.23825879e-03, -2.02755023e-01,  2.41696125e+00], [-3.73319419e-01,  2.68157638e-14,  6.44210210e-14, -4.41343299e-01,
#        -2.16082942e+00,  1.57079630e+00,  1.74029460e-01, -1.15577018e+00,
#        -5.63973004e-01,  1.53875241e-01, -2.10830533e+00,  1.57079600e+00,
#         4.12849455e-02, -9.54237641e-01,  1.83733338e+00], [-7.88301102e-01, -1.06065982e-13, -7.45300567e-14, -4.12061751e-01,
#        -2.14761742e+00,  1.57079630e+00, -1.10737677e+00, -1.30642434e+00,
#        -1.49601540e+00,  5.76352641e-01, -1.92120926e+00,  1.57079600e+00,
#         3.17603099e-02, -7.45570900e-01,  1.82570359e+00], [-5.35899062e-01,  4.95657845e-12, -2.65840866e-12, -4.00359578e-01,
#        -2.10096921e+00,  1.57079630e+00, -3.97871646e-01, -1.15165688e+00,
#        -1.05890412e+00,  3.33555274e-01, -1.99332410e+00,  1.57079600e+00,
#        -1.19819019e-04, -7.28638287e-01,  1.77677651e+00], [-3.36451974e-01,  4.53336854e-14,  2.57981985e-14, -3.43305793e-01,
#        -2.14157678e+00,  1.57079630e+00,  1.13232421e-01, -9.78759946e-01,
#        -7.65356791e-01,  1.58149232e-01, -2.05256457e+00,  1.57079600e+00,
#         3.45628311e-02, -7.95704204e-01,  1.78886597e+00], [-2.41287441e-02,  2.09632007e-14,  4.18380453e-14, -5.01467781e-02,
#        -9.57246178e-01,  1.67610085e-01,  1.63599118e-04,  7.89010205e-01,
#        -1.49663205e+00,  3.65936795e-01, -9.84059716e-01,  2.13595120e-01,
#         4.34550486e-04,  7.66101266e-01,  1.23027844e+00], [ 1.78220357e-01, -4.12995307e-18,  3.53275974e-17, -1.60879641e-01,
#        -3.32635590e-02, -1.93582824e-01,  2.04782991e-18,  2.26846383e-01,
#        -1.51110728e+00,  4.47243211e-01,  1.31672487e-01, -3.52361825e-01,
#         2.40708438e-18,  2.20689338e-01,  8.69810557e-01]])

#     path = np.array([[-4.88294048e-01,  3.92096586e-17,  1.95172965e-17, -7.16814094e-01,
#     7.19954859e-02, -2.97292784e-01,  7.47902815e-18,  2.25297298e-01,
#    -2.88658422e-01, -2.97808375e-02, -3.02837503e-01,  1.10090761e-01,
#     3.80500469e-18,  1.92746742e-01,  2.01334901e+00], [-4.88294048e-01,  3.92096586e-17,  1.95172965e-17, -7.16814094e-01,
#     7.19954859e-02, -2.97292784e-01,  7.47902815e-18,  2.25297298e-01,
#    -2.88658422e-01, -2.97808375e-02, -3.02837503e-01,  1.10090761e-01,
#     3.80500469e-18,  1.92746742e-01,  2.01334901e+00], [-3.98072072e-01,  1.34890117e-17,  2.37053593e-17, -5.96296231e-01,
#    -7.57522333e-02, -3.62259495e-01,  1.58590821e-17,  4.38011729e-01,
#    -5.76246431e-01,  3.40648096e-02, -4.43108153e-01,  7.93590026e-02,
#     1.00118383e-17,  3.63749150e-01,  1.93620712e+00], [-3.98072072e-01,  1.34890117e-17,  2.37053593e-17, -5.96296231e-01,
#    -7.57522333e-02, -3.62259495e-01,  1.58590821e-17,  4.38011729e-01,
#    -5.76246431e-01,  3.40648096e-02, -4.43108153e-01,  7.93590026e-02,
#     1.00118383e-17,  3.63749150e-01,  1.93620712e+00], [-2.02296909e-01,  4.12168077e-12,  1.87246183e-12, -2.00831178e-01,
#    -1.04507004e+00,  5.38399386e-01, -4.84988239e-05,  5.06664244e-01,
#    -1.16778838e+00,  1.64788708e-01, -1.41131287e+00,  1.17163753e+00,
#    -5.91199028e-04,  2.34171139e-01,  1.60871495e+00], [-2.02296909e-01,  4.12168077e-12,  1.87246183e-12, -2.00831178e-01,
#    -1.04507004e+00,  5.38399386e-01, -4.84988239e-05,  5.06664244e-01,
#    -1.16778838e+00,  1.64788708e-01, -1.41131287e+00,  1.17163753e+00,
#    -5.91199028e-04,  2.34171139e-01,  1.60871495e+00], [-1.51229954e-01, -1.33981776e-13, -3.92965115e-13, -2.43041956e-01,
#    -1.32098183e+00,  7.17466326e-01, -9.31134072e-19,  6.03515507e-01,
#    -1.17651900e+00,  1.18766386e-01, -1.80660423e+00,  1.57079600e+00,
#    -1.87517515e-04,  2.18051826e-01,  1.60488300e+00], [-1.51229954e-01, -1.33981776e-13, -3.92965115e-13, -2.43041956e-01,
#    -1.32098183e+00,  7.17466326e-01, -9.31134072e-19,  6.03515507e-01,
#    -1.17651900e+00,  1.18766386e-01, -1.80660423e+00,  1.57079600e+00,
#    -1.87517515e-04,  2.18051826e-01,  1.60488300e+00], [-1.74880046e-01, -3.91637734e-14, -6.16069980e-14,  2.81407784e-01,
#    -2.26244627e+00,  1.57079630e+00, -4.64262572e-03, -1.59420289e-01,
#    -1.67821526e+00,  6.15247718e-01, -2.02780664e+00,  1.57079600e+00,
#    -4.25615801e-03, -6.17225315e-04,  1.13227983e+00], [-1.74880046e-01, -3.91637734e-14, -6.16069980e-14,  2.81407784e-01,
#    -2.26244627e+00,  1.57079630e+00, -4.64262572e-03, -1.59420289e-01,
#    -1.67821526e+00,  6.15247718e-01, -2.02780664e+00,  1.57079600e+00,
#    -4.25615801e-03, -6.17225315e-04,  1.13227983e+00], [ 1.78220357e-01, -4.12995307e-18,  3.53275974e-17, -1.60879641e-01,
#    -3.32635590e-02, -1.93582824e-01,  2.04782991e-18,  2.26846383e-01,
#    -1.51110728e+00,  4.47243211e-01,  1.31672487e-01, -3.52361825e-01,
#     2.40708438e-18,  2.20689338e-01,  8.69810557e-01], [ 1.78220357e-01, -4.12995307e-18,  3.53275974e-17, -1.60879641e-01,
#    -3.32635590e-02, -1.93582824e-01,  2.04782991e-18,  2.26846383e-01,
#    -1.51110728e+00,  4.47243211e-01,  1.31672487e-01, -3.52361825e-01,
#     2.40708438e-18,  2.20689338e-01,  8.69810557e-01]])
    
    new_path = []
    for row in path:
        new_path.append(row)
        new_path.append(row)
    new_path = np.array(new_path)
    
    
    #setting initial configuration
    sim.setqsim(q0)
    
    #TODO this is just an example, you are free to do as you please.
    #In any case this trajectory does not follow the path 
    #0 init and end velocities
    def maketraj(q0,q1,T): #TODO compute a real trajectory !
            
        q_of_t = Bezier(new_path, t_max=T)
        vq_of_t = q_of_t.derivative(1)
        vvq_of_t = vq_of_t.derivative(1)
        return q_of_t, vq_of_t, vvq_of_t
    
    #TODO this is just a random trajectory, you need to do this yourself
    total_time=4. #tried 3 and 5 but taking too long. What does this mean?
    trajs = maketraj(q0, qe, total_time)   
    
    tcur = 0.
    
    q_of_t, vq_of_t, vvq_of_t = trajs
    
    
    while tcur < total_time:
        rununtil(controllaw, DT, sim, robot, trajs, tcur, cube)
        tcur += DT
    
    
    
