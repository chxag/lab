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
Kp = 100000
Kv = 2 * np.sqrt(Kp)   # derivative gain (D of PD)

# proportional gain (P of PD)
def controllaw(sim, robot, trajs, tcurrent, cube):
    q, vq = sim.getpybulletstate()
    q_t, vq_t, aq_t = trajs
    
    aq = aq_t(tcurrent) + Kp * (q_t(tcurrent) - q) + Kv * (vq_t(tcurrent) - vq)
    
    fc_value = -250
    
    fc_value_z = -200
    
    fc = np.array([0, fc_value, fc_value_z, 0, 0, 0])
    
    pin.computeJointJacobians(robot.model, robot.data, q)
    
    left_hand = robot.model.getFrameId(LEFT_HAND)
    right_hand = robot.model.getFrameId(RIGHT_HAND)
    
    Jleft = pin.computeFrameJacobian(robot.model, robot.data, q, left_hand)
    Jright = pin.computeFrameJacobian(robot.model, robot.data, q, right_hand)
    
    Jtotal = np.vstack((Jleft, Jright))
    fcM = np.hstack((fc, fc))
    
    fcJ = np.dot(Jtotal.T, fcM)
    
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
    import matplotlib.pyplot as plt
    
    q0,successinit = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT, None)
    qe,successend = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET,  None)
    path = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)

    
    new_path = []
    n = len(path)
    new_path.append(path[0]) 
    new_path.append(path[0]) 
    new_path.append(path[0]) 
    for row in path: 
        new_path.append(row)
        new_path.append(row)
        new_path.append(row)
        new_path.append(row)
        new_path.append(row)
        new_path.append(row)
        new_path.append(row)
        new_path.append(row)
#         new_path.append(row)
    new_path.append(path[n-1])
    new_path.append(path[n-1]) 
    new_path.append(path[n-1])
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
    
    fig, axs = plt.subplot(4,1,figsize=(10,15))
    
    positions = np.array(q_of_t)
    velocities = np.array(vq_of_t)
    accelerations = vvq_of_t
    
    axs[0].plot(positions[:, 0], positions[:, 1], label='Position Trajectory', color='b')
    axs[0].set_title('Trajectory')
    axs[0].set_xlabel('X Position')
    axs[0].set_ylabel('Y Position')
    axs[0].grid()
    axs[0].legend()
    
    plt.tight_layout()
    plt.show()
    
