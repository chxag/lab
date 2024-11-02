#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:32:51 2023

@author: stonneau
"""

import pinocchio as pin 
import numpy as np
from numpy.linalg import pinv,inv,norm,svd,eig
from tools import collision, getcubeplacement, setcubeplacement, projecttojointlimits
from config import LEFT_HOOK, RIGHT_HOOK, LEFT_HAND, RIGHT_HAND, EPSILON
from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET

from tools import setcubeplacement

def computeqgrasppose(robot, qcurrent, cube, cubetarget, viz=None):
    '''Return a collision free configuration grasping a cube at a specific location and a success flag'''
    setcubeplacement(robot, cube, cubetarget)
    #TODO implement
    for i in range(300):
        pin.framesForwardKinematics(robot.model, robot.data, qcurrent)
        pin.computeJointJacobians(robot.model, robot.data, qcurrent)

        left_hand = robot.model.getFrameId(LEFT_HAND)
        right_hand = robot.model.getFrameId(RIGHT_HAND)
        left_hook = robot.model.getFrameId(LEFT_HOOK)
        right_hook = robot.model.getFrameId(RIGHT_HOOK)

    #     print(left_hook, right_hook, left_hand, right_hand)

        oMleft_hand = robot.data.oMf[left_hand]
        oMright_hand = robot.data.oMf[right_hand]

    #     print(oMleft_hand, oMright_hand)

        oMleft_hook = getcubeplacement(cube, LEFT_HOOK)
        oMright_hook = getcubeplacement(cube, RIGHT_HOOK)

        error_left = oMleft_hand.inverse()*oMleft_hook
        error_right = oMright_hand.inverse()*oMright_hook

        error_left_vec = pin.log(error_left).vector
        error_right_vec = pin.log(error_right).vector

        error_both = np.vstack((error_left_vec, error_right_vec))
        error_both = error_both.reshape(-1)

        Jleft = pin.computeFrameJacobian(robot.model, robot.data, qcurrent, left_hand)
        Jright = pin.computeFrameJacobian(robot.model, robot.data, qcurrent, right_hand)

        Jtotal = np.vstack((Jleft, Jright))

        v_q = pinv(Jtotal)@error_both

#         if not collision(robot, qcurrent):
        qcurrent = pin.integrate(robot.model, qcurrent, v_q * 1e-2)
        updatevisuals(viz, robot, cube, qcurrent)
    
#     print ("TODO: implement me")
    return qcurrent, collision(robot, qcurrent)
    # distance between qcurrent and the goal < epsilon
#     return robot.q0, False
            
if __name__ == "__main__":
    from tools import setupwithmeshcat
    from setup_meshcat import updatevisuals
    robot, cube, viz = setupwithmeshcat()
    
    q = robot.q0.copy()
    
    q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
    qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)
    
    updatevisuals(viz, robot, cube, q0)
    
    
    
