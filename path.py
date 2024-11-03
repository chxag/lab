#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 21 11:44:32 2023

@author: stonneau
"""

import pinocchio as pin
import numpy as np
import random
from numpy.linalg import pinv
# from tools import 

from config import LEFT_HAND, RIGHT_HAND
import time

#returns a collision free path from qinit to qgoal under grasping constraints
#the path is expressed as a list of configurations
def computepath(qinit,qgoal,cubeplacementq0, cubeplacementqgoal):
    #TODO
    #sampling configurations
    
    #Constrain np.rand() to 3.2, -3.2
    #Calculate linear interpolation between cube init placement and cube_rand
    #Discretise, and cmpute grap pose at each step until you get grasp pose that isn't valid
    #Take the grasp pose that is valid before the unvalid one 
    
    # first get random configuration of the cube with constraints in consideration
    G = [(None, qinit)]
    
    rotation = cubeplacementq0.rotation
    translation = cubeplacementq0.translation
    
    rotation_goal = cubeplacementqgoal.rotation
    translation_goal = cubeplacementqgoal.translation
    
    sample_positions = set()
    
    k = 100 #iteration number
    delta_q = 3.
    min_dist = float('inf')
    
    discretisationsteps_newconf = 20
    discretisationsteps_validedge = 20
    
    
    
    idx = -1
    
    
    for _ in range(k):
        
        while True:
            #bias towards the goal config
            b = np.random.uniform(0.0, 1.) # adjust the value to control the bias
            
            #cube position
            cube_x_rand = (1-b) * translation[0] + b * translation_goal[0] + np.random.uniform(-0.1,0.1)
            cube_y_rand = (1-b) * translation[1] + b * translation_goal[1] + np.random.uniform(-0.1,0.1)
            cube_z_rand = translation[2]

            cube_rand_translation = np.array([cube_x_rand, cube_y_rand, cube_z_rand])
            cube_q_rand = pin.SE3(rotation, cube_rand_translation)
            
            position_tuple = (cube_x_rand, cube_y_rand, cube_z_rand)
            if position_tuple not in sample_positions:
                sample_positions.add(position_tuple)
                break

        q_rand, success = computeqgrasppose(robot, qinit, cube, cube_q_rand, viz)
        
        if not success:
            continue
        
        #nearest vertex
        for (i, node) in enumerate(G):
            dist = np.linalg.norm(q_rand - node[1])
            if dist < min_dist:
                min_dist = dist
                idx = i
        near_idx = idx
        q_near = G[near_idx][1]
        
        #new configuration
        q_end = q_rand.copy()
        dist = np.linalg.norm(q_rand - q_near)
        if delta_q is not None and dist > delta_q:
            q_end = q_near * (1 - delta_q/dist) + q_rand * (delta_q/dist)
            dist = delta_q
        
        dt = dist / discretisationsteps_newconf
        
        for n in range(1, discretisationsteps_newconf):
            q_new = q_near * (1 - dt*n) + q_end * (dt*n)
            q_new, success = computeqgrasppose(robot, qinit, cube, cube_q_rand, viz)
            if success==False:
                q_new = q_near * (1 - dt*(n-1)) + q_end * (dt*(n-1))
            else:
                q_new = q_end
        
        G += [(near_idx, q_new)]
        
        #valid edge check
        q_end_check = qgoal.copy()
        dist_check = np.linalg.norm(qgoal - q_new)
        if delta_q is not None and dist_check > delta_q:
            q_end_check = q_new * (1-delta_q/dist_check) + qgoal*(delta_q/dist_check)
            dist_check = delta_q
        dt_check = dist_check / discretisationsteps_validedge
        for i in range(1, discretisationsteps_validedge):
            q = q_new * (1-dt_check * i) + q_end_check * (dt_check * i)
            q, success_two = computegrasppose(robot, qinit, cube, cube_q_rand, viz)
            if success_two == False:
                q_new_check = q_new * (q-dt_check * (i-1)) + q_end_check * (dt_check * (i-1))    
            else:
                q_new_check = q_end_check
                
        if np.linalg.norm(qgoal - q_new_check) < 1e-3:
            print("Path found!")
            path += [(len(G) - 1, qgoal)]
            break 
    	
    return G
    
    #return [qinit, qgoal]
    #pass


def displaypath(robot,path,dt,viz):
    for q in path:
        viz.display(q)
        time.sleep(dt)


if __name__ == "__main__":
    from tools import setupwithmeshcat
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
    from setup_meshcat import updatevisuals #was not imported initially
    from inverse_geometry import computeqgrasppose
    
    robot, cube, viz = setupwithmeshcat()
    
    
    q = robot.q0.copy()
    q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
    qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)
    
    if not(successinit and successend):
        print ("error: invalid initial or end configuration")
    
    path = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)
    
    displaypath(robot,path,dt=0.5,viz=viz) #you ll probably want to lower dt
    
