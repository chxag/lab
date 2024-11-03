#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 21 11:44:32 2023

@author: stonneau
"""

import pinocchio as pin
import numpy as np
from numpy.linalg import pinv

from config import LEFT_HAND, RIGHT_HAND
import time

#returns a collision free path from qinit to qgoal under grasping constraints
#the path is expressed as a list of configurations
def computepath(qinit,qgoal,cubeplacementq0, cubeplacementqgoal):
    #TODO
    discretisationsteps_newconf = 200 
    discretisationsteps_validedge = 200 
    k = 1000
    delta_q = 3.
    min_dist = 10e4
    idx = -1
    
    G = [(None,qinit)]
    
    rotation = cubeplacementq0.rotation
    sampled_positions = set()
    goal_bias = 0.1
    
    for iteration in range(k):
        
        # Sampling configurations for the cube 
        if np.random.uniform() < goal_bias:
            cube_x_rand = cubeplacementqgoal.translation[0]
            cube_y_rand = cubeplacementqgoal.translation[1]
            cube_z_rand = cubeplacementqgoal.translation[2]
        else:
            cube_x_rand = np.random.uniform(0, 0.5)
            cube_y_rand = np.random.uniform(-0.2, 0.2)
            cube_z_rand = 0.93 

        cube_rand_translation = np.array([cube_x_rand, cube_y_rand, cube_z_rand])
        cube_q_rand = pin.SE3(rotation, cube_rand_translation)
        position_tuple = (cube_x_rand, cube_y_rand, cube_z_rand) 
        if position_tuple not in sampled_positions: 
            continue
        sampled_positions.add(position_tuple)
        print(f"Sampled position: {cube_rand_translation}")

        # Generating valid pose for the randomly sampled cube position 
        q_rand, success = computeqgrasppose(robot, qinit, cube, cube_q_rand, viz)
        print(f"Sampled configuration: {q_rand}, success: {success}")
        
        if success:
            continue
        
        # Find the nearest vertex to q_rand called q_near 
        for (i,node) in enumerate(G):
            dist = np.linalg.norm(q_rand - node[1])
            if dist < min_dist: 
                min_dist = dist
                idx = i
        q_near_index = idx
        q_near = G[q_near_index][1]
        
        print(f"Nearest vertex: {q_near}")

    # Return the closest configuration q_new such that the path q_near => q_new is the longest 
    # along the linear interpolation (q_near,q_rand) that is collision free and of length <  delta_q
        q_end = q_rand.copy()
        dist_two = np.linalg.norm(q_rand - q_near)
        if delta_q is not None and dist_two > delta_q:
            q_end = q_near * (1 - delta_q / dist) + q_rand * (delta_q / dist_two)
            dist_two = delta_q  
        dt = dist_two / discretisationsteps_newconf
        for i in range(1, discretisationsteps_newconf):
            q_new = q_near * (1 - dt*i) + q_end * (dt*i)
            q_newt, success = computeqgrasppose(robot, qinit, cube, cube_q_rand, viz)
            if success:
                q_new = q_near * (1 - dt*(i-1)) + q_end * (dt*(i-1))
            else:
                q_new = q_end
        print(f"New configuration: {q_new}")
    # Add the edge and vertex from q_near to q_new to the tree G
        G += [(q_near_index, q_new)]

    # Return the closest configuration q such that the path q => q_new is the longest 
    # along the linear interpolation (q_new,qgoal) that is collision free and of length <  delta_q
        q_end_two = qgoal.copy()
        dist_three = np.linalg.norm(qgoal - q_new) 
        if delta_q is not None and dist_three > delta_q:
            q_end_two = q_new * (1 - delta_q / dist_three) + qgoal * (delta_q / dist_three)
            dist_three = delta_q
        dt = dist_three / discretisationsteps_validedge
        for i in range(1, discretisationsteps_validedge):
            q = q_new * (1 - dt*i) + q_end_two * (dt*i)
            q1, success_two = computeqgrasppose(robot, qinit, cube, cube_q_rand, viz) 
            if success_two:
                q = q_new * (1 - dt*(i-1)) + q_end_two * (dt*(i-1))
            else:
                q = q_end_two
        print(f"New configuration: {q}")
        # If the edge between q_new and q_goal is valid then a path has been found 
        if np.linalg.norm(qgoal - q) < 1e-3:
            print("Path found!")
            G += [(len(G)-1, qgoal)]
            return G
    return G

    # Reconstruct the path from qinit to qgoal
#     path = []
#     node = G[-1]
#     while node[0] is not None:
#         path = [node[1]] + path
#         node = G[node[0]]
#     path = G[0][1] + path
#     return path
                                             

def displaypath(robot,path,dt,viz):
    for q in path:
        viz.display(q)
        time.sleep(dt)


if __name__ == "__main__":
    from tools import setupwithmeshcat
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
    from inverse_geometry import computeqgrasppose
    
    robot, cube, viz = setupwithmeshcat()
    
    
    q = robot.q0.copy()
    q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
    qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)
    
    if not(successinit and successend):
        print ("error: invalid initial or end configuration")
    
    path = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)
    
    displaypath(robot,path,dt=0.5,viz=viz) #you ll probably want to lower dt
    
