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
    
    G = [(None,np.array(qinit))]
    
    rotation = cubeplacementq0.rotation
    sampled_positions = set()
    goal_bias = 0.1

    x_range = (0, 0.5) 
    y_range = (-0.2, 0.2)
    z_range = (0.93, 1.2)
    
    for iteration in range(k):
        
        while True: 
        # Sampling configurations for the cube 
            cube_x_rand = np.random.uniform(*x_range)
            cube_y_rand = np.random.uniform(*y_range)
            cube_z_rand = np.random.uniform(*z_range) 

            cube_rand_translation = np.array([cube_x_rand, cube_y_rand, cube_z_rand])
            cube_q_rand = pin.SE3(rotation, cube_rand_translation)
            position_tuple = (cube_x_rand, cube_y_rand, cube_z_rand) 
            if position_tuple not in sampled_positions: 
                sampled_positions.add(position_tuple)
                print(f"Sampled position: {cube_rand_translation}")

                # Generating valid pose for the randomly sampled cube position 
                q_rand, success = computeqgrasppose(robot, qinit, cube, cube_q_rand, viz)
                print(f"Sampled configuration: {q_rand}, success: {success}")
            
                if not success:
                    break
                else: 
                    # Change the range of the sampling space 
                    x_range = (min(0, cube_x_rand - 0.1), max(0.5, cube_x_rand + 0.1))
                    y_range = (min(-0.2, cube_y_rand - 0.1), max(0.2, cube_y_rand + 0.1))   
                    z_range = (min(0.93, cube_z_rand - 0.1), max(1.2, cube_z_rand + 0.1))
            
        # Find the nearest vertex to q_rand called q_near 
        for i, (parent,node) in enumerate(G):
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
        q_new = np.array(q_new)
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
        q = np.array(q)
        print(f"New configuration: {q}")
        # If the edge between q_new and q_goal is valid then a path has been found 
        if np.linalg.norm(qgoal - q) < 1e-3:
            print("Path found!")
            G += [(len(G)-1, np.array(qgoal))]
            break

    # Reconstruct the path from qinit to qgoal
    path = []
    node = G[-1]
    while node[0] is not None:
        path = [node[1]] + path
        node = G[node[0]]
    path = [G[0][1]] + path
    print(path)
    return path

"""
Sampled position: [ 0.31008664 -0.30402431  1.08291382]
Sampled configuration: [-5.45553308e-01  3.15567393e-17 -1.06750904e-18 -8.02136878e-01
 -9.95786087e-02 -6.12747111e-01  7.00853009e-18  7.12325719e-01
 -2.19332026e-01 -1.63863791e-02 -5.08921791e-01 -9.44339030e-02
  4.69469089e-18  6.03355694e-01  2.13054339e+00], success: False
Nearest vertex: [-4.88294048e-01  7.18141757e-17 -7.48577510e-18 -7.16814094e-01
  7.19954859e-02 -2.97292784e-01  5.06354381e-18  2.25297298e-01
 -2.88658422e-01 -2.97808375e-02 -3.02837503e-01  1.10090761e-01
  1.38259284e-17  1.92746742e-01  2.01334901e+00]
  New configuration: [-5.45553308e-01  3.15567393e-17 -1.06750904e-18 -8.02136878e-01
 -9.95786087e-02 -6.12747111e-01  7.00853009e-18  7.12325719e-01
 -2.19332026e-01 -1.63863791e-02 -5.08921791e-01 -9.44339030e-02
  4.69469089e-18  6.03355694e-01  2.13054339e+00]
   New configuration: [ 1.78220357e-01  1.07355539e-17 -2.20063922e-17 -1.60879641e-01
 -3.32635590e-02 -1.93582824e-01 -6.30109638e-18  2.26846383e-01
 -1.51110728e+00  4.47243211e-01  1.31672487e-01 -3.52361825e-01
 -6.78156657e-18  2.20689338e-01  8.69810557e-01]
Path found!
[array([-4.88294048e-01,  7.18141757e-17, -7.48577510e-18, -7.16814094e-01,
        7.19954859e-02, -2.97292784e-01,  5.06354381e-18,  2.25297298e-01,
       -2.88658422e-01, -2.97808375e-02, -3.02837503e-01,  1.10090761e-01,
        1.38259284e-17,  1.92746742e-01,  2.01334901e+00]), array([-5.45553308e-01,  3.15567393e-17, -1.06750904e-18, -8.02136878e-01,
       -9.95786087e-02, -6.12747111e-01,  7.00853009e-18,  7.12325719e-01,
       -2.19332026e-01, -1.63863791e-02, -5.08921791e-01, -9.44339030e-02,
        4.69469089e-18,  6.03355694e-01,  2.13054339e+00]), array([ 1.78220357e-01,  1.07355539e-17, -2.20063922e-17, -1.60879641e-01,
       -3.32635590e-02, -1.93582824e-01, -6.30109638e-18,  2.26846383e-01,
       -1.51110728e+00,  4.47243211e-01,  1.31672487e-01, -3.52361825e-01,
       -6.78156657e-18,  2.20689338e-01,  8.69810557e-01])]

"""
                                             

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
    
