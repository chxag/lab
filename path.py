#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 21 11:44:32 2023

@author: stonneau
"""

import pinocchio as pin
import numpy as np
from numpy.linalg import pinv
# from setup_meshcat import updatevisuals

from config import LEFT_HAND, RIGHT_HAND
from tools import collision, getcubeplacement, setcubeplacement, projecttojointlimits
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
    
    G = [(None,np.array(cubeplacementq0))]
#     T = [(None, np.array(cubeplacementq0))]
#     cube_path = [cubeplacementq0]
    
    rotation = cubeplacementq0.rotation
    translation = cubeplacementq0.translation
    translation_goal = cubeplacementqgoal.translation
    
    cube_q_target = pin.SE3(cubeplacementqgoal.rotation, translation_goal)
    
    sampled_positions = set()
    goal_bias = 0.1

    x_range = (translation[0], translation_goal[0])
    y_range = (translation[1], translation_goal[1])
    z_range = (translation[2], translation[2] + 0.1)
    
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
#                     T+=[(cube, np.array(cube_q_rand))]
#                     cube_path.append(cube_q_rand)
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
            q_new = q_near * (1 - dt*i/discretisationsteps_newconf) + q_end * (dt*i/discretisationsteps_newconf)
            q_newt, success = computeqgrasppose(robot, qinit, cube, cube_q_rand, viz)
            if success:
                q_new = q_near * (1 - dt*(i-1)/discretisationsteps_newconf) + q_end * (dt*(i-1)/discretisationsteps_newconf)
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
    cube_path = []
    node = G[-1]
    while node[0] is not None:
        path = [node[1]] + path
        #get the cube configuration based on the robot configuration at this path step
        q_path, success_path = computeqgrasppose(robot, qinit, cube, cube_q_target,viz)
        if success_path:
            cube_path.insert(0, getcubeplacement(cube))
        node = G[node[0]]
    path = [G[0][1]] + path
    cube_path.insert(0,cubeplacementq0)
    
    print("robot path: ", path)
    print("cube path: ", cube_path)
    return path, cube_path

def lerp(q0,q1,t):    
    return q0 * (1 - t) + q1 * t 

def displayedge (q0, q1, cube_q0, cube_q1, vel=2.):
    from math import ceil
    from time import sleep
    from setup_meshcat import updatevisuals
    dist = np.linalg.norm(q1-q0)
    duration = dist / vel
    nframes = ceil(48. * duration)
    f = 1. / 48.
    for i in range(nframes-1):
        interp_q = lerp(q0, q1, float(i)/nframes)
        interp_cube_q = lerp(cube_q0.translation, cube_q1.translation, float(i)/nframes)
        viz.display(interp_q)
        setcubeplacement(robot, cube, cube_q0)
        updatevisuals(viz, robot, cube, interp_q)
        sleep(f)
    viz.display(q1)
    setcubeplacement(robot, cube, cube_q1)
    updatevisuals(viz, robot, cube, q1)
    sleep(f)
                                            
def displaypath(robot,path,cube_path,dt,viz):
    for (q0, q1), (cube_q0, cube_q1) in zip(zip(path[:-1],path[1:]), zip(cube_path[:-1], cube_path[1:])):
        displayedge(q0,q1, cube_q0, cube_q1)

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
    
    path, cube_path = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)
    
    displaypath(robot,path,cube_path, dt=0.5,viz=viz) #you ll probably want to lower dt
    
