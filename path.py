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
from tools import collision, getcubeplacement, setcubeplacement, projecttojointlimits, distanceToObstacle
from config import EPSILON
import time

def coll(q):
    return distanceToObstacle(robot, q) > 0

def cube_collision(oMf):
    setcubeplacement(robot, cube, oMf)
    dist_obst = pin.computeDistance(cube.collision_model, cube.collision_data, 1).min_distance
    return dist_obst < 0.02

def distance(q1, q2):
    return np.linalg.norm(q2.translation - q1.translation)

def NEAREST_VERTEX(G,q_rand):
    min_dist = 10e4
    idx=-1
    for (i,node) in enumerate(G):
#         print("setcubeplacement", setcubeplacement(robot, 
        dist = distance(node[1],q_rand)
        if dist < min_dist:
            min_dist = dist
            idx = i
    return idx

def ADD_EDGE_AND_VERTEX(G,parent, cube_placement, robot_q):
    G += [(parent, cube_placement, robot_q)]
    
def lerp(q0,q1,t):    
    return q0 * (1 - t) + q1 * t 

def NEW_CONF(q_near, q_rand, discretisationsteps, delta_q=None):
    q_end = q_rand.copy()
    dist = distance(q_near, q_rand)
    if delta_q is not None and dist > delta_q:
        print("q_near", q_near)
        print("q_rand", q_rand)
        print("t", delta_q/dist)
        q_end = lerp(q_near, q_rand, delta_q/dist)
        dist = delta_q
    dt = dist / discretisationsteps
    for i in range(1, discretisationsteps):
        q_lerp = lerp(q_near, q_end, (dt*i)/dist)
        if cube_collision(pin.SE3(q_lerp)):
            return lerp(q_near, q_end, (dt*(i-1))/dist)
    return q_end

def VALID_EDGE(q_new, q_goal, discretisationsteps):
    return np.linalg.norm(q_goal - NEW_CONF(q_new, q_goal, discretisationsteps)) < 1e-3

def getpath(G):
    path = []
    node = G[-1]
    while node[0] is not None:
        path = [node[1]] + path
        node = G[node[0]]
    path = [G[0][1]] + path
    return path

def shortcut(path):
    for i, q in enumerate(path):
        for j in reversed(range(i+1, len(path))):
            q2 = path[j]
            q_new = NEW_CONF(q,q2, discretisationsteps_newconf, delta_q = delta_q)
            if VALID_EDGE(q,q2,discretisationsteps_validedge):
                path = path[:i+1]+path[j:]
                return path
    return path

#returns a collision free path from qinit to qgoal under grasping constraints
#the path is expressed as a list of configurations
def computepath(qinit,qgoal,cubeplacementq0, cubeplacementqgoal):
    #TODO
    discretisationsteps_newconf = 200 
    discretisationsteps_validedge = 200 
    k = 1000
    delta_q = 3.
    
    G = [(None,cubeplacementq0, qinit)]
    
    rotation_init = cubeplacementq0.rotation
    translation_init = cubeplacementq0.translation
    translation_goal = cubeplacementqgoal.translation
    
    cube_q_target = pin.SE3(cubeplacementqgoal.rotation, translation_goal)
    
    sampled_positions = set()
    goal_bias = 0.1

    x_range = (translation_init[0], translation_goal[0])
    y_range = (translation_init[1], translation_goal[1])
    z_range = (translation_init[2], translation_init[2] + 0.1)
    
    for iteration in range(k):
        
        # q_rand = RAND_CONF()
        while True: 
        # Sampling configurations for the cube 
            cube_x_rand = np.random.uniform(*x_range)
            cube_y_rand = np.random.uniform(*y_range)
            cube_z_rand = np.random.uniform(*z_range) 

            cube_rand_translation = np.array([cube_x_rand, cube_y_rand, cube_z_rand])
            cube_q_rand = pin.SE3(rotation_init, cube_rand_translation)
            setcubeplacement(robot, cube, cube_q_rand)
            position_tuple = (cube_x_rand, cube_y_rand, cube_z_rand)
            if position_tuple not in sampled_positions: 
                sampled_positions.add(position_tuple)

                # Generating valid pose for the randomly sampled cube position 
                q_rand, success = computeqgrasppose(robot, qinit, cube, cube_q_rand, viz)
            
                if not success:
                    break
                else: 
                    # Change the range of the sampling space 
                    x_range = (min(0, cube_x_rand - 0.1), max(0.5, cube_x_rand + 0.1))
                    y_range = (min(-0.2, cube_y_rand - 0.1), max(0.2, cube_y_rand + 0.1))   
                    z_range = (min(0.93, cube_z_rand - 0.1), max(1.2, cube_z_rand + 0.1))
        cubeplacementqrand = getcubeplacement(cube)
        # Find the nearest vertex to q_rand called q_near 
        q_near_index = NEAREST_VERTEX(G, cube_q_rand)
        q_near = G[q_near_index][1]
        
    # Return the closest configuration q_new such that the path q_near => q_new is the longest 
    # along the linear interpolation (q_near,q_rand) that is collision free and of length <  delta_q
        
        q_new = NEW_CONF(q_near, cube_q_rand, discretisationsteps_newconf, delta_q=None)
        robot_new, success_coll = computeqgrasppose(robot, q_rand, cube, q_new, viz)
        #compute grasppose for the new cube config
    # Add the edge and vertex from q_near to q_new to the tree G
        ADD_EDGE_AND_VERTEX(G, q_near_index, q_new, robot_new)
        break

    # Return the closest configuration q such that the path q => q_new is the longest 
    # along the linear interpolation (q_new,qgoal) that is collision free and of length <  delta_q
        if VALID_EDGE(q_new, cubeplacementqgoal, discretisationsteps_validedge):
            print ("Path found!")
            robot_goal, success_coll = computeqgrasppose(robot, q_new, cube, cubeplacementgoal,viz) # compute grasppose for the goal cube config
            ADD_EDGE_AND_VERTEX(G,len(G)-1, cubeplacementqgoal, robot_goal)
        print("path not found")

    # Reconstruct the path from qinit to qgoal
    path = getpath(G)
    
    print("path found", path)
    return path #, cube_path

def displayedge (q0, q1, vel=2.):
    from math import ceil
    from time import sleep
    from setup_meshcat import updatevisuals
    dist = distance(q0,q1)
    duration = dist / vel
    nframes = ceil(48. * duration)
    f = 1. / 48.
    print("q0", q0)
    print("q1", q1)
    for i in range(nframes-1):
        t = float(i) / nframes
        print(t)
        interp = lerp(q0, q1, t)
        viz.display(pin.SE3(interp))
#         setcubeplacement(robot, cube, q0)
        updatevisuals(viz, robot, cube, pin.SE3(interp))
        sleep(f)
    viz.display(q1)
#     setcubeplacement(robot, cube, cube_q1)
    updatevisuals(viz, robot, cube, q1)
    sleep(f)
                                            
def displaypath(robot,path,dt,viz):
    for q0, q1 in zip(path[:-1],path[1:]):
        displayedge(q0,q1)

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
    
#     displaypath(robot,path, dt=0.5,viz=viz) #you ll probably want to lower dt
    
