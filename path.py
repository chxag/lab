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

def robot_collision(q):
    pin.updateGeometryPlacements(robot.model, robot.data, robot.collision_model, robot.collision_data, q)
    return pin.computeCollisions(robot.collision_model, robot.collision_data, False)

def distance(q1, q2):
    q1 = pin.SE3(q1)
    q2 = pin.SE3(q2)
    return np.linalg.norm(q2.translation - q1.translation)

def distance_robot(q1, q2):
    return np.linalg.norm(q2 - q1)

def robot_distance(q1, q2):
    return np.linalg.norm(q2-q1)

def NEAREST_VERTEX_ROBOT_Q(G, q_rand):
    min_dist = 10e4
    idx = -1
    for i, (parent, cube_q, robot_q) in enumerate(G):
        dist = robot_distance(robot_q, q_rand) #q_rand is not SE3
        if dist < min_dist:
            min_dist = dist
            idx = i
    return idx

def NEAREST_VERTEX_CUBE_Q(G,cube_q_rand):
    min_dist = 10e4
    idx=-1
    for i, (parent, cube_q, robot_q) in enumerate(G):
        cube_q = pin.SE3(cube_q)
        dist = distance(cube_q,cube_q_rand)
        if dist < min_dist:
            min_dist = dist
            idx = i
    return idx

def ADD_EDGE_AND_VERTEX(G,parent, cube_placement, robot_q):
    G += [(parent, cube_placement, robot_q)]
    
def lerp(q0,q1,t):    
    return q0 * (1 - t) + q1 * t 

def NEW_CONF_CUBE(robot_q_near, q_near, q_rand, discretisationsteps, delta_q=None):
    q_end = q_rand.copy()
    robot_q, success = computeqgrasppose(robot, robot_q_near, cube, q_end, viz)
    dist = distance(q_near, q_rand)

    if delta_q is not None and dist > delta_q:
        q_end = lerp(np.array(q_near), np.array(q_rand), delta_q / dist)
        dist = delta_q

    dt = dist / discretisationsteps
    last_valid = q_near
    last_valid = pin.SE3(last_valid)
    
    if not cube_collision(last_valid) and not collision(robot, robot_q):
        print(f"no Collision detected at step 0 with q = {last_valid}")
        return last_valid, robot_q
        
    
    for i in range(1, discretisationsteps):
        last_valid = lerp(np.array(q_near), np.array(last_valid), dt * (i - 1) / dist)
        last_valid = pin.SE3(last_valid)
        print(f"Step {i}: q = {last_valid}")

        robot_q, success = computeqgrasppose(robot, robot_q_near, cube, last_valid, viz)
        viz.display(robot_q)

        if not cube_collision(last_valid) and not collision(robot, robot_q):
            print(f"no Collision detected at step {i} with q = {last_valid}")
    #             last_valid = lerp(np.array(q_near), np.array(last_valid), dt * (i - 1) / dist)
    #             last_valid = pin.SE3(last_valid)
            robot_q, success = computeqgrasppose(robot, robot_q_near, cube, last_valid, viz)
    #             viz.display(robot_q)
            return last_valid, robot_q
        
#     print(f"No collision detected, returning q_end = {q_end, robot_q_end}")
    return q_end, robot_q


def VALID_EDGE(robot_q_new, q_new, q_goal, discretisationsteps):
    cube_q, robot_q = NEW_CONF_CUBE(robot_q_new, q_new, q_goal, discretisationsteps)
    print(np.linalg.norm(np.array(q_goal) - np.array(cube_q)) < 1e-3)
    print(f"VALID_EDGE: q_new={cube_q}, q_goal={q_goal}, new_conf={robot_q}, valid={np.linalg.norm(np.array(q_goal) - np.array(cube_q)) < 1e-3}")

    return np.linalg.norm(np.array(q_goal) - np.array(cube_q)) < 1e-3

def getpath(G):
    path = []
    node = G[-1]
    while node[0] is not None:
        path = [node[2]] + path
        node = G[node[0]]
    path = [G[0][2]] + path
    return path

def shortcut(path):
    for i, q in enumerate(path):
        for j in reversed(range(i+1, len(path))):
            q2 = path[j]
            q_new = NEW_CONF(q,q2,discretisationsteps_newconf, delta_q = delta_q)
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
    delta_q = .1
    
    G = [(None,np.array(cubeplacementq0), np.array(qinit))]
    
    rotation_init = cubeplacementq0.rotation
    translation_init = cubeplacementq0.translation
    translation_goal = cubeplacementqgoal.translation
        
    sampled_positions = set()
    goal_bias = 0.1

    x_range = (translation_init[0], translation_goal[0])
    y_range = (translation_init[1], translation_goal[1])
    z_range = (translation_init[2], translation_goal[2] + 0.6)

#     x_range = (0, 0.5) 
#     y_range = (-0.2, 0.2)
#     z_range = (0.93, 1.2)
    
    for iteration in range(k):
        
        # q_rand = RAND_CONF()
        while True: 
        # Sampling configurations for the cube 
            cube_x_rand = np.random.uniform(*x_range)
            cube_y_rand = np.random.uniform(*y_range)
            cube_z_rand = np.random.uniform(*z_range) 

            cube_rand_translation = np.array([cube_x_rand, cube_y_rand, cube_z_rand])
            cube_q_rand = pin.SE3(rotation_init, cube_rand_translation)

            position_tuple = (cube_x_rand, cube_y_rand, cube_z_rand)
            
            if position_tuple not in sampled_positions: 
                sampled_positions.add(position_tuple)

                # Generating valid pose for the randomly sampled cube position 
                q_rand, success = computeqgrasppose(robot, qinit, cube, cube_q_rand, viz)
            
                if not robot_collision(q_rand):
                    break
#                 else: 
#                     # Change the range of the sampling space 
#                     x_range = (min(0, cube_x_rand - 0.1), max(0.5, cube_x_rand + 0.1))
#                     y_range = (min(-0.2, cube_y_rand - 0.1), max(0.2, cube_y_rand + 0.1))   
#                     z_range = (min(0.93, cube_z_rand - 0.1), max(1.2, cube_z_rand + 0.1))
                    
        cube_q_near_index = NEAREST_VERTEX_CUBE_Q(G, cube_q_rand)
        cube_q_near = G[cube_q_near_index][1]
        cube_q_near = pin.SE3(cube_q_near)
        
        q_near_index = NEAREST_VERTEX_ROBOT_Q(G, q_rand)
        q_near, success = computeqgrasppose(robot, q_rand, cube, cube_q_near, viz)
        
    # Return the closest configuration q_new such that the path q_near => q_new is the longest 
    # along the linear interpolation (q_near,q_rand) that is collision free and of length <  delta_q
        print("q_near", q_near)
        print("cube_q_near", cube_q_near)
        print("cube_q_rand", cube_q_rand)
        
        cube_q_new, robot_q_new = NEW_CONF_CUBE(q_near, cube_q_near, cube_q_rand, discretisationsteps_newconf, delta_q)
        cube_q_new = pin.SE3(cube_q_new)
        
        ADD_EDGE_AND_VERTEX(G, q_near_index, np.array(cube_q_new), np.array(robot_q_new))
           
        #compute grasppose for the new cube config
    # Add the edge and vertex from q_near to q_new to the tree G
#         break

    # Return the closest configuration q such that the path q => q_new is the longest 
    # along the linear interpolation (q_new,qgoal) that is collision free and of length <  delta_q
        if VALID_EDGE(robot_q_new, cube_q_new, cubeplacementqgoal, discretisationsteps_validedge):
            print ("Path found!")
#                 ADD_EDGE_AND_VERTEX(G,len(G)-1, np.array(cubeplacementqgoal), np.array(qgoal))
            ADD_EDGE_AND_VERTEX(G,len(G)-1, np.array(cubeplacementqgoal), np.array(qgoal))
            print(getpath(G))
            return getpath(G)
    
    print("path not found")
    # Reconstruct the path from qinit to qgoal
    path = []
    print(path)
    return path

def displayedge (q0, q1, vel=2.):
    from math import ceil
    from time import sleep
    from setup_meshcat import updatevisuals
    dist = robot_distance(q0,q1)
    duration = dist / vel
    nframes = ceil(48. * duration)
    f = 1. / 48.
    print(q0)
    print(q1)
    for i in range(nframes-1):
        t = float(i) / nframes
        print(t)
        interp = lerp(q0, q1, t)
        viz.display(interp)
#         setcubeplacement(robot, cube, q0)
        updatevisuals(viz, robot, cube, interp)
        sleep(f)
    viz.display(q1)
#     setcubeplacement(robot, cube, cube_q1)
    updatevisuals(viz, robot, cube, q1)
    sleep(f)
                                            
def displaypath(robot,path,dt,viz):
    for q0, q1 in zip(path[:-1],path[1:]):
        displayedge(q0,q1)
#     for q in path:
#         viz.display(q)
#         time.sleep(dt)

def plotpaths(paths, colq_end,ors = ['r','c']):
    plotConfigurationSpace(hcol,hfree)
    for path, color in zip(paths,colors):
        patharray = np.array(path)
        plt.plot(patharray[:,0],patharray[:,1],color,lw=3)

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
    
#     plotpaths([path])
    
    displaypath(robot,path, dt=0.5,viz=viz) #you ll probably want to lower dt
    
