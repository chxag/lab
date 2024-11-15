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
from tools import setupwithmeshcat
from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
from inverse_geometry import computeqgrasppose
import matplotlib.pyplot as plt
from config import LEFT_HAND, RIGHT_HAND
from tools import collision, getcubeplacement, setcubeplacement, projecttojointlimits, distanceToObstacle, jointlimitsviolated
from config import EPSILON
import time
    
robot, cube, viz = setupwithmeshcat()

def coll(q):
    return distanceToObstacle(robot, q) > 0

def cube_collision(robot,cube, oMf):
    setcubeplacement(robot, cube, oMf)
    dist_obst = pin.computeDistance(cube.collision_model, cube.collision_data, 1).min_distance
    return dist_obst < 0.03

def robot_collision(robot, q):
    dist_obst = distanceToObstacle(robot, q)
    return dist_obst < 0.0029

def sample_cube_higher(q):
    rotation_q = q.rotation
    translation_q = q.translation
    transition = np.array([translation_q[0], translation_q[1], translation_q[2] + 0.2])
    cube_q = pin.SE3(rotation_q, transition)
    return cube_q
    
def distance(q1, q2):
    q1 = pin.SE3(q1)
    q2 = pin.SE3(q2)
    return np.linalg.norm(q2.translation - q1.translation)

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
    dist = distance(q_near, q_rand)

    if delta_q is not None and dist > delta_q:
        q_end = lerp(np.array(q_near), np.array(q_rand), delta_q / dist)
        dist = delta_q
    dt = dist / discretisationsteps
    last_valid_cube = q_near
    last_valid_robot = robot_q_near

    for  i in range(1, discretisationsteps):
        q_lerp_cube = lerp(np.array(q_near), np.array(q_end), (dt * i) / dist)
        q_lerp_cube = pin.SE3(q_lerp_cube)
        robot_q, success = computeqgrasppose(robot, robot_q_near, cube, q_lerp_cube, viz)
        viz.display(robot_q)
        if not cube_collision(robot,cube,q_lerp_cube) and not robot_collision(robot, robot_q):
            last_valid_cube = q_lerp_cube
            last_valid_robot = robot_q
        else:
            while cube_collision(robot, cube, q_lerp_cube) or robot_collision(robot, robot_q):
                q_lerp_cube = lerp(np.array(q_near), np.array(last_valid_cube), (dt * (i-1)) / dist)
                q_lerp_cube = pin.SE3(q_lerp_cube)
                robot_q, success= computeqgrasppose(robot, robot_q_near, cube, q_lerp_cube, viz)
                viz.display(robot_q)

                if not cube_collision(robot, cube, q_lerp_cube) and not robot_collision(robot, robot_q):
                    last_valid_cube = q_lerp_cube
                    last_valid_robot = robot_q
                    return last_valid_cube, last_valid_robot
                if np.array_equal(last_valid_cube, q_near):
                    return q_near, robot_q_near
            break
    return last_valid_cube, last_valid_robot

def VALID_EDGE(robot_q_new, q_new, q_goal, discretisationsteps):
    cube_q, robot_q = NEW_CONF_CUBE(robot_q_new, q_new, q_goal, discretisationsteps)
    return np.linalg.norm(np.array(q_goal) - np.array(cube_q)) < 2e-2

def getpath(G):
    path = []
    node = G[-1]
    while node[0] is not None:
        path = [node[2]] + path
        node = G[node[0]]
    path = [G[0][2]] + path
    return path

def computepath(qinit,qgoal,cubeplacementq0, cubeplacementqgoal, k):
    #TODO
    discretisationsteps_newconf = 20
    discretisationsteps_validedge = 20 
    k = 1000
    delta_q = .1
    
    G = [(None,np.array(cubeplacementq0), np.array(qinit))]
    
    rotation_init = cubeplacementq0.rotation
    translation_init = cubeplacementq0.translation
    translation_goal = cubeplacementqgoal.translation
        
    sampled_positions = set()

    x_range = (translation_init[0], translation_goal[0]+0.5)
    y_range = (translation_init[1], translation_goal[1]+0.5)
    z_range = (translation_init[2], translation_goal[2]+0.5)

    sample_higher = True
    path_found = False
    
    for iteration in range(k):
        
        while True:
            if sample_higher:
                cube_q_rand  = sample_cube_higher(cubeplacementq0) 
                cube_q_rand_translation = cube_q_rand.translation 
                position_tuple = (cube_q_rand_translation[0], cube_q_rand_translation[1], cube_q_rand_translation[2])
                if position_tuple not in sampled_positions:
                    sampled_positions.add(position_tuple)
                sample_higher = False
            else:
                cube_x_rand = np.random.uniform(*x_range)
                cube_y_rand = np.random.uniform(*y_range)
                cube_z_rand = np.random.uniform(*z_range) 

                cube_rand_translation = np.array([cube_x_rand, cube_y_rand, cube_z_rand])
                cube_q_rand = pin.SE3(rotation_init, cube_rand_translation)

                position_tuple = (cube_x_rand, cube_y_rand, cube_z_rand)

                if position_tuple not in sampled_positions:
                    sampled_positions.add(position_tuple)
 
            q_rand, success = computeqgrasppose(robot, qinit, cube, cube_q_rand, viz)

            if not robot_collision(robot, q_rand) and not jointlimitsviolated(robot, q_rand):
                break
                    
        cube_q_near_index = NEAREST_VERTEX_CUBE_Q(G, cube_q_rand)
        cube_q_near = G[cube_q_near_index][1]
        cube_q_near = pin.SE3(cube_q_near)

        q_near_index = NEAREST_VERTEX_ROBOT_Q(G, q_rand)
        q_near, success = computeqgrasppose(robot, q_rand, cube, cube_q_near, viz)

        if jointlimitsviolated(robot, q_near):
            q_near = projecttojointlimits(robot, q_near)
        
        cube_q_new, robot_q_new = NEW_CONF_CUBE(q_near, cube_q_near, cube_q_rand, discretisationsteps_newconf, delta_q)
        cube_q_new = pin.SE3(cube_q_new)
        if jointlimitsviolated(robot, robot_q_new):
            projecttojointlimits(robot, robot_q_new)
        
        ADD_EDGE_AND_VERTEX(G, q_near_index, np.array(cube_q_new), np.array(robot_q_new))

        if VALID_EDGE(robot_q_new, cube_q_new, cubeplacementqgoal, discretisationsteps_validedge):
            print ("Path found!")
            path_found = True
            ADD_EDGE_AND_VERTEX(G,len(G)-1, np.array(cubeplacementqgoal), np.array(qgoal))
            print(getpath(G))
            return getpath(G), iteration, path_found
    
    print("Path not found")
    return [], k, path_found

def displayedge (q0, q1, vel=2.):
    from math import ceil
    from time import sleep
    from setup_meshcat import updatevisuals
    dist = robot_distance(q0,q1)
    duration = dist / vel
    nframes = ceil(48. * duration)
    f = 1. / 48.
    for i in range(nframes-1):
        t = float(i) / nframes
        interp = lerp(q0, q1, t)
        updatevisuals(viz, robot, cube, interp)
        sleep(f)
    updatevisuals(viz, robot, cube, q1)
    sleep(f)
                                            
def displaypath(robot,path,dt,viz):
    for q0, q1 in zip(path[:-1],path[1:]):
        displayedge(q0,q1)
#     for q in path:
#         viz.display(q)
#         time.sleep(dt)

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
    
    k_values = [100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 2000, 3000, 4000, 5000]
    iterations = []
    iterations_not_found = []

    for i,k in enumerate(k_values):        
        path, iteration, path_found = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET, k)
        if path_found == True:
            iterations.append(i)
        else:
            iterations_not_found.append(i)
    
    displaypath(robot,path, dt=0.5,viz=viz) #you ll probably want to lower dt
    
