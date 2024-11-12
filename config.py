#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep  4 11:43:26 2023

@author: stonneau
"""

from os.path import dirname, join, abspath
import numpy as np
import pinocchio as pin #the pinocchio library
from pinocchio.utils import rotate

#These parameters can be edited
USE_MESHCAT = True # part 1 uses meshcat
USE_PYBULLET = True # the second part of the lab will use pybullet
MESHCAT_URL ="tcp://127.0.0.1:6000"
USE_PYBULLET_GUI = USE_PYBULLET and True
USE_PYBULLET_REALTIME = USE_PYBULLET and False

DT = 1e-3 #simulation tick time (s)
EPSILON = 1e-3 #almost 0

#the remaining variables should not be edited in theory
LEFT_HAND  = 'LARM_EFF'
RIGHT_HAND = 'RARM_EFF'

LEFT_HOOK = "LARM_HOOK"
RIGHT_HOOK = "RARM_HOOK"

    
#scene placements
ROBOT_PLACEMENT= pin.XYZQUATToSE3(np.array([0.,0.,0.85,0.,0.,0.,1.]))
TABLE_PLACEMENT= pin.SE3(rotate('z',-np.pi/2),np.array([0.8,0.,0.]))
OBSTACLE_PLACEMENT= pin.SE3(rotate('z',0),np.array([0.43,-0.1,0.94]))
#original

CUBE_PLACEMENT = pin.SE3(rotate('z', 0.),np.array([0.33, -0.3, 0.93]))
CUBE_PLACEMENT_TARGET= pin.SE3(rotate('z', 0),np.array([0.4, 0.11, 0.93]))

#tests

# CUBE_PLACEMENT = pin.SE3(rotate('z', 0),np.array([0.44, -0.4, 0.93]))
# CUBE_PLACEMENT_TARGET= pin.SE3(rotate('z', 0),np.array([0.5, 0.12, 0.93]))


# CUBE_PLACEMENT = pin.SE3(rotate('z', 10),np.array([0.44, -0.4, 0.93]))
# CUBE_PLACEMENT_TARGET= pin.SE3(rotate('z', 10),np.array([0.5, 0.12, 0.93]))


# CUBE_PLACEMENT = pin.SE3(rotate('z', -8),np.array([0.2, -0.1, 0.93]))
# CUBE_PLACEMENT_TARGET= pin.SE3(rotate('z', -3),np.array([0.7, 0.16, 0.93]))

# CUBE_PLACEMENT = pin.SE3(rotate('z', 0.),np.array([0.6, -0.3, 0.93]))
# CUBE_PLACEMENT_TARGET= pin.SE3(rotate('z', 0),np.array([0.3, 0.2, 0.93]))


# CUBE_PLACEMENT = pin.SE3(rotate('z', -4.),np.array([0.6, -0.3, 0.93]))
# CUBE_PLACEMENT_TARGET= pin.SE3(rotate('z', 3),np.array([0.3, 0.2, 0.93]))

# CUBE_PLACEMENT = pin.SE3(rotate('z', 18),np.array([0.23, -0.2, 0.93]))
# CUBE_PLACEMENT_TARGET= pin.SE3(rotate('z', -20),np.array([0.65, 0.25, 0.93]))

# CUBE_PLACEMENT = pin.SE3(rotate('z', 0),np.array([0.53, -0.45, 0.93]))
# CUBE_PLACEMENT_TARGET= pin.SE3(rotate('z', 0),np.array([0.8, 0.19, 0.93]))


# CUBE_PLACEMENT = pin.SE3(rotate('z', 5),np.array([0.53, -0.45, 0.93]))
# CUBE_PLACEMENT_TARGET= pin.SE3(rotate('z', -4),np.array([0.54, 0.19, 0.93]))



# CUBE_PLACEMENT = pin.SE3(rotate('z', 18),np.array([0.23, -0.2, 0.93]))
# CUBE_PLACEMENT_TARGET= pin.SE3(rotate('z', -20),np.array([0.65, 0.25, 0.93]))




#do not edit this part unless you know what you are doing
MODELS_PATH = join(dirname(str(abspath(__file__))), "models") 
MESH_DIR = MODELS_PATH 
NEXTAGE_URDF_PATH = MODELS_PATH + '/nextagea_description/urdf/'
NEXTAGE_URDF = NEXTAGE_URDF_PATH + 'NextageaOpen.urdf'
NEXTAGE_SRDF = NEXTAGE_URDF_PATH + 'NextageAOpen.srdf'
TABLE_URDF = MODELS_PATH + '/table/table_tallerscaled.urdf'
TABLE_MESH = MODELS_PATH +  "/table/" 
OBSTACLE_URDF = MODELS_PATH + '/cubes/obstacle.urdf'
OBSTACLE_MESH = MODELS_PATH + '/cubes/'
CUBE_URDF = MODELS_PATH + '/cubes/cube_small.urdf'
CUBE_MESH = MODELS_PATH + '/cubes/'
