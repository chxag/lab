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
    
    fc_value = -400 #seems to work better
    
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
    print(torques)
    
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
    import matplotlib.patches as mpatches
    import matplotlib.path as mpath
    
    q0,successinit = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT, None)
    qe,successend = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET,  None)
    path = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)

    def new_path_g(path): 
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
        return new_path
    
    
    #setting initial configuration
    sim.setqsim(q0)
    
    #TODO this is just an example, you are free to do as you please.
    #In any case this trajectory does not follow the path 
    #0 init and end velocities
    def maketraj(q0,q1,T): #TODO compute a real trajectory !
        traj_path = new_path_g(computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET))
        q_of_t = Bezier(traj_path, t_max=T)
        vq_of_t = q_of_t.derivative(1)
        vvq_of_t = vq_of_t.derivative(1)
        return q_of_t, vq_of_t, vvq_of_t
    
    #TODO this is just a random trajectory, you need to do this yourself
    total_time=4. #tried 3 and 5 but taking too long. What does this mean?
    trajs = maketraj(q0, qe, total_time)   
    
    tcur = 0.
    
    q_of_t, vq_of_t, vvq_of_t = trajs
    
#     t_values = np.linspace(0, total_time, num=100)  # Sample 100 points over the time span

    # Sample the position trajectory q_of_t at different time points
#     position_values = np.array([q_of_t(t) for t in t_values])

#     # Plot position (q_of_t) over time
#     plt.figure(figsize=(8, 6))
#     plt.plot(t_values, position_values, label="Position Trajectory (q_of_t)", color='b')



#     # Adding labels and title
#     plt.title("Trajectory Plot")
#     plt.xlabel("Time")
#     plt.ylabel("Position")
#     plt.legend()
#     plt.grid(True)

#     # Show the plot
#     plt.show()

#     sampled_points = []
#     position_values = []

    
    while tcur < total_time:
#         t_values = np.linspace(0, tcur, total_time)
#         sampled_points = np.array([q_of_t(t) for t in t_values])
#         q_t_value = q_of_t(tcur)
#         print("error", q_t_value - path[tcur])
#         sampled_points.append(q_t_value)
#         position_values.append(q_t_value)
        
        rununtil(controllaw, DT, sim, robot, trajs, tcur, cube)
        tcur += DT
    
#     sampled_points = np.array(sampled_points)
#     sampled_points_z = sampled_points[:, 2]

#     fig, ax = plt.subplots()

#     ax.plot(np.arange(len(sampled_points_z)), sampled_points_z, 'b-', label='Trajectory Points')
    
#     # Add title, labels, and grid
#     ax.legend()
#     ax.set_title("Bezier Trajectory")
#     ax.set_xlabel("Time Step / Point Index")
#     ax.set_ylabel("Z Value")
#     ax.grid(True)
    
#     plt.show()
    
#     z_values = [point[2] for point in path]
#     # Plot z_values using plt
#     plt.plot(z_values, 'ro--', label='Z Values')

#     # Add labels and legend to the second plot (using plt)
#     plt.legend()
#     plt.title("Z Dimension of Reference Path")
#     plt.xlabel("Point Index")
#     plt.ylabel("Z Value")
#     plt.grid(True)

#     plt.show()
    

#     # Calculate the error (difference between Bezier and the reference path)
#     errors = []
#     for i, point in enumerate(position_values):
#         # Assuming the reference path is a 2D array where each point corresponds to a state
#         # Here we compare the Z component, but this can be extended to other components as needed
#         error = np.linalg.norm(point - path[i])
#         errors.append(error)

#     # Convert the error list to a NumPy array for easier manipulation
#     errors = np.array(errors)

#     # Calculate the standard deviation of the error
#     std_dev = np.std(errors)

#     # Plotting the error between the Bezier trajectory and the reference path
#     plt.figure(figsize=(8, 6))
#     plt.plot(t_values, errors, label="Error between Path and Bezier", color='r')

#     # Optionally, plot the standard deviation as a shaded area
#     plt.fill_between(t_values, errors - std_dev, errors + std_dev, color='gray', alpha=0.5, label="Standard Deviation")

#     # Add labels and title
#     plt.title("Error between Path and Bezier Curve with Standard Deviation")
#     plt.xlabel("Time")
#     plt.ylabel("Error (Z Value Difference)")
#     plt.legend()
#     plt.grid(True)

#     # Show the plot
#     plt.show()


