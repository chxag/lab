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
    from mpl_toolkits.mplot3d import Axes3D
    
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
    
    trajectory_points = []
    trajectory_velocities = []
    trajectory_accelerations = []
    time = []
    
    while tcur < total_time:
        position = q_of_t(tcur)[:3]  # Extracting the x, y, z position
        
        position_0 = q_of_t(tcur)
        velocity = vq_of_t(tcur)
        acceleration = vvq_of_t(tcur)
       
        trajectory_points.append(position_0)
        trajectory_velocities.append(velocity)
        trajectory_accelerations.append(acceleration)
        time.append(tcur)
        
        rununtil(controllaw, DT, sim, robot, trajs, tcur, cube)
        tcur += DT
        
        
    # Convert trajectory points to numpy array for easy indexing
    trajectory_points = np.array(trajectory_points)
    trajectory_velocities = np.array(trajectory_velocities)
    trajectory_accelerations = np.array(trajectory_accelerations)
    time = np.array(time)
    
    print("Sample trajectory points:", trajectory_points[:5])
    print("Sample trajectory velocities:", trajectory_velocities[:5])
    print("Sample trajectory accelerations:", trajectory_accelerations[:5])
    
#     print(trajectory_points)
    # Plotting the trajectory in 3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(trajectory_points[:, 0], trajectory_points[:, 1], trajectory_points[:, 2], 'b-', label='Trajectory')
    ax.scatter(qe[0], qe[1], qe[2], color='r', label='Goal', zorder=5)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Trajectory")
    ax.legend()
    plt.show()
    
    #path plot 
    # Assuming all necessary imports and previous definitions are present
    
    path = new_path_g(path)
    # Compute 3D positions for each configuration in the path
    x_values = [point[0] for point in path]
    y_values = [point[1] for point in path]
    z_values = [point[2] for point in path]
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x_values, y_values, z_values, 'yo-', label='Computed Path')
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Path")
    ax.legend()
    plt.show()
    
#     def plot_trajectory(trajectory, u_s, p_goal, dt=1):
#     positions = np.array([x[0] for x in trajectory])
#     velocities = np.array([x[1] for x in trajectory])
#     accelerations = np.array(u_s)

    # Create time array for plotting
#     time = np.arange(0, len(trajectory) * dt, dt)

    # Plotting
#     fig, axs = plt.subplots(4, 1, figsize=(10, 15))

#     # Plot trajectory
#     axs[0].plot(positions[:, 0], positions[:, 1], label='Position Trajectory', color='b')
#     axs[0].scatter(p_goal[0], p_goal[1], color='r', label='Goal', zorder=5)  # Plot the goal point   
#     axs[0].set_title('Trajectory')
#     axs[0].set_xlabel('X Position')
#     axs[0].set_ylabel('Y Position')
#     axs[0].grid()
#     axs[0].legend()

    fig, axs = plt.subplots(3, 1, figsize=(10, 15))

    # Position Plot
    axs[0].plot(time, trajectory_points[:, 0], label='X Position', color='b')
    axs[0].plot(time, trajectory_points[:, 1], label='Y Position', color='r')
    axs[0].plot(time, trajectory_points[:, 2], label='Z Position', color='y')
    axs[0].set_title('Position vs Time')
    axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel('Position')
    axs[0].legend()
    axs[0].grid()

    # Velocity Plot
    axs[1].plot(time, trajectory_velocities[:, 0], label='X Velocity', color='b')
    axs[1].plot(time, trajectory_velocities[:, 1], label='Y Velocity', color='r')
    axs[1].plot(time, trajectory_velocities[:, 2], label='Z Velocity', color='y')
    axs[1].set_title('Velocity vs Time')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Velocity')
    axs[1].legend()
    axs[1].grid()

    # Acceleration Plot
    axs[2].plot(time, trajectory_accelerations[:, 0], label='X Acceleration', color='b')
    axs[2].plot(time, trajectory_accelerations[:, 1], label='Y Acceleration', color='r')
    axs[2].plot(time, trajectory_accelerations[:, 2], label='Z Acceleration', color='y')
    axs[2].set_title('Acceleration vs Time')
    axs[2].set_xlabel('Time (s)')
    axs[2].set_ylabel('Acceleration')
    axs[2].legend()
    axs[2].grid()

    plt.tight_layout()
    plt.show()

    
    

    

