import time
import math
from lib import my_math
from lib import puzz_msgs
import numpy as np

class DeadReckoning():
    def __init__(self):
        self.pose = [0, 0, 0]

        self.k = 0.1

        self.w_r = 0.0 # right wheel velocities
        self.w_l = 0.0 # left wheel velocities

        self.R = 0.05 # radius
        self.L = 0.18 # wheel distance

        self.Sig = np.array([[0.0, 0.0, 0.0],
                             [0.0, 0.0, 0.0],
                             [0.0, 0.0, 0.0]])
        
        self.t_start = time.time()
    
    def spin(self,topics):

        dt = time.time() - self.t_start
        self.t_start = time.time()

        # Read wheel angular velocities from topics (msg type is Float32)
        if "VelocityEncR" in topics:
            self.w_r = topics["VelocityEncR"].data
        if "VelocityEncL" in topics:
            self.w_l = topics["VelocityEncL"].data
        if "Pose" in topics:
            self.pose = topics["Pose"].pose
            self.Sig = topics["Pose"].cov


        # ====================================================================================================
        # =========== Task 1 - Dead Reckoning Localization ===================================================
        # =========== Start Here =============================================================================

        # Compute linear and angular velocities of the robot
        # linear  velocities 
        v_l = (self.R * self.w_r + self.R * self.w_l)/2
        # angular  velocities 
        v_w = (self.R * self.w_r - self.R * self.w_l)/self.L

        # Update pose (self.pose)
        # current robot pose
        x,y,theta = self.pose
        # update x,y,theta
        x_new  = x + v_l*np.cos(theta)*dt
        y_new  = y + v_l*np.sin(theta)*dt
        theta_new  = theta + v_w*dt

        # update pose
        self.pose=[x_new,y_new,theta_new]

        # Computer robot coveriance Sig (self.Sig), using the jacobian matrix H and the covariance matrix Q.
        H_K = np.array([[1, 0, -dt*v_l*np.sin(self.pose[self.k-1][2])],
                             [0, 1, dt*v_l*np.cos(self.pose[self.k-1][2])],
                             [0, 0, 1]])
        H_T_K = H_K.T # # Use transpose directly
        # v = R(ωr + ωl)/2
        # 由误差传播定律：
        sigma_v_squared = (self.R/2)**2 * (self.w_r + self.w_l)
        # ω = R(ωr - ωl)/L
        # L是轮距(self.L = 0.18)
        sigma_omega_squared = (self.R/self.L)**2 * (self.w_r + self.w_l)
        # θ = θ_previous + ω*dt
        sigma_theta_squared = sigma_omega_squared * (dt**2)
        Q_K = np.array([[sigma_v_squared,0,0],
                             [0,sigma_omega_squared,0],
                             [0,0,sigma_theta_squared]])
        self.Sig = np.dot(H_K, np.dot(self.Sig, H_T_K)) + Q_K # Matrix Multiplication
        # ====================================================================================================
        # =========== End Here =============================================================================
        # ====================================================================================================


        # Publish dead-reckoning pose and covariance
        msg_pose = puzz_msgs.Pose()
        msg_pose.pose = self.pose
        msg_pose.cov = self.Sig

        topics["Pose"] = msg_pose

        return topics

