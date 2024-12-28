
import time
import math
from lib import puzz_msgs
from my_examples import dead_reckoning as dr
from lib import my_math
import numpy as np
class DriveToGoal():
    def __init__(self):

        self.dead_reckon = dr.DeadReckoning()

        # Target points        
        self.target_x = 2
        self.target_y = 1
        
        self.v_max = 0.2
        self.w_max = 2

        self.w_setR = 0.0
        self.w_setL = 0.0

    def spin(self,topics):

        self.dead_reckon.spin(topics)

        est_pose = self.dead_reckon.pose

        # ====================================================================================================
        # =========== Task 1 - Motion Control ===================================================
        # =========== Start Here =============================================================================
        
        # Hint: You will need to caluclate the distance and angular error first, and then use these information for your propotional controller design. 
        # Note that, you need to set a condition such that the robot will stop moving once it is close enough to the goal, e.g., 0.1 m. 
        # Besides, considering the motor stauration, the directly generated control signal may be unrealistic, how you can address this issue in your controller design?
        # In the end, you will need to obtain the desired left and right motor speed to achieve the goal.
        # Good luck!
        # Calculate distance error to target
        dx = self.target_x - est_pose[0]
        dy = self.target_y - est_pose[1]
        distance_error = np.sqrt(dx**2 + dy**2)
        def wrap_angle(angle):
            return ((angle + np.pi) % (2 * np.pi)) - np.pi
        # Calculate angular error (desired heading vs current heading)
        desired_heading = np.arctan2(dy, dx) 
        angular_error = wrap_angle(desired_heading - est_pose[2])

        # Proportional control gains
        Kp_v = 0.3  # Reduce linear velocity gain
        Kp_w = 1.0  # Reduce angular velocity gain to reduce oscillation

        # 首先初始化 v 和 w
        v = Kp_v * distance_error
        w = Kp_w * angular_error

        # 1. 增大死区阈值
        position_deadzone = 0.08  # 从0.05增加到0.08
        angle_deadzone = 0.15    # 从0.1增加到0.15

        # 2. 更严格的分阶段控制
        if distance_error > 0.3:  # 远距离
            v = Kp_v * distance_error
            w = Kp_w * angular_error
        elif distance_error > 0.15:  # 中等距离
            v = 0.3 * Kp_v * distance_error
            w = 0.4 * Kp_w * angular_error
        else:  # 非常接近
            if distance_error < position_deadzone:
                v = 0
            else:
                v = 0.08 * Kp_v * distance_error  # 进一步降低线速度系数
                
            if abs(angular_error) < angle_deadzone:
                w = 0
            else:
                w = 0.15 * Kp_w * angular_error   # 更低的角速度系数

        # 添加一个计数器，记录稳定时间
        self.stable_count = 0
        self.max_stable_count = 50  # 需要保持稳定的周期数
        # 在停止条件中
        if distance_error < position_deadzone and abs(angular_error) < angle_deadzone:
            self.stable_count += 1
        else:
            self.stable_count = 0

        # 只有当持续稳定一段时间后才真正停止
        if self.stable_count >= self.max_stable_count:
            self.w_setR = 0.0
            self.w_setL = 0.0
            return topics

        # 4. 添加一个额外的保护，防止车轮速度过小时的抖动
        wheel_deadzone = 0.01
        if abs(self.w_setR) < wheel_deadzone:
            self.w_setR = 0
        if abs(self.w_setL) < wheel_deadzone:
            self.w_setL = 0

        # 应用速度限制
        v = np.clip(v, -self.v_max, self.v_max)
        w = np.clip(w, -self.w_max, self.w_max)

        # Convert to wheel velocities
        # Assuming differential drive with wheel radius R and base width L
        R = 0.033  # Wheel radius in meters
        L = 0.17   # Base width in meters

        # Calculate left and right wheel velocities
        self.w_setR = (2*v + w*L)/(2*R)
        self.w_setL = (2*v - w*L)/(2*R)

        # Apply wheel velocity saturation
        wheel_max = self.v_max/R
        self.w_setR = np.clip(self.w_setR, -wheel_max, wheel_max)
        self.w_setL = np.clip(self.w_setL, -wheel_max, wheel_max)
        
        print(f"Target: ({self.target_x}, {self.target_y})")
        print(f"Current pose: ({est_pose[0]:.2f}, {est_pose[1]:.2f}, {est_pose[2]:.2f})")
        print(f"Distance error: {distance_error:.3f}")
        print(f"Angular error: {angular_error:.3f}")

        # ====================================================================================================
        # =========== End Here =============================================================================
        # ====================================================================================================

        # Publish wheel velocities
        msg_w_setR = puzz_msgs.Float32()
        msg_w_setL = puzz_msgs.Float32()
        msg_w_setR.data = self.w_setR
        msg_w_setL.data = self.w_setL

        topics["VelocitySetR"] = msg_w_setR
        topics["VelocitySetL"] = msg_w_setL

        return topics

