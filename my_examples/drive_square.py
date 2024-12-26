import time
from lib import puzz_msgs
import math  
class DriveSquare():
    def __init__(self):
        self.t_start = time.time()
    
    def spin(self,topics):

        t_total = time.time() - self.t_start

        cmdR = puzz_msgs.Float32()
        cmdL = puzz_msgs.Float32()

        # =====================================================================
        # ===== Start Here ====================================================   
        # =====================================================================     
        # Task 2: Linear Motion
        # wheel_radius = 0.05
        # # 计算线速度：角速度 × 半径
        # linear_velocity = 2 * wheel_radius  # 2 rad/s × 半径
        # # 计算所需时间：距离/速度 （0.15m / linear_velocity）
        # required_time = 0.15 / linear_velocity

        # if t_total <= required_time:
        #     # 如果未到达目标距离，设置速度为2rad/s
        #     cmdR.data = 2.0
        #     cmdL.data = 2.0
        # else:
        #     # 到达目标距离后停止
        #     cmdR.data = 0.0
        #     cmdL.data = 0.0
        #     topics["IsDone"] = True

        #Task 3: Turn in Place
        # 计算所需的角速度：90度 = π/2弧度，需要在3秒内完成
        # 角速度 = 转动角度/所需时间 = (π/2)/3 弧度/秒
        # angular_velocity = (math.pi/2) / 3.0
        
        # if t_total <= 3.0:  # 3秒内执行旋转
        #     # 顺时针旋转：右轮后退，左轮前进
        #     cmdR.data = -2*angular_velocity  # 右轮反方向
        #     cmdL.data = 2*angular_velocity   # 左轮正方向
        # else:
        #     # 3秒后停止
        #     cmdR.data = 0.0
        #     cmdL.data = 0.0
        #     topics["IsDone"] = True

        # Task 4: Drive Square
        # 定义运动参数
        # straight_time = 3.0  # 直线运动时间
        # turn_time = 1.0     # 转弯时间
        # straight_speed = 2.0  # 直线速度
        # turn_speed = (math.pi/2) / turn_time  # 转弯角速度
        # phase_time = straight_time + turn_time  # 每个阶段总时间
        
        # # 计算当前所在的阶段（0-3表示四个边）
        # current_phase = int(t_total / phase_time)
        # time_in_phase = t_total % phase_time
        
        # if current_phase < 4:  # 还未完成正方形
        #     if time_in_phase <= straight_time:  # 直线运动
        #         cmdR.data = straight_speed
        #         cmdL.data = straight_speed
        #     elif time_in_phase <= phase_time:  # 转弯
        #         cmdR.data = -turn_speed
        #         cmdL.data = turn_speed
        # else:  # 完成正方形
        #     cmdR.data = 0.0
        #     cmdL.data = 0.0
        #     topics["IsDone"] = True


        # ===================================================================
        # ===== End Here ====================================================   
        # ===================================================================  
        topics["VelocitySetR"] = cmdR
        topics["VelocitySetL"] = cmdL

        return topics