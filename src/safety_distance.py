"""
安全距离计算模块
根据车辆速度和类型动态计算安全距离
"""

import numpy as np
from src.config import *


class SafetyDistanceCalculator:
    """安全距离计算器"""
    
    def __init__(self):
        pass
    
    def calculate_safe_distance(self, follower_speed: float, leader_speed: float,
                               is_within_platoon: bool = False) -> float:
        """
        计算安全距离
        
        公式: d = v_f * T + (v_f² - v_l²)/(2*b) + d_min
        
        Args:
            follower_speed: 跟随车速度 (m/s)
            leader_speed: 前车速度 (m/s)
            is_within_platoon: 是否为车队内跟驰
            
        Returns:
            安全距离 (m)
        """
        if is_within_platoon:
            # 车队内安全距离（更紧密）
            time_headway = SAFE_TIME_HEADWAY_WITHIN
            min_gap = MIN_GAP_WITHIN
        else:
            # 车队间安全距离
            time_headway = SAFE_TIME_HEADWAY_BETWEEN
            min_gap = MIN_GAP_BETWEEN
        
        # 基于速度的间距
        speed_term = follower_speed * time_headway
        
        # 基于速度差的制动距离
        speed_diff = follower_speed - leader_speed
        if speed_diff > 0:
            # 跟随车更快，需要额外制动距离
            brake_term = (speed_diff ** 2) / (2 * SAFE_DECEL)
        else:
            brake_term = 0.0
        
        # 总安全距离
        safe_distance = speed_term + brake_term + min_gap
        
        return max(safe_distance, min_gap)
    
    def calculate_desired_gap_idm(self, follower_speed: float, leader_speed: float,
                                  is_within_platoon: bool = False) -> float:
        """
        使用IDM模型计算期望间距
        
        d* = d_min + v*T + v*Δv/(2*√(a*b))
        
        Args:
            follower_speed: 跟随车速度 (m/s)
            leader_speed: 前车速度 (m/s)
            is_within_platoon: 是否为车队内跟驰
            
        Returns:
            期望间距 (m)
        """
        if is_within_platoon:
            time_headway = SAFE_TIME_HEADWAY_WITHIN
            min_gap = MIN_GAP_WITHIN
        else:
            time_headway = SAFE_TIME_HEADWAY_BETWEEN
            min_gap = MIN_GAP_BETWEEN
        
        # IDM期望间距
        v = follower_speed
        dv = follower_speed - leader_speed
        
        desired_gap = (min_gap + 
                      v * time_headway + 
                      v * dv / (2 * np.sqrt(IDM_A * IDM_B)))
        
        return max(desired_gap, min_gap)
    
    def calculate_idm_acceleration(self, follower_speed: float, leader_speed: float,
                                  current_gap: float, desired_speed: float,
                                  is_within_platoon: bool = False) -> float:
        """
        使用IDM模型计算加速度
        
        a = a_max * [1 - (v/v_desired)^δ - (d*/d)²]
        
        Args:
            follower_speed: 跟随车速度 (m/s)
            leader_speed: 前车速度 (m/s)
            current_gap: 当前间距 (m)
            desired_speed: 期望速度 (m/s)
            is_within_platoon: 是否为车队内跟驰
            
        Returns:
            加速度 (m/s²)
        """
        # 避免除零
        if current_gap < 0.1:
            return -MAX_DECELERATION
        
        # 期望间距
        desired_gap = self.calculate_desired_gap_idm(
            follower_speed, leader_speed, is_within_platoon
        )
        
        # 自由流加速项
        v_ratio = follower_speed / max(desired_speed, 1.0)
        free_term = 1.0 - (v_ratio ** IDM_DELTA)
        
        # 交互项
        gap_ratio = desired_gap / current_gap
        interaction_term = gap_ratio ** 2
        
        # 总加速度
        acceleration = IDM_A * (free_term - interaction_term)
        
        # 限制加速度范围
        acceleration = np.clip(acceleration, -MAX_DECELERATION, MAX_ACCELERATION)
        
        return acceleration
    
    def check_safety(self, follower_speed: float, leader_speed: float,
                    current_gap: float, is_within_platoon: bool = False) -> bool:
        """
        检查当前间距是否安全
        
        Args:
            follower_speed: 跟随车速度 (m/s)
            leader_speed: 前车速度 (m/s)
            current_gap: 当前间距 (m)
            is_within_platoon: 是否为车队内跟驰
            
        Returns:
            是否安全
        """
        required_distance = self.calculate_safe_distance(
            follower_speed, leader_speed, is_within_platoon
        )
        
        return current_gap >= required_distance
    
    def calculate_time_to_collision(self, follower_speed: float, leader_speed: float,
                                   current_gap: float) -> float:
        """
        计算碰撞时间 (TTC)
        
        Args:
            follower_speed: 跟随车速度 (m/s)
            leader_speed: 前车速度 (m/s)
            current_gap: 当前间距 (m)
            
        Returns:
            碰撞时间 (s)，如果不会碰撞返回inf
        """
        relative_speed = follower_speed - leader_speed
        
        if relative_speed <= 0:
            # 跟随车不比前车快，不会碰撞
            return float('inf')
        
        ttc = current_gap / relative_speed
        return ttc


if __name__ == "__main__":
    # 测试安全距离计算
    calculator = SafetyDistanceCalculator()
    
    print("=== 安全距离计算器测试 ===\n")
    
    # 测试场景1：车队间跟驰
    print("场景1: 车队间跟驰")
    follower_v = 30.0  # m/s
    leader_v = 28.0  # m/s
    safe_dist = calculator.calculate_safe_distance(follower_v, leader_v, False)
    print(f"跟随车速度: {follower_v} m/s, 前车速度: {leader_v} m/s")
    print(f"安全距离: {safe_dist:.2f} m")
    
    current_gap = 50.0
    is_safe = calculator.check_safety(follower_v, leader_v, current_gap, False)
    print(f"当前间距: {current_gap} m, 是否安全: {is_safe}")
    
    ttc = calculator.calculate_time_to_collision(follower_v, leader_v, current_gap)
    print(f"碰撞时间: {ttc:.2f} s\n")
    
    # 测试场景2: 车队内跟驰
    print("场景2: 车队内跟驰")
    safe_dist_within = calculator.calculate_safe_distance(follower_v, leader_v, True)
    print(f"跟随车速度: {follower_v} m/s, 前车速度: {leader_v} m/s")
    print(f"安全距离: {safe_dist_within:.2f} m")
    print(f"(注意: 车队内安全距离更小)\n")
    
    # 测试场景3: IDM加速度计算
    print("场景3: IDM加速度计算")
    desired_speed = 30.0
    current_gap = 40.0
    acceleration = calculator.calculate_idm_acceleration(
        follower_v, leader_v, current_gap, desired_speed, False
    )
    print(f"当前间距: {current_gap} m, 期望速度: {desired_speed} m/s")
    print(f"计算加速度: {acceleration:.3f} m/s²")
    
    # 测试不同间距下的加速度
    print("\n不同间距下的IDM加速度:")
    for gap in [20, 30, 40, 50, 60, 80, 100]:
        acc = calculator.calculate_idm_acceleration(
            follower_v, leader_v, gap, desired_speed, False
        )
        print(f"  间距 {gap:3d}m -> 加速度 {acc:6.3f} m/s²")


