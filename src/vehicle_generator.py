"""
车辆生成模块
使用泊松分布生成主线车队和匝道车辆
"""

import numpy as np
from typing import List, Dict, Tuple
from src.config import *


class Vehicle:
    """车辆类"""
    
    def __init__(self, veh_id: str, veh_type: str, platoon_id: str = None,
                 desired_speed: float = 30.0, is_leader: bool = False, 
                 position_in_platoon: int = -1, min_gap: float = None):
        self.id = veh_id
        self.type = veh_type  # 'mainline'、'fast_mainline' or 'ramp'
        self.platoon_id = platoon_id  # 所属车队ID（匝道车辆为None）
        self.is_leader = is_leader  # 是否为车队头车
        self.desired_speed = desired_speed
        self.position_in_platoon = position_in_platoon  # 在车队中的位置（0=队首，-1=非车队成员）
        self.platoon_max_size = None
        self.platoon_remaining_slots = None
        self.current_speed = 0.0
        self.position = 0.0
        self.lane = 0
        self.acceleration = 0.0
        self.min_gap = min_gap
        
    def __repr__(self):
        return f"Vehicle({self.id}, {self.type}, platoon={self.platoon_id})"


class VehicleGenerator:
    """车辆生成器"""
    
    def __init__(self, random_seed=None):
        # 设置随机种子
        if random_seed is not None:
            np.random.seed(random_seed)
            print(f"[车辆生成器] 使用随机种子: {random_seed}")
        else:
            print(f"[车辆生成器] 未设置随机种子，使用系统随机")
        
        self.mainline_counter = 1  # 主线车辆计数（从1开始）
        self.ramp_counter = 1  # 匝道车辆计数（从1开始）
        self.fast_mainline_counter = 1  # 快车道车辆技术（从1开始）
        self.platoon_counter = 0  # 车队计数
        
        self.last_mainline_time = -1000  # 上次生成主线车辆的时间
        self.last_ramp_time = -1000  # 上次生成匝道车辆的时间

        self.platoon_size_counts = {}  # 车队规模统计
        
        # 预先生成时间序列（泊松过程）
        self.mainline_generation_times = self._generate_poisson_times(
            MAINLINE_ARRIVAL_RATE,
            MAINLINE_GENERATION_START,
            MAINLINE_GENERATION_END
        )
        
        self.ramp_generation_times = self._generate_poisson_times(
            RAMP_ARRIVAL_RATE,
            RAMP_GENERATION_START,
            RAMP_GENERATION_END
        )

        self.fast_mainline_generation_times = self._generate_poisson_times(
            FAST_MAINLINE_ARRIVAL_RATE,
            FAST_MAINLINE_GENERATION_START,
            FAST_MAINLINE_GENERATION_END
        )

        self.mainline_idx = 0
        self.ramp_idx = 0
        self.fast_mainline_idx = 0
        
    def _generate_poisson_times(self, rate: float, start_time: float,
                                end_time: float) -> List[float]:
        """
        生成泊松过程的到达时间序列
        
        Args:
            rate: 到达率（事件/秒）
            start_time: 开始时间
            end_time: 结束时间
            
        Returns:
            到达时间列表
        """
        times = []
        current_time = start_time
        
        while current_time < end_time:
            # 生成指数分布的时间间隔
            interval = np.random.exponential(1.0 / rate)
            current_time += interval
            if current_time < end_time:
                times.append(current_time)
                
        return times
    
    def generate_mainline_platoon(self, current_time: float) -> List[Vehicle]:
        """
        生成主线车队（按权重比例随机选择规模）
        
        Args:
            current_time: 当前仿真时间
            
        Returns:
            车队中的车辆列表
        """
        # 按权重比例随机确定车队规模
        sizes = list(PLATOON_SIZE_WEIGHTS.keys())
        weights = list(PLATOON_SIZE_WEIGHTS.values())
        platoon_size = np.random.choice(sizes, p=weights)
        # 完全按随机比例确定车队规模
        # platoon_size = np.random.randint(PLATOON_SIZE_MIN, PLATOON_SIZE_MAX + 1)
        
        # 车队ID
        platoon_id = f"p{self.platoon_counter}"
        self.platoon_counter += 1

        # 更新车队规模统计
        if platoon_size not in self.platoon_size_counts:
            self.platoon_size_counts[platoon_size] = 0
        self.platoon_size_counts[platoon_size] += 1

        # 车队期望速度（带随机扰动）
        platoon_speed = PLATOON_DESIRED_SPEED + np.random.uniform(
            -PLATOON_SPEED_DEVIATION, PLATOON_SPEED_DEVIATION
        )
        platoon_speed = np.clip(platoon_speed, 20.0, MAINLINE_SPEED_LIMIT)
        
        vehicles = []
        for i in range(platoon_size):
            veh_id = f"m{self.mainline_counter}"
            self.mainline_counter += 1
            
            is_leader = (i == 0)
            
            vehicle = Vehicle(
                veh_id=veh_id,
                veh_type='mainline',
                platoon_id=platoon_id,
                desired_speed=platoon_speed,
                is_leader=is_leader,
                position_in_platoon=i  # 直接使用循环变量i作为车队位置
            )
            
            vehicles.append(vehicle)
            
        return vehicles
    
    def generate_ramp_vehicle(self, current_time: float) -> Vehicle:
        """
        生成匝道车辆
        
        Args:
            current_time: 当前仿真时间
            
        Returns:
            匝道车辆对象
        """
        veh_id = f"r{self.ramp_counter}"
        self.ramp_counter += 1
        
        # 匝道车辆期望速度
        desired_speed = RAMP_DESIRED_SPEED + np.random.uniform(
            -RAMP_SPEED_DEVIATION, RAMP_SPEED_DEVIATION
        )
        desired_speed = np.clip(desired_speed, 15.0, RAMP_SPEED_LIMIT)
        
        vehicle = Vehicle(
            veh_id=veh_id,
            veh_type='ramp',
            platoon_id=None,
            desired_speed=desired_speed,
            is_leader=False
        )
        
        return vehicle

    def generate_fast_mainline_vehicle(self, current_time: float) -> Vehicle:
        """生成快车道主线车辆（单车行驶）"""
        veh_id = f"f{self.fast_mainline_counter}"
        self.fast_mainline_counter += 1

        desired_speed = FAST_MAINLINE_DESIRED_SPEED + np.random.uniform(
            -FAST_MAINLINE_SPEED_DEVIATION, FAST_MAINLINE_SPEED_DEVIATION
        )
        desired_speed = np.clip(desired_speed, 20.0, FAST_MAINLINE_LANE_SPEED)

        vehicle = Vehicle(
            veh_id=veh_id,
            veh_type='fast_mainline',
            platoon_id=None,
            desired_speed=desired_speed,
            is_leader=False,
            position_in_platoon=-1,
            min_gap=FAST_MAINLINE_MIN_GAP
        )

        return vehicle

    def get_vehicles_to_generate(self, current_time: float) -> Tuple[List[Vehicle], List[Vehicle], List[Vehicle]]:
        """
        获取当前时刻需要生成的车辆
        
        Args:
            current_time: 当前仿真时间
            
        Returns:
            (主线车辆列表, 匝道车辆列表, 快车道车辆列表)
        """
        mainline_vehicles = []
        ramp_vehicles = []
        fast_mainline_vehicles = []
        
        # 检查是否需要生成主线车队
        while (self.mainline_idx < len(self.mainline_generation_times) and
               self.mainline_generation_times[self.mainline_idx] <= current_time):
            platoon = self.generate_mainline_platoon(current_time)
            mainline_vehicles.extend(platoon)
            self.mainline_idx += 1
            
        # 检查是否需要生成匝道车辆
        while (self.ramp_idx < len(self.ramp_generation_times) and
               self.ramp_generation_times[self.ramp_idx] <= current_time):
            vehicle = self.generate_ramp_vehicle(current_time)
            ramp_vehicles.append(vehicle)
            self.ramp_idx += 1

        # 检查是否需要生成快车道车辆（单车）
        while (self.fast_mainline_idx < len(self.fast_mainline_generation_times) and
                self.fast_mainline_generation_times[self.fast_mainline_idx] <= current_time):
            vehicle = self.generate_fast_mainline_vehicle(current_time)
            fast_mainline_vehicles.append(vehicle)
            self.fast_mainline_idx += 1

        return mainline_vehicles, ramp_vehicles, fast_mainline_vehicles
    
    def get_statistics(self) -> Dict:
        """获取生成统计信息"""
        return {
            'total_mainline_vehicles': self.mainline_counter - 1,  # 减1因为从1开始计数
            'total_ramp_vehicles': self.ramp_counter - 1,
            'total_fast_mainline_vehicles': self.fast_mainline_counter - 1,
            'total_platoons': self.platoon_counter,
            'mainline_generation_events': len(self.mainline_generation_times),
            'ramp_generation_events': len(self.ramp_generation_times),
            'fast_mainline_generation_events': len(self.fast_mainline_generation_times),
            'platoon_size_distribution': self.platoon_size_counts
        }
    
    def print_generation_plan(self):
        """打印车辆生成计划"""
        print("\n" + "="*70)
        print("  车辆生成计划")
        print("="*70)
        print(f"  预计生成主线车队: {len(self.mainline_generation_times)} 个")
        print(f"  预计生成匝道车辆: {len(self.ramp_generation_times)} 辆")
        print(f"  预计生成快车道车辆: {len(self.fast_mainline_generation_times)}辆")
        print(f"  主线车队规模范围: {PLATOON_SIZE_MIN}-{PLATOON_SIZE_MAX} 辆/队")
        print(f"  车队规模权重分布：{PLATOON_SIZE_WEIGHTS}")
        print(f"  预计主线车辆总数: ~{len(self.mainline_generation_times) * (PLATOON_SIZE_MIN + PLATOON_SIZE_MAX) / 2:.0f} 辆")
        print("")
        print("  车辆编号规则:")
        print(f"    - 主线车辆: m1, m2, m3, ... (连续编号)")
        print(f"    - 匝道车辆: r1, r2, r3, ... (连续编号)")
        print(f"    - 快车道车辆: f1, f2, f3, ... (连续编号)")
        print(f"    - 车队编号: p0, p1, p2, ... (连续编号)")
        print("")
        print("  车队内车辆位置:")
        print(f"    - position_in_platoon: 0表示队首，1表示第2辆，以此类推")
        print(f"    - 匝道车辆的position_in_platoon为-1")
        print("="*70 + "\n")

    def _get_expected_platoon_size(self) -> float:
        """
        计算期望的车队规模（加权平均）

        Returns：
            期望的车队规模
        """
        return sum(size * weight for size, weight in PLATOON_SIZE_WEIGHTS.items())


if __name__ == "__main__":
    # 测试车辆生成器
    generator = VehicleGenerator()
    
    print("=== 车辆生成器测试 ===")
    print(f"预计生成主线车队: {len(generator.mainline_generation_times)}")
    print(f"预计生成匝道车辆: {len(generator.ramp_generation_times)}")
    print()
    
    # 模拟前10秒
    for t in np.arange(0, 10, 0.1):
        mainline, ramp, fast_mainline = generator.get_vehicles_to_generate(t)
        
        if mainline:
            print(f"时刻 {t:.1f}s: 生成主线车队，共{len(mainline)}辆")
            for v in mainline:
                print(f"  - {v}")
                
        if ramp:
            print(f"时刻 {t:.1f}s: 生成匝道车辆")
            for v in ramp:
                print(f"  - {v}")

        if fast_mainline:
            print(f"时刻 {t:.1f}s: 生成快车道车辆")
            for v in fast_mainline:
                print(f"  - {v}")
    
    print("\n统计信息:")
    stats = generator.get_statistics()
    for key, value in stats.items():
        print(f"  {key}: {value}")
