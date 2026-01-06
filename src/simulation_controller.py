"""
仿真控制主程序
负责SUMO仿真的初始化、运行和控制
"""

import os
import sys
import traci
import numpy as np
from typing import Dict, List, Tuple
from src.config import *
from src.config import USE_OPTIMIZATION
from src.vehicle_generator import VehicleGenerator
from src.safety_distance import SafetyDistanceCalculator
from src.merge_optimizer import MergeOptimizer
from src.data_collector import DataCollector
from src.logger import SimulationLogger, PerformanceLogger
from src.tripinfo_analyzer import TripInfoAnalyzer


class SimulationController:
    """仿真控制器"""
    
    def __init__(self, use_gui: bool = True, random_seed: int = None, use_optimization: bool = None):
        self.use_gui = use_gui
        # 设置优化算法开关：如果参数为None则使用配置文件中的默认值
        if use_optimization is None:
            self.use_optimization = USE_OPTIMIZATION
        else:
            self.use_optimization = use_optimization
        self.enable_platoon_reorg = ENABLE_PLATOON_REORG
        self.mode_description = self._get_mode_description()
        # 在初始化日志中记录当前模式
        print(f"[仿真控制器]模式设置:{self.mode_description}")

        self.vehicle_generator = VehicleGenerator(random_seed=random_seed)
        self.safety_calculator = SafetyDistanceCalculator()
        self.merge_optimizer = MergeOptimizer()
        self.data_collector = DataCollector(use_optimization=self.use_optimization)
        self.logger = SimulationLogger()
        self.perf_logger = PerformanceLogger()
        
        # 车辆跟踪
        self.active_vehicles = {}  # 当前活动车辆
        self.vehicle_info = {}  # 车辆详细信息
        self.platoons: Dict[str, Dict[str, int]] = {}  # 车队状态（最大规模、当前规模、剩余容量）
        
        # 合流优化控制
        self.last_optimization_time = -OPTIMIZATION_INTERVAL
        self.current_sequence = []
        
        # 统计计数
        self.total_generated_mainline = 0
        self.total_generated_ramp = 0
        self.total_generated_fast_mainline = 0
        self.optimization_count = 0

        # 记录匝道车完成合流的事件（避免重复记录）
        self.ramp_merge_records: Dict[str, Dict[str, float]] = {}

    def _get_mode_description(self) -> str:
        """根据优化和车队重组开关返回本次仿真的模式描述"""
        if not self.use_optimization:
            return "FIFO模式"
        if self.enable_platoon_reorg:
            return "优化+车队重组模式"
        return "优化（不重组）模式"

    def _register_platoon(self, platoon_id: str, initial_size: int):
        """注册新车队并设置最大容量/剩余容量"""
        max_size = PLATOON_SIZE_MAX
        self.platoons[platoon_id] = {
            'max_size': max_size,
            'current_size': initial_size,
            'remaining_slots': max(max_size - initial_size, 0)
        }
        self._sync_platoon_state_to_vehicles()

    def _update_platoon_states(self):
        """根据当前活动车辆刷新车队规模和剩余容量"""
        platoon_counts: Dict[str, int] = {}  # 统计每个车队的当前车辆数

        # 遍历sumo中的车辆，确保统计的是当前仿真里真实存在的车辆
        for veh_id in traci.vehicle.getIDList():
            platoon_id = None

            # 优先从vehicle_info读取（始终保存车队元数据）
            if veh_id in self.vehicle_info:
                platoon_id = self.vehicle_info[veh_id].get('platoon_id')
            elif veh_id in self.active_vehicles:
                platoon_id = getattr(self.active_vehicles[veh_id], 'platoon_id', None)

            if platoon_id is None:  # 跳过没有车队ID的车辆（匝道车）
                continue

            platoon_counts[platoon_id] = platoon_counts.get(platoon_id, 0) + 1  # 车队数量统计，车队ID从0开始，因此+1

        for platoon_id, count in platoon_counts.items():
            if platoon_id not in self.platoons:
                # 新发现的车队，使用默认的最大容量
                max_size = PLATOON_SIZE_MAX
                capped_size = min(count, max_size)
                self.platoons[platoon_id] = {
                    'max_size': max_size,
                    'current_size': capped_size,
                    'remaining_slots': max(max_size - capped_size, 0)
                }
            else:  # 处理已存在的车队，更新当前规模和剩余空位
                state = self.platoons[platoon_id]
                state['max_size'] = min(state.get('max_size', PLATOON_SIZE_MAX), PLATOON_SIZE_MAX)
                capped_size = min(count, state['max_size'])
                state['current_size'] = capped_size
                state['remaining_slots'] = max(state['max_size'] - capped_size, 0)

        # 未在当前活动列表中的车队规模标记为0
        for platoon_id, state in self.platoons.items():
            if platoon_id not in platoon_counts:
                state['current_size'] = 0
                state['remaining_slots'] = max(state['max_size'], 0)

        self._sync_platoon_state_to_vehicles()

    def _sync_platoon_state_to_vehicles(self):
        """将车队容量信息同步到车辆对象和车辆信息表中"""
        # 遍历所有活跃车辆，跳过没有车队ID的车辆（筛选出需要同步信息的车辆对象）
        for veh_id, vehicle in self.active_vehicles.items():
            platoon_id = getattr(vehicle, 'platoon_id', None)
            if platoon_id is None:
                continue
            state = self.platoons.get(platoon_id)
            if not state:
                continue

            # 将车队状态信息直接复制给车辆对象的属性，同时更新车辆信息表
            vehicle.platoon_max_size = state['max_size']
            vehicle.platoon_remaining_slots = state['remaining_slots']
            if veh_id in self.vehicle_info:
                self.vehicle_info[veh_id]['platoon_max_size'] = state['max_size']
                self.vehicle_info[veh_id]['platoon_remaining_slots'] = state['remaining_slots']

    def _commit_platoon_join(self, ramp_vehicle_id: str, platoon_id: str) -> int:
        """将匝道车加入指定车队并刷新容量信息，返回车队内位置"""
        if ramp_vehicle_id in self.vehicle_info:
            self.vehicle_info[ramp_vehicle_id]['platoon_id'] = platoon_id
        else:
            self.vehicle_info[ramp_vehicle_id] = {
                'type': 'ramp',
                'platoon_id': platoon_id,
                'position_in_platoon': -1,
                'is_leader': True,
                'desired_speed': RAMP_DESIRED_SPEED,
                'depart_time': traci.simulation.getTime()
            }

        if ramp_vehicle_id in self.active_vehicles:
            setattr(self.active_vehicles[ramp_vehicle_id], 'platoon_id', platoon_id)

        # 若车队不存在则创建默认容量，存在则增加当前规模
        if platoon_id not in self.platoons:
            self._register_platoon(platoon_id, 0)

        state = self.platoons.get(platoon_id, {})
        state['max_size'] = min(state.get('max_size', PLATOON_SIZE_MAX), PLATOON_SIZE_MAX)
        if state.get('current_size', 0) >= state['max_size']:
            # 车队已满，跳过加入
            state['current_size'] = state['max_size']
            state['remaining_slots'] = 0
            self._sync_platoon_state_to_vehicles()
            if ramp_vehicle_id in self.vehicle_info:
                veh_info = self.vehicle_info[ramp_vehicle_id]
                veh_info['position_in_platoon'] = max(state.get('current_size', 0) - 1, 0)
            return state.get('current_size', 0) - 1

        join_position = state.get('current_size', 0)

        state['current_size'] = min(join_position + 1, state['max_size'])
        state['remaining_slots'] = max(state['max_size'] - state['current_size'], 0)

        # 更新车辆的车队位置信息
        if ramp_vehicle_id in self.vehicle_info:
            veh_info = self.vehicle_info[ramp_vehicle_id]
            veh_info['platoon_id'] = platoon_id
            veh_info['position_in_platoon'] = join_position
            veh_info['is_leader'] = veh_info.get('is_leader', False) or join_position == 0

        if ramp_vehicle_id in self.active_vehicles:
            vehicle = self.active_vehicles[ramp_vehicle_id]
            vehicle.platoon_id = platoon_id
            vehicle.position_in_platoon = join_position
            vehicle.is_leader = getattr(vehicle, 'is_leader', False) or join_position == 0

        self.platoons[platoon_id] = state
        self._sync_platoon_state_to_vehicles()

        return join_position

    def _platoon_has_capacity(self, platoon_id: str) -> bool:
        """检查指定车队当前是否还有剩余容量"""
        state = self.platoons.get(platoon_id)
        if not state:
            # 未注册的车队按默认容量处理
            return True
        return state.get('current_size', 0) < state.get('max_size', PLATOON_SIZE_MAX)

    def _ensure_ramp_platoon_membership(self, veh_id: str) -> Tuple[str, int]:
        """确保匝道车在完成合流后拥有车队编号并返回其位置"""
        veh_info = self.vehicle_info.get(veh_id, {'type': 'ramp'})
        self.vehicle_info.setdefault(veh_id, veh_info)

        platoon_id = veh_info.get('platoon_id')
        if platoon_id is None:
            platoon_id = f"p{self.vehicle_generator.platoon_counter}"
            self.vehicle_generator.platoon_counter += 1

        join_position = self._commit_platoon_join(veh_id, platoon_id)
        return platoon_id, join_position

    def _handle_ramp_merge_completion(self, veh_id: str, lane_id: str, current_time: float):
        """检测匝道车完成合流后的记录与车队同步"""
        if veh_id in self.ramp_merge_records:
            return

        if 'ramp' in lane_id or 'mainline_merge_0' in lane_id or lane_id.startswith(":"):
            return

        platoon_id, join_position = self._ensure_ramp_platoon_membership(veh_id)
        self.ramp_merge_records[veh_id] = {
            'time': current_time,
            'platoon_id': platoon_id
        }

        self.logger.log(
            f"匝道车{veh_id}完成合流换道，加入车队{platoon_id}，位置索引{join_position}，时间{current_time:.1f}s"
        )

    def initialize_sumo(self):
        """初始化SUMO仿真"""
        # 记录配置参数
        config = {
            '仿真时长': f"{SIMULATION_TIME}秒",
            '仿真步长': f"{SIMULATION_STEP}秒",
            '数据采集间隔': f"{DATA_COLLECTION_INTERVAL}秒",
            '主线车道数': f"{MAINLINE_LANES}条（车道1快车道+车道2慢车道）",
            '主线长度': f"{MAINLINE_LENGTH}米",
            '匝道长度': f"{RAMP_LENGTH}米",
            '加速车道长度': f"{RAMP_ACCELERATION_LANE}米",
            '合流区域': f"{MERGE_ZONE_START}-{MERGE_ZONE_START+MERGE_ZONE_LENGTH}米",
            '主线到达率': f"{MAINLINE_ARRIVAL_RATE}车队/秒",
            '匝道到达率': f"{RAMP_ARRIVAL_RATE}车辆/秒",
            '车队规模': f"{PLATOON_SIZE_MIN}-{PLATOON_SIZE_MAX}辆",
            '主线车队期望速度': f"{PLATOON_DESIRED_SPEED}m/s ({PLATOON_DESIRED_SPEED*3.6:.1f}km/h)",
            '匝道车辆期望速度': f"{RAMP_DESIRED_SPEED}m/s ({RAMP_DESIRED_SPEED*3.6:.1f}km/h)",
            '启用优化算法': self.use_optimization,
            '启用车队重组': self.enable_platoon_reorg,
            '仿真模式': self.mode_description,
            '车队内安全时间间隔': f"{SAFE_TIME_HEADWAY_WITHIN}秒",
            '车队间安全时间间隔': f"{SAFE_TIME_HEADWAY_BETWEEN}秒",
            '主线权重': WEIGHT_MAINLINE_TIME,
            '匝道权重': WEIGHT_RAMP_FAILED,
            '优化间隔': f"{OPTIMIZATION_INTERVAL}秒"
        }
        self.logger.log_simulation_start(config)
        
        # 检查SUMO_HOME环境变量
        if 'SUMO_HOME' in os.environ:
            tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
            sys.path.append(tools)
            self.logger.log(f"SUMO_HOME: {os.environ['SUMO_HOME']}")
        else:
            self.logger.log_error("SUMO配置", "未设置SUMO_HOME环境变量")
            sys.exit("请设置环境变量 'SUMO_HOME'")
        
        # 选择SUMO可执行文件
        if self.use_gui:
            sumo_binary = "sumo-gui"
        else:
            sumo_binary = "sumo"
        
        # SUMO启动参数
        sumo_cmd = [
            sumo_binary,
            "-c", SUMO_CONFIG_FILE,
            "--step-length", str(SIMULATION_STEP),
            "--collision.action", "warn",
            "--no-warnings", "true",
            "--duration-log.disable", "true",
            "--no-step-log", "true",
            "--tripinfo-output", TRIPINFO_OUTPUT_FILE,
            "--tripinfo-output.write-unfinished", "true"
        ]
        
        # 启动SUMO
        traci.start(sumo_cmd)
        print("SUMO仿真已启动")
        self.logger.log("SUMO仿真引擎已成功启动")
    
    def add_vehicle_to_sumo(self, vehicle, route_id: str, depart_lane: int = 0,
                            depart_pos: float = 0.0, depart_speed: float = 0.0):
        """
        在SUMO中添加车辆
        
        Args:
            vehicle: 车辆对象
            route_id: 路由ID
            depart_lane: 出发车道
            depart_pos: 出发位置
            depart_speed: 出发速度
        """
        try:
            # 根据车辆类型选择车辆类型ID
            if vehicle.type in ('mainline', 'fast_mainline'):
                type_id = "mainline_vehicle"
            else:
                type_id = "ramp_vehicle"
            
            # 添加车辆到SUMO
            traci.vehicle.add(
                vehID=vehicle.id,
                routeID=route_id,
                typeID=type_id,
                depart="now",
                departLane=str(depart_lane),
                departPos=str(depart_pos),
                departSpeed=str(depart_speed)
            )
            
            # 设置期望速度
            traci.vehicle.setMaxSpeed(vehicle.id, vehicle.desired_speed)
            
            # 设置车辆颜色和变道模式
            if vehicle.type == 'mainline':
                traci.vehicle.setColor(vehicle.id, (0, 100, 255, 255))  # 蓝色
                # 主线车辆：完全禁止换道，保持在车道0（慢车道）
                traci.vehicle.setLaneChangeMode(vehicle.id, 0b000000000000)  # 完全禁止换道
                # 设置速度模式：允许正常跟驰
                traci.vehicle.setSpeedMode(vehicle.id, 0b011111)
                
                # 为车队跟随车辆设置更紧密的跟驰参数
                if not vehicle.is_leader:
                    # 车队跟随车辆：设置更小的tau值以保持更紧密的间距
                    traci.vehicle.setTau(vehicle.id, 0.8)  # 比队首车辆更小的反应时间
                    traci.vehicle.setMinGap(vehicle.id, MIN_GAP_WITHIN)  # 最小间距
                else:
                    # 车队队首车辆：使用正常的tau值
                    traci.vehicle.setTau(vehicle.id, SAFE_TIME_HEADWAY_WITHIN)
            elif vehicle.type == 'fast_mainline':
                try:
                    fast_color = tuple(int(float(c.strip()) * 255) if float(c.strip()) <= 1 else int(float(c.strip()))
                                       for c in FAST_MAINLINE_COLOR.split(','))
                    if len(fast_color) == 3:
                        fast_color = (*fast_color, 255)
                except Exception:
                    fast_color = (255, 0, 0, 255)
                traci.vehicle.setColor(vehicle.id, fast_color)
                traci.vehicle.setLaneChangeMode(vehicle.id, 0b000000000000)
                traci.vehicle.setSpeedMode(vehicle.id, 0b011111)
                traci.vehicle.setTau(vehicle.id, FAST_TAU)
                traci.vehicle.setMinGap(vehicle.id, vehicle.min_gap or FAST_MAINLINE_MIN_GAP)
                traci.vehicle.setAccel(vehicle.id, FAST_IDM_A)
                traci.vehicle.setDecel(vehicle.id, FAST_IDM_B)
                traci.vehicle.setEmergencyDecel(vehicle.id, FAST_MAINLINE_EMERGENCY_DECEL)
                traci.vehicle.setMaxSpeed(vehicle.id, FAST_MAINLINE_LANE_SPEED)
                traci.vehicle.setLength(vehicle.id, FAST_MAINLINE_VEHICLE_LENGTH)
            else:
                traci.vehicle.setColor(vehicle.id, (255, 200, 0, 255))  # 黄色
                # 匝道车辆：完全禁止SUMO自主换道，只通过TraCI控制换道
                traci.vehicle.setLaneChangeMode(vehicle.id, 0b000000000000)  # 完全禁止换道
                traci.vehicle.setSpeedMode(vehicle.id, 0b011111)  # 允许正常速度控制
                # 禁用所有换道参数，确保SUMO不会自主换道
                try:
                    traci.vehicle.setParameter(vehicle.id, "laneChangeModel.lcStrategic", "0")
                    traci.vehicle.setParameter(vehicle.id, "laneChangeModel.lcSpeedGain", "0")
                    traci.vehicle.setParameter(vehicle.id, "laneChangeModel.lcCooperative", "0")
                    traci.vehicle.setParameter(vehicle.id, "laneChangeModel.lcKeepRight", "0")
                    traci.vehicle.setParameter(vehicle.id, "laneChangeModel.lcAssertive", "0")
                except:
                    pass
            
            # 记录车辆信息
            self.active_vehicles[vehicle.id] = vehicle
            
            # 使用车辆对象中的车队位置信息
            position_in_platoon = getattr(vehicle, 'position_in_platoon', -1)
            
            self.vehicle_info[vehicle.id] = {
                'type': vehicle.type,
                'platoon_id': vehicle.platoon_id,
                'position_in_platoon': position_in_platoon,
                'is_leader': vehicle.is_leader,
                'desired_speed': vehicle.desired_speed,
                'depart_time': traci.simulation.getTime(),
                'platoon_max_size': getattr(vehicle, 'platoon_max_size', None),
                'platoon_remaining_slots': getattr(vehicle, 'platoon_remaining_slots', None)
            }
            
            return True
            
        except traci.exceptions.TraCIException as e:
            print(f"添加车辆失败 {vehicle.id}: {e}")
            return False
    
    def update_vehicle_states(self, current_time: float = None):
        """更新所有车辆状态"""
        current_vehicles = traci.vehicle.getIDList()
        
        for veh_id in current_vehicles:
            if veh_id in self.active_vehicles:
                # 更新车辆状态
                vehicle = self.active_vehicles[veh_id]
                vehicle.position = traci.vehicle.getPosition(veh_id)[0]  # x坐标
                vehicle.current_speed = traci.vehicle.getSpeed(veh_id)
                vehicle.acceleration = traci.vehicle.getAcceleration(veh_id)
                vehicle.lane = traci.vehicle.getLaneIndex(veh_id)
                
                # 【全局检查】禁止任何车辆进入lane 2（最上侧车道）
                current_lane_index = traci.vehicle.getLaneIndex(veh_id)
                current_lane_id = traci.vehicle.getLaneID(veh_id)
                
                # 检查所有可能在lane2的情况：mainline_merge、merge_end等
                if current_lane_index >= 2:
                    # 检测到车辆在lane2，立即强制换回lane1
                    try:
                        traci.vehicle.setLaneChangeMode(veh_id, 0b000000000000)  # 禁止换道
                        # 强制换到lane1（如果是mainline_merge）或lane0（如果是mainline_after）
                        if 'mainline_merge' in current_lane_id or ':merge' in current_lane_id:
                            traci.vehicle.changeLane(veh_id, 1, 0)  # 立即换到lane1
                        elif 'mainline_after' in current_lane_id:
                            traci.vehicle.changeLane(veh_id, 0, 0)  # 立即换到lane0
                        else:
                            traci.vehicle.changeLane(veh_id, 1, 0)  # 默认换到lane1
                        # 禁用所有换道参数
                        try:
                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcStrategic", "0")
                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcSpeedGain", "0")
                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcCooperative", "0")
                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcKeepRight", "0")
                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcAssertive", "0")
                        except:
                            pass
                        # 降低速度确保安全
                        current_speed = traci.vehicle.getSpeed(veh_id)
                        traci.vehicle.setSpeed(veh_id, max(8.0, current_speed * 0.8))
                    except:
                        pass
                
                # 匝道车辆主动变道策略：从加速车道（车道0，最右侧）合流到主线慢车道（车道1，中间）
                if vehicle.type == 'ramp':
                    try:
                        lane_id = traci.vehicle.getLaneID(veh_id)
                        lane_index = traci.vehicle.getLaneIndex(veh_id)
                        position = traci.vehicle.getLanePosition(veh_id)
                        
                        # 如果在mainline_merge的加速车道（车道0，最右侧）上
                        if 'mainline_merge' in lane_id and lane_index == 0:
                            lane_length = traci.lane.getLength(lane_id)
                            remaining_distance = lane_length - position
                            
                            # 在加速车道上行驶一段后开始尝试向左换道到车道1（慢车道）
                            if position > 20:  # 至少行驶20米后开始尝试换道
                                try:
                                    # 【关键修复】先检查lane1是否有空隙，如果没有空隙，完全禁止换道
                                    # 车辆应该在lane0等待，而不是跳到lane2
                                    
                                    # 检查lane1的空隙情况
                                    try:
                                        left_leaders = traci.vehicle.getLeftLeaders(veh_id)
                                        left_followers = traci.vehicle.getLeftFollowers(veh_id)
                                        
                                        # 判断lane1是否有足够空隙
                                        has_safe_gap = True
                                        min_gap = 999.0
                                        
                                        if left_leaders:
                                            for leader_id, gap in left_leaders:
                                                min_gap = min(min_gap, gap)
                                                # 如果前车间隙小于15米，认为不安全
                                                if gap < 15.0:
                                                    has_safe_gap = False
                                        
                                        # 检查后车距离
                                        if left_followers:
                                            for follower_id, gap in left_followers:
                                                # 如果后车距离太近（<5米），可能不安全
                                                if gap < 5.0:
                                                    has_safe_gap = False
                                    except:
                                        # 无法检查间隙时，默认没有安全间隙，禁止换道
                                        has_safe_gap = False
                                    
                                    check_lane_index = traci.vehicle.getLaneIndex(veh_id)
                                    
                                    # 【关键】只有当lane1有空隙时，才允许尝试换道
                                    # 如果lane1拥堵，车辆应该在lane0等待，完全禁止换道
                                    if check_lane_index == 0 and has_safe_gap:
                                        # lane1有空隙，允许换到lane1（相邻车道）
                                        # 使用最小权限：只允许strategic left到相邻车道
                                        traci.vehicle.setLaneChangeMode(veh_id, 0b000000000010)  # 只允许strategic left
                                        try:
                                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcStrategic", "0.1")  # 极小值
                                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcSpeedGain", "0")  # 禁止speedGain（防止跳到lane2）
                                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcCooperative", "0")
                                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcKeepRight", "0")
                                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcAssertive", "0")
                                        except:
                                            pass
                                        # 使用极短duration，只换到相邻车道（lane1）
                                        traci.vehicle.changeLane(veh_id, 1, 0.1)
                                        
                                        # 【立即检查】如果错误换到了lane2，立即纠正
                                        immediate_check = traci.vehicle.getLaneIndex(veh_id)
                                        if immediate_check >= 2:
                                            # 立即强制换回lane1或lane0
                                            traci.vehicle.setLaneChangeMode(veh_id, 0b000000000000)
                                            traci.vehicle.changeLane(veh_id, 1, 0)
                                            try:
                                                traci.vehicle.setParameter(veh_id, "laneChangeModel.lcStrategic", "0")
                                                traci.vehicle.setParameter(veh_id, "laneChangeModel.lcSpeedGain", "0")
                                                traci.vehicle.setParameter(veh_id, "laneChangeModel.lcCooperative", "0")
                                                traci.vehicle.setParameter(veh_id, "laneChangeModel.lcKeepRight", "0")
                                                traci.vehicle.setParameter(veh_id, "laneChangeModel.lcAssertive", "0")
                                            except:
                                                pass
                                    elif check_lane_index == 0 and not has_safe_gap:
                                        # 【关键】lane1拥堵，没有空隙，完全禁止换道
                                        # 车辆应该在lane0等待，不允许任何换道（包括到lane2）
                                        traci.vehicle.setLaneChangeMode(veh_id, 0b000000000000)  # 完全禁止换道
                                        try:
                                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcStrategic", "0")
                                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcSpeedGain", "0")  # 禁止speedGain（防止跳到lane2）
                                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcCooperative", "0")
                                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcKeepRight", "0")
                                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcAssertive", "0")
                                        except:
                                            pass
                                        # 如果接近末端且lane1拥堵，适度减速等待
                                        if remaining_distance < 30:
                                            current_speed = traci.vehicle.getSpeed(veh_id)
                                            traci.vehicle.setSpeed(veh_id, max(3.0, current_speed * 0.5))
                                    else:
                                        # 已经不在lane0，确保不在lane2
                                        if check_lane_index >= 2:
                                            traci.vehicle.setLaneChangeMode(veh_id, 0b000000000000)
                                            traci.vehicle.changeLane(veh_id, 1, 0)
                                            try:
                                                traci.vehicle.setParameter(veh_id, "laneChangeModel.lcStrategic", "0")
                                                traci.vehicle.setParameter(veh_id, "laneChangeModel.lcSpeedGain", "0")
                                                traci.vehicle.setParameter(veh_id, "laneChangeModel.lcCooperative", "0")
                                                traci.vehicle.setParameter(veh_id, "laneChangeModel.lcKeepRight", "0")
                                                traci.vehicle.setParameter(veh_id, "laneChangeModel.lcAssertive", "0")
                                            except:
                                                pass
                                    
                                    # 如果距离末端很近还没换道成功，主动减速等待间隙
                                    if remaining_distance < 30:
                                        # 检查左侧车道（慢车道）的前车和后车
                                        left_leaders = traci.vehicle.getLeftLeaders(veh_id)
                                        left_followers = traci.vehicle.getLeftFollowers(veh_id)
                                        
                                        # 如果左侧间隙不足，减速等待
                                        has_safe_gap = True
                                        if left_leaders:
                                            for leader_id, gap in left_leaders:
                                                if gap < 20.0:
                                                    has_safe_gap = False
                                                    break
                                        
                                        if not has_safe_gap:
                                            current_speed = traci.vehicle.getSpeed(veh_id)
                                            traci.vehicle.setSpeed(veh_id, max(5.0, current_speed * 0.6))
                                except Exception as e:
                                    pass
                        
                        # 【严格锁定逻辑】如果已经换到lane 1，绝对禁止任何换道到lane2
                        # 这是最关键的控制点：一旦进入lane1，就永远不允许换到lane2
                        if lane_index == 1 or 'mainline_after' in lane_id:
                            # 【第一步】每个时间步都完全禁止换道，防止SUMO重新评估
                            traci.vehicle.setLaneChangeMode(veh_id, 0b000000000000)  # 完全禁止换道
                            try:
                                # 【第二步】持续禁用所有换道参数，防止SUMO重新评估
                                traci.vehicle.setParameter(veh_id, "laneChangeModel.lcStrategic", "0")
                                traci.vehicle.setParameter(veh_id, "laneChangeModel.lcSpeedGain", "0")  # 特别重要：禁止speedGain（防止跳到lane2）
                                traci.vehicle.setParameter(veh_id, "laneChangeModel.lcCooperative", "0")
                                traci.vehicle.setParameter(veh_id, "laneChangeModel.lcKeepRight", "0")
                                traci.vehicle.setParameter(veh_id, "laneChangeModel.lcAssertive", "0")
                                
                                # 【第三步】额外禁用可能触发换道的参数
                                traci.vehicle.setParameter(veh_id, "laneChangeModel.lcOvertakeRight", "0")
                                traci.vehicle.setParameter(veh_id, "laneChangeModel.lcOvertake", "0")
                            except:
                                pass

                            merge_record_time = current_time if current_time is not None else traci.simulation.getTime()
                            self._handle_ramp_merge_completion(veh_id, lane_id, merge_record_time)
                            
                            # 【第四步】无论速度如何，都持续禁止换道
                            # 不只在等待时禁止，任何时候都禁止
                            traci.vehicle.setLaneChangeMode(veh_id, 0b000000000000)
                            try:
                                # 重复设置参数，确保生效
                                traci.vehicle.setParameter(veh_id, "laneChangeModel.lcStrategic", "0")
                                traci.vehicle.setParameter(veh_id, "laneChangeModel.lcSpeedGain", "0")
                                traci.vehicle.setParameter(veh_id, "laneChangeModel.lcCooperative", "0")
                                traci.vehicle.setParameter(veh_id, "laneChangeModel.lcKeepRight", "0")
                                traci.vehicle.setParameter(veh_id, "laneChangeModel.lcAssertive", "0")
                            except:
                                pass
                            
                            # 【第五步】恢复速度控制（但不允许换道）
                            traci.vehicle.setSpeed(veh_id, -1)
                            
                            # 【第六步】多次检查，确保不在lane2
                            # 第一次检查
                            check1 = traci.vehicle.getLaneIndex(veh_id)
                            if check1 >= 2:
                                # 强制换回lane1
                                traci.vehicle.setLaneChangeMode(veh_id, 0b000000000000)
                                traci.vehicle.changeLane(veh_id, 1, 0)
                                # 立即再次禁止换道
                                try:
                                    traci.vehicle.setParameter(veh_id, "laneChangeModel.lcStrategic", "0")
                                    traci.vehicle.setParameter(veh_id, "laneChangeModel.lcSpeedGain", "0")
                                    traci.vehicle.setParameter(veh_id, "laneChangeModel.lcCooperative", "0")
                                    traci.vehicle.setParameter(veh_id, "laneChangeModel.lcKeepRight", "0")
                                    traci.vehicle.setParameter(veh_id, "laneChangeModel.lcAssertive", "0")
                                except:
                                    pass
                            
                            # 第二次检查（延迟检查，确保生效）
                            check2 = traci.vehicle.getLaneIndex(veh_id)
                            if check2 >= 2:
                                # 再次强制换回lane1
                                traci.vehicle.setLaneChangeMode(veh_id, 0b000000000000)
                                traci.vehicle.changeLane(veh_id, 1, 0)
                                # 降低速度，确保安全
                                current_speed = traci.vehicle.getSpeed(veh_id)
                                traci.vehicle.setSpeed(veh_id, max(5.0, current_speed * 0.7))
                                try:
                                    traci.vehicle.setParameter(veh_id, "laneChangeModel.lcStrategic", "0")
                                    traci.vehicle.setParameter(veh_id, "laneChangeModel.lcSpeedGain", "0")
                                    traci.vehicle.setParameter(veh_id, "laneChangeModel.lcCooperative", "0")
                                    traci.vehicle.setParameter(veh_id, "laneChangeModel.lcKeepRight", "0")
                                    traci.vehicle.setParameter(veh_id, "laneChangeModel.lcAssertive", "0")
                                except:
                                    pass
                    except:
                        pass
        
        # 移除已离开的车辆
        for veh_id in list(self.active_vehicles.keys()):
            if veh_id not in current_vehicles:
                del self.active_vehicles[veh_id]

        # 同步车队规模和剩余容量
        self._update_platoon_states()
    
    def get_vehicles_in_merge_zone(self) -> Tuple[List[Dict], List[Dict]]:
        """
        获取合流区域内的车辆（更新为新路网结构）
        
        Returns:
            (主线车辆列表, 匝道车辆列表)
        """
        mainline_vehicles = []
        ramp_vehicles = []
        
        # 遍历SUMO中所有活动车辆
        for veh_id in traci.vehicle.getIDList():
            try:
                lane_id = traci.vehicle.getLaneID(veh_id)
                position = traci.vehicle.getLanePosition(veh_id)
                speed = traci.vehicle.getSpeed(veh_id)
                
                # 判断车辆类型
                if veh_id.startswith('m'):
                    veh_type = 'mainline'
                elif veh_id.startswith('r'):
                    veh_type = 'ramp'
                elif veh_id.startswith('f'):
                    # 快车道车辆不参与合流优化/FIFO
                    continue
                else:
                    continue
                
                # 检查是否应该纳入优化（更新为新路网结构）
                should_include = False


                # 主线车辆：检查是否在合流区域内（mainline的位置范围）
                if veh_type == 'mainline':
                    if 'mainline' in lane_id:
                        if MERGE_ZONE_START <= position <= (MERGE_ZONE_START + MERGE_ZONE_LENGTH):
                            should_include = True
                
                # 匝道车辆：在加速车道或匝道上
                elif veh_type == 'ramp':
                    if 'acceleration_lane' in lane_id or 'ramp' in lane_id or 'mainline' in lane_id:
                        should_include = True
                
                if should_include:
                    veh_dict = {
                        'id': veh_id,
                        'type': veh_type,
                        'position': position,
                        'speed': speed,
                        'lane': lane_id,
                        'platoon_id': self.vehicle_info.get(veh_id, {}).get('platoon_id', None),
                        'is_leader': self.vehicle_info.get(veh_id, {}).get('is_leader', False),
                        'desired_speed': self.vehicle_info.get(veh_id, {}).get('desired_speed', 30.0),
                        'platoon_size': 1,
                        'platoon_remaining_slots': None
                    }

                    platoon_state = self.platoons.get(veh_dict['platoon_id'])
                    if platoon_state:
                        veh_dict['platoon_size'] = platoon_state['current_size']
                        veh_dict['platoon_remaining_slots'] = platoon_state['remaining_slots']

                    # 添加前车信息
                    try:
                        leader = traci.vehicle.getLeader(veh_id)
                        if leader is not None:
                            leader_id, gap = leader
                            veh_dict['distance_to_leader'] = gap
                            # 获取前车速度
                            if leader_id in traci.vehicle.getIDList():
                                veh_dict['leader_speed'] = traci.vehicle.getSpeed(leader_id)
                            else:
                                veh_dict['leader_speed'] = speed  # 默认使用当前速度
                    except:
                        veh_dict['distance_to_leader'] = float('inf')
                        veh_dict['leader_speed'] = speed

                    if veh_type == 'mainline':
                        mainline_vehicles.append(veh_dict)
                    else:
                        ramp_vehicles.append(veh_dict)
                        
            except traci.exceptions.TraCIException:
                continue
        
        return mainline_vehicles, ramp_vehicles

    def _calculate_fifo_speed(self, vehicle: Dict) -> float:
        """在FIFO模式下基于安全距离计算目标速度"""

        current_speed = vehicle.get('speed', 0.0)
        desired_speed = min(vehicle.get('desired_speed', MAINLINE_SPEED_LIMIT), MAINLINE_SPEED_LIMIT)
        distance_to_leader = vehicle.get('distance_to_leader', float('inf'))
        leader_speed = vehicle.get('leader_speed', current_speed)

        # 根据当前速度差计算安全距离
        safe_distance = self.safety_calculator.calculate_safe_distance(
            current_speed,
            leader_speed,
            False
        )

        # 距离不足时跟随前车，否则按照期望速度行驶
        if distance_to_leader < safe_distance:
            return max(min(leader_speed, desired_speed), 0.0)

        return desired_speed
    
    def apply_merge_control(self, current_time: float):
        """应用合流控制策略（优化/FIFO模式）"""
        # 检查是否需要优化
        if current_time - self.last_optimization_time < OPTIMIZATION_INTERVAL:
            return
        
        # 获取合流区域内的车辆
        mainline_vehicles, ramp_vehicles = self.get_vehicles_in_merge_zone()

        # 在此防御性过滤，确保快车道车辆不参与优化/FIFO/重组
        mainline_vehicles = [v for v in mainline_vehicles if not v.get('id', '').startswith('f')]
        ramp_vehicles = [v for v in ramp_vehicles if not v.get('id', '').startswith('f')]
        
        # 如果有车辆在合流区域，记录日志
        if mainline_vehicles or ramp_vehicles:
            self.logger.log(f"时刻{current_time:.1f}s: 合流区域检测到 "
                            f"主线{len(mainline_vehicles)}辆, 匝道{len(ramp_vehicles)}辆")
        
        if not mainline_vehicles and not ramp_vehicles:
            return

        # 根据模式选择FIFO或优化算法
        initial_cost = 0.0
        final_cost = 0.0
        iterations = 0.0
        if not self.use_optimization:  # 优化未开启，采用FIFO模式
            # FIFO模式：使用FIFO序列
            sequence = self.merge_optimizer._generate_fifo_sequence(mainline_vehicles, ramp_vehicles, current_time)
            optimized_sequence = sequence
        else:  # 优化开启
            # 运行优化算法
            optimized_sequence, initial_cost, final_cost, iterations = self.merge_optimizer.optimize_merge_sequence(
                mainline_vehicles, ramp_vehicles, current_time
            )
            sequence = optimized_sequence

            self.optimization_count += 1
        
            # 记录优化运行
            self.logger.log_optimization_run(
                current_time, len(mainline_vehicles), len(ramp_vehicles),
                initial_cost, final_cost, iterations
            )

        if self.enable_platoon_reorg:
            baseline_cost = final_cost
            # 逐个评估匝道车的重组候选，并在改进时即时应用
            for vehicle in ramp_vehicles:
                merge_candidates = self.merge_optimizer.search_merge_candidates(
                    vehicle, mainline_vehicles, current_time, sequence
                )
                if merge_candidates:
                    vehicle['merge_candidates'] = merge_candidates
                best_candidate = self.merge_optimizer.select_best_candidate(
                    merge_candidates, baseline_cost
                ) if merge_candidates else None
                if best_candidate and best_candidate.get('platoon_join'):
                    platoon_id = best_candidate.get('platoon_id')
                    if platoon_id and not self._platoon_has_capacity(platoon_id):
                        continue
                    sequence = self.merge_optimizer.apply_candidate_sequence(best_candidate, sequence)
                    baseline_cost = best_candidate['metrics'].get('objective_cost', baseline_cost)
                    if platoon_id:
                        self._commit_platoon_join(vehicle['id'], platoon_id)

            if baseline_cost < final_cost:
                optimized_sequence = sequence
                self.merge_optimizer.last_final_cost = baseline_cost
                self.merge_optimizer.last_optimized_sequence = optimized_sequence
                self.merge_optimizer._rebuild_virtual_platoons(optimized_sequence)

        self.current_sequence = sequence
        self.last_optimization_time = current_time
        
        # 应用控制指令和强制变道
        for vehicle in ramp_vehicles:
            veh_id = vehicle['id']
            
            if veh_id not in traci.vehicle.getIDList():
                continue
            
            try:
                lane_id = traci.vehicle.getLaneID(veh_id)
                
                # 匝道车辆：在ramp上时，允许换到mainline_merge的lane0
                # ⚠️ 注意：不设置允许换道的模式，保持禁止状态，避免跳到lane2
                if 'ramp' in lane_id:
                    # 如果还在ramp上，需要换到mainline_merge的lane0
                    # 但使用最小权限，不启用大量换道模式
                    try:
                        # 只允许必要的换道到相邻车道，不允许跳到lane2
                        traci.vehicle.setLaneChangeMode(veh_id, 0b000000000010)  # 只允许strategic left
                        # 禁用speedGain等可能导致跳到lane2的参数
                        try:
                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcSpeedGain", "0")
                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcCooperative", "0")
                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcKeepRight", "0")
                            traci.vehicle.setParameter(veh_id, "laneChangeModel.lcAssertive", "0")
                        except:
                            pass
                        # 换到mainline_merge的lane0（不是lane2！）
                        traci.vehicle.changeLane(veh_id, 0, 2.0)  # 2秒内完成，只换到lane0
                        # 立即检查，确保不在lane2
                        current_check = traci.vehicle.getLaneIndex(veh_id)
                        if current_check >= 2:
                            # 如果在lane2，立即纠正
                            traci.vehicle.setLaneChangeMode(veh_id, 0b000000000000)
                            traci.vehicle.changeLane(veh_id, 0, 0)
                    except:
                        pass
                
                # 计算控制指令
                if self.use_optimization:
                    # 优化模式，用merge_optimizer的跟驰控制方式
                    command = self.merge_optimizer.calculate_control_commands(vehicle, optimized_sequence, current_time)

                    target_speed = command['target_speed']
                else:
                    # FIFO模式，用基于安全距离的简单跟驰
                    target_speed = self._calculate_fifo_speed(vehicle)
                
                # 应用速度控制
                traci.vehicle.setSpeed(veh_id, target_speed)
                
            except traci.exceptions.TraCIException:
                pass
    
    def collect_data(self, current_time: float):
        """收集仿真数据（增强版：包含距离监测和安全检查）"""
        if not self.data_collector.should_collect(current_time):
            return
        
        for veh_id in traci.vehicle.getIDList():
            try:
                position = traci.vehicle.getLanePosition(veh_id)
                velocity = traci.vehicle.getSpeed(veh_id)
                acceleration = traci.vehicle.getAcceleration(veh_id)
                lane_id = traci.vehicle.getLaneID(veh_id)  # 获取完整的lane ID字符串
                lane_index = traci.vehicle.getLaneIndex(veh_id)
                
                # 获取车队信息（修复版：正确处理字符串和整数）
                platoon_id = -1
                position_in_platoon = -1
                platoon_max_size = None
                platoon_current_size = None
                platoon_remaining_slots = None
                stored_platoon_id = None
                platoon_label = -1
                if veh_id in self.vehicle_info:
                    veh_info = self.vehicle_info[veh_id]
                    stored_platoon_id = veh_info.get('platoon_id')
                    
                    if stored_platoon_id is not None:
                        try:
                            # 处理字符串格式 "p0" 或整数格式
                            if isinstance(stored_platoon_id, str):
                                platoon_id = int(stored_platoon_id.replace('p', ''))
                                platoon_label = stored_platoon_id
                            else:
                                platoon_id = int(stored_platoon_id)
                                platoon_label = f"p{platoon_id}"
                                
                            position_in_platoon = veh_info.get('position_in_platoon', -1)
                            
                            # 调试：首次采集时打印车队信息
                            if current_time < 20.0 and position_in_platoon >= 0:
                                print(f"    [调试] {veh_id}: platoon={platoon_id}, pos_in_platoon={position_in_platoon}")
                        except (ValueError, AttributeError, TypeError):
                            # 转换失败，保持默认值-1
                            platoon_id = -1
                            platoon_label = -1

                    # 同步车队容量信息以便写入采集记录
                    if stored_platoon_id is not None:
                        platoon_state = self.platoons.get(stored_platoon_id)
                        if platoon_state:
                            platoon_max_size = platoon_state.get('max_size')
                            platoon_current_size = platoon_state.get('current_size')
                            platoon_remaining_slots = platoon_state.get('remaining_slots')
                        else:
                            platoon_max_size = veh_info.get('platoon_max_size')
                            platoon_remaining_slots = veh_info.get('platoon_remaining_slots')

                # 计算与前车的距离和所需安全距离
                distance_to_leader = -1.0
                safe_distance_required = -1.0
                
                try:
                    leader = traci.vehicle.getLeader(veh_id, dist=100.0)
                    if leader is not None:
                        leader_id, gap = leader
                        distance_to_leader = gap
                        
                        # 根据车队关系计算所需安全距离（修复版：正确比较车队ID）
                        if platoon_id >= 0:
                            # 主线车辆
                            # 检查前车是否在同一车队
                            if leader_id in self.vehicle_info:
                                leader_info = self.vehicle_info[leader_id]
                                leader_stored_platoon_id = leader_info.get('platoon_id')
                                
                                # 获取前车的车队编号
                                leader_platoon_num = -1
                                if leader_stored_platoon_id is not None:
                                    try:
                                        if isinstance(leader_stored_platoon_id, str):
                                            leader_platoon_num = int(leader_stored_platoon_id.replace('p', ''))
                                        else:
                                            leader_platoon_num = int(leader_stored_platoon_id)
                                    except:
                                        leader_platoon_num = -1
                                
                                # 比较车队编号（都是整数）
                                if leader_platoon_num >= 0 and leader_platoon_num == platoon_id:
                                    # 同一车队：车队内安全距离
                                    safe_distance_required = MIN_GAP_WITHIN + velocity * SAFE_TIME_HEADWAY_WITHIN
                                else:
                                    # 不同车队：车队间安全距离
                                    safe_distance_required = MIN_GAP_BETWEEN + velocity * SAFE_TIME_HEADWAY_BETWEEN
                            else:
                                # 前车是匝道车：车队间安全距离
                                safe_distance_required = MIN_GAP_BETWEEN + velocity * SAFE_TIME_HEADWAY_BETWEEN
                        else:
                            # 匝道车辆：使用车队间安全距离
                            safe_distance_required = MIN_GAP_BETWEEN + velocity * SAFE_TIME_HEADWAY_BETWEEN
                except:
                    pass
                
                # 使用lane_id字符串来判断车辆在哪条边,
                # 如果包含"ramp"则在匝道，包含"mainline"则在主线
                self.data_collector.collect_vehicle_data(
                    vehicle_id=veh_id,
                    timestamp=current_time,
                    position=position,
                    velocity=velocity,
                    acceleration=acceleration,
                    lane_id=1 if 'mainline' in lane_id else 0,  # 0=匝道，1=主线
                    platoon_id=platoon_label if platoon_label !=-1 else platoon_id,
                    position_in_platoon=position_in_platoon,
                    platoon_max_size=platoon_max_size,
                    platoon_current_size=platoon_current_size,
                    platoon_remaining_slots=platoon_remaining_slots,
                    lane_id_full=lane_id,  # 保存完整的lane_id用于坐标转换
                    lane_index=lane_index,  # SUMO的车道索引（0, 1, 2）
                    distance_to_leader=distance_to_leader,
                    safe_distance_required=safe_distance_required
                )
            except traci.exceptions.TraCIException:
                continue
        
        self.data_collector.update_collection_time(current_time)
    
    def run_simulation(self):
        """运行仿真主循环"""
        print("开始仿真...")
        
        # 打印车辆生成计划
        self.vehicle_generator.print_generation_plan()
        
        step = 0
        current_time = 0.0
        
        # 记录生成的车辆数（用于车队内车辆位置计算）
        last_mainline_position = 0.0
        last_ramp_position = 0.0
        last_fast_mainline_position = 0.0
        
        while current_time < SIMULATION_TIME:
            # 执行一步仿真
            traci.simulationStep()
            current_time = traci.simulation.getTime()
            step += 1
            
            # 生成新车辆
            mainline_vehicles, ramp_vehicles, fast_mainline_vehicle = self.vehicle_generator.get_vehicles_to_generate(
                current_time
            )

            platoon_id = mainline_vehicles[0].platoon_id if mainline_vehicles else None

            # 添加主线车辆（车队形式）
            if mainline_vehicles:
                self.total_generated_mainline += len(mainline_vehicles)
                self._register_platoon(platoon_id, len(mainline_vehicles))
                vehicle_ids = [v.id for v in mainline_vehicles]
                print(f"[{current_time:.1f}s] 生成主线车队{platoon_id}，共{len(mainline_vehicles)}辆: {vehicle_ids}")
                
                # 验证车辆编号连续性
                expected_ids = [f"m{self.total_generated_mainline - len(mainline_vehicles) + i + 1}" 
                                for i in range(len(mainline_vehicles))]
                if vehicle_ids != expected_ids:
                    print(f"  警告: 车辆编号不连续！期望{expected_ids}，实际{vehicle_ids}")
                    self.logger.log(f"警告: 车队{platoon_id}编号不连续", "WARNING")
                
                self.logger.log_vehicle_generation(current_time, len(mainline_vehicles), 0,
                                                   [f"主线车队{platoon_id}生成: {vehicle_ids}"],
                                                   fast_mainline_count=0)
            
            # 计算车队需要的总长度，确保头车位置足够远
            max_platoon_size = len(mainline_vehicles)
            estimated_gap = MIN_GAP_WITHIN + PLATOON_DESIRED_SPEED * SAFE_TIME_HEADWAY_WITHIN
            estimated_platoon_length = max_platoon_size * (VEHICLE_LENGTH + estimated_gap)
            # 头车位置 = 检测区域起点(150) + 车队长度 + 安全余量(50)
            head_vehicle_pos = max(200.0, MERGE_ZONE_START + estimated_platoon_length + 50.0)
            
            for i, vehicle in enumerate(mainline_vehicles):
                if i == 0:
                    # 车队头车 - 从计算的安全位置生成
                    depart_pos = head_vehicle_pos
                    depart_speed = max(0.0, vehicle.desired_speed * 0.8)
                    last_mainline_position = depart_pos
                else:
                    # 车队内跟随车 - 按真实间距排列
                    gap = MIN_GAP_WITHIN + vehicle.desired_speed * SAFE_TIME_HEADWAY_WITHIN
                    depart_pos = last_mainline_position - VEHICLE_LENGTH - gap
                    depart_speed = vehicle.desired_speed
                    last_mainline_position = depart_pos
                
                # 调试信息
                print(f"  添加车辆{vehicle.id}: position_in_platoon={vehicle.position_in_platoon}, depart_pos={depart_pos:.1f}")
                
                platoon_state = self.platoons.get(platoon_id)
                if platoon_state:
                    vehicle.platoon_max_size = platoon_state['max_size']
                    vehicle.platoon_remaining_slots = platoon_state['remaining_slots']

                # 主线车辆全部从车道0（最右侧慢车道）行驶
                success = self.add_vehicle_to_sumo(
                    vehicle,
                    route_id="route_mainline",
                    depart_lane=0,  # mainline_before的车道0（慢车道，靠匝道侧）
                    depart_pos=depart_pos,  # 使用计算的真实位置，不再强制夹值
                    depart_speed=depart_speed
                )
                
                if not success:
                    print(f"  警告: 车辆{vehicle.id}添加失败！位置={depart_pos:.1f}")

            # 添加快车道单车
            if fast_mainline_vehicle:
                self.total_generated_fast_mainline += len(fast_mainline_vehicle)
                vehicle_ids = [v.id for v in fast_mainline_vehicle]
                print(f"[{current_time:.1f}s] 生成快车道车辆: {vehicle_ids}")

                expected_ids = [f"f{self.total_generated_fast_mainline - len(fast_mainline_vehicle) + i + 1}"
                                for i in range(len(fast_mainline_vehicle))]
                if vehicle_ids != expected_ids:
                    print(f"  警告: 快车道车辆编号不连续！期望{expected_ids}，实际{vehicle_ids}")
                    self.logger.log(f"警告: 快车道车辆编号不连续", "WARNING")

                self.logger.log_vehicle_generation(current_time, 0, 0,
                                                   [f"快车道车辆生成: {vehicle_ids}"],
                                                   fast_mainline_count=len(fast_mainline_vehicle))

            fast_reference_speed = fast_mainline_vehicle[0].desired_speed if fast_mainline_vehicle else FAST_MAINLINE_DESIRED_SPEED
            fast_gap = FAST_MAINLINE_MIN_GAP + fast_reference_speed * FAST_TAU
            head_fast_pos = max(200.0, MERGE_ZONE_START + fast_gap + FAST_MAINLINE_VEHICLE_LENGTH + 50.0)

            for i, vehicle in enumerate(fast_mainline_vehicle):
                if i == 0:
                    depart_pos = head_fast_pos
                    last_fast_mainline_position = depart_pos
                else:
                    gap = FAST_MAINLINE_MIN_GAP + vehicle.desired_speed * FAST_TAU
                    depart_pos = max(0.0, last_fast_mainline_position - FAST_MAINLINE_VEHICLE_LENGTH - gap)
                    last_fast_mainline_position = depart_pos

                success = self.add_vehicle_to_sumo(
                    vehicle,
                    route_id="route_mainline",
                    depart_lane=1,
                    depart_pos=depart_pos,
                    depart_speed=max(0.0, vehicle.desired_speed * 0.9)
                )

                if not success:
                    print(f"  警告: 快车道车辆{vehicle.id}添加失败！位置={depart_pos:.1f}")

            # 添加匝道车辆
            if ramp_vehicles:
                self.total_generated_ramp += len(ramp_vehicles)
                vehicle_ids = [v.id for v in ramp_vehicles]
                print(f"[{current_time:.1f}s] 生成匝道车辆: {vehicle_ids}")
                
                # 验证车辆编号连续性
                expected_ids = [f"r{self.total_generated_ramp - len(ramp_vehicles) + i + 1}" 
                                for i in range(len(ramp_vehicles))]
                if vehicle_ids != expected_ids:
                    print(f"  警告: 匝道车辆编号不连续！期望{expected_ids}，实际{vehicle_ids}")
                    self.logger.log(f"警告: 匝道车辆编号不连续", "WARNING")
                
                self.logger.log_vehicle_generation(current_time, 0, len(ramp_vehicles),
                                                   [f"匝道车辆生成: {vehicle_ids}"],
                                                   fast_mainline_count=0)
            
            for vehicle in ramp_vehicles:
                self.add_vehicle_to_sumo(
                    vehicle,
                    route_id="route_ramp",
                    depart_lane=0,
                    depart_pos=5.0,
                    depart_speed=max(15.0, vehicle.desired_speed * 0.8)  # 提高初始速度
                )
            
            # 更新车辆状态
            self.update_vehicle_states(current_time)
            
            # 应用合流控制
            self.apply_merge_control(current_time)
            
            # 收集数据
            self.collect_data(current_time)
            
            # 每10秒输出一次进度
            if step % 100 == 0:
                current_vehicles = len(traci.vehicle.getIDList())
                print(f"仿真进度: {current_time:.1f}/{SIMULATION_TIME}秒, "
                      f"活动车辆数: {current_vehicles}")
                self.logger.log_simulation_progress(
                    current_time, SIMULATION_TIME, current_vehicles,
                    self.total_generated_mainline + self.total_generated_ramp + self.total_generated_fast_mainline
                )
            
            # 每30秒记录一次状态快照
            if step % 300 == 0 and traci.vehicle.getIDList():
                self.logger.log_vehicle_state_snapshot(current_time, self.active_vehicles)
        
        print("仿真完成！")

    def finalize(self):
        """结束仿真并保存结果"""
        # 关闭SUMO
        traci.close()
        print("SUMO已关闭")
        
        # 导出数据
        self.data_collector.export_to_excel()
        
        # 显示统计信息
        print("\n=== 仿真统计 ===")
        stats = self.data_collector.get_statistics()
        for key, value in stats.items():
            print(f"{key}: {value}")

        print("\n===车队规模分布统计===")
        generator_stats = self.vehicle_generator.get_statistics()
        if 'platoon_size_distribution' in generator_stats:
            platoon_dist = generator_stats['platoon_size_distribution']
            total_platoons = sum(platoon_dist.values())

            print(f"总车队数量:{total_platoons}")
            print("各规模车队分布：")
            for size in sorted(platoon_dist.keys()):
                count = platoon_dist[size]
                percentage = (count / total_platoons * 100) if total_platoons > 0 else 0
                expected_percentage = PLATOON_SIZE_WEIGHTS.get(size, 0) * 100
                print(f"  {size}辆车队：{count}个({percentage:.1f}%), 预期:{expected_percentage:.1f}%")

        print("\n=== 生成统计 ===")
        print(f"慢车道车辆总数: {generator_stats.get('total_mainline_vehicles', 0)}")
        print(f"快车道车辆总数: {generator_stats.get('total_fast_mainline_vehicles', 0)}")
        print(f"匝道车辆总数: {generator_stats.get('total_ramp_vehicles', 0)}")
        print(f"慢车道车队生成事件: {generator_stats.get('mainline_generation_events', 0)}")
        print(f"快车道车辆生成事件: {generator_stats.get('fast_mainline_generation_events', 0)}")
        print(f"匝道车辆生成事件: {generator_stats.get('ramp_generation_events', 0)}")

        print("\n=== 性能指标 ===")
        metrics = self.data_collector.calculate_performance_metrics()
        for key, value in metrics.items():
            if isinstance(value, float):
                print(f"{key}: {value:.2f}")
            else:
                print(f"{key}: {value}")

        # tripinfo等待时间统计
        tripinfo_analyzer = TripInfoAnalyzer()
        tripinfo_report = tripinfo_analyzer.format_report()
        print("\n" + tripinfo_report)

        # 生成报告
        from src.data_collector import PerformanceAnalyzer
        analyzer = PerformanceAnalyzer(self.data_collector)
        print("\n" + analyzer.generate_report())
        
        # 记录日志
        self.logger.log_performance_metrics(metrics)
        self.logger.log_simulation_end(stats)
        
        # 生成结论
        conclusion = {
            'success': metrics.get('ramp_merge_success_rate', 0) > 70,
            'evaluation': [
                f"优化算法运行{self.optimization_count}次",
                f"主线车辆{stats.get('mainline_slow_vehicles', 0) + stats.get('mainline_fast_vehicles', 0)}辆",
                f"匝道车辆{stats.get('ramp_vehicles', 0)}辆",
                f"合流成功率{metrics.get('ramp_merge_success_rate', 0):.1f}%"
            ],
            'recommendations': []
        }
        
        if metrics.get('ramp_merge_success_rate', 0) < 70:
            conclusion['recommendations'].append("合流成功率偏低，需要进一步优化")
        if self.optimization_count < 10:
            conclusion['recommendations'].append("优化算法运行次数较少，建议检查触发条件")
            
        self.logger.log_conclusion(conclusion)
        self.logger.finalize()
        
        print(f"\n日志已保存到: output/simulation_log.txt")
        print(f"优化算法运行次数: {self.optimization_count}")
        
        return self.data_collector, self.merge_optimizer


if __name__ == "__main__":
    # 测试仿真控制器
    print("注意: 此测试需要SUMO环境和完整的路网文件")
    print("请使用 main.py 运行完整仿真")
