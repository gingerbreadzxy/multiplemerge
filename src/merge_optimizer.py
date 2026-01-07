"""
合流优化算法模块
对匝道车辆和主线车辆的合流顺序进行优化排序
"""
import copy
import numpy as np
from typing import List, Dict, Tuple, Optional
import traci
from src.config import *
from src.safety_distance import SafetyDistanceCalculator


class MergeOptimizer:
    """合流优化器"""
    
    def __init__(self):
        self.safety_calculator = SafetyDistanceCalculator()
        self.optimization_history = []  # 记录优化历史用于绘制收敛曲线
        # 虚拟车队状态：用于在多轮优化排序中考虑匝道车加入或新建车队后的容量变化
        self.virtual_platoon: Dict[str, Dict[str, float]] = {}
        self.virtual_vehicle_platoon: Dict[str, str] = {}
        self.last_fifo_sequence: List[Dict] = []
        self.last_fifo_cost: Optional[float] = None
        self.last_optimized_sequence: List[Dict] = []
        self.last_final_cost: Optional[float] = None
        self.reorg_bonus_weight = (
            PLATOON_REORG_BONUS
            if (USE_OPTIMIZATION and ENABLE_PLATOON_REORG)
            else 0.0
        )

    def _filter_fast_lane(self, sequence: List[Dict]) -> List[Dict]:
        """过滤掉快车道车辆，防止影响合流优化逻辑"""
        return [
            veh for veh in sequence
            if not str(veh.get('id', '')).startswith('f') and veh.get('type') != 'fast_mainline'
        ]

    def search_merge_candidates(self, ramp_vehicle: Dict, mainline_vehicles: List[Dict],
                                current_time: float, sequence_context: Optional[List[Dict]] = None) -> List[Dict]:
        """为匝道车生成合流候选方案（含加入车队的选项），并用统一的目标函数评价"""

        # 识别合流前/后的主线目标车
        front_vehicle, back_vehicle = self._find_adjacent_vehicles(
            ramp_vehicle, mainline_vehicles, current_time, sequence_context
        )

        # 选择用于计算的基准序列和代价：优先使用优化结果，其次退回到FIFO
        base_sequence = sequence_context or self.last_optimized_sequence
        if not base_sequence:
            base_sequence = self._generate_fifo_sequence(mainline_vehicles, [ramp_vehicle], current_time)

        fifo_cost = self.last_fifo_cost
        if fifo_cost is None:
            fifo_cost = self._calculate_objective_function(base_sequence, current_time, self.reorg_bonus_weight)

        baseline_cost = self._calculate_objective_function(base_sequence, current_time, self.reorg_bonus_weight)

        candidates: List[Dict] = []

        # 方案1：常规合流（不加入车队）
        candidates.append(self._build_candidate_result(
            'regular_merge', None, False, base_sequence, fifo_cost, baseline_cost,
            current_time, safety_ok=True
        ))

        if ramp_vehicle.get('is_small_car'):
            return candidates

        # 方案2：尝试加入前车所在车队
        if front_vehicle and self._can_join_platoon(front_vehicle):
            self._append_join_candidate(
                candidates, ramp_vehicle, front_vehicle, 'front', base_sequence,
                fifo_cost, baseline_cost, current_time
            )

        # 方案3： 尝试加入后车所在车队
        if back_vehicle and self._can_join_platoon(back_vehicle):
            self._append_join_candidate(
                candidates, ramp_vehicle, back_vehicle, 'back', base_sequence,
                fifo_cost, baseline_cost, current_time
            )

        return candidates

    def _find_adjacent_vehicles(self, ramp_vehicle: Dict, mainline_vehicles: List[Dict],
                                current_time: float, sequence_context: Optional[List[Dict]] = None
                                ) -> Tuple[Optional[Dict], Optional[Dict]]:
        """按预计到达时间找到匝道车前后的目标车辆（包含已排序匝道车）"""
        if not mainline_vehicles and not sequence_context:
            return None, None

        candidates = {veh.get('id'): veh for veh in mainline_vehicles
                      if not veh.get('id', '').startswith('f')}
        if sequence_context:
            # 允许前序匝道车（及其虚拟车队身份）参与前后车判断
            for veh in sequence_context:
                veh_id = veh.get('id')
                if veh_id is None or veh_id == ramp_vehicle.get('id') or veh_id.startswith('f'):
                    continue
                candidates.setdefault(veh_id, veh)

        ramp_arrival = self._estimate_arrival_time(ramp_vehicle, current_time)
        sorted_candidates = sorted(
            candidates.values(),
            key=lambda v: self._estimate_arrival_time(v, current_time)
        )

        front_vehicle = None
        back_vehicle = None
        for veh in sorted_candidates:
            if self._estimate_arrival_time(veh, current_time) <= ramp_arrival:
                front_vehicle = veh
            else:
                back_vehicle = veh
                break

        return front_vehicle, back_vehicle

    def _estimate_arrival_time(self, vehicle: Dict, current_time: float) -> float:
        """估计单车到达合流点的时间，优先复用已写入的arrival_time"""
        if 'arrival_time' in vehicle:
            return vehicle['arrival_time']

        distance_to_merge = MERGE_POINT - vehicle.get('position', 0.0)
        speed = vehicle.get('speed', 20.0)
        travel_time = distance_to_merge / max(speed, 1.0) if distance_to_merge > 0 else 0
        return current_time + travel_time

    def _can_join_platoon(self, target_vehicle: Dict) -> bool:
        """检查目标车辆所在车队是否有空余容量可接纳匝道车"""
        if target_vehicle.get('is_small_car'):
            return False
        platoon_id = self._get_effective_platoon_id(target_vehicle)
        if platoon_id is None:
            return False

        remaining_slots = target_vehicle.get('platoon_remaining_slots')
        # 优先使用虚拟车队容量（考虑前序匝道车是否已加入/占用）
        if platoon_id in self.virtual_platoon:
            remaining_slots = self.virtual_platoon[platoon_id].get('remaining_slots', remaining_slots)

        return remaining_slots is not None and remaining_slots > 0

    def _get_effective_platoon_id(self, vehicle: Dict) -> Optional[str]:
        """返回车辆当前或虚拟的车队id"""
        platoon_id = vehicle.get('planned_join_platoon_id') or vehicle.get('platoon_id')
        if platoon_id is None:
            platoon_id = self.virtual_vehicle_platoon.get(vehicle.get('id'))
        return platoon_id

    def _build_candidate_result(self, candidate_type: str, target_id: Optional[str], platoon_join: bool,
                                candidate_sequence: List[Dict], fifo_cost: float,
                                baseline_cost: float, current_time: float,
                                platoon_id: Optional[str] = None, safety_ok: bool = True) -> Dict:
        """统一封装候选方案的目标函数评价结果"""
        candidate_cost = self._calculate_objective_function(
            candidate_sequence, current_time, self.reorg_bonus_weight
        ) if candidate_sequence else baseline_cost

        return {
            'type': candidate_type,
            'target': target_id,
            'platoon_join': platoon_join,
            'platoon_id': platoon_id,
            'sequence': candidate_sequence,
            'metrics': {
                'objective_cost': candidate_cost,
                'improvement_vs_fifo': fifo_cost - candidate_cost,
                'improvement_vs_current': baseline_cost - candidate_cost,
                'safety_ok': safety_ok
            }
        }

    def _apply_join_plan(self, base_sequence: List[Dict], ramp_vehicle: Dict,
                         target_vehicle: Dict) -> Tuple[List[Dict], Optional[str]]:
        """在有序副本上写入规划的车队加入信息"""
        if not base_sequence:
            return [], None

        platoon_id = self._get_effective_platoon_id(target_vehicle)
        sequence_copy = copy.deepcopy(base_sequence)
        ramp_id = ramp_vehicle.get('id')
        updated = False

        for veh in sequence_copy:
            if veh.get('id') == ramp_id:
                veh['planned_join_platoon_id'] = platoon_id
                updated = True
                break

        if not updated:
            ramp_clone = copy.deepcopy(ramp_vehicle)
            ramp_clone['planned_join_planned_id'] = platoon_id
            sequence_copy.append(ramp_clone)

        return sequence_copy, platoon_id

    def _append_join_candidate(self, candidates: List[Dict], ramp_vehicle: Dict, target_vehicle: Dict,
                               position: str, base_sequence: List[Dict],
                               fifo_cost: float, baseline_cost: float, current_time: float):
        """将加入车队的候选方案添加到列表并用目标函数评价"""
        simulation = self._simulated_platoon_merge(ramp_vehicle, target_vehicle, position)
        if not simulation:
            return

        candidate_sequence, platoon_id = self._apply_join_plan(base_sequence, ramp_vehicle, target_vehicle)
        if platoon_id is None:
            return

        candidate = self._build_candidate_result(
            f'join_platoon_{position}', target_vehicle.get('id'), True,
            candidate_sequence, fifo_cost, baseline_cost, current_time,
            platoon_id=platoon_id, safety_ok=simulation.get('feasible', True)
        )
        candidates.append(candidate)

    def select_best_candidate(self, candidates: List[Dict], baseline_cost: float) -> Optional[Dict]:
        """从候选列表中选出在目标函数上最优且安全的方案"""
        best: Optional[Dict] = None
        for candidate in candidates:
            metrics = candidate.get('metrics', {})
            if not metrics.get('safety_ok', True):
                continue
            cost = metrics.get('objective_cost', baseline_cost)
            if best is None or cost < best['metrics'].get('objective_cost', baseline_cost):
                best = candidate
        # 仅在比当前基线更优时返回
        if best and best['metrics'].get('objective_cost', baseline_cost) < baseline_cost:
            return best
        return None

    def apply_candidate_sequence(self, candidate: Dict, baseline_sequence: List[Dict]) -> List[Dict]:
        """根据候选方案返回更新后的序列（默认回退到原序列）"""
        if not candidate:
            return baseline_sequence
        candidate_sequence = candidate.get('sequence')
        if candidate_sequence:
            return candidate_sequence
        return baseline_sequence

    def _simulated_platoon_merge(self, ramp_vehicle: Dict, target_vehicle: Dict, position: str) -> Optional[Dict]:
        """简化仿真：判断匝道车加入目标车队的安全可行性"""

        # 基本属性
        ramp_speed = ramp_vehicle.get('speed', 20.0)
        target_speed = target_vehicle.get('speed', 20.0)
        ramp_position = ramp_vehicle.get('position', 0.0)
        target_position = target_vehicle.get('position', ramp_position + 50.0)

        # 初始间距（正值代表前方有距离可用）
        ramp_length = ramp_vehicle.get('length', VEHICLE_LENGTH)
        initial_gap = max((target_position - ramp_position) - ramp_length, 1.0)

        time_step = 0.5
        horizon = 10.0
        time_elapsed = 0.0

        within_gap = initial_gap

        while time_elapsed < horizon:
            desired_gap = self.safety_calculator.calculate_desired_gap_idm(
                ramp_speed, target_speed, True
            )
            acceleration = self.safety_calculator.calculate_idm_acceleration(
                ramp_speed, target_speed, within_gap, target_vehicle.get('desired_speed', MAINLINE_SPEED_LIMIT), True
            )

            # 更新状态
            ramp_speed = max(ramp_speed + acceleration * time_step, 0.0)
            target_speed = max(target_speed + 0.0 * time_step, 0.0)  # 假设目标车速度保持不变
            within_gap = max(within_gap + (target_speed - ramp_speed) * time_step, 0.1)
            time_elapsed += time_step

        # 以期望间距作为加入车队后的稳定状态是否可行的判据
        final_desired_gap = self.safety_calculator.calculate_desired_gap_idm(
            ramp_speed, target_speed, True
        )
        safety_ok = within_gap >= final_desired_gap

        delay = max(final_desired_gap - within_gap, 0.0) / max(target_speed, 1.0)

        platoon_id = self._get_effective_platoon_id(target_vehicle)

        return {
            'feasible': safety_ok,
            'platoon_id': platoon_id,
            'position': position,
            'delay': max(delay, 0.0)
        }

    def optimize_merge_sequence(self, mainline_vehicles: List[Dict],
                                ramp_vehicles: List[Dict],
                                current_time: float) -> Tuple[List[Dict], float, float, int]:
        """
        优化合流序列
        
        Args:
            mainline_vehicles: 主线车辆信息列表
            ramp_vehicles: 匝道车辆信息列表
            current_time: 当前仿真时间
            
        Returns:
            (优化后的车辆序列, 初始代价, 最终代价, 迭代次数)
        """
        # 过滤掉快车道车辆（ID以'f'开头）
        mainline_vehicles = [v for v in mainline_vehicles if not v.get('id', '').startswith('f')]
        ramp_vehicles = [v for v in ramp_vehicles if not v.get('id', '').startswith('f')]

        if not mainline_vehicles and not ramp_vehicles:
            return [], 0.0, 0.0, 0
        
        # 1. FIFO基准序列（不优化）
        fifo_sequence = self._filter_fast_lane(self._generate_fifo_sequence(
            mainline_vehicles, ramp_vehicles, current_time
        ))
        fifo_cost = self._calculate_objective_function(fifo_sequence, current_time, self.reorg_bonus_weight)
        
        # 2. 计算初始序列（贪心策略）
        initial_sequence = self._filter_fast_lane(self._generate_initial_sequence(
            mainline_vehicles, ramp_vehicles, current_time
        ))
        
        # 计算初始目标函数值
        initial_cost = self._calculate_objective_function(initial_sequence, current_time, self.reorg_bonus_weight)
        
        # 3. 局部优化
        optimized_sequence, final_cost, iterations = self._local_optimization(
            initial_sequence, current_time, initial_cost
        )

        self.last_fifo_sequence = fifo_sequence
        self.last_fifo_cost = fifo_cost
        self.last_optimized_sequence = optimized_sequence
        self.last_final_cost = final_cost

        # 4. 根据优化结果刷新虚拟车队状态，供后续匝道车候选生成使用
        self._rebuild_virtual_platoons(optimized_sequence)

        # 5. 记录优化历史（包含FIFO对比）
        self.optimization_history.append({
            'time': current_time,
            'fifo_cost': fifo_cost,  # FIFO基准
            'initial_cost': initial_cost,  # 贪心策略
            'final_cost': final_cost,  # 优化后
            'improvement_vs_fifo': (fifo_cost - final_cost) / max(fifo_cost, 1e-3) if fifo_cost > 0 else 0,
            'improvement_vs_initial': (initial_cost - final_cost) / max(initial_cost, 1e-6) if initial_cost > 0 else 0
        })
        
        return optimized_sequence, initial_cost, final_cost, iterations

    def _rebuild_virtual_platoons(self, sequence: List[Dict]):
        """根据最新优化序列重建虚拟车队状态"""
        self.virtual_platoon = {}
        self.virtual_vehicle_platoon = {}

        for veh in sequence:
            veh_id_raw = veh.get('id')
            veh_id_str = str(veh_id_raw) if veh_id_raw is not None else ""
            # 快车道车辆不参与虚拟车队构建
            if veh_id_str.startswith('f') or veh.get('type') == 'fast_mainline':
                continue

            veh_id = veh.get('id')
            if veh_id is None:
                continue

            # 1) 优先使用明确的合并目标车队（planned_join_platoon_id），否则使用自身车队
            platoon_id = veh.get('planned_join_platoon_id') or veh.get('platoon_id')

            # 2)若匝道车未归属任何车队，将其视为新创建的单车车队，便于后续匝道车加入
            if platoon_id is None and veh.get('type') == 'ramp' and not veh.get('is_small_car'):
                platoon_id = f"virtual_ramp_{veh_id}"
                self.virtual_platoon[platoon_id] = {
                    'max_size': PLATOON_SIZE_MAX,
                    'current_size': 0,
                    'remaining_slots': max(PLATOON_SIZE_MAX - 1, 0)
                }

            if platoon_id is None:
                continue

            state = self.virtual_platoon.get(platoon_id)
            if state:
                state['max_size'] = min(state.get('max_size', PLATOON_SIZE_MAX), PLATOON_SIZE_MAX)
                if state.get('remaining_slots') is not None:
                    state['remaining_slots'] = max(
                        min(state['remaining_slots'], state['max_size'] - state.get('current_size', 0)), 0
                    )
            if not state:
                # 初始化：使用车辆自身容量信息，如果缺失则以最大车队容量为上限
                max_size = min(veh.get('platoon_max_size', PLATOON_SIZE_MAX), PLATOON_SIZE_MAX)
                remaining_slots = veh.get('platoon_remaining_slots', max_size)
                if remaining_slots is not None:
                    remaining_slots = max(min(remaining_slots, max_size), 0)
                state = {
                    'max_size': max_size,
                    'current_size': 0,
                    'remaining_slots': remaining_slots
                }

            # 将当前车辆计入车队规模，占用一个容量
            if state['current_size'] < state['max_size']:
                state['current_size'] += 1
                if state['remaining_slots'] is None:
                    state['remaining_slots'] = max(state['max_size'] - state['current_size'], 0)
                else:
                    state['remaining_slots'] = max(
                        min(state['remaining_slots'] - 1, state['max_size'] - state['current_size']), 0
                    )
            else:
                # 车队已满，保持容量上限
                state['current_size'] = state['max_size']
                state['remaining_slots'] = 0

            self.virtual_platoon[platoon_id] = state
            self.virtual_vehicle_platoon[veh_id] = platoon_id

    def _generate_fifo_sequence(self, mainline_vehicles: List[Dict],
                                ramp_vehicles: List[Dict],
                                current_time: float) -> List[Dict]:
        """
        生成FIFO序列（先到先服务，不优化）
        
        Args:
            mainline_vehicles: 主线车辆信息列表
            ramp_vehicles: 匝道车辆信息列表
            current_time: 当前仿真时间
            
        Returns:
            FIFO排序的车辆序列
        """
        all_vehicles = []
        
        # 所有车辆按到达时间排序
        for veh in mainline_vehicles:
            if veh.get('id', '').startswith('f'):
                continue
            position = veh.get('position', 0.0)
            speed = veh.get('speed', 20.0)
            distance_to_merge = MERGE_POINT - position
            time_to_merge = distance_to_merge / max(speed, 1.0) if distance_to_merge > 0 else 0
            arrival_time = current_time + time_to_merge
            
            all_vehicles.append({
                'vehicle': veh,
                'arrival_time': arrival_time,
                'type': 'mainline'
            })
        
        for veh in ramp_vehicles:
            if veh.get('id', '').startswith('f'):
                continue
            position = veh.get('position', 0.0)
            speed = veh.get('speed', 20.0)
            distance_to_merge = MERGE_POINT - position
            time_to_merge = distance_to_merge / max(speed, 1.0) if distance_to_merge > 0 else 0
            arrival_time = current_time + time_to_merge
            
            all_vehicles.append({
                'vehicle': veh,
                'arrival_time': arrival_time,
                'type': 'ramp'
            })
        
        # 按到达时间排序（FIFO）
        all_vehicles.sort(key=lambda x: x['arrival_time'])
        
        # 返回车辆序列
        return [item['vehicle'] for item in all_vehicles]
    
    def _generate_initial_sequence(self, mainline_vehicles: List[Dict],
                                   ramp_vehicles: List[Dict],
                                   current_time: float) -> List[Dict]:
        """
        生成初始序列（基于贪心策略）
        
        策略：
        1. 计算每辆车到达合流点的预计时间
        2. 主线车队头车有更高优先级
        3. 考虑匝道车的合流可行性
        """
        sequence = []
        
        # 为所有车辆计算优先级
        all_vehicles = []
        
        for veh in mainline_vehicles:
            if veh.get('id', '').startswith('f'):
                continue
            priority = self._calculate_priority(veh, 'mainline', current_time)
            all_vehicles.append({
                'vehicle': veh,
                'priority': priority,
                'type': 'mainline'
            })
        
        for veh in ramp_vehicles:
            if veh.get('id', '').startswith('f'):
                continue
            priority = self._calculate_priority(veh, 'ramp', current_time)
            all_vehicles.append({
                'vehicle': veh,
                'priority': priority,
                'type': 'ramp'
            })
        
        # 按优先级排序
        all_vehicles.sort(key=lambda x: x['priority'])
        
        # 构建序列
        for item in all_vehicles:
            sequence.append(item['vehicle'])
        
        return sequence
    
    def _calculate_priority(self, vehicle: Dict, veh_type: str,
                            current_time: float) -> float:
        """
        计算车辆优先级
        
        优先级越小，越优先通过合流点
        """
        position = vehicle.get('position', 0.0)
        speed = vehicle.get('speed', 20.0)
        
        # 估计到达合流点的时间
        distance_to_merge = MERGE_POINT - position
        if distance_to_merge <= 0:
            # 已经通过合流点
            return -1000
        
        time_to_merge = distance_to_merge / max(speed, 1.0)
        estimated_arrival = current_time + time_to_merge
        
        if veh_type == 'mainline':
            # 主线车辆优先级
            is_leader = vehicle.get('is_leader', False)
            platoon_size = vehicle.get('platoon_size', 1)
            
            if is_leader:
                # 车队头车：考虑车队规模给予优先
                priority = estimated_arrival - 0.5 * platoon_size
            else:
                # 车队内跟随车：继承头车优先级
                priority = estimated_arrival - 0.3 * platoon_size
        else:
            # 匝道车辆
            # 检查合流可行性，如果快要错过合流机会，提高优先级
            distance_to_merge_end = (MERGE_POINT + RAMP_ACCELERATION_LANE) - position
            urgency = max(0, 1.0 - distance_to_merge_end / RAMP_ACCELERATION_LANE)
            
            priority = estimated_arrival - urgency * 5.0
        
        return priority
    
    def _calculate_objective_function(self, sequence: List[Dict],
                                      current_time: float,
                                      platoon_join_bonus: float = 0.0) -> float:
        """
        计算目标函数值（改进版）
        
        目标：最小化 α×主线延误时间 + β×匝道延误时间 + γ×匝道失败惩罚
        """
        if not sequence:
            return 0.0

        # 防御性过滤：确保快车道车辆完全不参与目标函数计算
        sequence = [
            veh for veh in sequence
            if not str(veh.get('id', '')).startswith('f') and veh.get('type') != 'fast_mainline'
        ]
        if not sequence:
            return 0.0
        
        total_mainline_delay = 0.0  # 主线延误
        total_ramp_delay = 0.0      # 匝道延误
        failed_ramp_count = 0       # 匝道失败数
        rewarded_reorg = 0          # 车队重组次数
        
        last_merge_time = current_time
        
        for i, veh in enumerate(sequence):
            veh_type = veh.get('type', 'mainline')
            position = veh.get('position', 0.0)
            speed = veh.get('speed', 20.0)
            
            # 计算理想到达时间（无干扰情况下）
            distance_to_merge = MERGE_POINT - position
            if distance_to_merge > 0:
                travel_time = distance_to_merge / max(speed, 1.0)
                ideal_arrival_time = current_time + travel_time
            else:
                ideal_arrival_time = current_time
            
            # 计算实际合流时间（考虑前车影响）
            if i > 0:
                prev_veh = sequence[i-1]
                is_within_platoon = (
                    veh_type == 'mainline' and
                    prev_veh.get('type') == 'mainline' and
                    veh.get('platoon_id') == prev_veh.get('platoon_id')
                )
                
                if is_within_platoon:
                    min_time_gap = SAFE_TIME_HEADWAY_WITHIN
                else:
                    min_time_gap = SAFE_TIME_HEADWAY_BETWEEN
                
                # 实际合流时间 = max(理想到达时间, 前车合流时间 + 安全间隔)
                actual_merge_time = max(ideal_arrival_time, last_merge_time + min_time_gap)
            else:
                actual_merge_time = ideal_arrival_time
            
            # 计算延误时间 = 实际合流时间 - 理想到达时间
            delay = actual_merge_time - ideal_arrival_time
            
            # 累加延误
            if veh_type == 'mainline':
                total_mainline_delay += delay
            else:  # ramp
                total_ramp_delay += delay
                
                # 检查匝道车是否能在加速车道结束前合流
                max_merge_time = current_time + (
                    (MERGE_POINT + RAMP_ACCELERATION_LANE - position) / max(speed, 1.0)
                )
                if actual_merge_time > max_merge_time:
                    failed_ramp_count += 1
            
            last_merge_time = actual_merge_time

            if platoon_join_bonus > 0 and veh_type == 'ramp':
                planned_platoon = veh.get('planned_join_platoon_id')
                if planned_platoon and not str(planned_platoon).startswith("virtual_ramp_"):
                    rewarded_reorg += 1
        
        # 改进的目标函数：
        # - 主线延误权重高（0.5）
        # - 匝道延误也要考虑（0.3）
        # - 失败惩罚很高（0.2 × 1000）
        # - 重组奖励，重组模式下鼓励加入主线车队的方案
        objective = (
            0.5 * total_mainline_delay +
            0.3 * total_ramp_delay +
            0.2 * failed_ramp_count * 1000.0 -
            platoon_join_bonus * rewarded_reorg
        )
        
        return objective
    
    def _local_optimization(self, initial_sequence: List[Dict],
                            current_time: float,
                            initial_cost: float) -> Tuple[List[Dict], float, int]:
        """
        局部优化：尝试交换相邻车辆以改善目标函数
        
        Args:
            initial_sequence: 初始序列
            current_time: 当前时间
            initial_cost: 初始代价
            
        Returns:
            (优化后序列, 最终代价, 迭代次数)
        """
        # 过滤掉快车道车辆，避免对匝道合流排序造成影响
        initial_sequence = [
            veh for veh in initial_sequence
            if not str(veh.get('id', '')).startswith('f') and veh.get('type') != 'fast_mainline'
        ]
        if not initial_sequence:
            return [], 0.0, 0

        best_sequence = initial_sequence.copy()
        # 重新计算过滤后的基线代价，避免使用包含快车道车辆的旧值
        best_cost = self._calculate_objective_function(best_sequence, current_time, self.reorg_bonus_weight)
        
        improved = True
        iteration = 0
        
        while improved and iteration < MAX_ITERATIONS:
            improved = False
            iteration += 1
            
            # 尝试交换相邻车辆
            for i in range(len(best_sequence) - 1):
                # 创建新序列
                new_sequence = best_sequence.copy()
                new_sequence[i], new_sequence[i+1] = new_sequence[i+1], new_sequence[i]
                
                # 检查交换是否可行（不拆散车队）
                if not self._is_swap_feasible(best_sequence, i):
                    continue
                
                # 计算新的代价
                new_cost = self._calculate_objective_function(new_sequence, current_time, self.reorg_bonus_weight)
                
                # 如果改进，接受新序列
                if new_cost < best_cost * (1.0 - CONVERGENCE_THRESHOLD):
                    best_sequence = new_sequence
                    best_cost = new_cost
                    improved = True
                    break
        
        return best_sequence, best_cost, iteration
    
    def _is_swap_feasible(self, sequence: List[Dict], index: int) -> bool:
        """
        检查交换是否可行
        
        规则：不拆散车队（车队内车辆应保持顺序）
        """
        veh1 = sequence[index]
        veh2 = sequence[index + 1]
        
        # 如果两辆车属于同一车队，不允许交换
        if (veh1.get('type') == 'mainline' and
            veh2.get('type') == 'mainline' and
            veh1.get('platoon_id') == veh2.get('platoon_id')):
            return False
        
        return True
    
    def get_optimization_history(self) -> List[Dict]:
        """获取优化历史记录"""
        return self.optimization_history
    
    def calculate_control_commands(self, vehicle: Dict, target_sequence: List[Dict],
                                   current_time: float) -> Dict:
        """
        根据优化序列计算车辆控制指令
        
        Args:
            vehicle: 车辆信息
            target_sequence: 目标序列
            current_time: 当前时间
            
        Returns:
            控制指令（目标速度、加速度等）
        """
        veh_id = vehicle.get('id')
        veh_type = vehicle.get('type')
        current_lane = vehicle.get('lane', '')
        current_speed = vehicle.get('speed', 0.0)
        desired_speed = vehicle.get('desired_speed', 30.0)
        
        # 在序列中找到该车辆
        veh_index = None
        for i, v in enumerate(target_sequence):
            if v.get('id') == veh_id:
                veh_index = i
                break

        # 初始化默认值
        target_speed = desired_speed
        action = 'maintain'

        # 情况1：匝道车辆已经在主线车道上
        if veh_type == 'ramp' and 'mainline' in current_lane:
            # 获取前车信息
            distance_to_leader = vehicle.get('distance_to_leader', float('inf'))
            leader_speed = vehicle.get('leader_speed', current_speed)

            # 计算安全距离
            safe_distance = self.safety_calculator.calculate_safe_distance(current_speed, leader_speed, False)

            # 安全距离充足：加速到期望速度
            if distance_to_leader > safe_distance + 15.0:  # 额外15米安全缓冲，避免一下子加速导致安全距离不足
                target_speed = min(desired_speed, MAINLINE_SPEED_LIMIT)
                action = 'accelerate_to_desired'
            # 安全距离一般：遵循序列控制，但可以适当提高速度
            elif distance_to_leader > safe_distance + 5.0:
                if veh_index is not None and veh_index > 0:
                    prev_speed = target_sequence[veh_index - 1].get('speed', current_speed)
                    target_speed = min(prev_speed + 1.0, desired_speed)
                else:
                    target_speed = min(current_speed + 1.0, desired_speed)
                action = 'moderate_acceleration'
            # 安全距离紧张：严格遵循序列控制
            else:
                if veh_index is not None and veh_index > 0:
                    prev_vehicle = target_sequence[veh_index - 1]
                    target_speed = prev_vehicle.get('speed', current_speed)
                    action = 'follow_sequence'
                else:
                    target_speed = current_speed
                    action = 'maintain_safe'

        # 情况2：车辆不在优化序列中
        elif veh_index is None:
            # 车辆不在优化序列中，保持当前行为
            return {
                'target_speed': vehicle.get('desired_speed', 30.0),
                'acceleration': 0.0,
                'action': 'maintain'
            }
        
        # 情况3：序列中的车辆，但非主线匝道车
        # 根据序列位置计算控制指令
        else:
            if veh_index == 0:
                # 第一辆车，可以自由加速到期望速度
                target_speed = vehicle.get('desired_speed', 30.0)
                action = 'accelerate'
            else:
                # 跟随前车
                prev_vehicle = target_sequence[veh_index - 1]
                target_speed = prev_vehicle.get('speed', 20.0)
                action = 'follow'
        
        # 最终的安全检查，确保目标速度不超过安全限制
        target_speed = min(target_speed, MAINLINE_SPEED_LIMIT)

        # 计算加速度
        speed_diff = target_speed - current_speed
        
        if abs(speed_diff) < 0.3:  # 原0.5
            acceleration = 0.0
        else:
            # 平滑加速/减速
            acceleration = np.clip(
                speed_diff / 2.5,  # 原2.0
                -MAX_DECELERATION * 0.6,  # 舒适减速，原0.7
                MAX_ACCELERATION * 0.7     # 舒适加速，原0.8
            )
        
        return {
            'target_speed': target_speed,
            'acceleration': acceleration,
            'action': action
        }


if __name__ == "__main__":
    # 测试合流优化器
    optimizer = MergeOptimizer()
    
    print("=== 合流优化器测试 ===\n")
    
    # 创建测试数据 - 故意创建次优序列来测试优化效果
    mainline_vehicles = [
        {
            'id': 'm1', 'type': 'mainline', 'position': 900, 'speed': 20.0,  # 慢车在后面
            'is_leader': True, 'platoon_id': 'p1', 'platoon_size': 1,
            'desired_speed': 30.0
        },
        {
            'id': 'm2', 'type': 'mainline', 'position': 800, 'speed': 35.0,  # 快车在前面
            'is_leader': True, 'platoon_id': 'p2', 'platoon_size': 1,
            'desired_speed': 30.0
        },
    ]
    
    ramp_vehicles = [
        {
            'id': 'r1', 'type': 'ramp', 'position': 850, 'speed': 30.0,  # 中等速度
            'desired_speed': 25.0
        },
        {
            'id': 'r2', 'type': 'ramp', 'position': 820, 'speed': 25.0,  # 慢车
            'desired_speed': 26.0
        },
    ]
    
    current_time = 10.0
    
    # 运行优化
    optimized_sequence, initial_cost, final_cost, iterations = optimizer.optimize_merge_sequence(
        mainline_vehicles, ramp_vehicles, current_time
    )
    
    print(f"初始代价: {initial_cost:.2f}")
    print(f"最终代价: {final_cost:.2f}")
    print(f"改进率: {(initial_cost - final_cost) / max(initial_cost, 1e-6) * 100:.2f}%")
    print(f"迭代次数: {iterations}")
    print("")
    print("优化后的车辆序列:")
    for i, veh in enumerate(optimized_sequence):
        print(f"  {i+1}. {veh['id']} ({veh['type']}) - "
              f"位置: {veh['position']:.1f}m, 速度: {veh['speed']:.1f}m/s")
    
    # 测试控制指令生成
    print("\n控制指令:")
    for veh in mainline_vehicles + ramp_vehicles:
        command = optimizer.calculate_control_commands(veh, optimized_sequence, current_time)
        print(f"  {veh['id']}: {command}")
