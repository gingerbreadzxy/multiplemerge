"""
数据收集模块
收集仿真数据并导出为Excel
"""

import pandas as pd
import numpy as np
import traci
from typing import List, Dict
from openpyxl.utils import get_column_letter
from src.config import *


class DataCollector:
    """数据收集器"""
    
    def __init__(self, use_optimization: bool):
        self.use_optimization = use_optimization
        self.data_records = []  # 存储所有数据记录
        self.last_collection_time = -DATA_COLLECTION_INTERVAL
        self.vehicle_last_timestamp: Dict[str, float] = {}
        self.vehicle_fuel_consumption: Dict[str, float] = {}
        self.total_fuel_consumption: float = 0.0  # 总油耗（升）
        
    def should_collect(self, current_time: float) -> bool:
        """判断是否应该收集数据"""
        return current_time - self.last_collection_time >= DATA_COLLECTION_INTERVAL

    def collect_vehicle_data(self, vehicle_id: str, timestamp: float,
                             position: float, velocity: float,
                             acceleration: float, lane_id: int,
                             platoon_id=-1, position_in_platoon: int = -1,
                             platoon_max_size: int = None, platoon_current_size: int = None,
                             platoon_remaining_slots: int = None,
                             lane_id_full: str = "",
                             lane_index: int = -1,
                             distance_to_leader: float = -1.0,
                             safe_distance_required: float = -1.0,
                             vehicle_class: str = ""):
        """
        收集单个车辆数据
        
        Args:
            vehicle_id: 车辆ID
            timestamp: 时间戳
            position: 位置（边内局部坐标）
            velocity: 速度
            acceleration: 加速度
            lane_id: 车道类型（0=匝道，1=主线）
            platoon_id: 车队编号（主线车辆>=0，匝道车辆=-1）
            position_in_platoon: 车队内位置（0=队首，-1=匝道车辆）
            lane_id_full: 完整的lane_id字符串（用于坐标转换）
            lane_index: SUMO的车道索引（0, 1, 2）
            distance_to_leader: 与前车的距离（米）
            safe_distance_required: 所需安全距离（米）
        """
        # 转换为全局坐标
        global_position = self._convert_to_global_coordinates(lane_id_full, position)
        try:
            # 方法1: 使用getDistance - 从车辆进入路网开始的总距离
            raw_accumulated_distance = traci.vehicle.getDistance(vehicle_id)
        except:
            # 方法2: 使用getOdometer - 车辆的里程表读数
            raw_accumulated_distance = traci.vehicle.getOdometer(vehicle_id)

        # 主线车实际行驶距离
        mainline_distance = traci.lane.getLength("mainline_before_0") + traci.lane.getLength("mainline_merge_0")
        # 匝道车实际行驶距离
        ramp_distance = traci.lane.getLength("ramp_0") + traci.lane.getLength("mainline_merge_1")
        # 校准偏移量
        calibration_offset = mainline_distance - ramp_distance
        # 合流点位置
        merge_point_position = traci.lane.getLength("mainline_before_0") + traci.lane.getLength("mainline_merge_0")

        # 匝道车辆进行坐标位置计算时需加上校准偏移量，主线车辆保持原始累积距离不变
        if vehicle_id.startswith('r'):
            accumulated_distance = raw_accumulated_distance + calibration_offset
            vehicle_type = 'ramp'
        elif vehicle_id.startswith('f'):
            accumulated_distance = raw_accumulated_distance
            vehicle_type = 'fast_mainline'
        else:
            accumulated_distance = raw_accumulated_distance
            vehicle_type = 'mainline'

        # 计算到合流点的距离（使用校准后的累积距离）
        distance_to_merge = merge_point_position - accumulated_distance

        # 将车队ID转换为便于计算的数值行驶，并保留显示ID
        platoon_display_id = platoon_id
        platoon_numeric_id = -1
        try:
            if isinstance(platoon_id, str):
                platoon_numeric_id = int(platoon_id.replace('p', ''))
            elif platoon_id is not None:
                platoon_numeric_id = int(platoon_id)
                platoon_display_id = f"p{platoon_numeric_id}"
        except (ValueError, TypeError):
            platoon_numeric_id = -1

        # 检查安全距离是否符合要求
        safe_distance_ok = 1 if (distance_to_leader < 0 or distance_to_leader >= safe_distance_required) else 0

        # 检查加速度是否超出阈值
        harsh_braking = 1 if acceleration < -3.5 else 0  # 急刹车阈值 -3.5 m/s²
        harsh_accel = 1 if acceleration > 2.0 else 0  # 急加速阈值 2.0 m/s²

        # 计算瞬时油耗率并累积总油耗
        is_following = (position_in_platoon > 0)  # 车辆是否为跟随车
        fuel_rate = self._calculate_instantaneous_fuel(
            velocity,
            acceleration,
            is_following,
            vehicle_id,
            vehicle_class
        )
        fuel_increment = 0.0
        if vehicle_id in self.vehicle_last_timestamp:
            dt = timestamp - self.vehicle_last_timestamp[vehicle_id]
            if dt > 0:
                fuel_increment = fuel_rate * dt
        self.vehicle_last_timestamp[vehicle_id] = timestamp
        self.vehicle_fuel_consumption[vehicle_id] = self.vehicle_fuel_consumption.get(vehicle_id, 0.0) + fuel_increment
        self.total_fuel_consumption += fuel_increment

        record = {
            'timestamp': round(timestamp, 1),
            'car_name': vehicle_id,
            'vehicle_type': vehicle_type,
            'vehicle_class': vehicle_class,
            'platoon_id': platoon_display_id,
            'position_in_platoon': position_in_platoon,
            'platoon_max_size': platoon_max_size,
            'platoon_current_size': platoon_current_size,
            'platoon_remaining_slots': platoon_remaining_slots,
            'is_following': is_following,
            'is_platoon_member': 1 if platoon_numeric_id >= 0 else 0,
            'position': round(position, 6),  # 局部坐标（边内）
            'global_position': round(global_position, 6),  # 全局坐标
            'velocity': round(velocity, 8),
            'acceleration': round(acceleration, 1),
            'lane_id': lane_id,
            'lane_id_full': lane_id_full,  # 完整的lane_id字符串
            'lane_index': lane_index,  # SUMO的车道索引（0, 1, 2）
            'distance_to_leader': round(distance_to_leader, 2) if distance_to_leader >= 0 else -1,
            'safe_distance_required': round(safe_distance_required, 2) if safe_distance_required >= 0 else -1,
            'safe_distance_ok': safe_distance_ok,
            'harsh_braking': harsh_braking,
            'harsh_accel': harsh_accel,
            'fuel_rate': round(fuel_rate, 6),  # 升/秒
            'accumulated_distance': round(accumulated_distance, 6),
            'distance_to_merge': round(distance_to_merge, 6)
        }

        self.data_records.append(record)
    
    def _convert_to_global_coordinates(self, lane_id_full: str, position: float) -> float:
        """
        将SUMO局部坐标转换为全局坐标
        
        Args:
            lane_id_full: 完整的lane_id字符串（如"mainline_before_0", "mainline_after_0", "ramp_0"）
            position: 边内位置
        
        Returns:
            全局坐标
        """
        # 根据新网络定义（highway.nod.xml）：
        # mainline_before: 从(0, 0)到(1000, 0)，长度1000米
        # mainline_merge: 从(1000, 0)到(1200, 0)，长度200米
        # mainline_after: 从(1200, 0)到(2000, 0)，长度800米
        # ramp: 从(700, -50)到(1000, -7.2)，长度约304米
        # acceleration_lane: 从(1000, -7.2)到(1200, -7.2)，长度200米
        
        if 'mainline_before' in lane_id_full:
            return position
        elif 'mainline_merge' in lane_id_full:
            return 1000.0 + position
        elif 'mainline_after' in lane_id_full:
            return 1200.0 + position
        elif 'acceleration_lane' in lane_id_full:
            # 加速车道：从(1000, -7.2)到(1200, -7.2)，与主线平行
            return 1000.0 + position
        elif 'ramp' in lane_id_full:
            # 匝道：从(700, -50)到(1000, -7.2)
            # x方向距离: 1000 - 700 = 300米
            # 边长: sqrt(300^2 + 42.8^2) ≈ 303.04米
            # x方向投影比例: 300/303.04 ≈ 0.99
            return 700.0 + position * 0.99
        else:
            # 其他情况：直接返回position
            return position

    def _calculate_instantaneous_fuel(self, speed: float, acceleration: float, is_following: bool,
                                      vehicle_id: str = "", vehicle_class: str = "") -> float:
        """根据参考文献的模型计算瞬时油耗率（升/秒）"""
        v = max(speed, 0.0)
        a = acceleration

        # 根据车辆类型选择参数（快车道车辆可使用独立参数）
        use_fast_params = vehicle_id.startswith('f') or vehicle_class in {'ramp_car', 'fast_mainline_car'}
        vehicle_mass = FAST_FUEL_VEHICLE_MASS if use_fast_params else FUEL_VEHICLE_MASS

        # 功率项P(t)
        rolling = FUEL_DELTA1 * v
        engine = FUEL_DELTA2 * (v ** 2)
        if is_following:
            fuel_phi = FUEL_PHI
        else:
            fuel_phi = 1
        aerodynamic = FUEL_DELTA3 * fuel_phi * (v ** 3)
        resistance_power = rolling + engine + aerodynamic
        acceleration_power = vehicle_mass * a * v / 1000.0

        power_term = max(resistance_power + acceleration_power, 0.0)

        fuel_rate = (
            FUEL_ALPHA
            + FUEL_BETA1 * power_term
            + (FUEL_BETA2 * vehicle_mass * (a ** 2) * v)/1000.0
        ) / 1000.0

        return max(fuel_rate, 0.0)

    def update_collection_time(self, current_time: float):
        """更新最后收集时间"""
        self.last_collection_time = current_time
    
    def export_to_excel(self, filename: str = None):
        """
        导出数据到Excel文件
        
        Args:
            filename: 输出文件名，默认使用配置文件中的路径
        """
        if filename is None:
            filename = OUTPUT_EXCEL_FILE
        
        if not self.data_records:
            print("警告: 没有数据可导出")
            return
        
        # 转换为DataFrame
        df = pd.DataFrame(self.data_records)
        
        # 按时间戳和车辆ID排序
        df = df.sort_values(['timestamp', 'car_name'])
        
        # 重置索引
        df = df.reset_index(drop=True)
        
        # 导出到Excel
        with pd.ExcelWriter(filename, engine='openpyxl') as writer:
            df.to_excel(writer, sheet_name='Simulation Data', index=False)
            
            # 获取工作表以设置列宽
            worksheet = writer.sheets['Simulation Data']
            
            # 设置列宽
            column_widths = {
                'timestamp': 12,  # timestamp
                'car_name': 15,  # car_name
                'vehicle_type': 12,
                'platoon_id': 12,  # platoon_id
                'position_in_platoon': 12,  # position_in_platoon
                'platoon_max_size': 12,
                'platoon_current_size': 12,
                'platoon_remaining_slots': 12,
                'is_platoon_member': 18,  # is_platoon_member
                'position': 15,  # position (局部)
                'global_position': 15,  # global_position (全局)
                'velocity': 15,  # velocity
                'acceleration': 15,  # acceleration
                'lane_id': 10,  # lane_id
                'lane_id_full': 25,  # lane_id_full (完整lane_id)
                'lane_index': 12,  # lane_index (0, 1, 2)
                'distance_to_leader': 18,  # distance_to_leader
                'safe_distance_required': 20,  # safe_distance_required
                'safe_distance_ok': 16,  # safe_distance_ok
                'harsh_braking': 14,  # harsh_braking
                'harsh_accel': 14,  # harsh_accel
                'fuel_rate': 16,  # fuel_rate
                'accumulated_distance': 18,
                'distance_tp_merge': 18,
            }
            
            for idx, column_name in enumerate(df.columns, start=1):
                width = column_widths.get(column_name, 15)
                column_letter = get_column_letter(idx)
                worksheet.column_dimensions[column_letter].width = width
        
        print(f"数据已导出到: {filename}")
        print(f"总记录数: {len(df)}")
    
    def get_statistics(self) -> Dict:
        """
        获取数据统计信息
        
        Returns:
            统计信息字典
        """
        if not self.data_records:
            return {}
        
        df = pd.DataFrame(self.data_records)
        
        # 按车辆类型分组
        mainline_slow_df = df[df['car_name'].str.startswith('m')]
        mainline_fast_df = df[df['car_name'].str.startswith('f')]
        ramp_df = df[df['car_name'].str.startswith('r')]
        overall_df = df
        
        stats = {
            'total_records': len(df),
            'unique_vehicles': df['car_name'].nunique(),
            'mainline_slow_vehicles': mainline_slow_df['car_name'].nunique(),
            'mainline_fast_vehicles': mainline_fast_df['car_name'].nunique(),
            'ramp_vehicles': ramp_df['car_name'].nunique(),
            'simulation_duration': df['timestamp'].max() - df['timestamp'].min(),
            'avg_velocity_mainline_slow': mainline_slow_df['velocity'].mean() if len(mainline_slow_df) > 0 else 0,
            'avg_velocity_mainline_fast': mainline_fast_df['velocity'].mean() if len(mainline_fast_df) > 0 else 0,
            'avg_velocity_ramp': ramp_df['velocity'].mean() if len(ramp_df) > 0 else 0,
            'avg_velocity_overall': overall_df['velocity'].mean() if len(overall_df) > 0 else 0,
            'max_acceleration': df['acceleration'].max(),
            'min_acceleration': df['acceleration'].min(),
            'total_fuel_consumption_l': self.total_fuel_consumption,
            'avg_fuel_per_vehicle_l': self.total_fuel_consumption / df['car_name'].nunique()
            if df['car_name'].nunique() > 0 else 0,
        }
        
        return stats

    def get_vehicle_trajectory(self, vehicle_id: str) -> pd.DataFrame:
        """
        获取特定车辆的轨迹数据
        
        Args:
            vehicle_id: 车辆ID
            
        Returns:
            车辆轨迹DataFrame
        """
        df = pd.DataFrame(self.data_records)
        vehicle_df = df[df['car_name'] == vehicle_id].copy()
        vehicle_df = vehicle_df.sort_values('timestamp')
        return vehicle_df
    
    def calculate_performance_metrics(self) -> Dict:
        """
        计算性能指标
        
        Returns:
            性能指标字典
        """
        if not self.data_records:
            return {}
        
        df = pd.DataFrame(self.data_records)
        
        # 分离主线快慢车道和匝道车辆
        mainline_slow_df = df[df['car_name'].str.startswith('m')]
        mainline_fast_df = df[df['car_name'].str.startswith('f')]
        ramp_df = df[df['car_name'].str.startswith('r')]
        mainline_df = df[df['vehicle_type'].isin(['mainline', 'fast_mainline'])]

        metrics = {}

        # 主线性能指标
        if len(mainline_slow_df) > 0:
            mainline_vehicles = mainline_slow_df['car_name'].unique()
            travel_times = []
            
            for veh_id in mainline_vehicles:
                veh_data = mainline_slow_df[mainline_slow_df['car_name'] == veh_id]
                if len(veh_data) > 1:
                    # 计算通过合流区域的时间
                    travel_time = veh_data['timestamp'].max() - veh_data['timestamp'].min()
                    travel_times.append(travel_time)
            
            metrics['mainline_avg_travel_time'] = np.mean(travel_times) if travel_times else 0
            metrics['mainline_avg_speed'] = mainline_slow_df['velocity'].mean()
            metrics['mainline_std_speed'] = mainline_slow_df['velocity'].std()

        if len(mainline_fast_df) > 0:
            metrics['fast_mainline_avg_speed'] = mainline_fast_df['velocity'].mean()
            metrics['fast_mainline_std_speed'] = mainline_fast_df['velocity'].std()

        if len(mainline_df) > 0:
            per_vehicle_avg_speed = mainline_df.groupby('car_name')['velocity'].mean()

            entry_lane_index = (
                mainline_df.sort_values('timestamp')
                .groupby('car_name')
                .first()['lane_index']
            )
            entry_slow_ids = entry_lane_index[entry_lane_index == 0].index
            entry_fast_ids = entry_lane_index[entry_lane_index == 1].index

            metrics['mainline_entry_slow_avg_speed'] = (
                per_vehicle_avg_speed.loc[entry_slow_ids].mean()
                if len(entry_slow_ids) > 0 else 0
            )
            metrics['mainline_entry_fast_avg_speed'] = (
                per_vehicle_avg_speed.loc[entry_fast_ids].mean()
                if len(entry_fast_ids) > 0 else 0
            )

        if len(df) > 0:
            per_vehicle_avg_speed_all = df.groupby('car_name')['velocity'].mean()
            exit_lane_records_all = (
                df.sort_values('timestamp')
                .groupby('car_name')
                .last()[['lane_index', 'lane_id_full']]
            )
            exit_mainline_all = exit_lane_records_all[
                exit_lane_records_all['lane_id_full'].str.contains('mainline_after', na=False)
            ]
            exit_slow_ids_all = exit_mainline_all[exit_mainline_all['lane_index'] == 0].index
            exit_fast_ids_all = exit_mainline_all[exit_mainline_all['lane_index'] == 1].index

            metrics['overall_exit_slow_avg_speed'] = (
                per_vehicle_avg_speed_all.loc[exit_slow_ids_all].mean()
                if len(exit_slow_ids_all) > 0 else 0
            )
            metrics['overall_exit_fast_avg_speed'] = (
                per_vehicle_avg_speed_all.loc[exit_fast_ids_all].mean()
                if len(exit_fast_ids_all) > 0 else 0
            )
        
        # 匝道性能指标
        if len(ramp_df) > 0:
            ramp_vehicles = ramp_df['car_name'].unique()
            
            # 计算合流成功率
            successful_merges = 0
            for veh_id in ramp_vehicles:
                veh_data = ramp_df[ramp_df['car_name'] == veh_id]
                # 如果车辆进入主线车道（lane_id改变），认为合流成功
                if veh_data['lane_id'].nunique() > 1:
                    successful_merges += 1
            
            metrics['ramp_merge_success_rate'] = successful_merges / len(ramp_vehicles) * 100
            metrics['ramp_avg_speed'] = ramp_df['velocity'].mean()
            metrics['ramp_total_vehicles'] = len(ramp_vehicles)
            metrics['ramp_successful_merges'] = successful_merges
        
        # 整体性能指标
        metrics['overall_avg_speed'] = df['velocity'].mean()
        metrics['overall_avg_acceleration'] = df['acceleration'].mean()
        metrics['total_fuel_consumption_l'] = self.total_fuel_consumption
        metrics['avg_fuel_per_vehicle_l'] = (
            self.total_fuel_consumption / df['car_name'].nunique()
            if df['car_name'].nunique() > 0 else 0
        )

        # 安全性指标（改进算法：合并连续事件）
        harsh_braking, harsh_accel = self._count_harsh_events_improved(df)
        metrics['harsh_braking_events'] = harsh_braking
        metrics['harsh_acceleration_events'] = harsh_accel
        
        return metrics
    
    def _count_harsh_events_improved(self, df: pd.DataFrame) -> tuple:
        """
        改进的急刹/急加速统计：合并连续事件
        
        Args:
            df: 数据DataFrame
        
        Returns:
            (急刹次数, 急加速次数)
        """
        harsh_braking_count = 0
        harsh_accel_count = 0
        
        # 急刹/急加速阈值
        HARSH_BRAKING_THRESHOLD = -3.5
        HARSH_ACCEL_THRESHOLD = 2.0
        
        for vehicle_id in df['car_name'].unique():
            veh_data = df[df['car_name'] == vehicle_id].sort_values('timestamp')
            
            if len(veh_data) < 2:
                continue
            
            # 统计急刹事件（连续的只算一次）
            is_harsh_braking = (veh_data['acceleration'] < HARSH_BRAKING_THRESHOLD).values
            in_braking_event = False
            
            for harsh in is_harsh_braking:
                if harsh and not in_braking_event:
                    harsh_braking_count += 1
                    in_braking_event = True
                elif not harsh:
                    in_braking_event = False
            
            # 统计急加速事件（连续的只算一次）
            is_harsh_accel = (veh_data['acceleration'] > HARSH_ACCEL_THRESHOLD).values
            in_accel_event = False
            
            for harsh in is_harsh_accel:
                if harsh and not in_accel_event:
                    harsh_accel_count += 1
                    in_accel_event = True
                elif not harsh:
                    in_accel_event = False
        
        return harsh_braking_count, harsh_accel_count
    
    def get_data_for_convergence_plot(self, optimization_history: List[Dict]) -> Dict:
        """
        准备收敛曲线绘图数据
        
        Args:
            optimization_history: 优化历史记录
            
        Returns:
            绘图数据字典
        """
        if not optimization_history:
            return {
                'times': [],
                'initial_costs': [],
                'final_costs': [],
                'improvements': []
            }
        
        times = [h['time'] for h in optimization_history]
        initial_costs = [h['initial_cost'] for h in optimization_history]
        final_costs = [h['final_cost'] for h in optimization_history]
        improvements = [h['improvement'] * 100 for h in optimization_history]
        
        return {
            'times': times,
            'initial_costs': initial_costs,
            'final_costs': final_costs,
            'improvements': improvements
        }


class PerformanceAnalyzer:
    """性能分析器"""
    
    def __init__(self, data_collector: DataCollector):
        self.data_collector = data_collector
    
    def analyze_merge_efficiency(self) -> Dict:
        """分析合流效率"""
        metrics = self.data_collector.calculate_performance_metrics()
        
        analysis = {
            'summary': {},
            'recommendations': []
        }
        
        # 主线性能分析
        if 'mainline_avg_travel_time' in metrics:
            travel_time = metrics['mainline_avg_travel_time']
            entry_slow_speed = metrics.get('mainline_entry_slow_avg_speed')
            entry_fast_speed = metrics.get('mainline_entry_fast_avg_speed')
            avg_speed = metrics.get('mainline_avg_speed', 0)
            if entry_slow_speed or entry_fast_speed:
                speeds = [
                    speed for speed in [entry_slow_speed, entry_fast_speed]
                    if speed is not None and speed > 0
                ]
                if speeds:
                    avg_speed = float(np.mean(speeds))
            
            analysis['summary']['mainline'] = {
                'avg_travel_time': f"{travel_time:.2f} 秒",
                'avg_speed': f"{avg_speed:.2f} m/s ({avg_speed*3.6:.2f} km/h)",
                'performance': 'good' if avg_speed > 25 else 'moderate'
            }
            
            if avg_speed < 25:
                analysis['recommendations'].append("主线平均速度较低，建议优化车队间距")
        
        # 匝道性能分析
        if 'ramp_merge_success_rate' in metrics:
            success_rate = metrics['ramp_merge_success_rate']
            
            analysis['summary']['ramp'] = {
                'merge_success_rate': f"{success_rate:.1f}%",
                'total_vehicles': metrics['ramp_total_vehicles'],
                'successful_merges': metrics['ramp_successful_merges'],
                'performance': 'excellent' if success_rate > 90 else 'good' if success_rate > 80 else 'moderate'
            }
            
            if success_rate < 90:
                analysis['recommendations'].append("匝道合流成功率偏低，建议增加匝道车辆优先级")
        
        # 安全性分析
        if 'harsh_braking_events' in metrics:
            harsh_braking = metrics['harsh_braking_events']
            
            analysis['summary']['safety'] = {
                'harsh_braking_events': harsh_braking,
                'harsh_acceleration_events': metrics['harsh_acceleration_events'],
                'safety_level': 'high' if harsh_braking < 10 else 'moderate'
            }
            
            if harsh_braking > 10:
                analysis['recommendations'].append("存在较多急刹事件，建议增大安全距离")
        
        return analysis
    
    def generate_report(self) -> str:
        """生成文本报告"""
        stats = self.data_collector.get_statistics()
        metrics = self.data_collector.calculate_performance_metrics()
        analysis = self.analyze_merge_efficiency()
        
        report = []
        report.append("=" * 60)
        report.append("车辆合流仿真性能报告")
        report.append("=" * 60)
        report.append("")
        
        # 基本统计
        report.append("【基本统计】")
        report.append(f"  仿真时长: {stats.get('simulation_duration', 0):.1f} 秒")
        report.append(f"  总记录数: {stats.get('total_records', 0)}")
        report.append(f"  车辆总数: {stats.get('unique_vehicles', 0)}")
        report.append(f"    - 主线车辆: {stats.get('mainline_vehicles', 0)}")
        report.append(f"    - 匝道车辆: {stats.get('ramp_vehicles', 0)}")
        report.append(f"  总油耗: {stats.get('total_fuel_consumption_l', 0):.4f}升")
        report.append(f"  平均油耗: {stats.get('avg_fuel_per_vehicle_l', 0):.4f}升/车")
        report.append("")
        
        # 性能指标
        report.append("【性能指标】")
        
        if 'mainline' in analysis['summary']:
            mainline = analysis['summary']['mainline']
            report.append(f"  主线性能:")
            report.append(f"    - 平均通行时间: {mainline['avg_travel_time']}")
            report.append(f"    - 平均速度: {mainline['avg_speed']}")
            if 'mainline_entry_slow_avg_speed' in metrics:
                report.append(
                    "    - 入口慢车道平均速度: "
                    f"{metrics['mainline_entry_slow_avg_speed']:.2f} m/s "
                    f"({metrics['mainline_entry_slow_avg_speed']*3.6:.2f} km/h)"
                )
            if 'mainline_entry_fast_avg_speed' in metrics:
                report.append(
                    "    - 入口快车道平均速度: "
                    f"{metrics['mainline_entry_fast_avg_speed']:.2f} m/s "
                    f"({metrics['mainline_entry_fast_avg_speed']*3.6:.2f} km/h)"
                )
            report.append(f"    - 性能评级: {mainline['performance']}")
        
        if 'ramp' in analysis['summary']:
            ramp = analysis['summary']['ramp']
            report.append(f"  匝道性能:")
            report.append(f"    - 合流成功率: {ramp['merge_success_rate']}")
            report.append(f"    - 成功合流: {ramp['successful_merges']}/{ramp['total_vehicles']}")
            report.append(f"    - 性能评级: {ramp['performance']}")
        
        if 'safety' in analysis['summary']:
            safety = analysis['summary']['safety']
            report.append(f"  安全性:")
            report.append(f"    - 急刹事件: {safety['harsh_braking_events']}")
            report.append(f"    - 急加速事件: {safety['harsh_acceleration_events']}")
            report.append(f"    - 安全等级: {safety['safety_level']}")
        
        report.append("")
        
        # 建议
        if analysis['recommendations']:
            report.append("【优化建议】")
            for i, rec in enumerate(analysis['recommendations'], 1):
                report.append(f"  {i}. {rec}")
            report.append("")
        
        report.append("=" * 60)
        
        return "\n".join(report)


if __name__ == "__main__":
    # 测试数据收集器
    collector = DataCollector()
    
    print("=== 数据收集器测试 ===\n")
    
    # 模拟收集数据
    np.random.seed(42)
    for t in np.arange(0, 10, 0.5):
        # 主线车辆
        for i in range(3):
            collector.collect_vehicle_data(
                vehicle_id=f"m{i}",
                timestamp=t,
                position=500 + t * 30 + i * 20,
                velocity=30.0 + np.random.randn() * 2,
                acceleration=np.random.randn() * 0.5,
                lane_id=0
            )
        
        # 匝道车辆
        collector.collect_vehicle_data(
            vehicle_id="r0",
            timestamp=t,
            position=450 + t * 25,
            velocity=25.0 + np.random.randn() * 2,
            acceleration=np.random.randn() * 0.5,
            lane_id=1
        )
        
        collector.update_collection_time(t)
    
    # 导出Excel
    collector.export_to_excel("output/test_data.xlsx")
    
    # 显示统计信息
    print("\n统计信息:")
    stats = collector.get_statistics()
    for key, value in stats.items():
        print(f"  {key}: {value}")
    
    # 性能分析
    print("\n性能分析:")
    analyzer = PerformanceAnalyzer(collector)
    print(analyzer.generate_report())
