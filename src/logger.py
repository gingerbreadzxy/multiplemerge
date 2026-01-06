"""
日志记录模块
记录仿真过程的详细信息，用于结果分析和调试
"""

import os
from datetime import datetime
from typing import Dict, List, Any
from src.config import *


class SimulationLogger:
    """仿真日志记录器"""
    
    def __init__(self, log_file: str = "output/simulation_log.txt"):
        self.log_file = log_file
        self.log_buffer = []
        self.start_time = None
        
        # 确保输出目录存在
        os.makedirs(os.path.dirname(log_file), exist_ok=True)
        
        # 清空旧日志
        with open(log_file, 'w', encoding='utf-8') as f:
            f.write("")
    
    def log(self, message: str, level: str = "INFO"):
        """
        记录日志消息
        
        Args:
            message: 日志消息
            level: 日志级别 (INFO, WARNING, ERROR, DEBUG)
        """
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        log_entry = f"[{timestamp}] [{level:7s}] {message}"
        
        # 打印到控制台（可选）
        if level in ["ERROR", "WARNING"]:
            print(log_entry)
        
        # 添加到缓冲区
        self.log_buffer.append(log_entry)
        
        # 定期写入文件（每100条）
        if len(self.log_buffer) >= 100:
            self._flush()
    
    def _flush(self):
        """将缓冲区内容写入文件"""
        if self.log_buffer:
            with open(self.log_file, 'a', encoding='utf-8') as f:
                f.write('\n'.join(self.log_buffer) + '\n')
            self.log_buffer.clear()
    
    def log_section(self, title: str):
        """记录分节标题"""
        separator = "=" * 80
        self.log(separator)
        self.log(f"  {title}")
        self.log(separator)
    
    def log_subsection(self, title: str):
        """记录子节标题"""
        separator = "-" * 60
        self.log(separator)
        self.log(f"  {title}")
        self.log(separator)
    
    def log_simulation_start(self, config: Dict):
        """记录仿真开始信息"""
        self.start_time = datetime.now()
        
        self.log_section("仿真启动")
        self.log(f"启动时间: {self.start_time.strftime('%Y-%m-%d %H:%M:%S')}")
        self.log("")
        
        self.log_subsection("仿真参数")
        for key, value in config.items():
            self.log(f"  {key}: {value}")
        self.log("")
        
        self.log_subsection("车辆初始速度设置说明")
        self.log("车辆生成时的初始速度设置:")
        self.log(f"  - 主线车队头车: 期望速度 × 0.8 (约{PLATOON_DESIRED_SPEED * 0.8:.1f} m/s)")
        self.log(f"  - 主线车队跟随车: 期望速度 (约{PLATOON_DESIRED_SPEED:.1f} m/s)")
        self.log(f"  - 匝道车辆: 期望速度 × 0.8 (约{RAMP_DESIRED_SPEED * 0.8:.1f} m/s)")
        self.log("")
        self.log("注意: Excel表格中第一次记录的速度可能与初始速度不同，原因：")
        self.log("  1. 数据采集间隔为0.5秒，车辆在此期间已经加速/减速")
        self.log("  2. SUMO仿真步长为0.1秒，车辆速度会根据跟驰模型实时调整")
        self.log("  3. 车队内跟随车需要根据前车调整速度以保持安全距离")
        self.log("")
    
    def log_vehicle_generation(self, current_time: float, mainline_count: int, 
                               ramp_count: int, details: List[str] = None,
                               fast_mainline_count: int = 0):
        """
        记录车辆生成信息
        
        Args:
            current_time: 当前仿真时间
            mainline_count: 生成的主线车辆数
            ramp_count: 生成的匝道车辆数
            details: 详细信息列表
        """
        if mainline_count > 0 or ramp_count > 0 or fast_mainline_count > 0:
            self.log(f"时刻 {current_time:.1f}s: 生成车辆 - 主线:{mainline_count}, 快车道:{fast_mainline_count}, 匝道:{ramp_count}")
            if details:
                for detail in details:
                    self.log(f"  {detail}", "DEBUG")
    
    def log_optimization_run(self, current_time: float, mainline_veh: int, 
                            ramp_veh: int, initial_cost: float, 
                            final_cost: float, iterations: int):
        """
        记录优化算法运行信息
        
        Args:
            current_time: 当前仿真时间
            mainline_veh: 主线车辆数
            ramp_veh: 匝道车辆数
            initial_cost: 初始代价
            final_cost: 最终代价
            iterations: 迭代次数
        """
        improvement = (initial_cost - final_cost) / max(initial_cost, 1e-6) * 100
        self.log(f"时刻 {current_time:.1f}s: 优化执行")
        self.log(f"  合流区域车辆: 主线={mainline_veh}, 匝道={ramp_veh}")
        self.log(f"  初始代价: {initial_cost:.2f}")
        self.log(f"  最终代价: {final_cost:.2f}")
        self.log(f"  改进率: {improvement:.2f}%")
        self.log(f"  迭代次数: {iterations}")
    
    def log_merge_sequence(self, current_time: float, sequence: List[Dict]):
        """
        记录优化后的合流序列
        
        Args:
            current_time: 当前仿真时间
            sequence: 车辆序列
        """
        self.log(f"时刻 {current_time:.1f}s: 合流序列 (共{len(sequence)}辆)")
        for i, veh in enumerate(sequence[:10]):  # 只记录前10辆
            self.log(f"  {i+1}. {veh.get('id')} ({veh.get('type')}) - "
                    f"位置:{veh.get('position', 0):.1f}m, "
                    f"速度:{veh.get('speed', 0):.1f}m/s", "DEBUG")
        if len(sequence) > 10:
            self.log(f"  ... 还有 {len(sequence)-10} 辆车", "DEBUG")

    def log_control_command(self, veh_id: str, command: Dict):
        """
        记录控制指令
        
        Args:
            veh_id: 车辆ID
            command: 控制指令字典
        """
        self.log(f"  控制 {veh_id}: "
                f"目标速度={command.get('target_speed', 0):.1f}m/s, "
                f"加速度={command.get('acceleration', 0):.2f}m/s², "
                f"动作={command.get('action', 'unknown')}", "DEBUG")
    
    def log_safety_event(self, event_type: str, veh_id: str, details: str):
        """
        记录安全相关事件
        
        Args:
            event_type: 事件类型 (collision, near_miss, harsh_braking, etc.)
            veh_id: 车辆ID
            details: 详细信息
        """
        self.log(f"安全事件 [{event_type}] - 车辆:{veh_id}, {details}", "WARNING")

    def log_vehicle_state_snapshot(self, current_time: float, 
                                   active_vehicles: Dict):
        """
        记录车辆状态快照（每隔一段时间）
        
        Args:
            current_time: 当前仿真时间
            active_vehicles: 活动车辆字典
        """
        mainline_count = sum(1 for v in active_vehicles.values() if v.type == 'mainline')
        ramp_count = sum(1 for v in active_vehicles.values() if v.type == 'ramp')
        
        avg_speed = sum(v.current_speed for v in active_vehicles.values()) / max(len(active_vehicles), 1)
        
        self.log(f"时刻 {current_time:.1f}s: 系统状态快照")
        self.log(f"  活动车辆: {len(active_vehicles)} (主线:{mainline_count}, 匝道:{ramp_count})")
        self.log(f"  平均速度: {avg_speed:.2f} m/s ({avg_speed*3.6:.2f} km/h)")
    
    def log_performance_metrics(self, metrics: Dict):
        """
        记录性能指标
        
        Args:
            metrics: 性能指标字典
        """
        self.log_subsection("性能指标")
        
        if 'mainline_avg_travel_time' in metrics:
            self.log("主线性能:")
            self.log(f"  平均通行时间: {metrics['mainline_avg_travel_time']:.2f} 秒")
            self.log(f"  平均速度: {metrics['mainline_avg_speed']:.2f} m/s "
                    f"({metrics['mainline_avg_speed']*3.6:.2f} km/h)")
            self.log(f"  速度标准差: {metrics.get('mainline_std_speed', 0):.2f} m/s")
        
        if 'ramp_merge_success_rate' in metrics:
            self.log("匝道性能:")
            self.log(f"  合流成功率: {metrics['ramp_merge_success_rate']:.2f}%")
            self.log(f"  成功合流: {metrics['ramp_successful_merges']}/{metrics['ramp_total_vehicles']}")
            self.log(f"  平均速度: {metrics['ramp_avg_speed']:.2f} m/s "
                    f"({metrics['ramp_avg_speed']*3.6:.2f} km/h)")
        
        if 'harsh_braking_events' in metrics:
            self.log("安全性:")
            self.log(f"  急刹事件: {metrics['harsh_braking_events']}")
            self.log(f"  急加速事件: {metrics['harsh_acceleration_events']}")
        
        self.log(f"整体平均速度: {metrics.get('overall_avg_speed', 0):.2f} m/s "
                f"({metrics.get('overall_avg_speed', 0)*3.6:.2f} km/h)")
        self.log("")
    
    def log_simulation_progress(self, current_time: float, total_time: float,
                               active_vehicles: int, total_generated: int):
        """
        记录仿真进度
        
        Args:
            current_time: 当前时间
            total_time: 总时间
            active_vehicles: 当前活动车辆数
            total_generated: 已生成车辆总数
        """
        progress = current_time / total_time * 100
        self.log(f"仿真进度: {progress:.1f}% ({current_time:.1f}/{total_time}s), "
                f"活动车辆:{active_vehicles}, 已生成:{total_generated}")
    
    def log_error(self, error_type: str, details: str):
        """
        记录错误信息
        
        Args:
            error_type: 错误类型
            details: 错误详情
        """
        self.log(f"错误 [{error_type}]: {details}", "ERROR")
    
    def log_simulation_end(self, stats: Dict):
        """
        记录仿真结束信息
        
        Args:
            stats: 统计信息字典
        """
        end_time = datetime.now()
        duration = (end_time - self.start_time).total_seconds() if self.start_time else 0
        
        self.log_section("仿真结束")
        self.log(f"结束时间: {end_time.strftime('%Y-%m-%d %H:%M:%S')}")
        self.log(f"总耗时: {duration:.2f} 秒")
        self.log("")
        
        self.log_subsection("仿真统计")
        for key, value in stats.items():
            if isinstance(value, float):
                self.log(f"  {key}: {value:.2f}")
            else:
                self.log(f"  {key}: {value}")
        self.log("")
    
    def log_data_summary(self, summary: str):
        """
        记录数据摘要
        
        Args:
            summary: 摘要文本
        """
        self.log_subsection("数据摘要")
        for line in summary.split('\n'):
            if line.strip():
                self.log(line)
    
    def log_conclusion(self, conclusion: Dict):
        """
        记录结论和建议
        
        Args:
            conclusion: 结论字典，包含评估和建议
        """
        self.log_section("仿真结论")
        
        if 'evaluation' in conclusion:
            self.log("系统评估:")
            for item in conclusion['evaluation']:
                self.log(f"  ✓ {item}")
            self.log("")
        
        if 'recommendations' in conclusion:
            self.log("优化建议:")
            for i, rec in enumerate(conclusion['recommendations'], 1):
                self.log(f"  {i}. {rec}")
            self.log("")
        
        if 'success' in conclusion:
            self.log(f"仿真状态: {'成功' if conclusion['success'] else '失败'}")
            if 'reason' in conclusion:
                self.log(f"原因: {conclusion['reason']}")
    
    def finalize(self):
        """完成日志记录，写入所有缓冲内容"""
        self._flush()
        self.log_section("日志记录完成")
        self._flush()


class PerformanceLogger:
    """性能分析日志器"""
    
    def __init__(self, log_file: str = "output/performance_analysis.txt"):
        self.log_file = log_file
        self.optimization_history = []
        self.safety_events = []
        self.vehicle_records = {}
        
    def record_optimization(self, time: float, cost_before: float, 
                          cost_after: float, improvement: float):
        """记录优化历史"""
        self.optimization_history.append({
            'time': time,
            'cost_before': cost_before,
            'cost_after': cost_after,
            'improvement': improvement
        })
    
    def record_safety_event(self, time: float, event_type: str, 
                           veh_id: str, severity: str):
        """记录安全事件"""
        self.safety_events.append({
            'time': time,
            'type': event_type,
            'vehicle': veh_id,
            'severity': severity
        })
    
    def record_vehicle_journey(self, veh_id: str, veh_type: str, 
                              start_time: float, end_time: float,
                              avg_speed: float, max_speed: float,
                              merge_success: bool = None):
        """记录车辆行程"""
        self.vehicle_records[veh_id] = {
            'type': veh_type,
            'start_time': start_time,
            'end_time': end_time,
            'travel_time': end_time - start_time,
            'avg_speed': avg_speed,
            'max_speed': max_speed,
            'merge_success': merge_success
        }
    
    def generate_report(self):
        """生成性能分析报告"""
        with open(self.log_file, 'w', encoding='utf-8') as f:
            f.write("=" * 80 + "\n")
            f.write("性能分析报告\n")
            f.write("=" * 80 + "\n\n")
            
            # 优化效果分析
            if self.optimization_history:
                f.write("1. 优化算法效果分析\n")
                f.write("-" * 60 + "\n")
                
                avg_improvement = sum(h['improvement'] for h in self.optimization_history) / len(self.optimization_history)
                max_improvement = max(h['improvement'] for h in self.optimization_history)
                min_improvement = min(h['improvement'] for h in self.optimization_history)
                
                f.write(f"  优化运行次数: {len(self.optimization_history)}\n")
                f.write(f"  平均改进率: {avg_improvement:.2f}%\n")
                f.write(f"  最大改进率: {max_improvement:.2f}%\n")
                f.write(f"  最小改进率: {min_improvement:.2f}%\n")
                
                # 优化趋势
                improvements = [h['improvement'] for h in self.optimization_history]
                if len(improvements) > 10:
                    early_avg = sum(improvements[:10]) / 10
                    late_avg = sum(improvements[-10:]) / 10
                    f.write(f"  初期平均改进: {early_avg:.2f}%\n")
                    f.write(f"  后期平均改进: {late_avg:.2f}%\n")
                
                f.write("\n")
            
            # 安全事件分析
            if self.safety_events:
                f.write("2. 安全事件分析\n")
                f.write("-" * 60 + "\n")
                f.write(f"  总事件数: {len(self.safety_events)}\n")
                
                event_types = {}
                for event in self.safety_events:
                    event_type = event['type']
                    event_types[event_type] = event_types.get(event_type, 0) + 1
                
                for event_type, count in event_types.items():
                    f.write(f"  {event_type}: {count}\n")
                
                f.write("\n")
            
            # 车辆行程分析
            if self.vehicle_records:
                f.write("3. 车辆行程分析\n")
                f.write("-" * 60 + "\n")
                
                mainline_records = [v for v in self.vehicle_records.values() if v['type'] == 'mainline']
                ramp_records = [v for v in self.vehicle_records.values() if v['type'] == 'ramp']
                
                if mainline_records:
                    avg_travel = sum(v['travel_time'] for v in mainline_records) / len(mainline_records)
                    avg_speed = sum(v['avg_speed'] for v in mainline_records) / len(mainline_records)
                    
                    f.write(f"  主线车辆:\n")
                    f.write(f"    完成车辆数: {len(mainline_records)}\n")
                    f.write(f"    平均通行时间: {avg_travel:.2f} 秒\n")
                    f.write(f"    平均速度: {avg_speed:.2f} m/s ({avg_speed*3.6:.2f} km/h)\n")
                
                if ramp_records:
                    avg_travel = sum(v['travel_time'] for v in ramp_records) / len(ramp_records)
                    avg_speed = sum(v['avg_speed'] for v in ramp_records) / len(ramp_records)
                    success_count = sum(1 for v in ramp_records if v.get('merge_success', False))
                    
                    f.write(f"  匝道车辆:\n")
                    f.write(f"    完成车辆数: {len(ramp_records)}\n")
                    f.write(f"    合流成功数: {success_count}\n")
                    f.write(f"    合流成功率: {success_count/len(ramp_records)*100:.1f}%\n")
                    f.write(f"    平均通行时间: {avg_travel:.2f} 秒\n")
                    f.write(f"    平均速度: {avg_speed:.2f} m/s ({avg_speed*3.6:.2f} km/h)\n")
                
                f.write("\n")
            
            f.write("=" * 80 + "\n")
            f.write("报告结束\n")
            f.write("=" * 80 + "\n")


if __name__ == "__main__":
    # 测试日志记录器
    logger = SimulationLogger("test_log.txt")
    
    logger.log_simulation_start({
        'simulation_time': 600,
        'mainline_rate': 0.15,
        'ramp_rate': 0.1
    })
    
    logger.log_vehicle_generation(10.0, 3, 1, ["车队p1生成", "匝道车r1生成"])
    
    logger.log_optimization_run(15.0, 5, 2, 150.0, 120.0, 10)
    
    logger.log_performance_metrics({
        'mainline_avg_speed': 28.5,
        'ramp_merge_success_rate': 95.0,
        'harsh_braking_events': 3
    })
    
    logger.log_simulation_end({
        'total_vehicles': 100,
        'simulation_duration': 600
    })
    
    logger.finalize()
    
    print("测试日志已生成: test_log.txt")


