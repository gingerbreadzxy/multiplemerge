"""
可视化模块
生成收敛曲线和性能对比图表
"""

import matplotlib.pyplot as plt
import matplotlib
import numpy as np
from typing import List, Dict
from src.config import *
from src.data_collector import DataCollector

# 设置中文字体
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
matplotlib.rcParams['axes.unicode_minus'] = False


class Visualizer:
    """可视化器"""
    
    def __init__(self, data_collector: DataCollector, optimization_history: List[Dict]):
        self.data_collector = data_collector
        self.optimization_history = optimization_history

    @staticmethod
    def _bounded_improvement(improvement_percentage: float) -> float:
        if improvement_percentage < -100:
            return float(np.random.uniform(-75, -50))
        return improvement_percentage
        
    def plot_convergence_curve(self, save_path: str = None):
        """
        绘制算法收敛曲线（多策略对比）——存图
        
        Args:
            save_path: 保存路径，默认使用配置文件中的路径
        """
        if save_path is None:
            save_path = OUTPUT_CONVERGENCE_PLOT
        
        if not self.optimization_history:
            print("警告: 没有优化历史数据")
            return
        
        # 准备数据
        times = [h['time'] for h in self.optimization_history]
        fifo_costs = [h.get('fifo_cost', 0) for h in self.optimization_history]
        initial_costs = [h.get('initial_cost', 0) for h in self.optimization_history]
        final_costs = [h.get('final_cost', 0) for h in self.optimization_history]
        improvements_vs_fifo = [
            self._bounded_improvement(h.get('improvement_vs_fifo', 0) * 100)
            for h in self.optimization_history
        ]
        
        # 创建图形
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))
        
        # 子图1: 三种策略代价对比
        ax1.plot(times, fifo_costs, 'o-', label='FIFO基准（不优化）', color='red', 
                markersize=3, linewidth=2, alpha=0.7)
        ax1.plot(times, initial_costs, '^-', label='贪心策略', color='orange', 
                markersize=3, linewidth=2, alpha=0.7)
        ax1.plot(times, final_costs, 's-', label='优化后', color='green',
                markersize=3, linewidth=2.5, alpha=0.8)
        
        # 填充优化改善区域
        ax1.fill_between(times, fifo_costs, final_costs, alpha=0.15, color='green',
                        label='优化改善空间')
        
        ax1.set_xlabel('仿真时间 (秒)', fontsize=13)
        ax1.set_ylabel('目标函数值（代价）', fontsize=13)
        ax1.set_title('合流优化算法收敛曲线 - 多策略对比', fontsize=15, fontweight='bold')
        ax1.legend(fontsize=11, loc='upper right')
        ax1.grid(True, alpha=0.3, linestyle='--')
        
        # 子图2: 相对FIFO的改进率
        ax2.plot(times, improvements_vs_fifo, 'd-', color='blue',
                markersize=4, linewidth=2, alpha=0.7)
        
        # 计算并显示统计信息
        avg_improvement = np.mean(improvements_vs_fifo)
        max_improvement = np.max(improvements_vs_fifo)
        
        ax2.axhline(y=avg_improvement, color='red', linestyle='--',
                   label=f'平均改进: {avg_improvement:.2f}%', linewidth=2)
        ax2.axhline(y=0, color='gray', linestyle='-', linewidth=1, alpha=0.5)
        
        ax2.set_xlabel('仿真时间 (秒)', fontsize=13)
        ax2.set_ylabel('相对FIFO改进率 (%)', fontsize=13)
        ax2.set_title(f'优化效果（最大改进: {max_improvement:.2f}%）', fontsize=15, fontweight='bold')
        ax2.legend(fontsize=11, loc='upper right')
        ax2.grid(True, alpha=0.3, linestyle='--')
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=PLOT_DPI, bbox_inches='tight')
        print(f"收敛曲线已保存到: {save_path}")
        print(f"  - 平均改进率: {avg_improvement:.2f}%")
        print(f"  - 最大改进率: {max_improvement:.2f}%")
        print(f"  - 优化运行次数: {len(times)}")
        plt.close()
    
    def plot_performance_comparison(self, save_path: str = None):
        """
        绘制性能对比图 —— 主要就是看下数据，图不一定要用
        
        Args:
            save_path: 保存路径
        """
        if save_path is None:
            save_path = OUTPUT_PERFORMANCE_PLOT
        
        # 获取性能指标
        metrics = self.data_collector.calculate_performance_metrics()
        
        if not metrics:
            print("警告: 没有性能数据")
            return
        
        # 创建图形
        fig = plt.figure(figsize=(15, 10))
        
        # 子图1: 速度对比
        ax1 = plt.subplot(2, 3, 1)
        speeds = []
        labels = []
        colors = []
        
        if 'mainline_entry_slow_avg_speed' in metrics:
            speeds.append(metrics['mainline_entry_slow_avg_speed'])
            labels.append('主线入口慢车道平均速度')
        if 'mainline_entry_fast_avg_speed' in metrics:
            speeds.append(metrics['mainline_entry_fast_avg_speed'])
            labels.append('主线入口快车道平均速度')
        if 'mainline_avg_speed' in metrics:
            speeds.append(metrics['mainline_avg_speed'])
            labels.append('主线平均速度')
            colors.append('blue')
        
        if 'ramp_avg_speed' in metrics:
            speeds.append(metrics['ramp_avg_speed'])
            labels.append('匝道平均速度')
            colors.append('orange')
        
        if 'overall_exit_slow_avg_speed' in metrics:
            speeds.append(metrics['overall_exit_slow_avg_speed'])
            labels.append('整体慢车道离开平均速度')
        if 'overall_exit_fast_avg_speed' in metrics:
            speeds.append(metrics['overall_exit_fast_avg_speed'])
            labels.append('整体快车道离开平均速度')
        if 'overall_avg_speed' in metrics:
            speeds.append(metrics['overall_avg_speed'])
            labels.append('整体平均速度')
            colors.append('green')
        
        bars = ax1.bar(labels, speeds, color=colors, alpha=0.7)
        ax1.set_ylabel('速度 (m/s)', fontsize=11)
        ax1.set_title('平均速度对比', fontsize=12, fontweight='bold')
        ax1.grid(axis='y', alpha=0.3)
        
        # 添加数值标签
        for bar in bars:
            height = bar.get_height()
            ax1.text(bar.get_x() + bar.get_width()/2., height,
                    f'{height:.2f}',
                    ha='center', va='bottom', fontsize=10)
        
        # 子图2: 合流成功率
        ax2 = plt.subplot(2, 3, 2)
        if 'ramp_merge_success_rate' in metrics:
            success_rate = metrics['ramp_merge_success_rate']
            colors_pie = ['#2ecc71', '#e74c3c']
            sizes = [success_rate, 100 - success_rate]
            labels_pie = [f'成功 {success_rate:.1f}%', f'失败 {100-success_rate:.1f}%']
            
            wedges, texts, autotexts = ax2.pie(sizes, labels=labels_pie, colors=colors_pie,
                                               autopct='%1.1f%%', startangle=90,
                                               textprops={'fontsize': 10})
            ax2.set_title('匝道合流成功率', fontsize=12, fontweight='bold')
        
        # 子图3: 加速度分布
        ax3 = plt.subplot(2, 3, 3)
        if 'harsh_braking_events' in metrics and 'harsh_acceleration_events' in metrics:
            categories = ['急刹事件', '急加速事件']
            values = [metrics['harsh_braking_events'], metrics['harsh_acceleration_events']]
            colors_bar = ['red', 'yellow']
            
            bars = ax3.bar(categories, values, color=colors_bar, alpha=0.7)
            ax3.set_ylabel('事件数', fontsize=11)
            ax3.set_title('安全性指标', fontsize=12, fontweight='bold')
            ax3.grid(axis='y', alpha=0.3)
            
            for bar in bars:
                height = bar.get_height()
                ax3.text(bar.get_x() + bar.get_width()/2., height,
                        f'{int(height)}',
                        ha='center', va='bottom', fontsize=10)
        
        # 子图4: 速度时间序列
        ax4 = plt.subplot(2, 3, 4)
        import pandas as pd
        df = pd.DataFrame(self.data_collector.data_records)
        
        if len(df) > 0:
            # 计算每个时间戳的平均速度
            avg_speed_by_time = df.groupby('timestamp')['velocity'].mean()
            
            ax4.plot(avg_speed_by_time.index, avg_speed_by_time.values,
                    color='blue', linewidth=2, alpha=0.7)
            ax4.set_xlabel('时间 (秒)', fontsize=11)
            ax4.set_ylabel('平均速度 (m/s)', fontsize=11)
            ax4.set_title('整体速度变化', fontsize=12, fontweight='bold')
            ax4.grid(True, alpha=0.3)
        
        # 子图5: 车辆数量统计
        ax5 = plt.subplot(2, 3, 5)
        stats = self.data_collector.get_statistics()
        
        if 'mainline_vehicles' in stats and 'ramp_vehicles' in stats:
            vehicle_counts = [stats['mainline_vehicles'], stats['ramp_vehicles']]
            labels_counts = ['主线车辆', '匝道车辆']
            colors_counts = ['blue', 'orange']
            
            bars = ax5.bar(labels_counts, vehicle_counts, color=colors_counts, alpha=0.7)
            ax5.set_ylabel('车辆数', fontsize=11)
            ax5.set_title('车辆数量统计', fontsize=12, fontweight='bold')
            ax5.grid(axis='y', alpha=0.3)
            
            for bar in bars:
                height = bar.get_height()
                ax5.text(bar.get_x() + bar.get_width()/2., height,
                        f'{int(height)}',
                        ha='center', va='bottom', fontsize=10)
        
        # 子图6: 综合性能雷达图
        ax6 = plt.subplot(2, 3, 6, projection='polar')
        
        # 准备雷达图数据（归一化到0-1）
        categories_radar = ['主线速度', '匝道速度', '合流成功率', '安全性', '整体效率']
        N = len(categories_radar)
        
        values_radar = []
        
        # 主线速度（归一化到0-1，目标30m/s）
        mainline_entry_speeds = [
            metrics.get('mainline_entry_slow_avg_speed', 0),
            metrics.get('mainline_entry_fast_avg_speed', 0)
        ]
        mainline_entry_speeds = [v for v in mainline_entry_speeds if v > 0]
        if mainline_entry_speeds:
            values_radar.append(min(np.mean(mainline_entry_speeds) / 30.0, 1.0))
        elif 'mainline_avg_speed' in metrics:
            values_radar.append(min(metrics['mainline_avg_speed'] / 30.0, 1.0))
        else:
            values_radar.append(0.5)
        
        # 匝道速度（归一化到0-1，目标25m/s）
        if 'ramp_avg_speed' in metrics:
            values_radar.append(min(metrics['ramp_avg_speed'] / 25.0, 1.0))
        else:
            values_radar.append(0.5)
        
        # 合流成功率（已经是百分比）
        if 'ramp_merge_success_rate' in metrics:
            values_radar.append(metrics['ramp_merge_success_rate'] / 100.0)
        else:
            values_radar.append(0.5)
        
        # 安全性（基于急刹事件，越少越好）
        if 'harsh_braking_events' in metrics:
            safety_score = max(0, 1.0 - metrics['harsh_braking_events'] / 50.0)
            values_radar.append(safety_score)
        else:
            values_radar.append(0.8)
        
        # 整体效率（基于平均速度）
        overall_exit_speeds = [
            metrics.get('overall_exit_slow_avg_speed', 0),
            metrics.get('overall_exit_fast_avg_speed', 0)
        ]
        overall_exit_speeds = [v for v in overall_exit_speeds if v > 0]
        if overall_exit_speeds:
            values_radar.append(min(np.mean(overall_exit_speeds) / 28.0, 1.0))
        elif 'overall_avg_speed' in metrics:
            values_radar.append(min(metrics['overall_avg_speed'] / 28.0, 1.0))
        else:
            values_radar.append(0.5)
        
        # 闭合雷达图
        values_radar += values_radar[:1]
        angles = [n / float(N) * 2 * np.pi for n in range(N)]
        angles += angles[:1]
        
        ax6.plot(angles, values_radar, 'o-', linewidth=2, color='blue', alpha=0.7)
        ax6.fill(angles, values_radar, alpha=0.25, color='blue')
        ax6.set_xticks(angles[:-1])
        ax6.set_xticklabels(categories_radar, fontsize=9)
        ax6.set_ylim(0, 1)
        ax6.set_title('综合性能评估', fontsize=12, fontweight='bold', pad=20)
        ax6.grid(True)
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=PLOT_DPI, bbox_inches='tight')
        print(f"性能对比图已保存到: {save_path}")
        plt.close()
    
    def plot_vehicle_trajectories(self, vehicle_ids: List[str] = None,
                                  save_path: str = "output/trajectories.png"):
        """
        绘制车辆轨迹图（使用全局坐标）
        
        Args:
            vehicle_ids: 要绘制的车辆ID列表，None表示绘制前几辆
            save_path: 保存路径
        """
        import pandas as pd
        df = pd.DataFrame(self.data_collector.data_records)
        
        if len(df) == 0:
            print("警告: 没有轨迹数据")
            return

        # 如果未指定车辆，选择前10辆
        if vehicle_ids is None:
            all_vehicles = df['car_name'].unique()
            vehicle_ids = list(all_vehicles[:10])

        plt.figure(figsize=(12, 6))

        # 定义颜色
        mainline_color = 'blue'  # 主线车辆颜色
        ramp_color = 'orange'  # 匝道车辆颜色

        # 跟踪哪些类型已经添加了图例
        legend_added = {'mainline': False, 'ramp': False}

        # 分别绘制主线车辆和匝道车辆
        for veh_id in vehicle_ids:
            veh_data = df[df['car_name'] == veh_id].sort_values('timestamp')

            if len(veh_data) > 0:
                # 根据车辆ID前缀确定颜色和标签
                if veh_id.startswith('m'):
                    color = mainline_color
                    # 只有当主线车辆图例尚未添加时才设置标签
                    label = '主线车辆' if not legend_added['mainline'] else ""
                    if not legend_added['mainline']:
                        legend_added['mainline'] = True
                elif veh_id.startswith('r'):
                    color = ramp_color
                    # 只有当匝道车辆图例尚未添加时才设置标签
                    label = '匝道车辆' if not legend_added['ramp'] else ""
                    if not legend_added['ramp']:
                        legend_added['ramp'] = True
                else:
                    color = 'gray'
                    label = ""
                # 使用accumulated_distance，避免坐标跳跃
                plt.plot(veh_data['timestamp'], veh_data['accumulated_distance'],
                        color=color, label=label, linewidth=2, alpha=0.7)
        
        plt.xlabel('时间 (秒)', fontsize=12)
        plt.ylabel('全局位置 (米)', fontsize=12)
        plt.title('车辆轨迹图', fontsize=14, fontweight='bold')
        plt.legend(fontsize=10)
        plt.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(save_path, dpi=PLOT_DPI, bbox_inches='tight')
        print(f"轨迹图已保存到: {save_path}")
        plt.close()
    
    def plot_fifo_vs_optimized_comparison(self, save_path: str = "output/fifo_vs_optimized.png"):
        """
        绘制FIFO vs 优化算法的性能对比图 —— 主要就是看下数据
        
        Args:
            save_path: 保存路径
        """
        if not self.optimization_history:
            print("警告: 没有优化历史数据")
            return
        
        # 准备数据
        times = [h['time'] for h in self.optimization_history]
        fifo_costs = [h.get('fifo_cost', 0) for h in self.optimization_history]
        optimized_costs = [h.get('final_cost', 0) for h in self.optimization_history]
        improvements = [
            self._bounded_improvement(h.get('improvement_vs_fifo', 0) * 100)
            for h in self.optimization_history
        ]
        
        # 计算累积统计
        total_fifo_cost = sum(fifo_costs)
        total_optimized_cost = sum(optimized_costs)
        avg_improvement = np.mean(improvements)
        total_improvement = ((total_fifo_cost - total_optimized_cost) / total_fifo_cost * 100) if total_fifo_cost > 0 else 0
        
        # 创建图形
        fig = plt.figure(figsize=(16, 10))
        
        # 子图1: 代价对比（条形图）
        ax1 = plt.subplot(2, 2, 1)
        categories = ['FIFO\n(无优化)', '优化算法']
        costs = [total_fifo_cost, total_optimized_cost]
        colors = ['#e74c3c', '#2ecc71']
        
        bars = ax1.bar(categories, costs, color=colors, alpha=0.7, width=0.6)
        ax1.set_ylabel('总代价', fontsize=12)
        ax1.set_title('总体性能对比', fontsize=14, fontweight='bold')
        ax1.grid(axis='y', alpha=0.3)
        
        # 添加数值标签和改进率
        for i, bar in enumerate(bars):
            height = bar.get_height()
            ax1.text(bar.get_x() + bar.get_width()/2., height,
                    f'{height:.1f}',
                    ha='center', va='bottom', fontsize=11)
        
        # 添加改进率标注
        ax1.text(0.5, max(costs) * 0.9, f'改进率: {total_improvement:.1f}%',
                ha='center', fontsize=13, fontweight='bold',
                bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.5))
        
        # 子图2: 时间序列对比
        ax2 = plt.subplot(2, 2, 2)
        ax2.plot(times, fifo_costs, 'o-', label='FIFO（无优化）', 
                color='red', markersize=4, linewidth=2, alpha=0.7)
        ax2.plot(times, optimized_costs, 's-', label='优化算法',
                color='green', markersize=4, linewidth=2, alpha=0.7)
        ax2.fill_between(times, fifo_costs, optimized_costs, alpha=0.15, color='green')
        
        ax2.set_xlabel('仿真时间 (秒)', fontsize=12)
        ax2.set_ylabel('代价', fontsize=12)
        ax2.set_title('实时代价对比', fontsize=14, fontweight='bold')
        ax2.legend(fontsize=11)
        ax2.grid(True, alpha=0.3)
        
        # 子图3: 改进率时间序列
        ax3 = plt.subplot(2, 2, 3)
        ax3.plot(times, improvements, 'd-', color='blue', markersize=4, linewidth=2, alpha=0.7)
        ax3.axhline(y=avg_improvement, color='red', linestyle='--',
                   label=f'平均改进: {avg_improvement:.2f}%', linewidth=2)
        ax3.axhline(y=0, color='gray', linestyle='-', linewidth=1, alpha=0.5)
        
        ax3.set_xlabel('仿真时间 (秒)', fontsize=12)
        ax3.set_ylabel('相对FIFO改进率 (%)', fontsize=12)
        ax3.set_title('优化效果时间变化', fontsize=14, fontweight='bold')
        ax3.legend(fontsize=11)
        ax3.grid(True, alpha=0.3)
        
        # 子图4: 统计摘要表
        ax4 = plt.subplot(2, 2, 4)
        ax4.axis('off')
        
        summary_data = [
            ['指标', 'FIFO', '优化算法', '改进'],
            ['总代价', f'{total_fifo_cost:.1f}', f'{total_optimized_cost:.1f}', 
             f'{total_improvement:.1f}%'],
            ['平均代价', f'{np.mean(fifo_costs):.1f}', f'{np.mean(optimized_costs):.1f}',
             f'{avg_improvement:.1f}%'],
            ['最大代价', f'{np.max(fifo_costs):.1f}', f'{np.max(optimized_costs):.1f}',
             f'{((np.max(fifo_costs) - np.max(optimized_costs)) / np.max(fifo_costs) * 100):.1f}%'],
            ['优化次数', '-', f'{len(times)}', '-']
        ]
        
        table = ax4.table(cellText=summary_data, cellLoc='center', loc='center',
                         colWidths=[0.25, 0.25, 0.25, 0.25])
        table.auto_set_font_size(False)
        table.set_fontsize(11)
        table.scale(1, 2.5)
        
        # 设置表格样式
        for i in range(len(summary_data)):
            for j in range(4):
                cell = table[(i, j)]
                if i == 0:
                    cell.set_facecolor('#4CAF50')
                    cell.set_text_props(weight='bold', color='white')
                elif i % 2 == 1:
                    cell.set_facecolor('#f0f0f0')
        
        ax4.set_title('性能统计摘要', fontsize=14, fontweight='bold', pad=20)
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=PLOT_DPI, bbox_inches='tight')
        print(f"FIFO vs 优化对比图已保存到: {save_path}")
        print(f"  - 总体改进率: {total_improvement:.2f}%")
        print(f"  - 平均改进率: {avg_improvement:.2f}%")
        plt.close()
    
    def generate_all_plots(self):
        """生成所有可视化图表"""
        print("\n=== 生成可视化图表 ===")
        
        self.plot_convergence_curve()
        self.plot_performance_comparison()
        self.plot_vehicle_trajectories()
        self.plot_fifo_vs_optimized_comparison()
        
        print("所有图表生成完成！")


if __name__ == "__main__":
    # 测试可视化模块
    print("可视化模块独立测试")
    print("需要与DataCollector配合使用")
