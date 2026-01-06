"""
主程序入口
运行高速公路匝道合流优化仿真
"""

import os
import sys
import argparse
from src.simulation_controller import SimulationController
from src.visualization import Visualizer
from src.config import *


def check_environment():
    """检查运行环境"""
    print("=== 环境检查 ===")
    
    # 检查SUMO
    if 'SUMO_HOME' not in os.environ:
        print("错误: 未找到SUMO_HOME环境变量")
        print("请安装SUMO并设置SUMO_HOME环境变量")
        print("下载地址: https://sumo.dlr.de/docs/Downloads.php")
        return False
    else:
        sumo_home = os.environ['SUMO_HOME']
        print(f"✓ SUMO_HOME: {sumo_home}")
    
    # 检查SUMO配置文件
    if not os.path.exists(SUMO_CONFIG_FILE):
        print(f"错误: 未找到SUMO配置文件: {SUMO_CONFIG_FILE}")
        return False
    else:
        print(f"✓ SUMO配置文件: {SUMO_CONFIG_FILE}")
    
    # 检查输出目录
    if not os.path.exists('output'):
        os.makedirs('output')
        print("✓ 创建输出目录: output/")
    else:
        print("✓ 输出目录存在: output/")
    
    print()
    return True


def print_simulation_info():
    """打印仿真信息"""
    print("=" * 60)
    print("高速公路匝道合流优化仿真系统")
    print("=" * 60)
    print()
    print("【仿真参数】")
    print(f"  仿真时长: {SIMULATION_TIME} 秒")
    print(f"  仿真步长: {SIMULATION_STEP} 秒")
    print(f"  主线长度: {MAINLINE_LENGTH} 米")
    print(f"  匝道长度: {RAMP_LENGTH} 米")
    print(f"  合流区域: {MERGE_ZONE_START}-{MERGE_ZONE_START+MERGE_ZONE_LENGTH} 米")
    print()
    print("【车辆生成】")
    print(f"  主线车队到达率: {MAINLINE_ARRIVAL_RATE} 车队/秒")
    print(f"  车队规模: {PLATOON_SIZE_MIN}-{PLATOON_SIZE_MAX} 辆")
    print(f"  匝道车辆到达率: {RAMP_ARRIVAL_RATE} 车辆/秒")
    print()
    print("【优化算法】")
    print(f"  主线时间权重: {WEIGHT_MAINLINE_TIME}")
    print(f"  匝道失败权重: {WEIGHT_RAMP_FAILED}")
    print(f"  优化间隔: {OPTIMIZATION_INTERVAL} 秒")
    print()
    print("【输出文件】")
    print(f"  Excel数据: {OUTPUT_EXCEL_FILE}")
    print(f"  收敛曲线: {OUTPUT_CONVERGENCE_PLOT}")
    print(f"  性能对比: {OUTPUT_PERFORMANCE_PLOT}")
    print("=" * 60)
    print()


def run_simulation(use_gui=True):
    """
    运行仿真
    
    Args:
        use_gui: 是否使用SUMO GUI
    """
    # 环境检查
    if not check_environment():
        sys.exit(1)
    
    # 打印仿真信息
    print_simulation_info()
    
    # 打印随机种子信息
    print(f"[随机种子] 使用种子值: {RANDOM_SEED}")
    print(f"[提示] 修改config.py中的RANDOM_SEED可以得到不同的车辆到达模式")
    print()
    
    # 创建仿真控制器
    print("初始化仿真控制器...")
    controller = SimulationController(use_gui=use_gui, random_seed=RANDOM_SEED)
    
    try:
        # 初始化SUMO
        controller.initialize_sumo()
        
        # 运行仿真
        controller.run_simulation()
        
        # 结束仿真并保存结果
        data_collector, merge_optimizer = controller.finalize()
        
        # 生成可视化图表
        print("\n=== 生成可视化图表 ===")
        visualizer = Visualizer(
            data_collector=data_collector,
            optimization_history=merge_optimizer.get_optimization_history()
        )
        visualizer.generate_all_plots()
        
        print("\n" + "=" * 60)
        print("仿真完成！所有结果已保存到 output/ 目录")
        print("=" * 60)
        
    except KeyboardInterrupt:
        print("\n\n仿真被用户中断")
        try:
            controller.finalize()
        except:
            pass
        
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
        try:
            controller.finalize()
        except:
            pass


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='高速公路匝道合流优化仿真系统'
    )
    
    parser.add_argument(
        '--no-gui',
        action='store_true',
        help='不使用SUMO GUI，使用命令行模式运行'
    )
    
    parser.add_argument(
        '--check',
        action='store_true',
        help='仅检查环境配置，不运行仿真'
    )
    
    args = parser.parse_args()
    
    if args.check:
        # 仅检查环境
        if check_environment():
            print("环境检查通过！")
        else:
            print("环境检查失败！")
        return
    
    # 运行仿真
    use_gui = not args.no_gui
    run_simulation(use_gui=use_gui)


if __name__ == "__main__":
    main() 
