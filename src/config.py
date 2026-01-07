"""
配置参数文件
包含所有仿真和算法参数
"""

# ==================== SUMO配置 ====================
SUMO_CONFIG_FILE = "sumo_files/highway.sumocfg"
SUMO_GUI = True  # True使用GUI，False使用命令行版本
SIMULATION_STEP = 0.1  # 仿真步长（秒）
SIMULATION_TIME = 500  # 总仿真时间（秒）
WAITING_SPEED_THRESHOLD = 0.1  # 等待判定速度阈值(m/s)
TRIPINFO_OUTPUT_FILE = "output/tripinfo.xml"  # tripinfo输出

# ==================== 随机种子 ====================
RANDOM_SEED = 42  # 随机种子，改变此值会得到不同的车辆到达模式
# 常用种子值: 42, 123, 456, 789, 2024等

# ==================== 道路参数 ====================
# 主线道路
MAINLINE_LENGTH = 1500  # 主线长度（米）
MAINLINE_LANES = 2  # 主线车道数（车道1快车道+车道2慢车道）
MAINLINE_SPEED_LIMIT = 33.3  # 主线限速（m/s，约120km/h）
MAINLINE_LANE1_SPEED = 33.3  # 车道1（快车道）限速（m/s，约120km/h）
FAST_MAINLINE_LANE_SPEED = MAINLINE_LANE1_SPEED  # 车道限速（m/s）
MAINLINE_LANE2_SPEED = 27.78  # 车道2（慢车道）限速（m/s，约100km/h）

# 匝道
RAMP_LENGTH = 250  # 匝道长度（米）
RAMP_ACCELERATION_LANE = 180  # 加速车道长度（米）
RAMP_SPEED_LIMIT = 22.2  # 匝道限速（m/s，约80km/h）

# 合流区域
MERGE_ZONE_START = 150  # 合流区域起始位置（米）- 提前到150米，确保主线车进入检测区
MERGE_ZONE_LENGTH = 1000  # 合流区域长度（米）- 扩大到1000米，检测范围150-1150米
MERGE_POINT = 1000  # 合流点位置（米）

# ==================== 车辆参数 ====================
# 货车车辆物理参数
VEHICLE_LENGTH = 9.6  # 车辆长度（米）
MAX_ACCELERATION = 2.5  # 最大加速度（m/s²）
MAX_DECELERATION = 3.5  # 最大减速度（m/s²）
EMERGENCY_DECEL = 5.0  # 紧急制动减速度（m/s²）

# 小汽车车辆物理参数
FAST_MAINLINE_VEHICLE_LENGTH = 4.8  # 车辆长度（米）
FAST_FUEL_VEHICLE_MASS = 1680.0  # 车辆质量（kg）
FAST_MAINLINE_MAX_ACCELERATION = 3.5  # 最大加速度（m/s²）
FAST_MAINLINE_MAX_DECELERATION = 4.5  # 最大减速度（m/s²）
FAST_MAINLINE_EMERGENCY_DECEL = 7.0  # 紧急制动减速度（m/s²）
FAST_MAINLINE_COLOR = "1, 0, 0"  # 车辆颜色（RGB字符串）

# 主线车队参数
PLATOON_SIZE_MIN = 2  # 车队最小规模
PLATOON_SIZE_MAX = 5  # 车队最大规模
PLATOON_SIZE_WEIGHTS = {
    2: 0.35,  # 2辆车车队的权重
    3: 0.30,  # 3辆车车队的权重
    4: 0.20,  # 4辆车车队的权重
    5: 0.15   # 5辆车车队的权重
}
PLATOON_DESIRED_SPEED = 30.0  # 车队期望速度（m/s，约108km/h）
PLATOON_SPEED_DEVIATION = 2.0  # 速度偏差（m/s）

# 匝道车辆参数
RAMP_DESIRED_SPEED = 30.0  # 匝道车辆期望速度（m/s）
RAMP_SPEED_DEVIATION = 3.0  # 速度偏差（m/s）
RAMP_SMALL_CAR_RATIO = 0.3  # 匝道车辆中小汽车占比
RAMP_SMALL_CAR_COLOR = (0, 200, 80, 255)  # 匝道小汽车颜色（RGBA）

# 快车道主线车辆参数（小汽车）
FAST_MAINLINE_DESIRED_SPEED = 33.0  # 快车道期望速度（m/s，约118km/h）
FAST_MAINLINE_SPEED_DEVIATION = 1.5  # 速度偏差（m/s）
# ==================== 安全距离参数 ====================
# 车队间安全距离
SAFE_TIME_HEADWAY_BETWEEN = 2.0  # 车队间安全时间间隔（秒）
MIN_GAP_BETWEEN = 5.0  # 车队间最小间距（米）
SAFE_DECEL = 4.0  # 安全减速度（m/s²）

# 车队内安全距离
SAFE_TIME_HEADWAY_WITHIN = 0.9  # 车队内安全时间间隔（秒）
MIN_GAP_WITHIN = 3.0  # 车队内最小间距（米）

# 小汽车安全距离
FAST_MAINLINE_MIN_GAP = 5.0  # 最小车距（米）

# ==================== 泊松分布参数 ====================
# 车队车辆生成
MAINLINE_ARRIVAL_RATE = 0.15  # 主线车队到达率（车队/秒）- 提高到0.15以增加检测区域内车辆密度
MAINLINE_GENERATION_START = 0  # 开始生成时间（秒）
MAINLINE_GENERATION_END = 400  # 停止生成时间（秒）

# 匝道车辆生成
RAMP_ARRIVAL_RATE = 0.12  # 匝道车辆到达率（车辆/秒）- 提高到0.12以增加优化场景
RAMP_GENERATION_START = 0  # 开始生成时间（秒）
RAMP_GENERATION_END = 400  # 停止生成时间（秒）

# 快车车辆生成
FAST_MAINLINE_ARRIVAL_RATE = 0.3  # 到达率（车辆/秒）
FAST_MAINLINE_GENERATION_START = 0  # 开始生成时间（秒）
FAST_MAINLINE_GENERATION_END = 400  # 停止生成时间（秒）
# ==================== IDM跟驰模型参数 ====================
IDM_DELTA = 4.0  # IDM加速度指数
IDM_A = MAX_ACCELERATION  # 最大加速度
IDM_B = MAX_DECELERATION  # 舒适减速度

FAST_IDM_A = FAST_MAINLINE_MAX_ACCELERATION  # IDM最大加速度
FAST_IDM_B = FAST_MAINLINE_MAX_DECELERATION  # IDM舒适减速度
FAST_TAU = 0.9  # IDM期望时间间隔（秒）
# ==================== 仿真模式开关 ====================
USE_OPTIMIZATION = True  # True使用优化算法，False使用FIFO模式
# 车队重组开关（USE_OPTIMIZATION = True时才会生效）
ENABLE_PLATOON_REORG = True
# 车队重组奖励（目标函数中的奖励权重，鼓励采用重组方案；默认0表示不影响原有逻辑）
PLATOON_REORG_BONUS = 5.0

# ==================== 燃油消耗模型参数 ====================
# 参考文献的油耗模型设置的参数
FUEL_ALPHA = 0.666  # 怠速燃油消耗率(mL/s)
FUEL_BETA1 = 0.072  # 效率因子(mL/kJ)
FUEL_BETA2 = 0.034  # 效率因子(mL/(kJ*m/s^2))
FUEL_VEHICLE_MASS = 18000.0  # 货车车辆质量，后期根据货车和普通车区分、重设一下参数
FUEL_DELTA1 = 0.269    # 滚动阻力（kN）
FUEL_DELTA2 = 0.017  # 发动机阻力(kN/(m/s))
FUEL_DELTA3 = 0.000672  # 空气动力阻力（kN/(m/s)^2)
FUEL_PHI = 0.68  # 车队中跟随车的空气动力阻力折减系数

# ==================== 优化算法参数 ====================
# 目标函数权重
WEIGHT_MAINLINE_TIME = 0.6  # 主线通行时间权重
WEIGHT_RAMP_FAILED = 0.4  # 匝道失败权重

# 优化算法设置（滚动时距优化结构）
OPTIMIZATION_HORIZON = 30.0  # 优化时域（秒）- 预测未来30秒内的车辆行为
OPTIMIZATION_INTERVAL = 1.0  # 优化间隔（秒）- 每1秒重新优化一次（滚动优化）
MAX_ITERATIONS = 50  # 最大迭代次数 - 局部搜索的最大迭代次数
CONVERGENCE_THRESHOLD = 0.01  # 收敛阈值 - 当改进小于1%时停止迭代

# 注意：当前实现已经是滚动时距优化结构
# - 每OPTIMIZATION_INTERVAL秒执行一次优化
# - 每次优化都会重新获取合流区域内的车辆
# - 优化结果实时应用于车辆控制
# - 下一次优化会考虑新进入检测区域的车辆

# ==================== 数据收集参数 ====================
DATA_COLLECTION_INTERVAL = 0.5  # 数据采集间隔（秒）
OUTPUT_EXCEL_FILE = "output/simulation_data.xlsx"  # Excel输出文件
OUTPUT_CONVERGENCE_PLOT = "output/convergence_curve.png"  # 收敛曲线
OUTPUT_PERFORMANCE_PLOT = "output/performance_comparison.png"  # 性能对比图

# ==================== 可视化参数 ====================
PLOT_DPI = 300  # 图像分辨率
PLOT_FIGSIZE = (10, 6)  # 图像大小
PLOT_STYLE = 'seaborn-v0_8-darkgrid'  # 绘图风格
