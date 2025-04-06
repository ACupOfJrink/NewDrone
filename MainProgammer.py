import numpy as np
import matplotlib.pyplot as plt
from angle_utils import compute_angle3,compute_weiyi3,Follower_PI3,Leader_P,potential,potential_obs
import math
plt.rcParams['font.sans-serif'] = ['SimHei']
# 假设以下函数已经定义并可以导入使用
# from your_module import compute_angle3, compute_weiyi3, potential, potential_obs, Leader_P, Follower_PI3

# 初始化位置坐标矩阵
T = np.array([
    [200, 0, 500],      # Leader
    [-300, -200, 300],  # 1
    [200, -500, 200],   # 2
    [0, -600, 100],     # 3
    [0, -800, 100],     # 4
    [0, -450, 80],      # 5
    [100, 250, 0],      # 6
    [50, 200, 0],       # 7
    [20, 100, 0],       # 8
    [40, 50, 0],        # 9
    [160, 0, 0],        # 10
    [80, -130, 0],      # 11
    [100, -60, 0],      # 12
    [200, -500, 0],     # 13
    [100, -400, 0],     # 14
    [0, -300, 0],       # 15
    [-100, -200, 0],    # 16
    [-200, -100, 0],    # 17
    [-300, 0, 0],       # 18
    [-300, 300, 0],     # 19
    [-400, 400, 0]      # 20
])

# 创建3D图形
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

# 画出初始位置
colors = ['r', 'b', 'g', 'y'] * 6  # 颜色循环
for i in range(21):
    ax.scatter(T[i, 0], T[i, 1], T[i, 2], c=colors[i], s=100, marker='o')

# 参数设置
k = 1       # 引力增益
l = 300     # 队形距离
k_pot = 130 # 人工势场斥力的系数
V_L = 100   # leader步长
V_M = 200   # follower最大步长
V_m = 40    # follower最小步长
J = 200     # 步数
t = np.arange(0, J+1)
Rm = 20     # 跟随无人机最大航向角向心加速度
Qm = 0.1    # 最大俯仰角速度

# PI系数
kp = 0.5
ki = 0.1
krp = 0.9
kri = 0.4
kqp = 0.9
kqi = 0.4

# 队形矩阵
F = np.array([
    [0, 0, 0],
    [-l/2, -l, 0],
    [-l/2, l, 0],
    [-l, -2*l, 0],
    [-l, 0, 0],
    [-l, 2*l, 0],
    [-3*l/2, -3*l, 0],
    [-3*l/2, -l, 0],
    [-3*l/2, l, 0],
    [-3*l/2, 3*l, 0],
    [-2*l, -4*l, 0],
    [-2*l, -2*l, 0],
    [-2*l, 0, 0],
    [-2*l, 2*l, 0],
    [-2*l, 4*l, 0],
    [-5*l/2, -5*l, 0],
    [-5*l/2, -3*l, 0],
    [-5*l/2, -l, 0],
    [-5*l/2, l, 0],
    [-5*l/2, 3*l, 0],
    [-5*l/2, 5*l, 0]
])  # 编队矩阵，与长机之间的相对位移; 金字塔编队

# 初始化各种数组
U = np.zeros((21, 10*len(t)))    # 速度大小 (m/s)
R = np.zeros((21, 10*len(t)))    # 航向角 (rad)
Position_angle = np.zeros((21, 10*len(t)))
Rotat = np.zeros((21, 10*len(t)))
Rotatc = np.zeros((21, 10*len(t)))  # 航向角速度 (rad/s)
Q = np.zeros((21, 10*len(t)))    # 俯仰角 (rad)
fuyang_angle = np.zeros((21, 10*len(t)))
pitch = np.zeros((21, 10*len(t)))
pitchc = np.zeros((21, 10*len(t)))  # 俯仰角速度 (rad/s)
distancex1 = np.zeros((21, 10*len(t)))  # 僚机与长机间距离
distance = np.zeros((21, 10*len(t)))

# 初始化Leader
R[0, 0] = 0      # 初始朝向角
U[0, 0] = V_L

# 初始化followers
R[1, 0] = math.pi/2  # 初始朝向角
R[2, 0] = math.pi/3
Q[2, 0] = 0.13       # 初始俯仰角
R[3, 0] = math.pi/3
R[4, 0] = -math.pi/3
R[5, 0] = -math.pi/2

min_temp = 655360
min_distance = []

# 初始化跟随者速度和距离
for i_init in range(1, 21):
    U[i_init, 0] = V_L
    dx = T[i_init, 0] - T[0, 0]
    dy = T[i_init, 1] - T[0, 1]
    dz = T[i_init, 2] - T[0, 2]
    distancex1[i_init, 0] = math.sqrt(dx**2 + dy**2 + dz**2)

# 计算机间最小距离
for i_distance in range(20):
    for j_distance in range(i_distance+1, 21):
        distanceij = np.linalg.norm(T[i_distance] - T[j_distance])
        if distanceij < min_temp:
            min_temp = distanceij
min_distance.append(min_temp)

# 长机目标位置和障碍物
Xsum1 = np.array([200, 30000, 500])  # 长机目标位置
obs = np.array([
    [1000, 15000, 500],
    [-1000, 6000, 800]
])  # 圆柱状障碍物，[x坐标, y坐标, 半径]

# 画出Leader期望位置
ax.scatter(Xsum1[0], Xsum1[1], Xsum1[2], c='k', marker='x', s=100)

# 主循环
for j in range(1, J):
    # 领航无人机完成动作
    beta1, alpha1 = compute_angle3(T[0], Xsum1)
    Fatx1, Faty1, Fatz1, Fatr1 = compute_weiyi3(T[0], Xsum1, k, beta1, alpha1)
    
    Fsumy1 = Faty1  # y位移量
    Fsumx1 = Fatx1  # x位移量
    Fsumz1 = Fatz1  # z位移量
    Fsumr1 = Fatr1
    
    Position_angle[0, j] = math.atan2(Fsumy1, Fsumx1)  # 当前偏航角
    fuyang_angle[0, j] = math.atan2(Fsumz1, Fsumr1)    # 当前俯仰角
    
    T1 = T[0].copy()
    
    # 计算当前距离期望位置距离
    dx = T[0, 0] - Xsum1[0]
    dy = T[0, 1] - Xsum1[1]
    dz = T[0, 2] - Xsum1[2]
    distance[0, j] = math.sqrt(dx**2 + dy**2 + dz**2)
    
    if distance[0, j] < 0.5:
        Xnext1 = T1.copy()  # 到达
    else:
        Xnext1 = np.array([
            T1[0] + V_L * math.cos(fuyang_angle[0, j]) * math.cos(Position_angle[0, j]),
            T1[1] + V_L * math.cos(fuyang_angle[0, j]) * math.sin(Position_angle[0, j]),
            T1[2] + V_L * math.sin(fuyang_angle[0, j])
        ])
    
    T[0] = Xnext1
    
    # 队形根据长机运动方向进行旋转
    rot_yaw = np.array([
        [math.cos(Position_angle[0, j]), math.sin(Position_angle[0, j]), 0.0],
        [-math.sin(Position_angle[0, j]), math.cos(Position_angle[0, j]), 0.0],
        [0.0, 0.0, 1.0]
    ])
    
    rot_pitch = np.array([
        [math.cos(fuyang_angle[0, j]), 0.0, math.sin(fuyang_angle[0, j])],
        [0.0, 1.0, 0.0],
        [-math.sin(fuyang_angle[0, j]), 0.0, math.cos(fuyang_angle[0, j])]
    ])
    
    Transmatrix = np.dot(rot_yaw, rot_pitch)
    
    # 跟随无人机移动
    for i_follower in range(1, 21):
        Xsum_follower = T[0] + np.dot(F[i_follower], Transmatrix)  # 根据Leader确定期望位置
        
        pot_i = np.array([0.0, 0.0, 0.0])  # 人工势场斥力
        for j_follower in range(4):
            pot_i += k_pot * potential(i_follower, j_follower, T)  # 机间人工势场斥力
        
        pot_i += k_pot * potential_obs(i_follower, obs, T, U, R[i_follower, j-1], Q[i_follower, j-1])  # 障碍物斥力
        
        Xsum_follower += pot_i  # 人工势场斥力转化为期望位置改变（调整）量        
        beta, alpha = compute_angle3(T[i_follower], Xsum_follower)
        Fatx, Faty, Fatz, Fatr = compute_weiyi3(T[i_follower], Xsum_follower, k, beta, alpha)
        
        Fsumy = Faty  # y位移量
        Fsumx = Fatx  # x位移量
        Fsumz = Fatz  # z位移量
        Fsumr = Fatr
        
        Position_angle[i_follower, j] = beta  # 当前期望偏航角
        fuyang_angle[i_follower, j] = math.atan2(Fsumz, Fsumr)  # 当前期望俯仰角
        
        T_i = T[i_follower].copy()
        
        # 与长机间的距离
        distancex1[i_follower, j] = np.linalg.norm(T[i_follower] - T[0])
        # 与期望位置间的距离
        distance[i_follower, j] = np.linalg.norm(T_i - Xsum_follower)
        
        #跟随机PI控制器
        (U[i_follower, j], R[i_follower,:], Q[i_follower,:], 
         Rotatc[i_follower, j], pitchc[i_follower, j]) = Follower_PI3(
            distance[i_follower, :], V_M, V_m, Position_angle[i_follower, j], 
            fuyang_angle[i_follower, j], R[i_follower, :], Q[i_follower, :], 
            Rotat[i_follower, :], pitch[i_follower, :], Rm, Qm, j, kp, ki, 
            krp, kri, kqp, kqi)
        
        # 位置更新
        Xnext = np.array([
            T_i[0] + U[i_follower, j] * math.cos(Q[i_follower, j-1] + pitchc[i_follower, j]/2.0) * 
                math.cos(R[i_follower, j-1] + Rotatc[i_follower, j]/2.0),
            T_i[1] + U[i_follower, j] * math.cos(Q[i_follower, j-1] + pitchc[i_follower, j]/2.0) * 
                math.sin(R[i_follower, j-1] + Rotatc[i_follower, j]/2.0),
            T_i[2] + U[i_follower, j] * math.sin(Q[i_follower, j-1] + pitchc[i_follower, j]/2.0)
        ])
        T[i_follower] = Xnext
    
    # 求机间距离最小值
    min_temp = 655360
    for i_distance in range(20):
        for j_distance in range(i_distance+1, 21):
            distanceij = np.linalg.norm(T[i_distance] - T[j_distance])
            if distanceij < min_temp:
                min_temp = distanceij
    min_distance.append(min_temp)
    
    # 更新图形
    ax.clear()
    for i in range(21):
        ax.scatter(T[i, 0], T[i, 1], T[i, 2], c=colors[i], s=100, marker='o')
    
    ax.scatter(Xsum1[0], Xsum1[1], Xsum1[2], c='k', marker='x', s=100)
    ax.set_xlabel('x[m]')
    ax.set_ylabel('y[m]')
    ax.set_zlabel('z[m]')
    plt.title('无人机编队飞行模拟 - 阶段1')
    plt.pause(0.01)  # 短暂暂停以更新图形

# 编队重构
F = np.array([
    [0, 0, 0],
    [-l/2, -l, 0.04*l],
    [-l/2, l, 0.04*l],
    [-l/2, -l, -0.04*l],
    [-l/2, l, -0.04*l],
    [-l, -2*l, -2*0.04*l],
    [-l, 0, -2*0.04*l],
    [-l, 2*l, -2*0.04*l],
    [-l, -2*l, 0],
    [-l, 0, 2*0.04*l],
    [-l, 2*l, 0],
    [-l, -2*l, 2*0.04*l],
    [-l, 0, 0],
    [-l, 2*l, 2*0.04*l],
    [-3*l/2, -l, 0],
    [-3*l/2, l, 0],
    [-3*l/2, 3*l, 0],
    [-3*l/2, -3*l, 0],
    [-2*l, -2*l, 0],
    [-2*l, 0, 0],
    [-2*l, 2*l, 0]
])

# 更新初始条件
R[:, 0] = R[:, J-1]
Q[:, 0] = Q[:, J-1]
R[0, 0] = Position_angle[0, J-1]

# 第二阶段
plt.title('无人机编队飞行模拟 - 阶段2')
J2 = 2  # 650
for j in range(1, J2):
    # 领航无人机完成动作
    distance_j = 0
    
    # 设置领航无人机的操控输入
    if j < 300 and j >= 101:
        u_ctrl1 = np.array([0, math.pi/200, 0])  # 水平转弯
    elif j < 420 and j >= 400:
        u_ctrl1 = np.array([0.001, 0, 0.01])  # 爬升（起爬）
    elif j >= 480 and j < 500:
        u_ctrl1 = np.array([-0.001, 0, -0.01])  # 爬升（结束）
    else:
        u_ctrl1 = np.array([0, 0, 0])
    
    # 长机指令控制系统
    Vl_next, phi1_next, theta1_next, T1_next = Leader_P(u_ctrl1, T[0], U[0, 0], R[0, j-1], Q[0, j-1])
    
    # 参数更新
    R[0, j] = phi1_next
    Q[0, j] = theta1_next
    U[0, j] = Vl_next
    T[0] = T1_next
    
    # 队形根据长机运动方向进行旋转
    rot_yaw = np.array([
        [math.cos(R[0, j]), math.sin(R[0, j]), 0],
        [-math.sin(R[0, j]), math.cos(R[0, j]), 0],
        [0, 0, 1]
    ])
    
    rot_pitch = np.array([
        [math.cos(Q[0, j]), 0, math.sin(Q[0, j])],
        [0, 1, 0],
        [-math.sin(Q[0, j]), 0, math.cos(Q[0, j])]
    ])
    
    Transmatrix = np.dot(rot_yaw, rot_pitch)
    
    # 跟随无人机移动
    for i_follower in range(1, 21):
        Xsum_follower = T[0] + np.dot(F[i_follower], Transmatrix)  # 根据Leader确定期望位置
        
        pot_i = np.array([0.0, 0.0, 0.0])
        for j_follower in range(4):
            pot_i += np.array([k_pot,k_pot,k_pot]) * potential(i_follower, j_follower, T)  # 机间人工势场
        
        pot_i += k_pot * potential_obs(i_follower, obs, T, U[:, j-1], R[i_follower, j-1], Q[i_follower, j-1])  # 障碍物斥力
        
        Xsum_follower += pot_i  # 人工势场力转化为期望位置调整量
        
        beta, alpha = compute_angle3(T[i_follower], Xsum_follower)
        Fatx, Faty, Fatz, Fatr = compute_weiyi3(T[i_follower], Xsum_follower, k, beta, alpha)
        
        Fsumy = Faty  # y位移量
        Fsumx = Fatx  # x位移量
        Fsumz = Fatz  # z位移量
        Fsumr = Fatr
        
        Position_angle[i_follower, j] = beta  # 当前期望偏航角
        fuyang_angle[i_follower, j] = math.atan2(Fsumz, Fsumr)  # 当前期望俯仰角
        
        T_i = T[i_follower].copy()
        distancex1[i_follower, j] = np.linalg.norm(T[i_follower] - T[0])
        distance[i_follower, j] = np.linalg.norm(T_i - Xsum_follower)
        
        # 跟随机PI控制器
        (U[i_follower, j], R[i_follower,:], Q[i_follower,:], 
         Rotatc[i_follower, j], pitchc[i_follower, j]) = Follower_PI3(
            distance[i_follower, :], V_M, V_m, Position_angle[i_follower, j], 
            fuyang_angle[i_follower, j], R[i_follower, :], Q[i_follower, :], 
            Rotat[i_follower, :], pitch[i_follower, :], Rm, Qm, j, kp, ki, 
            krp, kri, kqp, kqi)
        
        # 位置更新
        Xnext = np.array([
            T_i[0] + U[i_follower, j] * math.cos(Q[i_follower, j-1] + pitchc[i_follower, j]/2) * 
                math.cos(R[i_follower, j-1] + Rotatc[i_follower, j]/2),
            T_i[1] + U[i_follower, j] * math.cos(Q[i_follower, j-1] + pitchc[i_follower, j]/2) * 
                math.sin(R[i_follower, j-1] + Rotatc[i_follower, j]/2),
            T_i[2] + U[i_follower, j] * math.sin(Q[i_follower, j-1] + pitchc[i_follower, j]/2)
        ])
        T[i_follower] = Xnext
    
    # 求机间最小距离
    min_temp = 655360
    for i_distance in range(20):
        for j_distance in range(i_distance+1, 21):
            distanceij = np.linalg.norm(T[i_distance] - T[j_distance])
            if distanceij < min_temp:
                min_temp = distanceij
    min_distance.append(min_temp)
    
    # 更新图形
    ax.clear()
    for i in range(21):
        ax.scatter(T[i, 0], T[i, 1], T[i, 2], c=colors[i], s=100, marker='o')
    
    ax.set_xlabel('x[m]')
    ax.set_ylabel('y[m]')
    ax.set_zlabel('z[m]')
    plt.title('无人机编队飞行模拟 - 阶段2')
    plt.pause(0.01)  # 短暂暂停以更新图形

# 绘制最终位置
for i in range(21):
    ax.scatter(T[i, 0], T[i, 1], T[i, 2], c=colors[i], s=100, marker='o')

ax.set_xlabel('x[m]')
ax.set_ylabel('y[m]')
ax.set_zlabel('z[m]')
plt.title('无人机编队飞行最终位置')
plt.grid(True)
plt.show()