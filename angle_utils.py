import numpy as np

def compute_angle3(X, Xsum1):
    r = np.sqrt((Xsum1[0] - X[0])**2 + (Xsum1[1] - X[1])**2)  # 与目标位置的水平距离
    beta = np.sign(Xsum1[1] - X[1]) * np.arccos((Xsum1[0] - X[0]) / r)  # 偏航角
    l = np.sqrt((Xsum1[0] - X[0])**2 + (Xsum1[1] - X[1])**2 + (Xsum1[2] - X[2])**2)  # 直线距离
    alpha = np.sign(Xsum1[2] - X[2]) * np.arccos(r / l)  # 俯仰角
    return beta, alpha

def compute_weiyi3(X, Xsum, k, beta, alpha):
    """
    计算位移量 Yatx, Yaty, Yatz, Yatr

    参数:
    X:     (list or array)  当前点的坐标 [x, y, z]
    Xsum:  (list or array)  目标点的坐标 [x, y, z]
    k:     (float)          比例因子
    beta:  (float)          偏航角 (弧度)
    alpha: (float)          俯仰角 (弧度)

    返回:
    Yatx, Yaty, Yatz, Yatr (四个位移量)
    """
    # 计算两点的平方距离
    R = (X[0] - Xsum[0])**2 + (X[1] - Xsum[1])**2 + (X[2] - Xsum[2])**2
    r = np.sqrt(R)  # 目标位置距离

    # 计算位移量
    Yatr = k * np.sqrt((X[0] - Xsum[0])**2 + (X[1] - Xsum[1])**2)
    Yatz = k * r * np.sin(alpha)   # z 方向位移
    Yatx = k * r * np.cos(alpha) * np.cos(beta)  # x 方向位移
    Yaty = k * r * np.cos(alpha) * np.sin(beta)  # y 方向位移

    return Yatx, Yaty, Yatz, Yatr

def Follower_PI3(distance, VM, Vm, Position_angle, fuyang_angle, 
                 R, Q, Rotat, pitch, Rm, Qm, j, 
                 kp, ki, krp, kri, kqp, kqi):
    """
    计算跟随无人机的速度和角度控制
    
    参数：
    distance        - 距离数组
    VM, Vm         - 最大/最小速度
    Position_angle - 目标位置角度
    fuyang_angle   - 俯仰角
    R, Q           - 速度方向角数组
    Rotat, pitch   - 旋转角度数组
    Rm, Qm         - 最大向心加速度和角速度
    j              - 当前索引
    kp, ki         - 速度控制 PI 参数
    krp, kri       - 旋转角控制 PI 参数
    kqp, kqi       - 俯仰角控制 PI 参数
    
    返回：
    U, R, Q, Rotatc, pitchc
    """
    # 计算误差 ed
    ed = distance[j] - distance[j - 1]

    # 速度 PI 控制
    U = kp * ed + ki * distance[j]
    U = abs(U)

    # 限制无人机速度
    if U > VM:
        U = VM
    elif U < Vm:
        U = Vm

    # 计算偏航角误差
    Rotat[j] = Position_angle - R[j - 1]
    
    # 角度范围转换到 (-pi, pi)
    if abs(Rotat[j]) > np.pi:
        Rotat[j] = (2 * np.pi - abs(Rotat[j])) * (-abs(Rotat[j]) / Rotat[j])

    # 角速度 PI 控制
    Rotatc = kri * (Rotat[j] - Rotat[j - 1]) + krp * Rotat[j - 1]

    # 限制向心加速度
    if abs(Rotatc) * U >= Rm:
        Rotatc = (Rotatc / abs(Rotatc)) * Rm / U
    
    # 更新 R 方向角
    R[j] = R[j - 1] + Rotatc

    # 再次转换 R 角度范围到 (-pi, pi)
    if abs(R[j]) > np.pi:
        R[j] = (2 * np.pi - abs(R[j])) * (-abs(R[j]) / R[j])

    # 计算俯仰角误差
    pitch[j] = fuyang_angle - Q[j - 1]

    # 角度范围转换到 (-pi, pi)
    if abs(pitch[j]) > np.pi:
        pitch[j] = (2 * np.pi - abs(pitch[j])) * (-abs(pitch[j]) / pitch[j])

    # 俯仰角速度 PI 控制
    pitchc = kqi * (pitch[j] - pitch[j - 1]) + kqp * pitch[j - 1]

    # 限制角速度
    if abs(pitchc) >= Qm:
        pitchc = (pitchc / abs(pitchc)) * Qm

    # 更新 Q 方向角
    Q[j] = Q[j - 1] + pitchc

    return U, R[j], Q[j], Rotatc, pitchc

def Leader_P(u, X1, V1, Position_phi, Position_theta):
    """
    计算长机的下一个状态

    参数：
    u               - 控制量 [a_tau, theta_dot, phi_dot] (加速度, 俯仰角角速度, 偏航角角速度)
    X1              - 当前地面坐标系下的长机坐标 [x, y, z]
    V1              - 当前长机飞行速度
    Position_phi    - 当前偏航角 (rad)
    Position_theta  - 当前俯仰角 (rad)

    返回：
    V1_next, phi1_next, theta1_next, X1_next
    """

    # 速度更新
    V1_next = V1 + u[0]

    # 航向角（偏航角）更新
    phi1_next = Position_phi + u[1]

    # 俯仰角更新
    theta1_next = Position_theta + u[2]

    # 角度范围转换 (-pi, pi)
    if abs(phi1_next) > np.pi:
        phi1_next = (2 * np.pi - abs(phi1_next)) * (-abs(phi1_next) / phi1_next)

    # 速度坐标变换
    v1x = np.cos(theta1_next) * np.cos(phi1_next) * V1_next
    v1y = np.cos(theta1_next) * np.sin(phi1_next) * V1_next
    v1z = np.sin(theta1_next) * V1_next

    # 位置更新
    X1_next = np.array([
        X1[0] + v1x,
        X1[1] + v1y,
        X1[2] + v1z
    ])

    return V1_next, phi1_next, theta1_next, X1_next

def potential(i, j, T):
    """
    计算个体之间的斥力
    
    参数：
    i, j - 两个个体的索引
    T    - 位置矩阵，形状为 (N, 3)，表示 N 个个体的三维坐标
    
    返回：
    pot  - 计算出的斥力向量 (3D)
    """
    d = 80.0  # 相对安全距离

    # 自己对自己没有作用力
    if i == j:
        return np.array([0.0, 0.0, 0.0])

    # 计算两点之间的欧几里得距离
    diff = T[i, :] - T[j, :]
    distance = np.linalg.norm(diff, 2)

    # 距离过远的个体没有排斥力
    if distance > 4.0 * d:
        return np.array([0.0, 0.0, 0.0])

    # 计算斥力（与距离的立方成反比）
    pot = 2 * d**2 * diff / (distance**3)

    # 如果距离小于 1.4d，则增加随机 z 方向的力以避障
    if distance < 1.4 * d:
        emg = 100000.0 * np.linalg.norm(pot) * np.array([0.0, 0.0, 8.0])
        pot = pot + emg if i < j else pot - emg

    return pot


def potential_obs(i, obs, T, U, R, Q):
    """
    计算个体与障碍物之间的斥力
    
    参数：
    i     - 目标个体索引
    obs   - 障碍物信息 (num_obs, 3)，每个障碍物的格式 [x, y, radius]
    T     - 个体位置矩阵 (N, 3)，N 个个体的坐标
    U     - 速度数组 (N, )，个体的速度
    R     - 偏航角数组 (N, )，个体的偏航角
    Q     - 俯仰角数组 (N, )，个体的俯仰角

    返回：
    pot   - 计算出的斥力向量 (3D)
    """
    d = 70  # 相对安全距离
    num_obs = obs.shape[0]

    pot = np.array([0.0, 0.0, 0.0])

    for i_obs in range(num_obs):
        # 取障碍物的 (x, y) 位置，z 取目标个体的 z
        obstacle = np.array([obs[i_obs, 0], obs[i_obs, 1], T[i, 2]])

        # 计算距离
        diff = T[i, :] - obstacle
        distance = np.linalg.norm(diff, 2) - obs[i_obs, 2]

        # 计算斥力
        pot += 2 * d**2 * diff / abs(distance**3)

        # 如果距离小于 5*d，则增加紧急避障力
        if distance < 5 * d:
            emg = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]]) * (diff / np.linalg.norm(diff, 2))
            emg_T = emg.T

            # 计算单位速度方向
            U_unit = np.array([
                np.cos(R) * np.cos(Q) * U[i%21-1,i//21 + (i%21==0)*0+1*(i%21!=0)-1],
                np.sin(R) * np.cos(Q) * U[i%21-1,i//21 + (i%21==0)*0+1*(i%21!=0)-1],
                np.sin(Q) * U[i%21-1,i//21 + (i%21==0)*0+1*(i%21!=0)-1]
            ])

            # 计算紧急避障力
            emg_1 = 100000.0 * np.linalg.norm(pot) * np.sign(U_unit @ emg_T) @ emg
            pot += emg_1

    return pot
