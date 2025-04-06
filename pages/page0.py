import streamlit as st
from PIL import Image
import base64
from io import BytesIO
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import atan, sin, cos, sqrt, pi
import math
from angle_utils import compute_angle3,compute_weiyi3,Follower_PI3,Leader_P,potential,potential_obs


def image_to_base64(image):
    buffered = BytesIO()
    image.save(buffered, format="PNG")
    return base64.b64encode(buffered.getvalue()).decode()

# 设置网页信息  page_icons用于显示表情
st.set_page_config(page_title="空中使者——队形算法", layout="wide")

# 使用 CSS style
st.markdown(
    """
    <style>
    .center {
        text-align: center;
    }
    </style>
    """,
    unsafe_allow_html=True
)

st.markdown(
    """
    <style>
    .kai-font {
        font-family: "KaiTi", "楷体", serif;
    }
    </style>
    """,
    unsafe_allow_html=True
)

st.markdown(
    """
    <style>
    .kai-font_center {
        font-family: "KaiTi", "楷体", serif;
        text-align: center;
    }
    </style>
    """,
    unsafe_allow_html=True
)

st.title("队形算法设计")

st.header('''目标问题''')

st.markdown(r'''假设队形变换在无障碍物情况下进行变换，则无人机$i$从$P_i$到$$P_{j}^{'}$$的所有路线中直线路径最短，那么问题就可以将队形变化变成优化问题。该优化问题需要求解在最小时间内的队形变化方案，该优化问题实际上是一个指派问题。若编队中有$$n$$架飞机，则潜在的指派方案就有$$n!$$种。目标函数为能量耗费$$E(K)$$和使用时间$$T(t)$$的加权平均，则优化问题的数学表达式为''')


#st.latex方法更加，直接可使公式居中
st.latex(r'''min\;\;J(\xi(k),\xi(k+1))=\alpha E(k)+(1-\alpha) \textit{T}(k) \\ \begin{cases} \textit{E}(k) = {\sum^{n}_{i=1}} \textit{E}_i (k) \\ \textit{T}(k)=max\{ t_1(k),t_2(k), \cdots , t_n(k) , \alpha\in [0,1]\} \end{cases}''')


st.header("算法运行")
'''
com3D.m：九架无人机由不同初始位置，集结为正方形编队，之后沿直线变换为三角编队，保持三角形编队转弯、直线飞行、爬升的仿真过程

Initialization.m：无人机初始化函数

compute_angle3.m：计算当前该无人机角度误差函数，包括偏航角与俯仰角 Follower_PI3.m：三维空间下三个控制通道的PI控制器函数

Leader_P.m：领航机控制函数

Rb1.m：相对位置坐标转换函数

potential.m：人工势场斥力场函数
'''
# 打开视频文件
col1, col2, col3 = st.columns([0.1,0.8,0.1])

plt.rcParams['font.sans-serif'] = ['SimHei']

#st.sidebar begin
with st.sidebar:

    SelfPro = st.sidebar.radio(
        "选择控制哪一个程序的参数",
        ("MainProgrammer", "Main1Programmer"),
    )
    if SelfPro == 'MainProgrammer':
        #环境调参
        st.write('参数设置')

        k = st.slider('引力增益k',max_value=10,min_value=0,value = 1)      # 
        l = st.slider('队形距离l',max_value=600,min_value=0,value = 300)    # 
        k_pot = st.slider('人工势场斥力的系数k_pot',max_value=200,min_value=0,value = 130) # 
        V_L = st.slider('leader步长V_L',max_value=300,min_value=0,value = 100)   # 
        V_M = st.slider('follower最大步长V_M',max_value=300,min_value=0,value = 200)   # 
        V_m = st.slider('follower最小步长V_N',max_value=60,min_value=0,value = 40)    # 
        J = st.slider('步数J',max_value=300,min_value=0,value = 200 )    # 
        t = np.arange(0, J+1)
        Rm = st.slider('跟随无人机最大航向角向心加速度Rm',max_value=30,min_value=0,value = 20  )    # 
        Qm = st.slider('最大俯仰角速度Qm',max_value=3.0,min_value=0.0,value = 0.1 )    # 
        # PI系数        
        st.divider()  
        st.write('PI系数')
        kp = st.slider('kp',max_value=1.0,min_value=0.0,value = 0.5 )
        ki = st.slider('ki',max_value=1.0,min_value=0.0,value = 0.1 )
        krp = st.slider('krp',max_value=1.0,min_value=0.0,value = 0.9 )
        kri = st.slider('kri',max_value=1.0,min_value=0.0,value = 0.4 )
        kqp = st.slider('kqp',max_value=1.0,min_value=0.0,value = 0.9 )
        kqi = st.slider('kqi',max_value=1.0,min_value=0.0,value = 0.4)
    else:
        st.write('参数设置')
        st.divider() 
        k = st.slider('Attraction gain k',max_value=5,min_value=0,value = 1 ) # 
        l = st.slider('Formation distance l',max_value=300,min_value=0,value = 200 ) # 
        Vl = st.slider('Leader velocity V1',max_value=20,min_value=0,value = 10 )  # 
        V_M = st.slider('Follower max velocity V_M',max_value=40,min_value=0,value = 20 ) # 
        V_m = st.slider('Follower min velocity V_N',max_value=10,min_value=0,value = 0 ) # 
        # J = st.slider('引力增益k',max_value=10,min_value=0,value = 1)200  # Time
        # t = np.arange(J+1)  # Time sequence
        Rm = st.slider('Max heading angular acceleration of follower Rm',max_value=6.0,min_value=0.0,value = 1.5 ) # 
        Qm = st.slider('Max pitch angular velocity Qm',max_value=1,min_value=0.0,value = 0.1 )  # 
        J = st.slider('Time J',max_value=1000,min_value=0,value = 650 )
        t = np.arange(1, J+1)
        # PI coefficients
        st.divider() 
        st.write('PI系数')
        kp = st.slider('kp',max_value=1.0,min_value=0.0,value = 0.5 )
        ki = st.slider('ki',max_value=1.0,min_value=0.0,value = 0.1 )
        krp = st.slider('krp',max_value=1.0,min_value=0.0,value = 0.9 )
        kri = st.slider('kri',max_value=1.0,min_value=0.0,value = 0.4 )
        kqp = st.slider('kqp',max_value=1.0,min_value=0.0,value = 0.9 )
        kqi = st.slider('kqi',max_value=1.0,min_value=0.0,value = 0.4
  )
#st.sidebar end

# 使用st.video函数播放视频
with st.expander(label= '队形程序运行:四架无人机从一个靠近的范围内，受人工势场影响形成三角形编队，并做编队转弯、直线飞行、爬升的仿真过程'):
    with st.container(height=500):
        st.video(open('./vedio/AP3.mp4', 'rb').read(),autoplay= True)
with st.expander(label= '队形程序运行:实现了21架无人机的编队形成、编队重构、编队保持以及避撞'):
    with st.container(height=500):
        st.video(open('./vedio/test3DMain.mp4', 'rb').read(),autoplay= True)
with st.expander(label= '交互模拟A:21架无人机在队形变换时的模拟仿真'):
    with st.container(height=500):
        st.video(open('./vedio/MainProgammer.mp4', 'rb').read(),autoplay= True)
with st.expander(label= '交互模拟B:4架无人机的编队组成、重构、避障的模拟仿真'):
    with st.container(height=500):
        st.video(open('./vedio/Main1Progammer.mp4', 'rb').read(),autoplay= True)
Interact1,Interact2=st.tabs(['21架无人机的编队控制','4架无人机的队形变换'])

with Interact1:
    Interact1button = st.button('MainProgammer点击运行',key = 'I1b')
    with st.expander(label= '模拟仿真',expanded=True):
        with st.container(height=500):
            placeholder = st.empty()        
            if Interact1button :
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
                # k = 1       # 引力增益
                # l = 300     # 队形距离
                # k_pot = 130 # 人工势场斥力的系数
                # V_L = 100   # leader步长
                # V_M = 200   # follower最大步长
                # V_m = 40    # follower最小步长
                # J = 200     # 步数
                # t = np.arange(0, J+1)
                # Rm = 20     # 跟随无人机最大航向角向心加速度
                # Qm = 0.1    # 最大俯仰角速度

                # # PI系数
                # kp = 0.5
                # ki = 0.1
                # krp = 0.9
                # kri = 0.4
                # kqp = 0.9
                # kqi = 0.4

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
                    #plt.pause(0.001)  # 短暂暂停以更新图形
                    placeholder.pyplot(fig,use_container_width=True)
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
                    #plt.pause(0.001)  # 短暂暂停以更新图形
                    placeholder.pyplot(fig,use_container_width=True)

                # 绘制最终位置
                for i in range(21):
                    ax.scatter(T[i, 0], T[i, 1], T[i, 2], c=colors[i], s=100, marker='o')

                ax.set_xlabel('x[m]')
                ax.set_ylabel('y[m]')
                ax.set_zlabel('z[m]')
                plt.title('无人机编队飞行最终位置')
                plt.grid(True)
                plt.show()
                placeholder.pyplot(fig,use_container_width=True)

with Interact2:
    Interact2button = st.button('Main1Progammer点击运行',key = 'I2b')
    with st.expander(label= '模拟仿真',expanded=True):
        with st.container(height=500):     
            placeholder1 = st.empty()    
            if Interact2button :
                # Assuming the following functions are already implemented in Python:
                # potential_obs, potential, Leader_P, Follower_PI3, compute_weiyi3, compute_angle3


                # Initial positions
                T1 = np.array([200, 0, 300])  # Leader
                T2 = np.array([130, -70, 300])  # 1
                T3 = np.array([270, -70, 300])  # 2
                T4 = np.array([200, -140, 300])  # 3
                T = np.vstack((T1, T2, T3, T4))  # Initial parameter matrix

                # Plot initial positions
                fig = plt.figure()
                ax = fig.add_subplot(111, projection='3d')
                ax.plot([T1[0]], [T1[1]], [T1[2]], 'o', markersize=10, color='r', label='UAV_1')
                ax.plot([T2[0]], [T2[1]], [T2[2]], 'o', markersize=10, color='b', label='UAV_2')
                ax.plot([T3[0]], [T3[1]], [T3[2]], 'o', markersize=10, color='g', label='UAV_3')
                ax.plot([T4[0]], [T4[1]], [T4[2]], 'o', markersize=10, color='y', label='UAV_4')

                # Parameters
                k = 1  # Attraction gain
                l = 200  # Formation distance
                Vl = 10  # Leader velocity
                V_M = 20  # Follower max velocity
                V_m = 0  # Follower min velocity
                J = 200  # Time
                t = np.arange(J+1)  # Time sequence
                Rm = 1.5  # Max heading angular acceleration of follower
                Qm = 0.1  # Max pitch angular velocity
                J = 650
                t = np.arange(1, J+1)
                # PI coefficients
                kp = 0.5
                ki = 0.1
                krp = 0.9
                kri = 0.4
                kqp = 0.9
                kqi = 0.4

                # Formation matrix (relative displacement to leader)
                F = np.array([
                    [0, 0, 0],
                    [-l, -l, 0],
                    [-l, l, 0],
                    [-2*l, 0, 0]
                ])

                # ----------------------------------------------- leader
                R1 = np.zeros_like(t)  # Heading angular velocity (rad/s)
                R1[0] = 0  # Initial heading angle
                Position_angle1 = np.zeros_like(t)
                Rotat1 = np.zeros_like(t)
                Rotatc1 = np.zeros_like(t)
                Q1 = np.zeros_like(t)  # Pitch angular velocity (rad/s)
                Q1[0] = 0  # Initial pitch angle
                fuyang_angle1 = np.zeros_like(t)
                pitch1 = np.zeros_like(t)
                pitchc1 = np.zeros_like(t)

                # ----------------------------------------------- follower_1
                R2 = np.zeros_like(t)
                R2[0] = pi/2  # Initial heading angle
                Position_angle2 = np.zeros_like(t)
                Rotat2 = np.zeros_like(t)
                Rotatc2 = np.zeros_like(t)
                Q2 = np.zeros_like(t)
                Q2[0] = 0  # Initial pitch angle
                fuyang_angle2 = np.zeros_like(t)
                pitch2 = np.zeros_like(t)
                pitchc2 = np.zeros_like(t)

                distance21 = [sqrt((T2[0]-T1[0])**2 + (T2[1]-T1[1])**2 + (T2[2]-T1[2])**2)]
                distance2 =[0]*1000
                # ----------------------------------------------- follower_2
                R3 = np.zeros_like(t)
                R3[0] = pi/2  # Initial heading angle
                Position_angle3 = np.zeros_like(t)
                Rotat3 = np.zeros_like(t)
                Rotatc3 = np.zeros_like(t)
                Q3 = np.zeros_like(t)
                Q3[0] = 0  # Initial pitch angle
                fuyang_angle3 = np.zeros_like(t)
                pitch3 = np.zeros_like(t)
                pitchc3 = np.zeros_like(t)

                distance31 = [sqrt((T3[0]-T1[0])**2 + (T3[1]-T1[1])**2 + (T3[2]-T1[2])**2)]
                distance3 =[0]*1000
                # ----------------------------------------------- follower_3
                R4 = np.zeros_like(t)
                R4[0] = pi/2  # Initial heading angle
                Position_angle4 = np.zeros_like(t)
                Rotat4 = np.zeros_like(t)
                Rotatc4 = np.zeros_like(t)
                Q4 = np.zeros_like(t)
                Q4[0] = 0  # Initial pitch angle
                fuyang_angle4 = np.zeros_like(t)
                pitch4 = np.zeros_like(t)
                pitchc4 = np.zeros_like(t)

                distance41 = [sqrt((T4[0]-T1[0])**2 + (T4[1]-T1[1])**2 + (T4[2]-T1[2])**2)]
                distance4 =[0]*1000

                U1 = [Vl]
                U2 = [Vl]
                U3 = [Vl]
                U4 = [Vl]

                l = 300
                F = np.array([
                    [0, 0, 0],
                    [-l/2*sqrt(3), l/2, 0],
                    [-l/2*sqrt(3), -2*l/3, 0],
                    [-l, 0, 0]
                ])

                T = np.vstack((T1, T2, T3, T4))
                k_pot = 130  # Artificial potential field repulsion coefficient
                V_m = 4
                a = [np.linalg.norm(T2 - T3)]
                b = [np.linalg.norm(T2 - T4)]
                c = [np.linalg.norm(T3 - T4)]
                d = [np.linalg.norm(T1 - T2)]
                e = [np.linalg.norm(T1 - T3)]
                f = [np.linalg.norm(T1 - T4)]
                Position_angle1[0] = pi/2


                for j in range(J-1):
                    # ------------------------------------------- Leader completes action
                    distance1 = [0]
                    
                    # Set leader UAV control input
                    if j < 300 and j >= 101:
                        u_ctrl1 = np.array([0, pi/200, 0])
                    elif j < 420 and j >= 400:
                        u_ctrl1 = np.array([0.001, 0, 0.01])
                    elif j >= 480 and j < 500:
                        u_ctrl1 = np.array([-0.001, 0, -0.01])
                    else:
                        u_ctrl1 = np.array([0, 0, 0])

                    Vl_next, phi1_next, theta1_next, T1_next = Leader_P(u_ctrl1, T[0,:], Vl, Position_angle1[j-1], fuyang_angle1[j-1])

                    Position_angle1[j] = phi1_next
                    fuyang_angle1[j] = theta1_next
                    Vl = Vl_next
                    T[0,:] = T1_next

                    Transmatrix = np.array([
                        [cos(Position_angle1[j-1]), sin(Position_angle1[j-1]), 0],
                        [-sin(Position_angle1[j-1]), cos(Position_angle1[j-1]), 0],
                        [0, 0, 1]
                    ]) @ np.array([
                        [cos(fuyang_angle1[j-1]), 0, sin(fuyang_angle1[j-1])],
                        [0, 1, 0],
                        [-sin(fuyang_angle1[j-1]), 0, cos(fuyang_angle1[j-1])]
                    ])

                    # ------------------------------------------- Follower 1 moves
                    Xsum2 = T[0,:] + F[1,:] @ Transmatrix
                    pot2 = np.array([0.0, 0.0, 0.0])
                    for i_follower in range(4):
                        pot2 = pot2 + k_pot * potential(1, i_follower, T)
                    Xsum2 = Xsum2 + pot2

                    beta2, alpha2 = compute_angle3(T[1,:], Xsum2)
                    Fatx2, Faty2, Fatz2, Fatr2 = compute_weiyi3(T[1,:], Xsum2, k, beta2, alpha2)
                    Fsumy2 = Faty2  # y displacement
                    Fsumx2 = Fatx2  # x displacement
                    Fsumz2 = Fatz2  # z displacement
                    Fsumr2 = Fatr2
                    Position_angle2[j] = beta2
                    fuyang_angle2[j] = atan(Fsumz2 / Fsumr2)
                    T2 = T[1,:]
                    distance21.append(sqrt((T2[0]-T[0,0])**2 + (T2[1]-T[0,1])**2 + (T2[2]-T[0,2])**2))
                    distance2[j] = (sqrt((T2[0]-Xsum2[0])**2 + (T2[1]-Xsum2[1])**2 + (T2[2]-Xsum2[2])**2))

                    U2_j, R2_j, Q2_j, Rotatc2_j, pitchc2_j = Follower_PI3(
                        distance2, V_M, V_m, Position_angle2[j], fuyang_angle2[j], 
                        R2, Q2, Rotat2, pitch2, Rm, Qm, j, 
                        kp, ki, krp, kri, kqp, kqi
                    )
                    U2.append(U2_j)
                    R2[j] = R2_j
                    Q2[j] = Q2_j
                    Rotatc2[j] = Rotatc2_j
                    pitchc2[j] = pitchc2_j

                    if distance2[j] < 0.5:
                        Xnext2 = T2.copy()
                    else:
                        Xnext2 = np.array([
                            T2[0] + U2[j] * cos(Q2[j-1] + pitchc2[j]/2) * cos(R2[j-1] + Rotatc2[j]/2),
                            T2[1] + U2[j] * cos(Q2[j-1] + pitchc2[j]/2) * sin(R2[j-1] + Rotatc2[j]/2),
                            T2[2] + U2[j] * sin(Q2[j-1] + pitchc2[j]/2)
                        ])
                    T[1,:] = Xnext2

                    # ------------------------------------------- Follower 2 moves
                    Xsum3 = T[0,:] + F[2,:] @ Transmatrix
                    pot3 = np.array([0.0, 0.0, 0.0])
                    for i_follower in range(4):
                        pot3 = pot3 + k_pot * potential(2, i_follower, T)
                    Xsum3 = Xsum3 + pot3

                    beta3, alpha3 = compute_angle3(T[2,:], Xsum3)
                    Fatx3, Faty3, Fatz3, Fatr3 = compute_weiyi3(T[2,:], Xsum3, k, beta3, alpha3)
                    Fsumy3 = Faty3
                    Fsumx3 = Fatx3
                    Fsumz3 = Fatz3
                    Fsumr3 = Fatr3
                    T3 = T[2,:]
                    Position_angle3[j] = beta3
                    fuyang_angle3[j] = atan(Fsumz3 / Fsumr3)
                    distance31.append(sqrt((T3[0]-T[0,0])**2 + (T3[1]-T[0,1])**2 + (T3[2]-T[0,2])**2))
                    distance3[j] = sqrt((T3[0]-Xsum3[0])**2 + (T3[1]-Xsum3[1])**2 + (T3[2]-Xsum3[2])**2)

                    U3_j, R3_j, Q3_j, Rotatc3_j, pitchc3_j = Follower_PI3(
                        distance3, V_M, V_m, Position_angle3[j], fuyang_angle3[j], 
                        R3, Q3, Rotat3, pitch3, Rm, Qm, j, 
                        kp, ki, krp, kri, kqp, kqi
                    )
                    U3.append(U3_j)
                    R3[j] = R3_j
                    Q3[j] = Q3_j
                    Rotatc3[j] = Rotatc3_j
                    pitchc3[j] = pitchc3_j

                    if distance3[j] < 0.5:
                        Xnext3 = T3.copy()
                    else:
                        Xnext3 = np.array([
                            T3[0] + U3[j] * cos(Q3[j-1] + pitchc3[j]/2) * cos(R3[j-1] + Rotatc3[j]/2),
                            T3[1] + U3[j] * cos(Q3[j-1] + pitchc3[j]/2) * sin(R3[j-1] + Rotatc3[j]/2),
                            T3[2] + U3[j] * sin(Q3[j-1] + pitchc3[j]/2)
                        ])
                    T[2,:] = Xnext3

                    # ------------------------------------------- Follower 3 moves
                    Xsum4 = T[0,:] + F[3,:] @ Transmatrix
                    pot4 = np.array([0, 0, 0])
                    for i_follower in range(4):
                        pot4 = pot4 + k_pot * potential(3, i_follower, T)
                    Xsum4 = Xsum4 + pot4

                    beta4, alpha4 = compute_angle3(T[3,:], Xsum4)
                    Fatx4, Faty4, Fatz4, Fatr4 = compute_weiyi3(T[3,:], Xsum4, k, beta4, alpha4)
                    Fsumy4 = Faty4
                    Fsumx4 = Fatx4
                    Fsumz4 = Fatz4
                    Fsumr4 = Fatr4
                    Position_angle4[j] = beta4
                    fuyang_angle4[j] = alpha4
                    T4 = T[3,:]
                    distance41.append(sqrt((T4[0]-T[0,0])**2 + (T4[1]-T[0,1])**2 + (T4[2]-T[0,2])**2))
                    distance4[j] = sqrt((T4[0]-Xsum4[0])**2 + (T4[1]-Xsum4[1])**2 + (T4[2]-Xsum4[2])**2)

                    U4_j, R4_j, Q4_j, Rotatc4_j, pitchc4_j = Follower_PI3(
                        distance4, V_M, V_m, Position_angle4[j], fuyang_angle4[j], 
                        R4, Q4, Rotat4, pitch4, Rm, Qm, j, 
                        kp, ki, krp, kri, kqp, kqi
                    )
                    U4.append(U4_j)
                    R4[j] = R4_j
                    Q4[j] = Q4_j
                    Rotatc4[j] = Rotatc4_j
                    pitchc4[j] = pitchc4_j

                    if distance4[j] < 0.5:
                        Xnext4 = T4.copy()
                    else:
                        Xnext4 = np.array([
                            T4[0] + U4[j] * cos(Q4[j-1] + pitchc4[j]/2) * cos(R4[j-1] + Rotatc4[j]/2),
                            T4[1] + U4[j] * cos(Q4[j-1] + pitchc4[j]/2) * sin(R4[j-1] + Rotatc4[j]/2),
                            T4[2] + U4[j] * sin(Q4[j-1] + pitchc4[j]/2)
                        ])
                    T[3,:] = Xnext4

                    # Update distances between UAVs
                    a.append(np.linalg.norm(T[1,:] - T[2,:]))
                    b.append(np.linalg.norm(T[1,:] - T[3,:]))
                    c.append(np.linalg.norm(T[2,:] - T[3,:]))
                    d.append(np.linalg.norm(T[1,:] - T[0,:]))
                    e.append(np.linalg.norm(T[2,:] - T[0,:]))
                    f.append(np.linalg.norm(T[3,:] - T[0,:]))
                    #distance1 < 1 and
                    # Check if reached destination and plot
                    if (distance2 [j-1] < 1 and distance3 [j-1] < 1 and distance4 [j-1] < 1 and j > 500):
                        break

                    # Plot current positions
                    ax.plot([T[0,0]], [T[0,1]], [T[0,2]], '.r')
                    ax.plot([T[1,0]], [T[1,1]], [T[1,2]], '.b')
                    ax.plot([T[2,0]], [T[2,1]], [T[2,2]], '.g')
                    ax.plot([T[3,0]], [T[3,1]], [T[3,2]], '.y')
                    ax.set_xlabel('x[m]')
                    ax.set_ylabel('y[m]')
                    ax.set_zlabel('z[m]')
                    #plt.pause(0.001)
                    placeholder1.pyplot(fig)
                # Final plot
                ax.plot([T[0,0]], [T[0,1]], [T[0,2]], 'o', markersize=10, color='r')
                ax.plot([T[1,0]], [T[1,1]], [T[1,2]], 'o', markersize=10, color='b')
                ax.plot([T[2,0]], [T[2,1]], [T[2,2]], 'o', markersize=10, color='g')
                ax.plot([T[3,0]], [T[3,1]], [T[3,2]], 'o', markersize=10, color='y')

                # Draw formation lines
                ax.plot([T[0,0], T[1,0]], [T[0,1], T[1,1]], [T[0,2], T[1,2]], 'k-')
                ax.plot([T[0,0], T[2,0]], [T[0,1], T[2,1]], [T[0,2], T[2,2]], 'k-')
                ax.plot([T[1,0], T[3,0]], [T[1,1], T[3,1]], [T[1,2], T[3,2]], 'k-')
                ax.plot([T[2,0], T[3,0]], [T[2,1], T[3,1]], [T[2,2], T[3,2]], 'k-')

                ax.legend(['UAV_1', 'UAV_2', 'UAV_3', 'UAV_4'])
                plt.grid(True)
                plt.show()

                # Plot distances between UAVs
                plt.figure()
                plt.plot(t[:len(a)], a, '-b+', linewidth=2, label='|p_2-p_3|')
                plt.plot(t[:len(b)], b, '-g*', linewidth=2, label='|p_2-p_4|')
                plt.plot(t[:len(c)], c, '-yx', linewidth=2, label='|p_3-p_4|')
                plt.legend()
                plt.axis([0, 200, 0, 1300])
                plt.xlabel('t(s)')
                plt.ylabel('L(m)')
                plt.show()
                placeholder1.pyplot(fig)

# col1, col2, col3 = st.columns(3)

# with col1:
#     st.markdown('''<p class="kai-font_center">队形实时切换</p>''',unsafe_allow_html=True)
#     st.image(Image.open('./images/image_duixing1.png'),caption='基于指派算法的实时切换',width=350)
# with col2: 
#     st.markdown('''<p class="kai-font_center">队形时变队形</p>''',unsafe_allow_html=True)
#     st.image(Image.open('./images/image_duixing3.png'),caption='基于阿基米德螺线的时变队形',width=350)
# with col3:
#     st.markdown('''<p class="kai-font_center">队形变化仿真</p>''',unsafe_allow_html=True)
#     st.image(Image.open('./images/image_duixing2.png'),caption='基于粒子群算法的变化仿真',width=350)


# col11, col12 = st.columns(2)
# col111, col112 = st.columns(2)

# st.header("算法对比")

# with col11:
#     st.image(Image.open('./images/result1block.png'),caption='队形变换示意图',width=560)  
      
# with col12:
#     #with col111:
#     st.image(Image.open('./images/result3UAV2.png'),caption='两种队形变换的概念图',width=580)

#     #with col112:
#     #st.markdown('')
#     st.image(Image.open('./images/result3UAV3.png'),caption='两种队形变换的概念图',width=580)

# #col30, col31, col32 = st.columns(3)

# st.image(Image.open('./images/result2juzhen.png'),caption='两种队形变换的概念图',width=1170)

# col22, col23 = st.columns(2)

# with col22:
#     st.image(Image.open(r'images\result4ConnectMatrix.png'),caption='两种队形变换的概念图',width=580)
# with col23:
#     st.image(Image.open(r'images\result5UAVroute.png'),caption='两种队形变换的概念图',width=580)

# col30, col31 = st.columns([0.55,0.45])
# with col30:
#     st.image(Image.open(r'images\result66pic.png'),caption='两种队形变换的概念图',width=635)

# with col31:
#     st.image(Image.open(r'images\result7computationtime.png'),caption='两种队形变换的概念图',width=525)


# st.header("算法设计")
