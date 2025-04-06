import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import atan, sin, cos, sqrt, pi
from angle_utils import compute_angle3,compute_weiyi3,Follower_PI3,Leader_P,potential,potential_obs

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
    plt.pause(0.01)

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

