import numpy as np
import matplotlib.pyplot as plt

import quadrotor

DEBUG = False

xdim = 6
udim = 2
N = 500
nvars = (N)*udim + (N+1)*xdim

x0 = np.zeros((6,1))
x_des = np.array([3,0,3,0,0,0])
I = np.identity(6)
Z = np.zeros((6,6))
A = quadrotor.A
B = quadrotor.B

M = np.zeros(((N+1)*xdim, nvars))
M[0:6, 0:6] = I

for i in range(N):
    M[(i+1)*xdim:(i+2)*xdim,i*(xdim+udim):i*(xdim+udim)+xdim] = A
    M[(i+1)*xdim:(i+2)*xdim,i*(xdim+udim)+xdim:(i+1)*(xdim+udim)] = B
    M[(i+1)*xdim:(i+2)*xdim,(i+1)*(xdim+udim):(i+1)*(xdim+udim)+xdim] = -I

Q = np.identity(6)
R = np.identity(2) * 0.01

G = np.zeros((nvars, nvars))
for i in range(N):
    G[i*(xdim+udim):i*(xdim+udim)+xdim,i*(xdim+udim):i*(xdim+udim)+xdim] = Q
    G[i*(xdim+udim)+xdim:(i+1)*(xdim+udim),i*(xdim+udim)+xdim:(i+1)*(xdim+udim)] = R
if DEBUG:
    print("G shape", G.shape)
    print("M shape", M.shape)

KKT = np.zeros((nvars+(N+1)*xdim, nvars+(N+1)*xdim))
KKT[0:G.shape[0],0:G.shape[1]] = G
KKT[G.shape[0]:,0:G.shape[1]] = M
KKT[0:G.shape[0],G.shape[1]:G.shape[1] + M.shape[0]] = M.T
if DEBUG:
    print("KKT shape", KKT.shape)

f = np.zeros((nvars+(N+1)*xdim,1))
p = np.zeros([xdim * (N+1)])
g = np.zeros((nvars))
for i in range(N):
    g[i*(xdim+udim):i*(xdim+udim)+xdim] = - x_des @ Q

p[0:6] = np.array([0,0,0,0,0,0])
f[0:nvars,0] = -g
f[nvars:,0] = p

x = np.linalg.solve(KKT, f)

u1 = x[6:nvars:8]
u2 = x[7:nvars:8]

new_u = np.concatenate([u1, u2], axis=1).T
quadrotor.animate_robot(x0, new_u, goal=[3,3])


pos_x = x[0:nvars:8]
vel_x = x[1:nvars:8]
pos_y = x[2:nvars:8]
vel_y = x[3:nvars:8]
orientation = x[4:nvars:8]
vel_angular = x[5:nvars:8]


fig, axes = plt.subplots(3, 2, figsize=(6, 8))
axes = axes.flatten()
t_state = np.arange(N+1)
axes[0].plot(t_state, pos_x)
axes[0].set_title("Position on X")
axes[0].set_xlabel("t")
axes[0].set_ylabel("Px")

axes[1].plot(t_state, pos_y)
axes[1].set_title("Position on Y")
axes[1].set_xlabel("t")
axes[1].set_ylabel("Py")

axes[2].plot(t_state, vel_x)
axes[2].set_title("Velocity on X")
axes[2].set_xlabel("t")
axes[2].set_ylabel("Vx")

axes[3].plot(t_state, vel_y)
axes[3].set_title("Velocity on Y")
axes[3].set_xlabel("t")
axes[3].set_ylabel("Vy")

axes[4].plot(t_state, orientation)
axes[4].set_title("Orientation")
axes[4].set_xlabel("t")
axes[4].set_ylabel("Orientation")

axes[5].plot(t_state, vel_angular)
axes[5].set_title("Angular Velocity")
axes[5].set_xlabel("t")
axes[5].set_ylabel("V_angular")

plt.tight_layout()
plt.savefig("imgs/x_info.png")
plt.show()


u1 = x[6:nvars:8]
u2 = x[7:nvars:8]
t_control = np.arange(N)

fig, axes = plt.subplots(1, 2, figsize=(6, 3))
axes = axes.flatten()
axes[0].plot(t_control, u1)
axes[0].set_title("u1")
axes[0].set_xlabel("t")
axes[0].set_ylabel("u1")
axes[1].plot(t_control, u2)
axes[1].set_title("u2")
axes[1].set_xlabel("t")
axes[1].set_ylabel("u2")
plt.tight_layout()
plt.savefig("imgs/u_info.png")
plt.show()