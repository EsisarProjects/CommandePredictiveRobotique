import numpy as np
import casadi as ca
import scipy.special
import matplotlib.pyplot as plt

# 1. Funções auxiliares de Bernstein e Matriz de Energia Q
def bernstein_poly(i, n, t):
    return scipy.special.comb(n, i) * ((1 - t) ** (n - i)) * (t ** i)

def compute_Q(n):
    Q = np.zeros((n, n))
    for i in range(n):
        for k in range(n):
            c_i = scipy.special.comb(n - 1, i)
            c_k = scipy.special.comb(n - 1, k)
            c_tot = scipy.special.comb(2 * n - 2, i + k)
            Q[i, k] = (n**2) * (c_i * c_k) / ((2 * n - 1) * c_tot)
    return Q

# 2. Definição dos Waypoints (W) e instantes de passagem (tw)
W = np.array([
    [0.0, 0.0],
    [1.0, 2.0],
    [3.0, 1.0],
    [4.0, 2.0],
    [5.0, 1.0]
])
tw = [0.0, 0.3, 0.5, 0.8, 1.0]
m = len(tw)
n = 6  # Grau da curva de Bézier (n + 1 = 7 pontos de controle)

# 3. Configuração do Otimizador CasADi (Opti stack)
solver = ca.Opti()
P = solver.variable(2, n + 1)  # Variável de decisão: pontos de controle [x; y]

# Função de custo: minimizar a energia da derivada primeira
Q = compute_Q(n)
dP = P[:, 1:] - P[:, :-1]
energy = ca.trace(dP @ ca.DM(Q) @ dP.T)
solver.minimize(energy)

# Restrições de igualdade nos waypoints: z(tw[j]) == W[j]
for j in range(m):
    Bs = ca.DM([bernstein_poly(i, n, tw[j]) for i in range(n + 1)])
    solver.subject_to(P @ Bs == W[j, :])

# 4. Resolução numérica via IPOPT
solver.solver('ipopt', {"expand": True}, {"print_level": 0})
sol = solver.solve()
P_opt = sol.value(P)

# 5. Avaliação da trajetória e derivadas para o Dubins Car
L_car = 0.5  # Distância entre eixos [m]
t_vals = np.linspace(0, 1, 200)

B_mat    = np.array([[bernstein_poly(i, n, t) for i in range(n + 1)] for t in t_vals])
B_dot    = np.array([[bernstein_poly(i, n - 1, t) for i in range(n)] for t in t_vals])
B_ddot   = np.array([[bernstein_poly(i, n - 2, t) for i in range(n - 1)] for t in t_vals])

dP_opt  = n * np.diff(P_opt, axis=1)
ddP_opt = n * (n - 1) * np.diff(P_opt, n=2, axis=1)

z   = B_mat @ P_opt.T
zd  = B_dot @ dP_opt.T
zdd = B_ddot @ ddP_opt.T

# Inversão algébrica (Differential Flatness do Carro de Dubins)
u_V     = np.sqrt(zd[:, 0]**2 + zd[:, 1]**2)
theta   = np.arctan2(zd[:, 1], zd[:, 0])
u_theta = np.arctan2(L_car * (zdd[:, 1]*zd[:, 0] - zd[:, 1]*zdd[:, 0]), u_V**3)

# 6. Visualização
fig, axs = plt.subplots(2, 2, figsize=(10, 7))
axs[0, 0].plot(z[:, 0], z[:, 1], 'b-', lw=2, label='Trajetória z(t)')
axs[0, 0].plot(P_opt[0, :], P_opt[1, :], 'ro--', label='Pontos de Controle P')
axs[0, 0].plot(W[:, 0], W[:, 1], 'ks', markersize=8, label='Waypoints W')
axs[0, 0].set_title('Trajetória Plana (x, y)')
axs[0, 0].legend(); axs[0, 0].grid(True)

axs[0, 1].plot(t_vals, theta, 'r-', lw=1.5)
axs[0, 1].set_title('Orientação θ(t) [rad]'); axs[0, 1].grid(True)

axs[1, 0].plot(t_vals, u_V, 'g-', lw=1.5)
axs[1, 0].set_title('Velocidade Linear u_V(t) [m/s]'); axs[1, 0].grid(True)

axs[1, 1].plot(t_vals, u_theta, 'm-', lw=1.5)
axs[1, 1].set_title('Ângulo de Esterçamento u_θ(t) [rad]'); axs[1, 1].grid(True)

plt.tight_layout()
plt.show()