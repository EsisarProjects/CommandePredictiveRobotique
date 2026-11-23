import numpy as np
import casadi as ca
import scipy.special
import matplotlib.pyplot as plt

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

def compute_path_length(P_opt, n):
    t_vals = np.linspace(0, 1, 400)
    B_mat = np.array([[bernstein_poly(i, n, t) for i in range(n + 1)] for t in t_vals])
    traj = B_mat @ P_opt.T
    diffs = np.diff(traj, axis=0)
    return np.sum(np.sqrt(np.sum(diffs**2, axis=1))), traj

def solve_dubins_opt(W, tw, n, delta=0.0, rho=None):
    m = len(tw)
    solver = ca.Opti()
    P = solver.variable(2, n + 1)

    Q = compute_Q(n)
    dP = P[:, 1:] - P[:, :-1]
    energy = ca.trace(dP @ ca.DM(Q) @ dP.T)
    solver.minimize(energy)

    # Restrição de waypoints (exata se delta == 0, ou vizinhança se delta > 0)
    for j in range(m):
        Bs = ca.DM([bernstein_poly(i, n, tw[j]) for i in range(n + 1)])
        if delta == 0.0:
            solver.subject_to(P @ Bs == W[j, :])
        else:
            diff = P @ Bs - W[j, :]
            solver.subject_to(ca.sumsqr(diff) <= delta**2)

    # Restrição opcional de velocidade máxima ||v(t)|| <= rho
    if rho is not None:
        t_grid = np.linspace(0, 1, 60)
        for tk in t_grid:
            B_dot = ca.DM([bernstein_poly(i, n - 1, tk) for i in range(n)])
            v_k = n * (dP @ B_dot)
            solver.subject_to(ca.sumsqr(v_k) <= rho**2)

    solver.solver('ipopt', {"expand": True}, {"print_level": 0, "sb": "yes"})
    try:
        sol = solver.solve()
        P_opt = sol.value(P)
        length, traj = compute_path_length(P_opt, n)
        return True, length, P_opt, traj
    except Exception:
        return False, None, None, None

# Dados do laboratório
W = np.array([[0, 0], [1, 2], [3, 1], [4, 2], [5, 1]], dtype=float)
tw = [0.0, 0.4, 0.6, 0.8, 1.0]
m = len(tw)

# ==============================================================================
# EXERCÍCIO 1: Comprimento vs Número de Pontos de Controle (n+1 >= m)
# ==============================================================================
ctrl_points = list(range(m, 13)) # n+1 de 5 até 12 (n de 4 até 11)
lengths_ex1 = []

for n_pts in ctrl_points:
    n = n_pts - 1
    ok, L_val, _, _ = solve_dubins_opt(W, tw, n)
    lengths_ex1.append(L_val)
    print(f"[Ex 1] n+1 = {n_pts:2d} (grau n={n:2d}) -> Comprimento L = {L_val:.4f} m")

# ==============================================================================
# EXERCÍCIO 2: Relaxamento delta-envelope (L(0) <= 1.10 * L(delta_bar))
# ==============================================================================
n_fixed = 6 # 7 pontos de controle
_, L_0, _, traj_0 = solve_dubins_opt(W, tw, n_fixed, delta=0.0)
L_target = L_0 / 1.10

deltas = np.linspace(0.01, 0.6, 60)
delta_bar = None
L_bar = None
traj_bar = None

for d in deltas:
    ok, L_d, _, traj_d = solve_dubins_opt(W, tw, n_fixed, delta=d)
    if ok and L_d <= L_target:
        delta_bar = d
        L_bar = L_d
        traj_bar = traj_d
        break

print(f"\n[Ex 2] L(delta=0) = {L_0:.4f} m | Limite (L0/1.10) = {L_target:.4f} m")
print(f"[Ex 2] delta_bar = {delta_bar:.4f} m -> L(delta_bar) = {L_bar:.4f} m (L0/L_bar = {L_0/L_bar:.4f})")

# ==============================================================================
# EXERCÍCIO 3: Limite de velocidade ||v(t)|| <= rho
# ==============================================================================
rhos = np.linspace(6.0, 25.0, 96)
rho_min = None
for r in rhos:
    ok, L_r, _, _ = solve_dubins_opt(W, tw, n_fixed, delta=0.0, rho=r)
    if ok:
        rho_min = r
        break

if rho_min is not None:
    print(f"\n[Ex 3] Intervalo de viabilidade: rho_min = {rho_min:.2f} m/s <= rho <= +inf (irrestrito)")
else:
    print("\n[Ex 3] Nenhuma solução viável encontrada até rho = 25.0 m/s.")

# ==============================================================================
# PLOT DOS RESULTADOS
# ==============================================================================
fig, axs = plt.subplots(1, 2, figsize=(13, 5))

# Gráfico Ex 1
axs[0].plot(ctrl_points, lengths_ex1, 'bo-', linewidth=2, markersize=7)
axs[0].set_xlabel('Número de Pontos de Controle (n + 1)')
axs[0].set_ylabel('Comprimento Total da Trajetória L [m]')
axs[0].set_title('Ex 3.2.1: Comprimento vs. Pontos de Controle')
axs[0].grid(True)

# Gráfico Ex 2
axs[1].plot(traj_0[:, 0], traj_0[:, 1], 'b-', linewidth=2, label=f'Exata δ=0 (L={L_0:.2f}m)')
axs[1].plot(traj_bar[:, 0], traj_bar[:, 1], 'g--', linewidth=2, label=f'Relaxada δ̄={delta_bar:.2f} (L={L_bar:.2f}m)')
axs[1].plot(W[:, 0], W[:, 1], 'ks', markersize=7, label='Waypoints W')
for wj in W:
    circle = plt.Circle(wj, delta_bar, color='r', fill=False, linestyle=':', linewidth=1.5)
    axs[1].add_patch(circle)
axs[1].set_aspect('equal', adjustable='datalim')
axs[1].set_xlabel('x [m]')
axs[1].set_ylabel('y [m]')
axs[1].set_title(f'Ex 3.2.2: Vizinhança δ̄ = {delta_bar:.3f} m')
axs[1].legend()
axs[1].grid(True)

plt.tight_layout()
plt.show()