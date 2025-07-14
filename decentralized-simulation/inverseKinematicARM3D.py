import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Paramètres
L1, L2, L3 = 15.0, 60.0, 129.0  # Longueurs en mm
target = np.array([150.0, 0.0, 100.0])  # Position cible en mm
theta = np.array([0.0, np.pi/4, 0.0])  # Angles initiaux en radians

# Fonction de cinématique directe
def forward_kinematics(theta):
    x = L1 * np.cos(theta[0]) + L2 * np.cos(theta[0]) * np.cos(theta[1]) + L3 * (np.cos(theta[0]) * np.cos(theta[1]) * np.cos(theta[2]) - np.sin(theta[0]) * np.sin(theta[2]))
    y = L1 * np.sin(theta[0]) + L2 * np.sin(theta[0]) * np.cos(theta[1]) + L3 * (np.sin(theta[0]) * np.cos(theta[1]) * np.cos(theta[2]) + np.cos(theta[0]) * np.sin(theta[2]))
    z = L2 * np.sin(theta[1]) + L3 * np.sin(theta[1]) * np.cos(theta[2])
    return np.array([x, y, z])

# Jacobienne approximative (différences finies)
def jacobian(theta):
    J = np.zeros((3, 3))
    eps = 1e-6
    for i in range(3):
        theta_eps = theta.copy()
        theta_eps[i] += eps
        pos_eps = forward_kinematics(theta_eps)
        J[:, i] = (pos_eps - forward_kinematics(theta)) / eps
    return J

# Mise à jour des angles
def update_angles(theta, target):
    pos = forward_kinematics(theta)
    e = target - pos
    J = jacobian(theta)
    if np.linalg.norm(J) > 1e-6:
        delta_theta = np.linalg.pinv(J) @ e
        theta += delta_theta * 0.1  # Pas d'itération contrôlé
    return theta, np.linalg.norm(e)

# Configuration de la figure 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-50, 200])
ax.set_ylim([-150, 150])
ax.set_zlim([0, 150])
ax.set_xlabel('X (mm)')
ax.set_ylabel('Y (mm)')
ax.set_zlabel('Z (mm)')
ax.set_title('Bras Robotique Darwin-OP - Cinématique Inverse')

# Initialisation des données pour l'animation
lines = [
    ax.plot([], [], [], 'r-', lw=2)[0],  # Tronc
    ax.plot([], [], [], 'g-', lw=2)[0],  # Bras
    ax.plot([], [], [], 'b-', lw=2)[0]   # Avant-bras
]
target_point, = ax.plot([target[0]], [target[1]], [target[2]], 'yo', markersize=10)  # Cible

# Fonction d'initialisation pour l'animation
def init():
    for line in lines:
        line.set_data([], [])
        line.set_3d_properties([])
    return lines + [target_point]

# Fonction d'animation
def update(frame):
    global theta
    theta, error = update_angles(theta, target)
    
    # Calcul des positions des articulations
    pos0 = np.array([0.0, 0.0, 0.0])  # Origine
    pos1 = np.array([L1 * np.cos(theta[0]), L1 * np.sin(theta[0]), 0.0])
    pos2 = pos1 + np.array([L2 * np.cos(theta[0]) * np.cos(theta[1]), L2 * np.sin(theta[0]) * np.cos(theta[1]), L2 * np.sin(theta[1])])
    pos3 = pos2 + np.array([L3 * (np.cos(theta[0]) * np.cos(theta[1]) * np.cos(theta[2]) - np.sin(theta[0]) * np.sin(theta[2])),
                           L3 * (np.sin(theta[0]) * np.cos(theta[1]) * np.cos(theta[2]) + np.cos(theta[0]) * np.sin(theta[2])),
                           L3 * np.sin(theta[1]) * np.cos(theta[2])])

    # Mise à jour des lignes
    lines[0].set_data([pos0[0], pos1[0]], [pos0[1], pos1[1]])
    lines[0].set_3d_properties([pos0[2], pos1[2]])
    lines[1].set_data([pos1[0], pos2[0]], [pos1[1], pos2[1]])
    lines[1].set_3d_properties([pos1[2], pos2[2]])
    lines[2].set_data([pos2[0], pos3[0]], [pos2[1], pos3[1]])
    lines[2].set_3d_properties([pos2[2], pos3[2]])

    if error < 1e-3:
        ani.event_source.stop()
        print(f"Angles finaux (radians): {theta}")
        print(f"Erreur finale: {error}")

    return lines + [target_point]

# Création de l'animation
ani = FuncAnimation(fig, update, init_func=init, frames=None, interval=50, blit=True, repeat=False)

plt.show()