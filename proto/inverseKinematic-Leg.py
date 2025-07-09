import math
import matplotlib.pyplot as plt
import numpy as np

class Angles:
    def __init__(self, angleThigh=0.0, angleKnee=0.0, angleFoot=0.0):
        self.angleThigh = angleThigh
        self.angleKnee = angleKnee
        self.angleFoot = angleFoot

def inverseKinematicLeg(x, y, z):
    angles = Angles()  # Initialiser à 0
    femur = 9.3
    tibia = 9.3
    LF = 3.35
    max_length = femur + tibia + LF

    # Vérifier si la position est atteignable (en 2D, on ignore y)
    dist = math.sqrt(x*x + z*z)
    if dist > max_length or dist < abs(femur - tibia):
        print(f"Error: Position ({x:.2f}, {y:.2f}, {z:.2f}) out of reach.")
        return angles

    x = x - LF  # Ajuster pour la longueur du pied
    h = math.sqrt(x*x + z*z)
    if h < 0.01:  # Éviter division par zéro
        print("Error: Invalid h value.")
        return angles

    A1 = math.atan2(z, x)
    A2 = math.acos((tibia*tibia + h*h - femur*femur) / (2*tibia*h))
    A3 = math.acos((femur*femur + h*h - tibia*tibia) / (2*femur*h))
    A4 = math.acos((z*z + h*h - x*x) / (2*z*h))
    A5 = math.acos((femur*femur + tibia*tibia - h*h) / (2*femur*tibia))

    # Vérifier si les calculs sont valides
    if math.isnan(A1) or math.isnan(A2) or math.isnan(A3) or math.isnan(A4) or math.isnan(A5):
        print("Error: Invalid angles calculated.")
        return angles

    angles.angleThigh = (A3 + A4) * 180.0 / math.pi
    angles.angleKnee = 180.0 - (A5 * 180.0 / math.pi)  # Inverser l'angle du genou
    angles.angleFoot = (A1 + A2) * 180.0 / math.pi

    print("Inverse Kinematics Angles:")
    print(f"Thigh: {angles.angleThigh:.2f}, Knee: {angles.angleKnee:.2f}, Foot: {angles.angleFoot:.2f}")
    return angles

def plot_leg(angles, x_target, z_target):
    femur = 9.3
    tibia = 9.3
    LF = 3.35

    # Convertir les angles en radians pour les calculs
    theta1 = angles.angleThigh * math.pi / 180.0
    theta2 = (180.0 - angles.angleKnee) * math.pi / 180.0  # Re-inverser pour la visualisation
    theta3 = angles.angleFoot * math.pi / 180.0

    # Calculer les positions des articulations
    # Origine (bassin) à (0, 0)
    x0, z0 = 0, 0
    # Fin du fémur (genou)
    x1 = x0 + femur * math.cos(theta1)
    z1 = z0 + femur * math.sin(theta1)
    # Fin du tibia (cheville)
    x2 = x1 + tibia * math.cos(theta1 + theta2)
    z2 = z1 + tibia * math.sin(theta1 + theta2)
    # Fin du pied
    x3 = x2 + LF * math.cos(theta1 + theta2 + theta3)
    z3 = z2 + LF * math.sin(theta1 + theta2 + theta3)

    # Créer la figure
    plt.figure(figsize=(8, 8))
    plt.plot([x0, x1], [z0, z1], 'b-', linewidth=3, label='Fémur')
    plt.plot([x1, x2], [z1, z2], 'r-', linewidth=3, label='Tibia')
    plt.plot([x2, x3], [z2, z3], 'g-', linewidth=3, label='Pied')
    plt.plot(x_target, z_target, 'ko', markersize=10, label='Position cible')
    plt.plot([x0, x1, x2, x3], [z0, z1, z2, z3], 'ko', markersize=5)  # Articulations
    plt.grid(True)
    plt.xlabel('X (cm)')
    plt.ylabel('Z (cm)')
    plt.title(f'Cinématique inverse 2D (Thigh: {angles.angleThigh:.2f}°, Knee: {angles.angleKnee:.2f}°, Foot: {angles.angleFoot:.2f}°)')
    plt.legend()
    plt.axis('equal')
    plt.show()

def main():
    # Position cible (exemple)
    x, y, z = 5.0, 0.0, 10.0
    angles = inverseKinematicLeg(x, y, z)
    if angles.angleThigh != 0.0 or angles.angleKnee != 0.0 or angles.angleFoot != 0.0:
        plot_leg(angles, x, z)

if __name__ == "__main__":
    main()