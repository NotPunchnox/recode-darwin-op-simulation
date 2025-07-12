#include <webots/robot.h>
#include <webots/motor.h>
#include <stdio.h>
#include <math.h>

// Structure pour les angles
struct Angles {
    double angleEpaule;   // Shoulder yaw (degrees)
    double angleEpaule2;  // Shoulder pitch (degrees)
    double angleCoude;    // Elbow pitch (degrees)
};

// Variable globale pour conserver les derniers angles valides
struct Angles lastValidAngles = {0.0, 90.0, 90.0};

// Paramètres des longueurs du bras en cm
const double Ltx = 12.22;  // Offset épaule (122.2 mm = 12.22 cm)
const double L2 = 6.0;     // Haut du bras (60 mm = 6 cm)
const double Ll = 12.9;    // Avant-bras (129 mm = 12.9 cm)


// Fonction de cinématique inverse
struct Angles inverseKinematicARM(double x, double y, double z) {
    struct Angles angles = lastValidAngles;

    // Calcul de la distance horizontale (x avant, y latéral)
    double r = sqrt(x * x + y * y);
    double h = z - Ltx; // Hauteur relative à l'articulation de l'épaule

    // Distance totale depuis l'épaule
    double D = sqrt(r * r + h * h);

    // Vérification de la portée en cm
    double maxReach = L2 + Ll; // 18.9 cm
    double minReach = fabs(L2 - Ll); // 6.9 cm
    if (D > maxReach || D < minReach) {
        printf("ERREUR: Position (%.2f, %.2f, %.2f cm) hors portée (max=%.2f cm, min=%.2f cm)\n",
               x, y, z, maxReach, minReach);
        return lastValidAngles;
    }

    // Calcul de theta1 (shoulder yaw)
    double theta1 = atan2(y, x) * 180.0 / M_PI; // Angle par rapport à l'axe x
    angles.angleEpaule = 180 - theta1;

    // Calcul de theta2 (shoulder pitch) et theta3 (elbow pitch)
    double cos_theta3 = (D * D - L2 * L2 - Ll * Ll) / (2 * L2 * Ll);
    cos_theta3 = fmax(-1.0, fmin(1.0, cos_theta3)); // Clamping
    double theta3_rad = acos(cos_theta3); // Angle du coude en radians
    double theta3 = theta3_rad * 180.0 / M_PI;

    // Calcul de theta2 (shoulder pitch)
    double beta = atan2(h, r) * 180.0 / M_PI;
    double alpha = acos((L2 * L2 + D * D - Ll * Ll) / (2 * L2 * D)) * 180.0 / M_PI;
    double theta2 = beta - alpha;

    // Ajustement des angles pour correspondre à votre convention
    angles.angleEpaule2 = theta2 + 90.0 + 37; // Offset +90°
    angles.angleCoude = theta3 - 90.0;   // Offset -90°

    // Mettre à jour les derniers angles valides
    lastValidAngles = angles;

    // Affichage des résultats
    printf("Position (%.2f, %.2f, %.2f cm) atteinte avec angles: épaule=%.1f°, épaule2=%.1f°, coude=%.1f°\n",
           x, y, z, angles.angleEpaule, angles.angleEpaule2, angles.angleCoude);

    return angles;
}