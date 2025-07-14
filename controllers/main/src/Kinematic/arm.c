#include <stdio.h>
#include <math.h>

// Structure pour les angles
struct Angles {
    double angleEpaule;   // Shoulder yaw (degrees)
    double angleEpaule2;  // Shoulder pitch (degrees)
    double angleCoude;    // Elbow pitch (degrees)
};

// Variable globale pour conserver les derniers angles valides
struct Angles lastValidAngles = {0.0, 90.0, 0.0};

// Paramètres des longueurs du bras en cm
const double Ltx = 12.2;  // Tronc (122.2 mm = 12.2 cm)
const double L2 = 6.0;    // Haut du bras (60 mm = 6 cm)
const double Ll = 12.9;   // Avant-bras (129 mm = 12.9 cm)

// Fonction de cinématique directe (pour vérification)
void forward_kinematics(double theta[], double *x, double *y, double *z) {
    *x = Ltx * cos(theta[0] * M_PI / 180.0) + 
         L2 * cos(theta[0] * M_PI / 180.0) * cos(theta[1] * M_PI / 180.0) + 
         Ll * (cos(theta[0] * M_PI / 180.0) * cos(theta[1] * M_PI / 180.0) * cos(theta[2] * M_PI / 180.0) - 
               sin(theta[0] * M_PI / 180.0) * sin(theta[2] * M_PI / 180.0));
    *y = Ltx * sin(theta[0] * M_PI / 180.0) + 
         L2 * sin(theta[0] * M_PI / 180.0) * cos(theta[1] * M_PI / 180.0) + 
         Ll * (sin(theta[0] * M_PI / 180.0) * cos(theta[1] * M_PI / 180.0) * cos(theta[2] * M_PI / 180.0) + 
               cos(theta[0] * M_PI / 180.0) * sin(theta[2] * M_PI / 180.0));
    *z = L2 * sin(theta[1] * M_PI / 180.0) + 
         Ll * sin(theta[1] * M_PI / 180.0) * cos(theta[2] * M_PI / 180.0);
}

// Fonction de cinématique inverse
struct Angles inverseKinematicARM(double x, double y, double z) {
    struct Angles angles = lastValidAngles;

    // Calcul de la distance horizontale dans le plan XY
    double r_xy = sqrt(x * x + y * y);
    double D = sqrt(r_xy * r_xy + z * z); // Distance totale depuis l'origine

    // Vérification de la portée (inclut le tronc)
    double maxReach = Ltx + L2 + Ll;
    double minReach = fabs(L2 - Ll);
    if (D > maxReach || D < minReach) {
        printf("ERREUR: Position (%.2f, %.2f, %.2f cm) hors portée (max=%.2f cm, min=%.2f cm)\n", x, y, z, maxReach, minReach);
        return lastValidAngles;
    }

    // === CALCUL DE L'ANGLE D'ÉPAULE (YAW) - Rotation autour de l'axe Z ===
    angles.angleEpaule = atan2(y, x) * 180.0 / M_PI;

    // === CALCUL DES ANGLES DANS LE PLAN VERTICAL ===
    // Distance effective après le tronc
    double D_eff = sqrt((D - Ltx) * (D - Ltx) + z * z); // Ajustement approximatif
    if (D_eff > (L2 + Ll) || D_eff < fabs(L2 - Ll)) {
        printf("ERREUR: Position verticale hors portée après tronc\n");
        return lastValidAngles;
    }

    // Angle d'élévation globale
    double elevation_angle = atan2(z, r_xy) * 180.0 / M_PI;

    // Loi des cosinus pour l'angle au coude
    double cos_elbow = (L2 * L2 + Ll * Ll - D_eff * D_eff) / (2 * L2 * Ll);
    cos_elbow = fmax(-1.0, fmin(1.0, cos_elbow)); // Clamping
    double elbow_angle = acos(cos_elbow) * 180.0 / M_PI;
    angles.angleCoude = 180.0 - elbow_angle; // Angle par rapport à l'extension complète

    // Angle interne à l'épaule
    double cos_shoulder = (L2 * L2 + D_eff * D_eff - Ll * Ll) / (2 * L2 * D_eff);
    cos_shoulder = fmax(-1.0, fmin(1.0, cos_shoulder)); // Clamping
    double shoulder_internal_angle = acos(cos_shoulder) * 180.0 / M_PI;

    // Angle de pitch de l'épaule
    angles.angleEpaule2 = elevation_angle - shoulder_internal_angle + 90.0; // Offset pour Darwin-OP

    // Ajustement final du coude
    angles.angleCoude -= 90.0; // Offset spécifique

    // Clamping des angles (limites typiques des servos)
    angles.angleEpaule = fmax(-90.0, fmin(90.0, angles.angleEpaule));
    angles.angleEpaule2 = fmax(0.0, fmin(180.0, angles.angleEpaule2));
    angles.angleCoude = fmax(-90.0, fmin(90.0, angles.angleCoude));

    lastValidAngles = angles;
    printf("Position (%.2f, %.2f, %.2f cm) -> Angles: épaule_yaw=%.1f°, épaule_pitch=%.1f°, coude=%.1f°\n",
           x, y, z, angles.angleEpaule, angles.angleEpaule2, angles.angleCoude);

    return angles;
}