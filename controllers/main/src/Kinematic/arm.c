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
const double Ltx = 1.22;  // Offset épaule (122.2 mm = 12.22 cm)
const double L2 = 6.0;     // Haut du bras (60 mm = 6 cm)
const double Ll = 12.9;    // Avant-bras (129 mm = 12.9 cm)


// Fonction de cinématique inverse
struct Angles inverseKinematicARM(double x, double y, double z) {
    struct Angles angles = lastValidAngles;
    
    // Calcul de la distance horizontale dans le plan XY
    double r_xy = sqrt(x * x + y * y);
    
    // Distance totale depuis l'épaule jusqu'au point cible
    double D = sqrt(r_xy * r_xy + z * z);
    
    // Vérification de la portée
    double maxReach = L2 + Ll; // Longueur bras + avant-bras
    double minReach = fabs(L2 - Ll); // Différence des longueurs
    
    if (D > maxReach || D < minReach) {
        //printf("ERREUR: Position (%.2f, %.2f, %.2f cm) hors portée (max=%.2f cm, min=%.2f cm)\n",x, y, z, maxReach, minReach);
        return lastValidAngles;
    }
    
    // === CALCUL DE L'ANGLE D'ÉPAULE (YAW) - Rotation autour de l'axe Z ===
    // Angle dans le plan horizontal XY
    double shoulder_yaw = atan2(y, x) * 180.0 / M_PI;
    angles.angleEpaule = shoulder_yaw;
    
    // === CALCUL DE L'ANGLE DU COUDE (ELBOW PITCH) ===
    // Utilisation de la loi des cosinus dans le triangle formé par le bras et l'avant-bras
    double cos_elbow = (L2 * L2 + Ll * Ll - D * D) / (2 * L2 * Ll);
    cos_elbow = fmax(-1.0, fmin(1.0, cos_elbow)); // Clamping pour éviter les erreurs d'arrondi
    
    double elbow_angle = acos(cos_elbow) * 180.0 / M_PI;
    // L'angle du coude est mesuré par rapport à l'extension complète (180°)
    angles.angleCoude = 180.0 - elbow_angle;
    
    // === CALCUL DE L'ANGLE D'ÉPAULE (PITCH) - Rotation dans le plan vertical ===
    // Angle d'élévation vers le point cible
    double elevation_angle = atan2(z, r_xy) * 180.0 / M_PI;
    
    // Angle interne du triangle au niveau de l'épaule
    double cos_shoulder = (L2 * L2 + D * D - Ll * Ll) / (2 * L2 * D);
    cos_shoulder = fmax(-1.0, fmin(1.0, cos_shoulder)); // Clamping
    double shoulder_internal_angle = acos(cos_shoulder) * 180.0 / M_PI;
    
    // L'angle de pitch de l'épaule combine l'élévation et l'angle interne
    double shoulder_pitch = elevation_angle - shoulder_internal_angle;
    
    // Application des offsets spécifiques au Darwin OP
    angles.angleEpaule2 = shoulder_pitch + 90.0;
    
    // Ajustement final du coude (offset spécifique au Darwin OP)
    angles.angleCoude = angles.angleCoude - 90.0;
    
    // Mettre à jour les derniers angles valides
    lastValidAngles = angles;
    
    // Vérification et affichage des résultats
    //printf("Position (%.2f, %.2f, %.2f cm) -> Angles: épaule_yaw=%.1f°, épaule_pitch=%.1f°, coude=%.1f°\n",x, y, z, angles.angleEpaule, angles.angleEpaule2, angles.angleCoude);
    
    return angles;
}