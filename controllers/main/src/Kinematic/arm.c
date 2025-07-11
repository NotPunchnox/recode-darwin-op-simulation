#include <webots/robot.h>
#include <webots/motor.h>
#include <stdio.h>
#include <math.h>

// Definition de la structure Angles
struct Angles {
    double angleEpaule;
    double angleEpaule2;
    double angleCoude;
};

// Variable globale pour conserver les derniers angles valides
struct Angles lastValidAngles = {0.0, 90.0, 0.0};

struct Angles inverseKinematicARM(double x, double y, double z) {
    struct Angles angles = lastValidAngles;

    // Longueurs des segments
    double biceps = 6.0;    // Haut du bras (cm)
    double avant_bras = 12.9; // Avant-bras (cm)

    // Calcul de la distance au point cible
    double h = sqrt(x * x + z * z);

    // Vérification de l'atteignabilité
    double maxReach = biceps + avant_bras;  // 18.0 cm
    double minReach = fabs(biceps - avant_bras);  // 6.0 cm
    if (h > maxReach) {
        printf("ERREUR: Position (%.2f, %.2f) trop éloignée (distance=%.2f, max=%.2f)\n", x, z, h, maxReach);
        return lastValidAngles;
    }
    if (h < minReach) {
        printf("ERREUR: Position (%.2f, %.2f) trop proche (distance=%.2f, min=%.2f)\n", x, z, h, minReach);
        return lastValidAngles;
    }

    // Calcul de l'angle initial (alpha) en radians
    double alpha = atan2(z, x);

    // Calcul du premier groupe d'angles (coude vers le bas)
    /*
    double cos_teta1_1 = (x * x + z * z + biceps * biceps - avant_bras * avant_bras) / (2 * biceps * h);
    cos_teta1_1 = fmax(-1.0, fmin(1.0, cos_teta1_1)); // Clamping pour éviter les erreurs numériques
    double teta1_1 = acos(cos_teta1_1) + alpha;
    double teta2_1 = atan2((z - biceps * sin(teta1_1)) / avant_bras, (x - biceps * cos(teta1_1)) / avant_bras) - teta1_1;
    */


    // Calcul du deuxième groupe d'angles (coude vers le haut)
    double cos_teta1_2 = (x * x + z * z + biceps * biceps - avant_bras * avant_bras) / (2 * biceps * h);
    cos_teta1_2 = fmax(-1.0, fmin(1.0, cos_teta1_2));
    double teta1_2 = -acos(cos_teta1_2) + alpha;
    double teta2_2 = atan2((z - biceps * sin(teta1_2)) / avant_bras, (x - biceps * cos(teta1_2)) / avant_bras) - teta1_2;

    // Stocker les angles en degrés (utilisation de la première solution)
    angles.angleEpaule = (teta1_2 * 180.0 / M_PI)+90;
    angles.angleEpaule2 = 35;
    angles.angleCoude = (teta2_2 * 180.0 / M_PI)-90;

    // Mettre à jour les derniers angles valides
    lastValidAngles = angles;

    // Affichage des résultats
    printf("Position (%.2f, %.2f) atteinte avec angles (coude bas): épaule=%.1f°, coude=%.1f°\n", x, z, angles.angleEpaule, angles.angleCoude);
    printf("Alternative (coude haut): épaule=%.1f°, coude=%.1f°\n", teta1_2 * 180.0 / M_PI, teta2_2 * 180.0 / M_PI);

    return angles;
}
