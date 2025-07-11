#include <webots/robot.h>
#include <webots/motor.h>
#include <stdio.h>
#include <math.h>

// Structure pour stocker les angles du bras
struct Angles {
    double shoulderPitch;    // Rotation épaule (haut/bas)
    double shoulderYaw;      // Rotation épaule (gauche/droite)
    double elbow;           // Angle coude
};

// Variable globale pour conserver les derniers angles valides
struct Angles lastValidAngles = {0.0, 0.0, 0.0};

// Fonction pour limiter les angles à des valeurs raisonnables
struct Angles constrainAngles(struct Angles angles) {
    angles.shoulderPitch = fmax(-90.0, fmin(90.0, angles.shoulderPitch));
    angles.shoulderYaw = fmax(-90.0, fmin(90.0, angles.shoulderYaw));
    angles.elbow = fmax(-150.0, fmin(0.0, angles.elbow));
    return angles;
}

// Cinématique inverse 3D pour le bras
struct Angles inverseKinematicARM(double x, double y, double z) {
    struct Angles angles = lastValidAngles;
    
    // Longueurs des segments (ajustées selon votre robot)
    double upperArm = 6.0;   // Longueur du bras supérieur
    double forearm = 6.0;    // Longueur de l'avant-bras (ajustée)
    
    // Distance totale jusqu'à la cible
    double distance_totale = sqrt(x*x + y*y + z*z);
    
    // Vérification de l'atteignabilité
    double maxReach = upperArm + forearm;
    double minReach = fabs(upperArm - forearm);
    
    if (distance_totale > maxReach) {
        printf("ERREUR: Position (%.2f, %.2f, %.2f) trop éloignée (distance=%.2f, max=%.2f)\n", x, y, z, distance_totale, maxReach);
        return lastValidAngles;
    }
    
    if (distance_totale < minReach) {
        printf("ERREUR: Position (%.2f, %.2f, %.2f) trop proche (distance=%.2f, min=%.2f)\n", x, y, z, distance_totale, minReach);
        return lastValidAngles;
    }
    
    // 1. Calcul de l'angle de rotation horizontale (Yaw)
    angles.shoulderYaw = atan2(y, x) * 180.0 / M_PI;
    
    // 2. Distance dans le plan vertical après rotation horizontale
    double horizontal_dist = sqrt(x*x + y*y);
    double vertical_dist = z;
    double planar_dist = sqrt(horizontal_dist*horizontal_dist + vertical_dist*vertical_dist);
    
    // 3. Calcul de l'angle du coude par la loi des cosinus
    double cos_elbow = (upperArm*upperArm + forearm*forearm - planar_dist*planar_dist) / (2.0 * upperArm * forearm);
    cos_elbow = fmax(-1.0, fmin(1.0, cos_elbow));
    double elbow_rad = acos(cos_elbow);
    angles.elbow = -(180.0 - (elbow_rad * 180.0 / M_PI)); // Négatif car coude se plie vers l'intérieur
    
    // 4. Calcul de l'angle de l'épaule (Pitch)
    double alpha = atan2(vertical_dist, horizontal_dist) * 180.0 / M_PI;
    
    double cos_beta = (upperArm*upperArm + planar_dist*planar_dist - forearm*forearm) / (2.0 * upperArm * planar_dist);
    cos_beta = fmax(-1.0, fmin(1.0, cos_beta));
    double beta = acos(cos_beta) * 180.0 / M_PI;
    
    angles.shoulderPitch = alpha + beta;
    
    // Application des contraintes
    angles = constrainAngles(angles);
    lastValidAngles = angles;
    
    printf("Position (%.2f, %.2f, %.2f) -> Angles: Pitch=%.1f°, Yaw=%.1f°, Elbow=%.1f°\n", x, y, z, angles.shoulderPitch, angles.shoulderYaw, angles.elbow);
    
    return angles;
}