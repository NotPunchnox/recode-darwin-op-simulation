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
     
     double biceps = 6.0;
     double avant_bras = 12.0;
     double distance_totale = sqrt(x*x + y*y + z*z);

     /* Vérification de l'atteignabilité de la cible */
     if (distance_totale > biceps + avant_bras) {
          printf("ERREUR: Position (%.2f, %.2f, %.2f) trop éloignée (distance=%.2f, max=%.2f)\n", x, y, z, distance_totale, biceps + avant_bras);
          return lastValidAngles;
     }
     
     if (distance_totale < 2.0) {
          printf("ERREUR: Position (%.2f, %.2f, %.2f) trop proche (distance=%.2f, min=2.0)\n", x, y, z, distance_totale);
          return lastValidAngles;
     }

     double distance_horizontale = sqrt(x*x + y*y);
     
     if (distance_horizontale > 0.001) {
          double angle_rotation = atan2(x, y) * 180.0 / M_PI;
          angles.angleEpaule2 = fmax(0.0, fmin(90.0, 37.0 + angle_rotation * 0.5));
     } else {
          angles.angleEpaule2 = 37.0;
     }

     double distance_verticale = sqrt(distance_horizontale*distance_horizontale + z*z);

     double cos_angle_coude = (biceps*biceps + avant_bras*avant_bras - distance_verticale*distance_verticale) / (2 * biceps * avant_bras);
     cos_angle_coude = fmax(-1.0, fmin(1.0, cos_angle_coude));
     double angle_coude = acos(cos_angle_coude);
     angles.angleCoude = 90.0 - (180.0 - (angle_coude * 180.0 / M_PI));

     double angle_vers_cible = atan2(z, distance_horizontale) * 180.0 / M_PI;
     double cos_angle_epaule = (biceps*biceps + distance_verticale*distance_verticale - avant_bras*avant_bras) / (2 * biceps * distance_verticale);
     cos_angle_epaule = fmax(-1.0, fmin(1.0, cos_angle_epaule));
     double angle_epaule_offset = acos(cos_angle_epaule) * 180.0 / M_PI;
     angles.angleEpaule = angle_epaule_offset - angle_vers_cible;

     lastValidAngles = angles;
     
     printf("Position (%.2f, %.2f, %.2f) atteinte avec angles: épaule=%.1f°, épaule2=%.1f°, coude=%.1f°\n",x, y, z, angles.angleEpaule, angles.angleEpaule2, angles.angleCoude);
     
     return angles;
}