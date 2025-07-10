#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h>
#include <stdio.h>
#include <math.h>

#include "src/motor.h"

#define TIME_STEP 32

struct Angles {
    double angleEpaule;
    double angleEpaule2;
    double angleCoude;
};

// Variable globale pour conserver les derniers angles valides
struct Angles lastValidAngles = {0.0, 37.0, 0.0};

// Fonction pour limiter les angles à des valeurs raisonnables
struct Angles constrainAngles(struct Angles angles) {
    angles.angleEpaule = fmax(-90.0, fmin(90.0, angles.angleEpaule));
    angles.angleEpaule2 = fmax(0.0, fmin(90.0, angles.angleEpaule2));
    angles.angleCoude = fmax(-150.0, fmin(0.0, angles.angleCoude));
    return angles;
}

// Cinématique inverse 3D
struct Angles inverseKinematicARM(double x, double y, double z) {
    struct Angles angles = lastValidAngles;
    
    double biceps = 6.0;
    double avant_bras = 12.0;
    double distance_totale = sqrt(x*x + y*y + z*z);

    if (distance_totale > biceps + avant_bras || distance_totale < 2.0) {
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

    angles = constrainAngles(angles);
    lastValidAngles = angles;
    return angles;
}

// Démo mouvements sur chaque axe & axes combinés
void axisDemo(double time, double *x, double *y, double *z) {
    double cycle_time = 8.0;
    double phase = fmod(time, cycle_time * 4);
    
    if (phase < cycle_time) {
        // Phase 1: Mouvement sur axe X uniquement
        *x = 5.0 * sin(phase * 2.0 * M_PI / cycle_time);
        *y = 8.0;
        *z = 0.0;
    } else if (phase < cycle_time * 2) {
        // Phase 2: Mouvement sur axe Z uniquement
        *x = 0.0;
        *y = 8.0;
        *z = 5.0 * sin((phase - cycle_time) * 2.0 * M_PI / cycle_time);
    } else if (phase < cycle_time * 3) {
        // Phase 3: Mouvement sur axe Y uniquement
        *x = 0.0;
        *y = 8.0 + 3.0 * sin((phase - cycle_time * 2) * 2.0 * M_PI / cycle_time);
        *z = 0.0;
    } else {
        // Phase 4: Mouvement combiné sur tous les axes
        double t = (phase - cycle_time * 3) * 2.0 * M_PI / cycle_time;
        *x = 4.0 * sin(t);
        *y = 8.0 + 2.0 * sin(t * 1.5);
        *z = 4.0 * cos(t * 0.8);
    }
}

int main() {
    wb_robot_init();

    RobotMotors motors;
    initMotor(&motors);

    // Position de repos
    moveMotor(motors.arm.ShoulderR, 0);
    moveMotor(motors.arm.ShoulderL, 0);
    moveMotor(motors.arm.ArmUpperR, 37);
    moveMotor(motors.arm.ArmUpperL, 37);
    moveMotor(motors.arm.ArmLowerR, -30);
    moveMotor(motors.arm.ArmLowerL, -30);

    // Jambes stables
    moveMotor(motors.leg.LegUpperR, 20);
    moveMotor(motors.leg.LegUpperL, 20);
    moveMotor(motors.leg.LegLowerR, 20);
    moveMotor(motors.leg.LegLowerL, 20);
    moveMotor(motors.leg.AnkleR, -30);
    moveMotor(motors.leg.AnkleL, -30);

    double time = 0.0;
    
    printf("=== DÉMONSTRATION PAR AXES ===\n");
    printf("Phase 1 (0-8s): Mouvement axe X\n");
    printf("Phase 2 (8-16s): Mouvement axe Z\n");
    printf("Phase 3 (16-24s): Mouvement axe Y\n");
    printf("Phase 4 (24-32s): Mouvement combiné\n");

    while (wb_robot_step(TIME_STEP) != -1) {
        time += 0.032;
        
        double x, y, z;
        axisDemo(time, &x, &y, &z);
        
        struct Angles angles = inverseKinematicARM(x, y, z);
        
        moveMotor(motors.arm.ShoulderR, angles.angleEpaule);
        moveMotor(motors.arm.ShoulderL, angles.angleEpaule);
        moveMotor(motors.arm.ArmUpperR, angles.angleEpaule2);
        moveMotor(motors.arm.ArmUpperL, angles.angleEpaule2);
        moveMotor(motors.arm.ArmLowerR, angles.angleCoude);
        moveMotor(motors.arm.ArmLowerL, angles.angleCoude);
        
        // Affichage de la phase actuelle
        int phase = (int)(time / 8.0) % 4;
        if ((int)(time * 5) % 10 == 0) {
            printf("Phase %d | Temps: %.1fs | Position: (%.1f, %.1f, %.1f)\n", 
                   phase + 1, time, x, y, z);
        }
    }

    wb_robot_cleanup();
    return 0;
}
