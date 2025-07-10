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

// Only 2d for this time
// Cinématique inverse
struct Angles inverseKinematicARM(double x, double y, double z) {
    struct Angles angles = lastValidAngles; // Initialiser avec les derniers angles valides
    
    double biceps = 6.0;
    double avant_bras = 12.0;

    // z: hauteur
    // x: distance horizontale
    // y: distance latérale

    double distance = sqrt(y*y + z*z);

    if (distance > biceps + avant_bras) {
        printf("Impossible de trouver une solution, la distance est trop grande.\n");
        return lastValidAngles; // Retourne les derniers angles valides
    }

    if (distance < fabs(biceps - avant_bras)) {
        printf("Impossible de trouver une solution, la distance est trop petite.\n");
        return lastValidAngles; // Retourne les derniers angles valides
    }

    // Calculer l'angle du biceps avec vérification des bornes
    double cos_angle_biceps = (biceps*biceps + avant_bras*avant_bras - distance*distance) / (2 * biceps * avant_bras);
    cos_angle_biceps = fmax(-1.0, fmin(1.0, cos_angle_biceps));
    double angle_biceps = acos(cos_angle_biceps);
    angles.angleCoude = 90-(180-(angle_biceps * 180.0 / M_PI));

    // Calculer l'angle de l'épaule avec atan2 pour gérer les quadrants
    double a1 = atan2(y, z) * 180.0 / M_PI;
    
    double cos_a2 = (biceps*biceps + distance*distance - avant_bras*avant_bras) / (2 * biceps * distance);
    cos_a2 = fmax(-1.0, fmin(1.0, cos_a2));
    double a2 = acos(cos_a2) * 180.0 / M_PI;
    
    angles.angleEpaule = a2 - a1;
    angles.angleEpaule2 = 37;

    // Sauvegarder les angles valides
    lastValidAngles = angles;

    return angles;
}

int main() {
    wb_robot_init();

    // Init motors
    RobotMotors motors;
    initMotor(&motors);

    wb_keyboard_enable(TIME_STEP);

    // Initialiser les moteurs à leur position par défaut

    // épaules
    moveMotor(motors.arm.ShoulderR, 0);
    moveMotor(motors.arm.ShoulderL, 0);

    // bras supérieurs
    moveMotor(motors.arm.ArmUpperR, 0);
    moveMotor(motors.arm.ArmUpperL, 0);

    // bras inférieurs
    moveMotor(motors.arm.ArmLowerR, 0);
    moveMotor(motors.arm.ArmLowerL, 0);

    // bassin
    moveMotor(motors.pelvis.PelvYR, 0);
    moveMotor(motors.pelvis.PelvYL, 0);
    moveMotor(motors.pelvis.PelvR, 0);
    moveMotor(motors.pelvis.PelvL, 0);

    // jambes supérieures
    moveMotor(motors.leg.LegUpperR, 20);
    moveMotor(motors.leg.LegUpperL, 20);

    // jambes inférieures
    moveMotor(motors.leg.LegLowerR, 20);
    moveMotor(motors.leg.LegLowerL, 20);

    //chevilles
    moveMotor(motors.leg.AnkleR, -30);
    moveMotor(motors.leg.AnkleL, -30);

    // pieds
    moveMotor(motors.leg.FootR, 0);
    moveMotor(motors.leg.FootL, 0);

    // cou
    moveMotor(motors.head.Neck, 0);

    // tête
    moveMotor(motors.head.Head, 20);

    double x = 0.0, y = 5.0, z = 0.0;
    double z_min = -12.0, z_max = 14.0;
    double z_speed = 0.1;
    int z_direction = 1; // 1 pour monter, -1 pour descendre

    // Simulation loop
    while (wb_robot_step(TIME_STEP) != -1) {
        // Animation de z
        z += z_speed * z_direction;
        
        // Inverser la direction quand on atteint les limites
        if (z >= z_max) {
            z_direction = -1;
        } else if (z <= z_min) {
            z_direction = 1;
        }

        // Calcul de la cinématique inverse
        struct Angles angles = inverseKinematicARM(x, y, z);
        
        // Appliquer les angles aux moteurs
        moveMotor(motors.arm.ShoulderR, angles.angleEpaule);
        moveMotor(motors.arm.ShoulderL, angles.angleEpaule);

        moveMotor(motors.arm.ArmUpperR, angles.angleEpaule2);
        moveMotor(motors.arm.ArmUpperL, angles.angleEpaule2);

        moveMotor(motors.arm.ArmLowerL, angles.angleCoude);
        moveMotor(motors.arm.ArmLowerR, angles.angleCoude);

        // Afficher les valeurs
        if ((int)(z * 10) % 50 == 0) {
            printf("z: %.2f, Angles - Epaule: %.2f, Epaule2: %.2f, Coude: %.2f\n", z, angles.angleEpaule, angles.angleEpaule2, angles.angleCoude);
        }

        int key = wb_keyboard_get_key();
        if (key == WB_KEYBOARD_UP) {
            moveMotor(motors.leg.LegUpperR, wb_motor_get_target_position(motors.leg.LegUpperR) * 180.0 / M_PI + 10);
        } else if (key == WB_KEYBOARD_DOWN) {
            moveMotor(motors.leg.LegUpperR, wb_motor_get_target_position(motors.leg.LegUpperR) * 180.0 / M_PI - 10);
        }
    }

    wb_robot_cleanup();
    return 0;
}