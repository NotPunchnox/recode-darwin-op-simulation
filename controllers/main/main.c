#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h>
#include <stdio.h>
#include <math.h>

#include "src/motor.h"

#define TIME_STEP 32

struct Angles {
    double angleThigh;
    double angleKnee;
    double angleFoot;
};

int femur = 9.3;
int tibia = 9.3;
double LF = 3.35;

// Only 2d for this time
// Cinématique inverse
struct Angles inverseKinematicLeg(double x, double y, double z) {
    struct Angles angles = {0.0, 0.0, 0.0}; // Initialiser à 0
    const double femur = 9.3, tibia = 9.3, LF = 3.35;
    const double max_length = femur + tibia + LF;

    // Vérifier si la position est atteignable
    double dist = sqrt(x*x + z*z);
    if (dist > max_length || dist < fabs(femur - tibia)) {
        printf("Error: Position (%.2f, %.2f, %.2f) out of reach.\n", x, y, z);
        return angles;
    }

    x = x - LF; // Ajuster pour la longueur du pied
    double h = sqrt(x*x + z*z);
    if (h < 0.01) { // Éviter division par zéro
        printf("Error: Invalid h value.\n");
        return angles;
    }

    double A1 = atan2(z, x);
    double A2 = acos((tibia*tibia + h*h - femur*femur) / (2*tibia*h));
    double A3 = acos((femur*femur + h*h - tibia*tibia) / (2*femur*h));
    double A4 = acos((z*z + h*h - x*x) / (2*z*h));
    double A5 = acos((femur*femur + tibia*tibia - h*h) / (2*femur*tibia));

    // Vérifier si les calculs sont valides
    if (isnan(A1) || isnan(A2) || isnan(A3) || isnan(A4) || isnan(A5)) {
        printf("Error: Invalid angles calculated.\n");
        return angles;
    }

    angles.angleThigh = (A3 + A4) * 180.0 / M_PI;
    angles.angleKnee = 180.0 - (A5 * 180.0 / M_PI); // Inverser l'angle du genou
    angles.angleFoot = (A1 + A2) * 180.0 / M_PI;

    // Limiter les angles (ajusté pour le genou)
    if (angles.angleThigh < -90 || angles.angleThigh > 90 ||
        angles.angleKnee < -150 || angles.angleKnee > 150 || // Limites typiques pour Dynamixel MX-28
        angles.angleFoot < -90 || angles.angleFoot > 90) {
        printf("Error: Angles out of motor limits (Thigh: %.2f, Knee: %.2f, Foot: %.2f).\n", 
               angles.angleThigh, angles.angleKnee, angles.angleFoot);
        angles.angleThigh = 0.0;
        angles.angleKnee = 0.0;
        angles.angleFoot = 0.0;
        return angles;
    }

    printf("Inverse Kinematics Angles:\n");
    printf("Thigh: %.2f, Knee: %.2f, Foot: %.2f\n", angles.angleThigh, angles.angleKnee, angles.angleFoot);
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
    moveMotor(motors.arm.ArmUpperR, 30);
    moveMotor(motors.arm.ArmUpperL, 30);

    // bras inférieurs
    moveMotor(motors.arm.ArmLowerR, 70);
    moveMotor(motors.arm.ArmLowerL, 70);

    // bassin
    moveMotor(motors.pelvis.PelvYR, 0);
    moveMotor(motors.pelvis.PelvYL, 0);
    moveMotor(motors.pelvis.PelvR, 0);
    moveMotor(motors.pelvis.PelvL, 0);

    // jambes supérieures
    moveMotor(motors.leg.LegUpperR, 0);
    moveMotor(motors.leg.LegUpperL, 25);

    // jambes inférieures
    moveMotor(motors.leg.LegLowerR, 0);
    moveMotor(motors.leg.LegLowerL, 20);

    //chevilles
    moveMotor(motors.leg.AnkleR, 0);
    // moveMotor(motors.leg.AnkleL, 10);

    // pieds
    moveMotor(motors.leg.FootR, 0);
    moveMotor(motors.leg.FootL, 0);

    // cou
    moveMotor(motors.head.Neck, 0);

    // tête
    moveMotor(motors.head.Head, 20);

    double x = 0.1, y = 0.0, z = 15.0;

    // Inverse kinematics example
    struct Angles legAngles = inverseKinematicLeg(x, y, z);
    moveMotor(motors.leg.LegUpperR, legAngles.angleThigh);
    moveMotor(motors.leg.LegLowerR, legAngles.angleKnee);
    moveMotor(motors.leg.AnkleR, legAngles.angleFoot);


    // Simulation loop
    while (wb_robot_step(TIME_STEP) != -1) {
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