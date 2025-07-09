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

// Only 2d for this time
struct Angles inverseKinematicLeg(double x, double y, double z) {
     // cinématique inversée jambe
    int femur = 9.3;
    int tibia = 9.3;
    double LF = 3.35;

    double max_length = femur + tibia + LF;
    
    if (y > max_length) {
        printf("Error: Position out of reach.\n");
    }

    //double original_x = x;
    x = x - LF;

    // Hypoténus formé par les coordonnées x;z
    double h = sqrt(x * x + z * z);

    // angle formé par les coordonnées x;z
    double A1 = atan2(z, x);

    // Angle formé près du sol
    double A2 = acos((tibia * tibia + h * h - femur * femur) / (2 * tibia * h));

    // angle coordonnées proche du bassin
    double A3 = acos((femur * femur + h * h - tibia * tibia) / (2 * femur * h));

    // angle coordonnées proche du bassin
    double A4 = acos((z * z + h * h - x * x) / (2 * z * h));

    // angle du genou
    double A5 = acos((femur * femur + tibia * tibia - h * h) / (2 * femur * tibia));

    double angleThigh = A3 + A4;
    double angleKnee = 180 - A5;
    double angleFoot = A2;

    // Convertir les angles en degrés
    struct Angles angles;
    angles.angleThigh = angleThigh * 180.0 / M_PI;
    angles.angleKnee = angleKnee * 180.0 / M_PI;
    angles.angleFoot = angleFoot * 180.0 / M_PI;

    // Afficher les angles
    printf("Inverse Kinematics Angles:\n");
    printf("Thigh Angle: %.2f degrees\n", angles.angleThigh);
    printf("Knee Angle: %.2f degrees\n", angles.angleKnee);
    printf("Foot Angle: %.2f degrees\n", angles.angleFoot);

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

    double x = 5.0, y = 0.0, z = 10.0;

    // Inverse kinematics example
    struct Angles legAngles = inverseKinematicLeg(x, y, z);
    if (legAngles.angleThigh != 0 || legAngles.angleKnee != 0 || legAngles.angleFoot != 0) {
        moveMotor(motors.leg.LegUpperR, legAngles.angleThigh);
        moveMotor(motors.leg.LegLowerR, legAngles.angleKnee);
        moveMotor(motors.leg.AnkleR, legAngles.angleFoot);
    } else {
        printf("Inverse kinematics failed.\n");
    }


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