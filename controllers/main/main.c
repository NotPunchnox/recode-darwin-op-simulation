#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h>
#include <stdio.h>
#include <math.h>

#include "src/motor.h"

#define TIME_STEP 32

// Only 2d for this time
void inverseKinematicLeg(double x, double y, double z) {
     // cinématique inversée jambe
    int femur = 9.3;
    int tibia = 9.3;
    double LF = 3.35;

    double max_length = femur + tibia + LF;
    
    if (y > max_length) {
        printf("Error: Position out of reach.\n");
        return;
    }

    double original_x = x;
    x = x - LF;

    // Hypoténus formé par les coordonnées x;y
    double h = sqrt(x * x + y * y);

    // angle formé par les coordonnées x;y
    double A1 = atan2(y, x);

    // Angle formé près du sol
    double A2 = acos((tibia * tibia + h * h - femur * femur) / (2 * tibia * h));

    // angle coordonnées proche du bassin
    double A3 = acos((femur * femur + h * h - tibia * tibia) / (2 * femur * h));

    // angle coordonnées proche du bassin
    double A4 = acos((y * y + h * h - x * x) / (2 * y * h));

    // angle du genou
    double A5 = acos((femur * femur + tibia * tibia - h * h) / (2 * femur * tibia));

    double angleThigh = A3 + A4;
    double angleKnee = A5;

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