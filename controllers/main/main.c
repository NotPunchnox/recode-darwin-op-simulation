#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h>
#include <stdio.h>
#include <math.h>

#include "src/motor.h"

#define TIME_STEP 32


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
    moveMotor(motors.leg.LegUpperR, 25);
    moveMotor(motors.leg.LegUpperL, 25);

    // jambes inférieures
    moveMotor(motors.leg.LegLowerR, 20);
    moveMotor(motors.leg.LegLowerL, 20);

    //chevilles
    // moveMotor(motors.leg.AnkleR, 10);
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