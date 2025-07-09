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
    moveMotor(motors.arm.ArmUpperR, 0);
    moveMotor(motors.arm.ArmUpperL, 0);

    // bras inférieurs
    moveMotor(motors.arm.ArmLowerR, 30);
    moveMotor(motors.arm.ArmLowerL, 0);

    // Simulation loop
    while (wb_robot_step(TIME_STEP) != -1) {
        int key = wb_keyboard_get_key();
        if (key == WB_KEYBOARD_UP) {
            moveMotor(motors.arm.ShoulderL, wb_motor_get_target_position(motors.arm.ShoulderL) * 180.0 / M_PI + 10);
        } else if (key == WB_KEYBOARD_DOWN) {
            moveMotor(motors.arm.ShoulderL, wb_motor_get_target_position(motors.arm.ShoulderL) * 180.0 / M_PI - 10);
        }
    }

    wb_robot_cleanup();
    return 0;
}