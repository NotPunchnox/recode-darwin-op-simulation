#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h>
#include <stdio.h>
#include <math.h>

#include "src/motor.h"
#include "src/Kinematic/kinematic.h"

#define TIME_STEP 32

int main() {
    wb_robot_init();

    RobotMotors motors;
    initMotor(&motors);

    // Jambes stables
    moveMotor(motors.leg.LegUpperR, 20);
    moveMotor(motors.leg.LegUpperL, 20);
    moveMotor(motors.leg.LegLowerR, 20);
    moveMotor(motors.leg.LegLowerL, 20);
    moveMotor(motors.leg.AnkleR, -30);
    moveMotor(motors.leg.AnkleL, -30);

    
    printf("=== DÉMONSTRATION DE POSES SIMPLES ===\n");

    while (wb_robot_step(TIME_STEP) != -1) {
        
        // Calcul de la cinématique inverse pour la pose actuelle
        struct Angles angles = inverseKinematicARM(0.0, 12.0, 1.0);
        
        // Application des angles aux moteurs
        moveMotor(motors.arm.ShoulderR, angles.angleEpaule);
        moveMotor(motors.arm.ShoulderL, angles.angleEpaule);
        moveMotor(motors.arm.ArmUpperR, angles.angleEpaule2);
        moveMotor(motors.arm.ArmUpperL, angles.angleEpaule2);
        moveMotor(motors.arm.ArmLowerR, angles.angleCoude);
        moveMotor(motors.arm.ArmLowerL, angles.angleCoude);
    }

    wb_robot_cleanup();
    return 0;
}
