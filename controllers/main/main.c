#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h>
#include <stdio.h>
#include <math.h>

#include "src/motor.h"
#include "src/Kinematic/kinematic.h"

#define TIME_STEP 32
#define CYCLE_DURATION_MS 2000
#define CYCLE_DURATION_STEPS (CYCLE_DURATION_MS / TIME_STEP)

static void initStableLegs(const RobotMotors* motors) {
    moveMotor(motors->leg.AnkleR, -30);
    moveMotor(motors->leg.AnkleL, -30);
}

static void applyArmMovement(const RobotMotors* motors, double z_angle) {
    printf("Z angle: %.1f degrees\n", z_angle);
    
    // Position fixe pour X et Y, seul Z varie
    double x = 0.0;  // Position fixe
    double y = 0.0;   // Position fixe
    
    struct Angles angles = inverseKinematicARM(x, y, z_angle);
    
    moveMotor(motors->arm.ShoulderR, angles.angleEpaule);
    moveMotor(motors->arm.ShoulderL, angles.angleEpaule);
    moveMotor(motors->arm.ArmUpperR, angles.angleEpaule2);
    moveMotor(motors->arm.ArmUpperL, angles.angleEpaule2);
    moveMotor(motors->arm.ArmLowerR, angles.angleCoude-90);
    moveMotor(motors->arm.ArmLowerL, angles.angleCoude-90);
}

int main() {
    wb_robot_init();

    RobotMotors motors;
    initMotor(&motors);
    initStableLegs(&motors);

    int step_count = 0;
    double z_min = -12.0;  // Angle Z minimum
    double z_max = 12.0;   // Angle Z maximum
    
    printf("Démarrage mouvement cyclique des bras...\n");

    while (wb_robot_step(TIME_STEP) != -1) {
        // Calcul de l'angle Z en fonction du temps (mouvement sinusoïdal)
        double progress = (double)step_count / CYCLE_DURATION_STEPS;
        double z_angle = z_min + (z_max - z_min) * (sin(2 * M_PI * progress) + 1) / 2;
        
        applyArmMovement(&motors, z_angle);
        
        step_count++;
        if (step_count >= CYCLE_DURATION_STEPS) {
            step_count = 0;  // Reset pour boucle continue
        }
    }
    
    wb_robot_cleanup();
    return 0;
}
