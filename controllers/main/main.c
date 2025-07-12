#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h>
#include <stdio.h>
#include <math.h>

#include "src/motor.h"
#include "src/Kinematic/kinematic.h"

#define TIME_STEP 32
#define POSITION_HOLD_STEPS 62  // ~2 secondes par position (62 * 32ms)

// Matrice des positions à atteindre [x, y, z]
static const double positions[][3] = {
    {5.0, 0.0, 12.0},
    {5.0, 0.0, 15.0},
    {5.0, 0.0, 13.0},
    {5.0, 0.0, 14.0},
    {5.0, 0.0, 12.5},
    {5.0, 0.0, 14.5}
};

#define NUM_POSITIONS (sizeof(positions) / sizeof(positions[0]))

static void initStableLegs(const RobotMotors* motors) {
    moveMotor(motors->leg.AnkleR, -30);
    moveMotor(motors->leg.AnkleL, -30);
}

static void applyArmMovement(const RobotMotors* motors, double x, double y, double z) {
    printf("Position: X=%.1f, Y=%.1f, Z=%.1f\n", x, y, z);
    
    struct Angles angles = inverseKinematicARM(x, y, z, 'c');
    
    moveMotor(motors->arm.ShoulderR, angles.angleEpaule);
    moveMotor(motors->arm.ShoulderL, angles.angleEpaule);
    moveMotor(motors->arm.ArmUpperR, angles.angleEpaule2);
    moveMotor(motors->arm.ArmUpperL, angles.angleEpaule2);
    moveMotor(motors->arm.ArmLowerR, angles.angleCoude);
    moveMotor(motors->arm.ArmLowerL, angles.angleCoude);
}

int main() {
    wb_robot_init();

    RobotMotors motors;
    initMotor(&motors);
    initStableLegs(&motors);

    int step_count = 0;
    int current_position = 0;
    
    printf("Démarrage séquence de positions des bras...\n");

    while (wb_robot_step(TIME_STEP) != -1) {
        // Appliquer la position actuelle
        applyArmMovement(&motors, 
                        positions[current_position][0],
                        positions[current_position][1], 
                        positions[current_position][2]);
        
        step_count++;
        
        // Passer à la position suivante après le délai
        if (step_count >= POSITION_HOLD_STEPS) {
            step_count = 0;
            current_position = (current_position + 1) % NUM_POSITIONS;
            printf("Passage à la position %d\n", current_position + 1);
        }
    }
    
    wb_robot_cleanup();
    return 0;
}
