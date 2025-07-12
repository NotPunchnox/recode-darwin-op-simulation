#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h>
#include <stdio.h>

#include "src/motor.h"
#include "src/Kinematic/kinematic.h"

#define TIME_STEP 32
#define MOVEMENT_DURATION 3000  // Durée en ms pour passer d'une position à l'autre
#define HOLD_DURATION 1000      // Durée en ms pour maintenir une position

// Matrice des positions à atteindre [x, y, z]
static const double positions[][3] = {
    {3.0, 0.0, 8.0},   // Position repos centrale
    {3.2, 0.5, 8.2},   // Légère variation droite
    {3.0, 0.0, 8.0},   // Retour au centre
    {3.0, 0.0, 8.0},   // Retour au centre
    {3.0, 0.2, 8.1},   // Légère variation avant
};

#define NUM_POSITIONS (sizeof(positions) / sizeof(positions[0]))

typedef enum {
    MOVING,
    HOLDING
} MovementState;

static void initStableLegs(const RobotMotors* motors) {
    moveMotor(motors->leg.AnkleR, -30);
    moveMotor(motors->leg.AnkleL, -30);
}

static void applyArmMovement(const RobotMotors* motors, double x, double y, double z) {
    struct Angles angles = inverseKinematicARM(x, y, z, 'c');
    
    moveMotor(motors->arm.ShoulderR, angles.angleEpaule);
    moveMotor(motors->arm.ShoulderL, angles.angleEpaule);
    moveMotor(motors->arm.ArmUpperR, angles.angleEpaule2);
    moveMotor(motors->arm.ArmUpperL, angles.angleEpaule2);
    moveMotor(motors->arm.ArmLowerR, angles.angleCoude);
    moveMotor(motors->arm.ArmLowerL, angles.angleCoude);
}

// Fonction d'interpolation avec easing (mouvement plus naturel)
static double smoothInterpolate(double start, double end, double t) {
    // Utilise une fonction d'easing smooth step (3t² - 2t³)
    double smooth_t = t * t * (3.0 - 2.0 * t);
    return start + (end - start) * smooth_t;
}

int main() {
    wb_robot_init();

    RobotMotors motors;
    initMotor(&motors);
    initStableLegs(&motors);

    int elapsed_time = 0;
    int current_position = 0;
    int next_position = 1;
    MovementState state = MOVING;
    
    printf("Démarrage séquence de positions des bras...\n");
    printf("Position initiale: X=%.1f, Y=%.1f, Z=%.1f\n", 
           positions[current_position][0], 
           positions[current_position][1], 
           positions[current_position][2]);

    while (wb_robot_step(TIME_STEP) != -1) {
        double current_x, current_y, current_z;
        
        switch (state) {
            case MOVING: {
                double progress = (double)elapsed_time / MOVEMENT_DURATION;
                
                if (progress >= 1.0) {
                    progress = 1.0;
                    // Transition vers HOLDING
                    state = HOLDING;
                    elapsed_time = 0;
                    current_position = next_position;
                    printf("Position %d atteinte: X=%.1f, Y=%.1f, Z=%.1f - Maintien...\n", 
                           current_position + 1,
                           positions[current_position][0],
                           positions[current_position][1],
                           positions[current_position][2]);
                }
                
                // Interpolation fluide sur tous les axes simultanément
                current_x = smoothInterpolate(positions[current_position][0], 
                                            positions[next_position][0], progress);
                current_y = smoothInterpolate(positions[current_position][1], 
                                            positions[next_position][1], progress);
                current_z = smoothInterpolate(positions[current_position][2], 
                                            positions[next_position][2], progress);
                break;
            }
            
            case HOLDING: {
                // Maintenir la position actuelle
                current_x = positions[current_position][0];
                current_y = positions[current_position][1];
                current_z = positions[current_position][2];
                
                if (elapsed_time >= HOLD_DURATION) {
                    // Transition vers le mouvement suivant
                    next_position = (current_position + 1) % NUM_POSITIONS;
                    state = MOVING;
                    elapsed_time = 0;
                    printf("Mouvement vers position %d: X=%.1f, Y=%.1f, Z=%.1f\n", 
                           next_position + 1,
                           positions[next_position][0],
                           positions[next_position][1],
                           positions[next_position][2]);
                }
                break;
            }
        }
        
        applyArmMovement(&motors, current_x, current_y, current_z);
        elapsed_time += TIME_STEP;
    }
    
    wb_robot_cleanup();
    return 0;
}
