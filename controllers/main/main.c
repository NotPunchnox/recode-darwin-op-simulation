#include <webots/robot.h>
#include <webots/motor.h>
#include <stdio.h>
#include <math.h>

#include "src/motor.h"
#include "src/Kinematic/kinematic.h"

#define ANIMATION_FRAMES 5
int TIME_STEP = 32; // Temps d'itération en ms

// Positions corrigées pour un robot de cette taille
double target_positions[ANIMATION_FRAMES][3] = {
    {8.0, 0.0, 2.0},       // Frame 0: Position initiale (bras vers l'avant)
    {6.0, 3.0, 4.0},       // Frame 1: Lever les bras (position atteignable)
    {4.0, 2.0, 6.0},       // Frame 2: Position haute
    {6.0, -2.0, 3.0},      // Frame 3: Côté opposé
    {8.0, 0.0, 2.0}        // Frame 4: Retour position initiale
};

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

    printf("=== DÉMONSTRATION AVEC CINÉMATIQUE INVERSE CORRIGÉE ===\n");

    // Animation loop avec cinématique inverse
    for (int frame = 0; frame < ANIMATION_FRAMES; frame++) {
        printf("Frame %d\n", frame);
        
        // Calcul cinématique inverse pour bras droit
        struct Angles anglesR = inverseKinematicARM(target_positions[frame][0], 
                                                   target_positions[frame][1], 
                                                   target_positions[frame][2]);
        
        // Calcul cinématique inverse pour bras gauche (symétrie en Y)
        struct Angles anglesL = inverseKinematicARM(target_positions[frame][0], 
                                                   -target_positions[frame][1], 
                                                   target_positions[frame][2]);
        
        // Application des angles calculés (CORRECTION PRINCIPALE)
        moveMotor(motors.arm.ShoulderR, anglesR.shoulderPitch);     // Épaule droite pitch
        moveMotor(motors.arm.ShoulderL, anglesL.shoulderPitch);     // Épaule gauche pitch
        moveMotor(motors.arm.ArmUpperR, anglesR.shoulderYaw);       // Épaule droite yaw
        moveMotor(motors.arm.ArmUpperL, -anglesL.shoulderYaw);      // Épaule gauche yaw (symétrie)
        moveMotor(motors.arm.ArmLowerR, anglesR.elbow);             // Coude droit
        moveMotor(motors.arm.ArmLowerL, anglesL.elbow);             // Coude gauche

        // Attendre entre les frames
        for (int i = 0; i < 50; i++) {
            wb_robot_step(TIME_STEP);
        }
    }

    wb_robot_cleanup();
    return 0;
}