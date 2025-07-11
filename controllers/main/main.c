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

    int pose_duration = 3000; // 3 secondes par pose
    int current_pose = 0;
    int step_count = 0;
    bool sequence_complete = false;
    
    // Définition des poses (x, y, z)
    double poses[][3] = {
        {0.0, 8.0, 8.0},
        {0.0, 9.0, 8.0},
        {0.0, 10.0, 8.0},
        {0.0, -5.0, 8.0},
        {0.0, -8.0, 8.0},
        {0.0, -10.0, 8.0},
    };
    int num_poses = sizeof(poses) / sizeof(poses[0]);
    
    printf("=== DÉMONSTRATION DE POSES SIMPLES ===\n");
    printf("7 poses différentes, 3 secondes chacune\n");
    printf("Pose 1: Bras tendu devant\n");
    printf("Pose 2: Bras vers la droite\n");
    printf("Pose 3: Bras vers la gauche\n");
    printf("Pose 4: Bras vers le haut\n");
    printf("Pose 5: Bras vers le bas\n");
    printf("Pose 6: Position diagonale\n");
    printf("Pose 7: Position proche\n\n");
    
    printf("Début avec pose %d: (%.1f, %.1f, %.1f)\n", 
           current_pose + 1, 
           poses[current_pose][0], 
           poses[current_pose][1], 
           poses[current_pose][2]);

    while (wb_robot_step(TIME_STEP) != -1) {
        if (!sequence_complete) {
            step_count++;
            
            // Changer de pose toutes les 3 secondes
            if (step_count >= pose_duration / TIME_STEP) {
                current_pose++;
                if (current_pose >= num_poses) {
                    sequence_complete = true;
                    printf("Séquence terminée !\n");
                } else {
                    step_count = 0;
                    printf("Changement vers pose %d: (%.1f, %.1f, %.1f)\n", 
                           current_pose + 1, 
                           poses[current_pose][0], 
                           poses[current_pose][1], 
                           poses[current_pose][2]);
                }
            }
            
            if (!sequence_complete) {
                // Calcul de la cinématique inverse pour la pose actuelle
                struct Angles angles = inverseKinematicARM(poses[current_pose][0], poses[current_pose][1], poses[current_pose][2]);
                
                // Application des angles aux moteurs
                moveMotor(motors.arm.ShoulderR, angles.angleEpaule);
                moveMotor(motors.arm.ShoulderL, angles.angleEpaule);
                moveMotor(motors.arm.ArmUpperR, angles.angleEpaule2);
                moveMotor(motors.arm.ArmUpperL, angles.angleEpaule2);
                moveMotor(motors.arm.ArmLowerR, angles.angleCoude);
                moveMotor(motors.arm.ArmLowerL, angles.angleCoude);
            }
        }
    }

    wb_robot_cleanup();
    return 0;
}
