#include <webots/robot.h>
#include <webots/motor.h>
#include <stdio.h>
#include <math.h>
#include "src/motor.h"
#include "src/Kinematic/kinematic.h"

#define TIME_STEP 32

int main() {
    wb_robot_init();
    
    // Initialisation des moteurs
    RobotMotors motors;
    if (initMotor(&motors) != 0) {
        printf("Erreur d'initialisation des moteurs\n");
        return -1;
    }

    printf("Démarrage du contrôle de bras avec cinématique inverse...\n");
    
    double time = 0.0;
    double base_z = 10.0;  // Position Z de base
    double amplitude = 5.0; // Amplitude du mouvement
    double frequency = 0.5; // Fréquence du mouvement
    
    while (wb_robot_step(TIME_STEP) != -1) {
        // Calcul de la position Z oscillante
        double z_position = base_z + amplitude * sin(2 * M_PI * frequency * time);
        printf("Temps: %.2f s, Position Z: %.2f cm\n", time, z_position);
        // Calcul de la cinématique inverse avec Z variable
        struct Angles new_angles = inverseKinematicARM(0.0, 0.0, 15.0);
        moveMotor(motors.arm.ShoulderR, new_angles.angleEpaule);
        moveMotor(motors.arm.ArmUpperR, new_angles.angleEpaule2-37);
        moveMotor(motors.arm.ArmLowerR, new_angles.angleCoude);
        
        // Incrément du temps
        time += TIME_STEP / 1000.0;
    }
    
    wb_robot_cleanup();
    return 0;
}
