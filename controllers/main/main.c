#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h>
#include <stdio.h>
#include <math.h>

#include "src/motor.h"

#define TIME_STEP 32

struct Angles {
    double angleEpaule;
    double angleEpaule2;
    double angleCoude;
};

// Variable globale pour conserver les derniers angles valides
struct Angles lastValidAngles = {0.0, 37.0, 0.0};

// Fonction pour limiter les angles à des valeurs raisonnables
struct Angles constrainAngles(struct Angles angles) {
    angles.angleEpaule = fmax(-90.0, fmin(90.0, angles.angleEpaule));
    angles.angleEpaule2 = fmax(0.0, fmin(90.0, angles.angleEpaule2));
    angles.angleCoude = fmax(-150.0, fmin(0.0, angles.angleCoude));
    return angles;
}

// Cinématique inverse 3D
struct Angles inverseKinematicARM(double x, double y, double z) {
    struct Angles angles = lastValidAngles;
    
    double biceps = 6.0;
    double avant_bras = 12.0;
    double distance_totale = sqrt(x*x + y*y + z*z);

    if (distance_totale > biceps + avant_bras || distance_totale < 2.0) {
        return lastValidAngles;
    }

    double distance_horizontale = sqrt(x*x + y*y);
    
    if (distance_horizontale > 0.001) {
        double angle_rotation = atan2(x, y) * 180.0 / M_PI;
        angles.angleEpaule2 = fmax(0.0, fmin(90.0, 37.0 + angle_rotation * 0.5));
    } else {
        angles.angleEpaule2 = 37.0;
    }

    double distance_verticale = sqrt(distance_horizontale*distance_horizontale + z*z);

    double cos_angle_coude = (biceps*biceps + avant_bras*avant_bras - distance_verticale*distance_verticale) / (2 * biceps * avant_bras);
    cos_angle_coude = fmax(-1.0, fmin(1.0, cos_angle_coude));
    double angle_coude = acos(cos_angle_coude);
    angles.angleCoude = 90.0 - (180.0 - (angle_coude * 180.0 / M_PI));

    double angle_vers_cible = atan2(z, distance_horizontale) * 180.0 / M_PI;
    double cos_angle_epaule = (biceps*biceps + distance_verticale*distance_verticale - avant_bras*avant_bras) / (2 * biceps * distance_verticale);
    cos_angle_epaule = fmax(-1.0, fmin(1.0, cos_angle_epaule));
    double angle_epaule_offset = acos(cos_angle_epaule) * 180.0 / M_PI;
    angles.angleEpaule = angle_epaule_offset - angle_vers_cible;

    angles = constrainAngles(angles);
    lastValidAngles = angles;
    return angles;
}

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
    
    // Définition des poses (x, y, z)
    double poses[][3] = {
        {0.0, 10.0, 0.0},    // Pose 1: Bras tendu devant
        {6.0, 8.0, 0.0},     // Pose 2: Bras vers la droite
        {-6.0, 8.0, 0.0},    // Pose 3: Bras vers la gauche
        {0.0, 8.0, 8.0},     // Pose 4: Bras vers le haut
        {0.0, 8.0, -6.0},    // Pose 5: Bras vers le bas
        {4.0, 12.0, 4.0},    // Pose 6: Position diagonale
        {0.0, 6.0, 0.0}      // Pose 7: Position proche
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

    while (wb_robot_step(TIME_STEP) != -1) {
        step_count++;
        
        // Changer de pose toutes les 3 secondes
        if (step_count >= pose_duration / TIME_STEP) {
            current_pose = (current_pose + 1) % num_poses;
            step_count = 0;
            printf("Changement vers pose %d: (%.1f, %.1f, %.1f)\n", 
                   current_pose + 1, 
                   poses[current_pose][0], 
                   poses[current_pose][1], 
                   poses[current_pose][2]);
        }
        
        // Calcul de la cinématique inverse pour la pose actuelle
        struct Angles angles = inverseKinematicARM(poses[current_pose][0], 
                                                   poses[current_pose][1], 
                                                   poses[current_pose][2]);
        
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
