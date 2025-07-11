#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h>
#include <stdio.h>
#include <math.h>

#include "src/motor.h"
#include "src/Kinematic/kinematic.h"

#define TIME_STEP 32
#define POSE_DURATION_MS 3000
#define POSE_DURATION_STEPS (POSE_DURATION_MS / TIME_STEP)

typedef struct {
    double x, y, z;
} Pose3D;

static const Pose3D poses[] = {
    {14.0, 0.0, 0.0},
    {13.0, 0.0, 0.0},
    {12.0, 0.0, 0.0},
    {11.0, 0.0, 0.0},
};
static const int num_poses = sizeof(poses) / sizeof(poses[0]);

static void initStableLegs(const RobotMotors* motors) {
    moveMotor(motors->leg.AnkleR, -30);
    moveMotor(motors->leg.AnkleL, -30);
}

static void applyPose(const RobotMotors* motors, const Pose3D* pose, int pose_index) {
    printf("Pose %d: (%.1f, %.1f, %.1f)\n", pose_index + 1, pose->x, pose->y, pose->z);
    
    struct Angles angles = inverseKinematicARM(pose->x, pose->y, pose->z);
    
    moveMotor(motors->arm.ShoulderR, angles.angleEpaule*-1);
    moveMotor(motors->arm.ShoulderL, angles.angleEpaule*-1);
    moveMotor(motors->arm.ArmUpperR, angles.angleEpaule2);
    moveMotor(motors->arm.ArmUpperL, angles.angleEpaule2);
    moveMotor(motors->arm.ArmLowerR, angles.angleCoude*-1);
    moveMotor(motors->arm.ArmLowerL, angles.angleCoude*-1);
}

int main() {
    wb_robot_init();

    RobotMotors motors;
    initMotor(&motors);
    initStableLegs(&motors);

    int current_pose = 0;
    int step_count = 0;
    bool pose_applied = false;
    
    printf("Démarrage séquence de %d poses...\n", num_poses);

    while (wb_robot_step(TIME_STEP) != -1 && current_pose < num_poses) {
        if (!pose_applied) {
            applyPose(&motors, &poses[current_pose], current_pose);
            pose_applied = true;
        }
        
        if (++step_count >= POSE_DURATION_STEPS) {
            current_pose++;
            step_count = 0;
            pose_applied = false;
        }
    }
    
    printf("Séquence terminée !\n");
    wb_robot_cleanup();
    return 0;
}
