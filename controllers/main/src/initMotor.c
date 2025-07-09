#include "motor.h"

static const char *motorNames[20] = {
    "ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL", "ArmLowerR",
    "ArmLowerL", "PelvYR", "PelvYL", "PelvR", "PelvL",
    "LegUpperR", "LegUpperL", "LegLowerR", "LegLowerL", "AnkleR",
    "AnkleL", "FootR", "FootL", "Neck", "Head"
};

int initMotor(RobotMotors *motors) {
    // Assigner les moteurs aux tags correspondants
    motors->arm.ShoulderR = wb_robot_get_device(motorNames[0]);
    motors->arm.ShoulderL = wb_robot_get_device(motorNames[1]);
    motors->arm.ArmUpperR = wb_robot_get_device(motorNames[2]);
    motors->arm.ArmUpperL = wb_robot_get_device(motorNames[3]);
    motors->arm.ArmLowerR = wb_robot_get_device(motorNames[4]);
    motors->arm.ArmLowerL = wb_robot_get_device(motorNames[5]);
    motors->pelvis.PelvYR = wb_robot_get_device(motorNames[6]);
    motors->pelvis.PelvYL = wb_robot_get_device(motorNames[7]);
    motors->pelvis.PelvR = wb_robot_get_device(motorNames[8]);
    motors->pelvis.PelvL = wb_robot_get_device(motorNames[9]);
    motors->leg.LegUpperR = wb_robot_get_device(motorNames[10]);
    motors->leg.LegUpperL = wb_robot_get_device(motorNames[11]);
    motors->leg.LegLowerR = wb_robot_get_device(motorNames[12]);
    motors->leg.LegLowerL = wb_robot_get_device(motorNames[13]);
    motors->leg.AnkleR = wb_robot_get_device(motorNames[14]);
    motors->leg.AnkleL = wb_robot_get_device(motorNames[15]);
    motors->leg.FootR = wb_robot_get_device(motorNames[16]);
    motors->leg.FootL = wb_robot_get_device(motorNames[17]);
    motors->head.Neck = wb_robot_get_device(motorNames[18]);
    motors->head.Head = wb_robot_get_device(motorNames[19]);

    // Vérifier si tous les moteurs sont correctement initialisés
    WbDeviceTag *allMotors[] = {
        &motors->arm.ShoulderR, &motors->arm.ShoulderL, &motors->arm.ArmUpperR, &motors->arm.ArmUpperL,
        &motors->arm.ArmLowerR, &motors->arm.ArmLowerL, &motors->pelvis.PelvYR, &motors->pelvis.PelvYL,
        &motors->pelvis.PelvR, &motors->pelvis.PelvL, &motors->leg.LegUpperR, &motors->leg.LegUpperL,
        &motors->leg.LegLowerR, &motors->leg.LegLowerL, &motors->leg.AnkleR, &motors->leg.AnkleL,
        &motors->leg.FootR, &motors->leg.FootL, &motors->head.Neck, &motors->head.Head
    };
    for (int i = 0; i < 20; i++) {
        if (*allMotors[i] == 0) {
            fprintf(stderr, "Error: Device '%s' not found.\n", motorNames[i]);
            wb_robot_cleanup();
            return -1;
        }
    }
    return 0;
}