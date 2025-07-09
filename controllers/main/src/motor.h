#ifndef MOTOR_H
#define MOTOR_H

#include <webots/motor.h>
#include <webots/robot.h>
#include <stdio.h>

// Définir les groupes de moteurs
typedef struct {
    WbDeviceTag ShoulderR;
    WbDeviceTag ShoulderL;
    WbDeviceTag ArmUpperR;
    WbDeviceTag ArmUpperL;
    WbDeviceTag ArmLowerR;
    WbDeviceTag ArmLowerL;
} ArmMotors;

typedef struct {
    WbDeviceTag PelvYR;
    WbDeviceTag PelvYL;
    WbDeviceTag PelvR;
    WbDeviceTag PelvL;
} PelvisMotors;

typedef struct {
    WbDeviceTag LegUpperR;
    WbDeviceTag LegUpperL;
    WbDeviceTag LegLowerR;
    WbDeviceTag LegLowerL;
    WbDeviceTag AnkleR;
    WbDeviceTag AnkleL;
    WbDeviceTag FootR;
    WbDeviceTag FootL;
} LegMotors;

typedef struct {
    WbDeviceTag Neck;
    WbDeviceTag Head;
} HeadMotors;

typedef struct {
    ArmMotors arm;
    PelvisMotors pelvis;
    LegMotors leg;
    HeadMotors head;
} RobotMotors;


// Fonction pour init les moteurs
int initMotor(RobotMotors *motors);

// Fonction pour bouger les moteurs
void moveMotor(WbDeviceTag motor, double degree);

#endif // MOTOR_H