#ifndef KINEMATIC_H
#define KINEMATIC_H

#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Angles {
    double angleEpaule;
    double angleEpaule2;
    double angleCoude;
};

// Variable globale pour conserver les derniers angles valides
extern struct Angles lastValidAngles;

// Fonction pour limiter les angles à des valeurs raisonnables
struct Angles constrainAngles(struct Angles angles);

// Fonction de cinématique inverse pour le bras
struct Angles inverseKinematicARM(double x, double y, double z);

#endif