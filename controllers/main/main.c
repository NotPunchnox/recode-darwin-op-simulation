#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gyro.h>
#include <webots/accelerometer.h>
#include <stdio.h>
#include <math.h>
#include "src/motor.h"

#define TIME_STEP 32
#define LEG 180.0        // Leg length parameter
#define M_PI 3.14159265358979323846

// Structure pour les angles des articulations
struct LegAngles {
    double hipYaw;
    double hipPitch;
    double hipRoll;
    double kneePitch;
    double anklePitch;
    double ankleRoll;
};

// Structure pour l'état du robot
struct RobotState {
    double gyro_angle_x;
    double gyro_angle_y;
    double gyro_vel_x;
    double gyro_vel_y;
    double accel_x;
    double accel_y;
    double accel_z;
};

// Structure pour le contrôle de marche
struct WalkController {
    // Paramètres de base
    double adjFR;        // Adjustment front-rear
    double autoH;        // Auto height
    double autoHs;       // Auto height start
    int mode;            // Mode de contrôle
    int walkF;           // Flag de marche
    
    // Variables de marche
    double dx[2];        // Position avant-arrière des pieds (0=droit, 1=gauche)
    double dy;           // Position latérale
    double dxi;          // Correction avant-arrière
    double dyi;          // Correction latérale
    double fwr0, fwr1;   // Position de référence des pieds
    int fwct;            // Compteur de cycle de marche
    int fwctEnd;         // Fin du cycle de marche
    int jikuasi;         // Pied d'appui (0=droit, 1=gauche)
    double fw;           // Amplitude de marche
    double swf;          // Amplitude de balancement latéral
    double fhMax;        // Hauteur max de levée de pied
    double landRate;     // Taux d'atterrissage
    double fh;           // Hauteur actuelle du pied
    
    // Angles des articulations
    double K0W[2];       // Hip pitch
    double K1W[2];       // Hip roll
    double HW[2];        // Knee pitch
    double A0W[2];       // Ankle pitch
    double A1W[2];       // Ankle roll
    
    // Variables de contrôle UVC
    double fbAV, lrAV;   // Correction gyro
    double fbRad, lrRad; // Angles du gyro
    double asiPress_r, asiPress_l; // Pression des pieds
    int uvcOff;          // Désactiver UVC
};

// Limites des articulations (en degrés)
const double joint_limits[6][2] = {
    {-45.0, 45.0}, {-90.0, 90.0}, {-45.0, 45.0},
    {0.0, 135.0}, {-90.0, 90.0}, {-45.0, 45.0}
};

// Contrôleur de marche global
struct WalkController walker;

// Clamp des angles
double clamp(double angle, double min, double max) {
    return fmax(min, fmin(max, angle));
}

// Initialisation du contrôleur de marche
void init_walk_controller() {
    walker.adjFR = 2.04;
    walker.autoH = 170;
    walker.autoHs = 180;
    walker.mode = 0;
    walker.walkF = 0;
    
    // Initialisation des variables
    walker.dx[0] = walker.dx[1] = 0;
    walker.dy = 0;
    walker.dxi = walker.dyi = 0;
    walker.fwr0 = walker.fwr1 = 0;
    walker.fwct = 0;
    walker.fwctEnd = 48;
    walker.jikuasi = 0;
    walker.fw = 20;
    walker.swf = 12;
    walker.fhMax = 20;
    walker.landRate = 0.2;
    walker.fh = 0;
    
    // Initialisation des angles
    for(int i = 0; i < 2; i++) {
        walker.K0W[i] = 0;
        walker.K1W[i] = 0;
        walker.HW[i] = 0;
        walker.A0W[i] = 0;
        walker.A1W[i] = 0;
    }
    
    walker.fbAV = walker.lrAV = 0;
    walker.fbRad = walker.lrRad = 0;
    walker.asiPress_r = walker.asiPress_l = 0;
    walker.uvcOff = 0;
}

// Contrôle de position des pieds
void foot_control(double x, double y, double h, int s) {
    // x: distance avant-arrière (avant +)
    // y: distance gauche-droite (droite +)
    // h: distance de l'axe de roulis de cheville à l'axe de roulis de hanche
    // s: pied d'appui (0) ou pied oscillant (1)
    
    double k = sqrt(x*x + (y*y + h*h)); // Distance de A0 à K0
    if(k > LEG) k = LEG; // Éviter les erreurs de calcul
    
    x = asin(x/k); // Angle de balancement de jambe K0
    k = acos(k/LEG); // Angle de flexion du genou K0
    
    walker.fbAV = 0; // Désactiver le gyro pour l'évaluation UVC
    walker.lrAV = 0;
    
    walker.K0W[s] = k + x;
    walker.HW[s] = k * 2;
    walker.A0W[s] = k - x - 0.003 * walker.fbAV;
    
    k = atan(y/h); // Angle K1
    walker.K1W[s] = k;
    
    if(s == 0) walker.A1W[s] = -k - 0.002 * walker.lrAV;
    else       walker.A1W[s] = -k + 0.002 * walker.lrAV;
}

// Contrôle principal de marche
void walk_control(struct RobotState *state) {
    int i;
    double k;
    
    // Mise à jour des angles du gyro
    walker.fbRad = state->gyro_angle_x;
    walker.lrRad = state->gyro_angle_y;
    
    switch(walker.mode) {
        
        ////////////////////////
        // Transition vers la posture initiale
        ////////////////////////
        case 0:
            if(walker.autoHs > walker.autoH) walker.autoHs -= 1;
            else walker.mode = 10;
            
            foot_control(-walker.adjFR, 0, walker.autoHs, 0);
            foot_control(-walker.adjFR, 0, walker.autoHs, 1);
            break;
            
        //////////////////////
        // État d'attente
        //////////////////////
        case 10:
            walker.K0W[0] = 0;
            walker.K0W[1] = 0;
            
            // Initialisation des paramètres
            walker.dx[0] = 0;
            walker.dx[1] = 0;
            walker.fwr0 = 0;
            walker.fwr1 = 0;
            walker.fwct = 0;
            walker.dxi = 0;
            walker.dyi = 0;
            walker.dy = 0;
            walker.jikuasi = 0;
            walker.fwctEnd = 48;
            walker.swf = 12;
            walker.fhMax = 20;
            walker.landRate = 0.2;
            walker.fh = 0;
            
            foot_control(-walker.adjFR, 0, walker.autoH, 0);
            foot_control(-walker.adjFR, 0, walker.autoH, 1);
            
            if(walker.walkF & 0x01) {
                walker.fw = 20;
                walker.mode = 20;
            }
            break;
            
        /////////////////////////////////////////////////////////////////
        // Contrôle de marche
        /////////////////////////////////////////////////////////////////
        case 20:
        case 30:
            
            //###########################################################
            // UVC (Contrôle vertical du corps supérieur)
            //###########################################################
            if((walker.jikuasi == 0 && walker.asiPress_r < -0.1 && walker.asiPress_l > -0.1) ||
               (walker.jikuasi == 1 && walker.asiPress_r > -0.1 && walker.asiPress_l < -0.1)) {
                
                k = 1.5 * 193 * sin(walker.lrRad); // Déplacement latéral
                if(walker.jikuasi == 0) walker.dyi += k;
                else walker.dyi -= k;
                
                if(walker.dyi > 0) walker.dyi = 0;
                if(walker.dyi < -30) walker.dyi = -30;
                
                k = 1.5 * 130 * sin(walker.fbRad); // Déplacement avant-arrière
                walker.dxi += k;
            }
            walker.dyi *= 0.90; // Amortissement
            
            if(walker.uvcOff == 1) {
                walker.dxi = 0;
                walker.dyi = 0;
            }
            
            //###########################################################
            // Démarche de base
            //###########################################################
            
            // Balancement latéral
            k = walker.swf * sin(M_PI * walker.fwct / walker.fwctEnd);
            if(walker.jikuasi == 0) walker.dy = k;   // Balancement droit
            else walker.dy = -k;  // Balancement gauche
            
            // Contrôle de balancement avant du pied d'appui
            if(walker.fwct < walker.fwctEnd/2) {
                walker.dx[walker.jikuasi] = walker.fwr0 * (1 - 2.0 * walker.fwct / walker.fwctEnd);
            } else {
                walker.dx[walker.jikuasi] = -(walker.fw - walker.dxi) * (2.0 * walker.fwct / walker.fwctEnd - 1);
            }
            
            // Contrôle de balancement avant du pied oscillant
            if(walker.mode == 20) { // Période de décalage des deux pieds
                if(walker.fwct < (walker.landRate * walker.fwctEnd)) {
                    walker.dx[walker.jikuasi^1] = walker.fwr1 - (walker.fwr0 - walker.dx[walker.jikuasi]);
                } else {
                    walker.fwr1 = walker.dx[walker.jikuasi^1];
                    walker.mode = 30;
                }
            }
            
            if(walker.mode == 30) { // Balancement avant
                k = (-cos(M_PI * (walker.fwct - walker.landRate * walker.fwctEnd) / 
                         ((1 - walker.landRate) * walker.fwctEnd)) + 1) / 2;
                walker.dx[walker.jikuasi^1] = walker.fwr1 + k * (walker.fw - walker.dxi - walker.fwr1);
            }
            
            // Limites d'amplitude de balancement
            if(walker.dx[walker.jikuasi] > 100) {
                walker.dxi -= walker.dx[walker.jikuasi] - 100;
                walker.dx[walker.jikuasi] = 100;
            }
            if(walker.dx[walker.jikuasi] < -100) {
                walker.dxi -= walker.dx[walker.jikuasi] + 100;
                walker.dx[walker.jikuasi] = -100;
            }
            if(walker.dx[walker.jikuasi^1] > 100) walker.dx[walker.jikuasi^1] = 100;
            if(walker.dx[walker.jikuasi^1] < -100) walker.dx[walker.jikuasi^1] = -100;
            
            // Contrôle de levée de pied
            i = walker.landRate * walker.fwctEnd;
            if(walker.fwct > i) {
                walker.fh = walker.fhMax * sin(M_PI * (walker.fwct - i) / (walker.fwctEnd - i));
            } else {
                walker.fh = 0;
            }
            
            // Appel des fonctions de contrôle des jambes
            if(walker.jikuasi == 0) {
                foot_control(walker.dx[0] - walker.adjFR, -walker.dy - walker.dyi + 1, walker.autoH, 0);
                foot_control(walker.dx[1] - walker.adjFR, walker.dy - walker.dyi + 1, walker.autoH - walker.fh, 1);
            } else {
                foot_control(walker.dx[0] - walker.adjFR, -walker.dy - walker.dyi + 1, walker.autoH - walker.fh, 0);
                foot_control(walker.dx[1] - walker.adjFR, walker.dy - walker.dyi + 1, walker.autoH, 1);
            }
            
            //###########################################################
            // CPG (Générateur de cycle de marche, version simplifiée)
            //###########################################################
            if(walker.fwct == walker.fwctEnd) {
                walker.jikuasi ^= 1;
                walker.fwct = 1;
                walker.dxi = 0;
                walker.fwr0 = walker.dx[walker.jikuasi];
                walker.fwr1 = walker.dx[walker.jikuasi^1];
                walker.fh = 0;
                walker.mode = 20;
                
                if(walker.fw == 20) {
                    walker.landRate = 0.1;
                    walker.fw = 40;
                }
            } else {
                ++walker.fwct;
            }
            break;
    }
}

// Conversion des angles de contrôle vers les angles des jambes
void convert_to_leg_angles(struct LegAngles *right_angles, struct LegAngles *left_angles) {
    // Conversion pour la jambe droite (index 0)
    right_angles->hipYaw = 0.0;
    right_angles->hipPitch = walker.K0W[0] * 180.0 / M_PI;
    right_angles->hipRoll = walker.K1W[0] * 180.0 / M_PI;
    right_angles->kneePitch = walker.HW[0] * 180.0 / M_PI;
    right_angles->anklePitch = walker.A0W[0] * 180.0 / M_PI;
    right_angles->ankleRoll = walker.A1W[0] * 180.0 / M_PI;
    
    // Conversion pour la jambe gauche (index 1)
    left_angles->hipYaw = 0.0;
    left_angles->hipPitch = walker.K0W[1] * 180.0 / M_PI;
    left_angles->hipRoll = -walker.K1W[1] * 180.0 / M_PI; // Inversion pour la symétrie
    left_angles->kneePitch = walker.HW[1] * 180.0 / M_PI;
    left_angles->anklePitch = walker.A0W[1] * 180.0 / M_PI;
    left_angles->ankleRoll = -walker.A1W[1] * 180.0 / M_PI; // Inversion pour la symétrie
    
    // Application des limites
    right_angles->hipPitch = clamp(right_angles->hipPitch, joint_limits[1][0], joint_limits[1][1]);
    right_angles->hipRoll = clamp(right_angles->hipRoll, joint_limits[2][0], joint_limits[2][1]);
    right_angles->kneePitch = clamp(right_angles->kneePitch, joint_limits[3][0], joint_limits[3][1]);
    right_angles->anklePitch = clamp(right_angles->anklePitch, joint_limits[4][0], joint_limits[4][1]);
    right_angles->ankleRoll = clamp(right_angles->ankleRoll, joint_limits[5][0], joint_limits[5][1]);
    
    left_angles->hipPitch = clamp(left_angles->hipPitch, joint_limits[1][0], joint_limits[1][1]);
    left_angles->hipRoll = clamp(left_angles->hipRoll, joint_limits[2][0], joint_limits[2][1]);
    left_angles->kneePitch = clamp(left_angles->kneePitch, joint_limits[3][0], joint_limits[3][1]);
    left_angles->anklePitch = clamp(left_angles->anklePitch, joint_limits[4][0], joint_limits[4][1]);
    left_angles->ankleRoll = clamp(left_angles->ankleRoll, joint_limits[5][0], joint_limits[5][1]);
}

// Application des angles aux moteurs
void apply_leg_angles(RobotMotors *motors, struct LegAngles right_angles, struct LegAngles left_angles) {
    // Application des angles aux moteurs de la jambe droite
    moveMotor(motors->leg.LegUpperR, right_angles.hipYaw);
    moveMotor(motors->leg.LegUpperR, right_angles.hipPitch);
    moveMotor(motors->leg.LegLowerR, right_angles.kneePitch);
    moveMotor(motors->leg.AnkleR, right_angles.anklePitch);
    moveMotor(motors->leg.AnkleR, right_angles.ankleRoll);

    // Application des angles aux moteurs de la jambe gauche
    moveMotor(motors->leg.LegUpperL, left_angles.hipYaw);
    moveMotor(motors->leg.LegUpperL, left_angles.hipPitch);
    moveMotor(motors->leg.LegLowerL, left_angles.kneePitch);
    moveMotor(motors->leg.AnkleL, left_angles.anklePitch);
    moveMotor(motors->leg.AnkleL, left_angles.ankleRoll);
}

int main() {
    wb_robot_init();
    
    // Initialisation des moteurs
    RobotMotors motors;
    if (initMotor(&motors) != 0) {
        printf("Erreur d'initialisation des moteurs\n");
        return -1;
    }

    // Initialisation des capteurs
    WbDeviceTag gyro = wb_robot_get_device("Gyro");
    WbDeviceTag accelerometer = wb_robot_get_device("Accelerometer");
    
    if (gyro == 0) {
        printf("Erreur : Capteur gyro non trouvé\n");
        return -1;
    }
    
    wb_gyro_enable(gyro, TIME_STEP);
    if (accelerometer != 0) {
        wb_accelerometer_enable(accelerometer, TIME_STEP);
    }

    // Initialisation du contrôleur de marche
    init_walk_controller();
    
    // Variables d'état
    struct RobotState robot_state = {0};
    struct LegAngles right_angles = {0};
    struct LegAngles left_angles = {0};
    
    // Filtrage des données capteurs
    double gyro_filter_alpha = 0.8;
    double filtered_gyro[3] = {0.0, 0.0, 0.0};
    
    printf("Démarrage du système de marche bipède...\n");
    
    // Attendre 3 secondes puis démarrer la marche
    int start_walking_timer = 3000 / TIME_STEP;
    int timer = 0;
    
    while (wb_robot_step(TIME_STEP) != -1) {
        timer++;
        
        // Lecture des capteurs avec filtrage
        const double *gyro_values = wb_gyro_get_values(gyro);
        filtered_gyro[0] = gyro_filter_alpha * filtered_gyro[0] + (1 - gyro_filter_alpha) * gyro_values[0];
        filtered_gyro[1] = gyro_filter_alpha * filtered_gyro[1] + (1 - gyro_filter_alpha) * gyro_values[1];
        
        // Mise à jour de l'état
        robot_state.gyro_vel_x = filtered_gyro[0];
        robot_state.gyro_vel_y = filtered_gyro[1];
        robot_state.gyro_angle_x += robot_state.gyro_vel_x * TIME_STEP / 1000.0;
        robot_state.gyro_angle_y += robot_state.gyro_vel_y * TIME_STEP / 1000.0;
        
        // Lecture de l'accéléromètre si disponible
        if (accelerometer != 0) {
            const double *accel_values = wb_accelerometer_get_values(accelerometer);
            robot_state.accel_x = accel_values[0];
            robot_state.accel_y = accel_values[1];
            robot_state.accel_z = accel_values[2];
        }
        
        // Démarrer la marche après le délai
        if (timer == start_walking_timer) {
            walker.walkF = 1;
            printf("Démarrage de la marche...\n");
        }
        
        // Exécution du contrôle de marche
        walk_control(&robot_state);
        
        // Conversion et application des angles
        convert_to_leg_angles(&right_angles, &left_angles);
        apply_leg_angles(&motors, right_angles, left_angles);
        
        // Affichage périodique du statut
        if (timer % 1000 == 0) {
            printf("Mode: %d, Cycle: %d/%d, Pied d'appui: %s\n", 
                   walker.mode, walker.fwct, walker.fwctEnd,
                   walker.jikuasi == 0 ? "Droit" : "Gauche");
        }
    }
    
    // Nettoyage
    wb_gyro_disable(gyro);
    if (accelerometer != 0) {
        wb_accelerometer_disable(accelerometer);
    }
    wb_robot_cleanup();
    
    return 0;
}