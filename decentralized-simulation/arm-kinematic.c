#include <math.h>
#include <stdio.h>
#include <string.h>

// Structure pour les paramètres DH fixes
typedef struct {
    double d;
    double a;
    double alpha;
} DH_fixed;

// Paramètres DH pour le bras droit (à ajuster selon le fichier proto Webots)
DH_fixed DH_arm_right[3] = {
    {122.2e-3, 0.0, M_PI / 2}, // Joint 1: shoulder yaw, d1 = Ltx, a1 = 0, alpha1 = 90°
    {0.0, 60.0e-3, 0.0},       // Joint 2: shoulder pitch, d2 = 0, a2 = L2, alpha2 = 0
    {0.0, 16.0e-3, 0.0}        // Joint 3: elbow pitch, d3 = 0, a3 = Ll, alpha3 = 0
};

// Limites des articulations (supposées, à ajuster)
double joint_limits[3][2] = {
    {-M_PI / 2, M_PI / 2}, // theta1: ±90°
    {-M_PI / 2, M_PI / 2}, // theta2: ±90°
    {-M_PI / 2, M_PI / 2}  // theta3: ±90°
};

// Fonction pour calculer la matrice de transformation DH
void get_T(double theta, double d, double a, double alpha, double T[4][4]) {
    double ct = cos(theta);
    double st = sin(theta);
    double ca = cos(alpha);
    double sa = sin(alpha);
    T[0][0] = ct;        T[0][1] = -st * ca;  T[0][2] = st * sa;   T[0][3] = a * ct;
    T[1][0] = st;        T[1][1] = ct * ca;   T[1][2] = -ct * sa;  T[1][3] = a * st;
    T[2][0] = 0.0;       T[2][1] = sa;        T[2][2] = ca;        T[2][3] = d;
    T[3][0] = 0.0;       T[3][1] = 0.0;       T[3][2] = 0.0;       T[3][3] = 1.0;
}

// Fonction pour multiplier deux matrices 4x4
void matrix_multiply(double A[4][4], double B[4][4], double C[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            C[i][j] = 0.0;
            for (int k = 0; k < 4; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

// Cinématique directe : calcule la position de l'effecteur final
void forward_kinematics(double theta[3], DH_fixed DH_params[3], double p[3]) {
    double T_total[4][4] = {{1.0, 0.0, 0.0, 0.0},
                            {0.0, 1.0, 0.0, 0.0},
                            {0.0, 0.0, 1.0, 0.0},
                            {0.0, 0.0, 0.0, 1.0}};
    for (int i = 0; i < 3; i++) {
        double T_i[4][4];
        get_T(theta[i], DH_params[i].d, DH_params[i].a, DH_params[i].alpha, T_i);
        double new_T_total[4][4];
        matrix_multiply(T_total, T_i, new_T_total);
        memcpy(T_total, new_T_total, sizeof(double) * 16);
    }
    p[0] = T_total[0][3];
    p[1] = T_total[1][3];
    p[2] = T_total[2][3];
}

// Cinématique inverse : calcule les angles des articulations
int inverse_kinematics(double p_desired[3], DH_fixed DH_params[3], double theta[3]) {
    double x = p_desired[0], y = p_desired[1], z = p_desired[2];
    double d1 = DH_params[0].d, L2 = DH_params[1].a, L3 = DH_params[2].a;

    // Calcul de theta1 (shoulder yaw)
    theta[0] = atan2(y, x);
    if (theta[0] < joint_limits[0][0] || theta[0] > joint_limits[0][1]) {
        return -1; // Hors limites
    }

    // Calcul de la distance dans le plan local
    double r = sqrt(x * x + y * y);
    double h = z - d1;
    double D = sqrt(r * r + h * h);

    // Vérification de la portée
    if (D > (L2 + L3) || D < fabs(L2 - L3)) {
        return -1; // Position hors de portée
    }

    // Calcul de theta3 (elbow pitch) avec la loi des cosinus
    double cos_theta3 = (r * r + h * h - L2 * L2 - L3 * L3) / (2 * L2 * L3);
    if (cos_theta3 < -1.0 || cos_theta3 > 1.0) {
        return -1; // Pas de solution
    }
    theta[2] = acos(cos_theta3); // Solution "elbow down"
    if (theta[2] < joint_limits[2][0] || theta[2] > joint_limits[2][1]) {
        theta[2] = -theta[2]; // Essayer "elbow up"
        if (theta[2] < joint_limits[2][0] || theta[2] > joint_limits[2][1]) {
            return -1;
        }
    }

    // Calcul de theta2 (shoulder pitch)
    double k1 = L2 + L3 * cos(theta[2]);
    double k2 = L3 * sin(theta[2]);
    double beta = atan2(h, r);
    double alpha = atan2(k2, k1);
    theta[1] = beta - alpha;
    if (theta[1] < joint_limits[1][0] || theta[1] > joint_limits[1][1]) {
        return -1; // Hors limites
    }

    return 0; // Succès
}

// Exemple d'utilisation
int main() {
    double p_desired[3] = {0.05, 0.05, 0.1}; // Position cible (x, y, z) en mètres
    double theta[3] = {0.0, 0.0, 0.0};       // Angles initiaux
    int result = inverse_kinematics(p_desired, DH_arm_right, theta);
    if (result == 0) {
        printf("Angles des articulations : theta1=%.2f°, theta2=%.2f°, theta3=%.2f°\n",
               theta[0] * 180.0 / M_PI, theta[1] * 180.0 / M_PI, theta[2] * 180.0 / M_PI);
        // Vérification avec cinématique directe
        double p_result[3];
        forward_kinematics(theta, DH_arm_right, p_result);
        printf("Position calculée : x=%.4f, y=%.4f, z=%.4f\n", p_result[0], p_result[1], p_result[2]);
    } else {
        printf("Aucune solution trouvée ou position hors limites.\n");
    }
    return 0;
}