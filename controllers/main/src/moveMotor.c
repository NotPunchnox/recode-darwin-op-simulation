#include <webots/motor.h>
#include <webots/robot.h>
#include <stdio.h>

void moveMotor(WbDeviceTag motor, double degree) {

  // Si le moteur appartient au groupe de droite mettre la position en négatif ( inclus R )
  static const char *right_side_motors[] = {
      "ShoulderR", "ArmUpperR", "ArmLowerR", "PelvYR", "PelvR",
      "LegUpperR", "LegLowerR", "AnkleR", "FootR"
  };

  // Vérifier si le moteur est dans le groupe de droite
  int is_right_side = 0;
  for (int i = 0; i < 9; i++) {
      if (motor == wb_robot_get_device(right_side_motors[i])) {
          is_right_side = 1;
          break;
      }
  }

  // convertir les degrées en radians
  double radian = degree * M_PI / 180.0;

  if (is_right_side > 0) {
  // Si le moteur est du côté droit, inverser la direction
    if (degree < 0) {
      radian = -radian;
    } else {
      radian = radian * -1;
    }
  }

  // Vérifier si la position est dans la limite ( max/min )
  double max_pos = wb_motor_get_max_position(motor);
  double min_pos = wb_motor_get_min_position(motor);

  if (radian > max_pos || radian < min_pos) {
    printf("Error: Position out of bounds. Max: %f, Min: %f, Requested degree: %f, Requested radian: %f\n", 
      (180 / M_PI) * max_pos,
      (180 / M_PI) * min_pos,
      degree,
      (180 / M_PI) * radian);
      
    return;
  }


  // Positionner le moteur
  wb_motor_set_position(motor, radian);
}