#include <webots/motor.h>
#include <webots/robot.h>
#include <stdio.h>
#include <string.h>

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

  if (is_right_side) {
    degree = -degree;
  }


  // convertir les degrées en radians
  double radian = degree * M_PI / 180.0;

  // Vérifier si la position est dans la limite ( max/min )
  double max_pos = wb_motor_get_max_position(motor);
  double min_pos = wb_motor_get_min_position(motor);

  if (radian > max_pos || radian < min_pos) {
    printf("Error: Position out of bounds. Max: %f, Min: %f, Requested: %f\n", max_pos, min_pos, radian);
    return;
  }


  // Positionner le moteur
  wb_motor_set_position(motor, radian);
}