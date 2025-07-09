#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h>
#include <stdio.h>
#include <math.h>

#define TIME_STEP 32
#define MAX_POS 1.5708  // 90°
#define MIN_POS -1.5708 // -90°

static const char *motorNames[20] = {
  "ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL", "ArmLowerR",
  "ArmLowerL", "PelvYR", "PelvYL", "PelvR", "PelvL",
  "LegUpperR", "LegUpperL", "LegLowerR", "LegLowerL", "AnkleR",
  "AnkleL", "FootR", "FootL", "Neck", "Head"
};



int main() {
  wb_robot_init();

  WbDeviceTag motor[20];

  for (int i = 0; i < 20; i++) {
    motor[i] = wb_robot_get_device(motorNames[i]);
    if (motor[i] == 0) {
      fprintf(stderr, "Error: Device '%s' not found.\n", motorNames[i]);
      return -1;
    }
  }
  wb_keyboard_enable(TIME_STEP);

  // Bouger le bras droit (left-arm)
  // Rotation progressive + logs
  printf("min position: %f\n", wb_motor_get_min_position(motor[0]));
  printf("max position: %f\n", wb_motor_get_max_position(motor[0]));

  wb_motor_set_position(motor[0], 0.1);
  wb_motor_set_position(motor[2], 0.1);
  wb_motor_set_position(motor[4], 0.1);
  



  wb_robot_cleanup();
  return 0;
}