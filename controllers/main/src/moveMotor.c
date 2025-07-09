void moveMotor(double degree, WbDeviceTag motor) {
  // convertir les degrées en radians
  double radian = degree * M_PI / 180.0;

  // Vérifier si la position est dans la limite ( max/min )
  double max_pos = wb_motor_get_max_position(motor);
  double min_pos = wb_motor_get_min_position(motor);

  if (radian > max_pos || radian < min_pos) {
    fprintf(stderr, "Error: Position out of bounds. Max: %f, Min: %f, Requested: %f\n", max_pos, min_pos, radian);
    return;
  }

  // Positionner le moteur
  wb_motor_set_position(motor, position);
}