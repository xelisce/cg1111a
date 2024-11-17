bool test_line() // sensing double black
{
  uint8_t sensor_state;
  sensor_state = lineFinder.readSensors();
  if (sensor_state == S1_IN_S2_IN)
  {
    stop_moving(); // stop moving while sensing
    movement = COLOR_SENSE;
#if PRINT
    Serial.println("DOUBLE BLACK SENSED");
#endif
  }
  else
  {
    return false;
  }
}