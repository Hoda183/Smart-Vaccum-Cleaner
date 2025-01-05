


// Function to control motor speeds
void controlMotors(int left_speed, int right_speed) {
  // Set motor directions and speeds based on input
  if (left_speed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    left_speed = -left_speed; // Convert to positive for PWM
  }

  if (right_speed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    right_speed = -right_speed; // Convert to positive for PWM
  }

  // Write speeds to the motor driver
  analogWrite(EN1, constrain(left_speed, 0, 255));
  analogWrite(EN2, constrain(right_speed, 0, 255));

}
