#ifndef MOTOR_H_
#define MOTOR_H_

enum directions_t {DIRECTION_1, DIRECTION_2};

//This is a struct that encapsulates motor functionality.
struct motor {
  int   pwm;              //its an int so overflow problems don't happen
  char  pwm_pin;          //pin that actuates speed.
  char  directiona;       //pin that makes the motor turn cw when it is high.
  char  directionb;       //pin that makes motor turn ccw when it is high.
  long  encoder_value;
  float command_velocity; //specified in rad/s
  float current_velocity; //specified in rad/s
  float command_position; //specified in rad
  float current_position; //specified in rad
};

//This function uses the motor properties to determine if the motor will turn
//clockwise or counter clockwise.
void moveMotor(const motor &active_motor) {
  if (active_motor.command_velocity < 0) {
    digitalWrite(active_motor.directiona,LOW);
    digitalWrite(active_motor.directionb,HIGH);
  }
  else if (active_motor.command_velocity > 0) {
    digitalWrite(active_motor.directiona,HIGH);
    digitalWrite(active_motor.directionb,LOW);
  }
}


//This function sets the duty cycle that will be used to control the motor.
void setMotorSpeed(const motor &active_motor, const char &duty) {
  analogWrite(active_motor.pwm_pin, duty);
}

//This function allows you to manually select the direction of the motor.
void setMotorDirection(const motor &active_motor,
    const directions_t &direction) {
   if (direction == DIRECTION_1) {
    digitalWrite(active_motor.directiona,LOW);
    digitalWrite(active_motor.directionb,HIGH);
  }
  else if (direction == DIRECTION_2) {
    digitalWrite(active_motor.directiona,HIGH);
    digitalWrite(active_motor.directionb,LOW);
  }
}

//This function forces the motor pins to match, forcing it to brake.
void stopMotor(const motor &active_motor) {
  digitalWrite(active_motor.directiona,LOW);
  digitalWrite(active_motor.directionb,LOW);
}

#endif
