/*
 * The motor gear ratio is 30
 * The encoder resolution is 64
 * That makes 1920 ticks per shaft rev.
 *
 * The timer interrupt uses the following formula
 * (16M/prescaler)/(desired frequency) = number of counts for CTC mode.
 * (16M/8)/(200) = CTC_MATCH = 10000
 *
 * Motor Pinout:
 * Red    | motor power (connects to one motor terminal)
 * Black  | motor power (connects to the other motor terminal)
 * Green  | encoder GND
 * Blue   | encoder Vcc (3.5 - 20 V)
 * Yellow | encoder A output
 * White  | encoder B output
 *
 * The pololu motor max speed is 350 rev/minute.
 * In radians per second that is:
 * 350/60 * 2pi = 36.651 rad/s @ theoretically 255 PWM
 */

#include <Arduino.h>
#include <AFMotor.h>
#include <PID.h>
#include <motor.h>
#include <encoder.h>

#define NUM_SAMPLES 10
#define CORNER_0 0
#define CORNER_1 1
#define CORNER_2 2
#define CORNER_3 3

#define MOTOR_SCALER -10

#define NUM_MOTORS 2
#define MOTOR_X 0
#define MOTOR_X_PWM 5
#define MOTOR_X_PIN1 4
#define MOTOR_X_PIN2 6

#define MOTOR_Y 1
#define MOTOR_Y_PWM 5
#define MOTOR_Y_PIN1 4
#define MOTOR_Y_PIN2 6

#define PWM_SCALER (255/36.651)

#define TICKS_PER_REVOLUTION 1680

//this is the number of ticks for CTC mode
#define SAMPLE_RATE 200 //Hz
#define CTC_MATCH 10000 //*should* run the interrupt at 200Hz
#define SAMPLE_TIME 0.005

#define PRESCALE_8    0x02

#define KP 7 //5
#define KI 0
#define KD 0.2


//PID constants for second motor.
#define KP2 7 //5
#define KI2 0
#define KD2 0.2


#define X_MAX 4*1680 //4.4
#define X_MIN 0

#define Y_MAX 4*1680
#define Y_MIN 0

#define ENCODER1_A 2
#define ENCODER1_B 3

//AF_DCMotor motor(1, MOTOR12_64KHZ); // create motor #2, 64KHz pwm
motor motors[NUM_MOTORS];

pid_data motor_pid[NUM_MOTORS];

ISR(TIMER1_COMPA_vect) {
  float current_error = 0;
  for (int i = 0; i < NUM_MOTORS; ++i) {
    motors[i].encoder_value = encoder::encoder_count; //set motor encoder count

    //ensure position commanded is within bounds
    motors[i].command_position = constrain (motors[i].command_position, X_MIN,
        X_MAX);

    current_error = motors[i].encoder_value - motors[i].command_position; //calc err
    updatePID(motor_pid[i], current_error, SAMPLE_TIME); //update PID

    //constrain pwm from 0 to 255
    motors[i].pwm = round(fabs( constrain(motor_pid[i].pid_output,-255,255)));

    //if output is < 0 switch directions
    if (motor_pid[i].pid_output < 0)
      setMotorDirection(motors[i], DIRECTION_1);
    else
      setMotorDirection(motors[i], DIRECTION_2);

    //move the motor
    analogWrite(motors[i].pwm_pin, motors[i].pwm);
  }
}

void setup() {
  noInterrupts();

  motors[MOTOR_X].pwm_pin = MOTOR_X_PWM;
  motors[MOTOR_X].directionb = MOTOR_X_PIN1;
  motors[MOTOR_X].directiona = MOTOR_X_PIN2;

  pinMode(motors[MOTOR_X].pwm_pin, OUTPUT);
  pinMode(motors[MOTOR_X].directionb, OUTPUT);
  pinMode(motors[MOTOR_X].directiona, OUTPUT);

  motors[MOTOR_X].command_position = 0;
  motors[MOTOR_X].prev_command_position = 0;

  setPIDConstants(motor_pid[MOTOR_X], KP, KI, KD, 1000);

  motors[MOTOR_Y].pwm_pin = MOTOR_Y_PWM;
  motors[MOTOR_Y].directionb = MOTOR_Y_PIN1;
  motors[MOTOR_Y].directiona = MOTOR_Y_PIN2;

  pinMode(motors[MOTOR_Y].pwm_pin, OUTPUT);
  pinMode(motors[MOTOR_Y].directionb, OUTPUT);
  pinMode(motors[MOTOR_Y].directiona, OUTPUT);

  motors[MOTOR_Y].command_position = 0;
  motors[MOTOR_Y].prev_command_position = 0;

  setPIDConstants(motor_pid[MOTOR_Y], KP, KI, KD, 1000);

  analogReference(DEFAULT);
  Serial.begin(9600);

  TCCR1A = 0;
  TCCR1B = PRESCALE_8; //sets the prescaler to 1
  TCNT1  = 0;             //resets the timer

  OCR1A = CTC_MATCH;
  TCCR1B |= (0x01 << WGM12);  //enables CTC mode
  TIMSK1 |= (0x01 << OCIE1A); //enables the interrupt CTC interrupt

  //associates encoder channels with pin numbers
  encoder::initEncoderChannelA(ENCODER1_A);
  encoder::initEncoderChannelB(ENCODER1_B);

  interrupts();
}

int main () {
  int moving_average_index = 0;
  int a, b, c, d, x, y, xout, yout, prev_x, prev_y;
  int xbuffer[NUM_SAMPLES];
  int ybuffer[NUM_SAMPLES];
  char incomingByte = 0;
  bool touchpad_enabled = false;

  init();
  setup();

  while (1) {
    if (Serial.available()) {
      incomingByte = Serial.read();
      switch(incomingByte) {
        case 'w':
          motors[MOTOR_Y].command_position -= 50;
          break;
        case 's':
          motors[MOTOR_Y].command_position += 50;
          break;
        case 'a':
          motors[MOTOR_X].command_position -= 50;
          break;
        case 'd':
          motors[MOTOR_X].command_position += 50;
          break;
        case ' ':
          motors[MOTOR_X].command_position = 0;
          motors[MOTOR_Y].command_position = 0;
          break;
        case 't':
          touchpad_enabled = !touchpad_enabled;
          break;
      }
    }

    /*
    Serial.print(motor_x.command_position, DEC);
    Serial.print("\t");
    Serial.print(motor_x.encoder_value, DEC);
    Serial.print("\t");
    Serial.print(motor_x_pid.pid_output, 4);
    Serial.print("\t");
    Serial.print(motor_x.pwm, DEC);
    Serial.print("\n");
    */

    //read the four corners
    a = analogRead(CORNER_0);
    b = analogRead(CORNER_1);
    c = analogRead(CORNER_2);
    d = analogRead(CORNER_3);

    //do the x magic
    x = c + a - b - d;

    //do the y magic
    y = b + a - c - d;

    //(the above formulas are derived from the properties of MY piece of paper + graphite and probably won't work well with any other)

    //moving average to smooth mouse motion
    xbuffer[moving_average_index % NUM_SAMPLES] = x;
    ybuffer[moving_average_index++ % NUM_SAMPLES] = y;

    xout = 0;
    yout = 0;

    for (int i = 0; i < NUM_SAMPLES; ++i) {
      xout += xbuffer[i];
      yout += ybuffer[i];
    }

    xout /= NUM_SAMPLES;
    yout /= NUM_SAMPLES;
    //end of smoothing
    if (touchpad_enabled) {
      motors[MOTOR_X].command_position = MOTOR_SCALER * xout;
      motors[MOTOR_Y].command_position = MOTOR_SCALER * yout;
    }

    //send x y positions to computer. A program of your own is required to receive this and interpret it into mouse movement.
    Serial.print("x ");
    Serial.print(xout);
    delay(25);
    Serial.print(" y ");
    Serial.print(yout);
    Serial.print("\n");
    delay(25);
    prev_x = xout;
    prev_y = yout;
  }
}
