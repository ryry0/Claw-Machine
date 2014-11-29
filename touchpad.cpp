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
 *
 */

#include <Arduino.h>
#include <PID.h>
#include <motor.h>

#define NUM_SAMPLES 10
#define CORNER_0 0
#define CORNER_1 1
#define CORNER_2 2
#define CORNER_3 3

//#define PWM_REMOVAL
#define MOTOR_SCALER -10

#define NUM_MOTORS 4
#define MOTOR_X 0
#define MOTOR_X_PWM 4
#define MOTOR_X_PIN1 40
#define MOTOR_X_PIN2 41

#define MOTOR_X2 1
#define MOTOR_X2_PWM 5
#define MOTOR_X2_PIN1 23
#define MOTOR_X2_PIN2 22

#define MOTOR_Y 2
#define MOTOR_Y_PWM 6
#define MOTOR_Y_PIN1 35
#define MOTOR_Y_PIN2 34

#define MOTOR_WINCH 3
#define MOTOR_WINCH_PWM 7
#define MOTOR_WINCH_PIN1 51
#define MOTOR_WINCH_PIN2 50

#define CLAW_MOTOR_PIN1 8
#define CLAW_MOTOR_PIN2 9

#define PWM_MAX 255
#define PWM_SCALER (255/36.651)

#define TICKS_PER_REVOLUTION 1680

//this is the number of ticks for CTC mode
#define SAMPLE_RATE 200 //Hz
#define CTC_MATCH 10000 //*should* run the interrupt at 200Hz
#define SAMPLE_TIME 0.005

#define PRESCALE_8    0x02

#define KP 0.9 //7 //5
#define KI 0
#define KD 4.5

#define ERR_TOLERANCE 1.0
#define ENC_ERROR 10.0

//PID constants for second motor.
#define KP2 7 //5
#define KI2 0
#define KD2 0.2


#define X_MAX 4*1680 //4.4
#define X_MIN -4*1680

#define Y_MAX 4*1680
#define Y_MIN -4*1680

//Y motor
#define ENCODER1_A 6
#define ENCODER1_B 5

//X motor
#define ENCODER2_A 3
#define ENCODER2_B 2

struct coordinates_t {
  int x;
  int y;
};

void setup();
//readkeyboard is for debugging purposes
void readKeyboard(bool &touchpad_enabled);
coordinates_t readTouchpad();

//global variables
motor motors[NUM_MOTORS]; //motor structs
pid_data motor_pid[NUM_MOTORS]; //pid structs
volatile int encoder_counts[NUM_MOTORS]; //separate encoder structs

//port B interrupt vector
ISR(PCINT2_vect) {
  static const int8_t rot_states[] = //lookup table of rotation states
  {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
  static uint8_t AB[2] = {0x03, 0x03};
  uint8_t t = PINK;  // read port status

  for (int i = 0; i < NUM_MOTORS; ++i) {
    // check for rotary state change button1
    AB[i] <<= 2;                  // save previous state
    AB[i] |= (t >> 2*i) & 0x03;     // add current state
    encoder_counts[i] += rot_states[AB[i] & 0x0f];
  }
}

//Timer interrupt
ISR(TIMER1_COMPA_vect) {
  float current_error = 0;
  for (int i = 0; i < NUM_MOTORS; ++i) { //for num motors
    motors[i].encoder_value = encoder_counts[i]; //set motor encoder count

    //ensure position commanded is within bounds
    motors[i].command_position += motors[i].command_velocity * SAMPLE_TIME;
    /*
    motors[i].command_position = constrain (motors[i].command_position, X_MIN,
        X_MAX);
        */

    current_error = motors[i].command_position - motors[i].encoder_value;  //calc err
    fixedUpdatePID(motor_pid[i], current_error); //update PID

    //constrain pwm from 0 to 255
    motors[i].pwm = round(fabs(
          constrain(motor_pid[i].pid_output,-PWM_MAX, PWM_MAX)));

    //if output is < 0 switch directions
    if (motor_pid[i].pid_output < 0)
      setMotorDirection(motors[i], DIRECTION_1);
    else
      setMotorDirection(motors[i], DIRECTION_2);

#ifdef PWM_REMOVAL //remove buzzing sound
    if (fabs(current_error) < ENC_ERROR) {
      motors[i].pwm = 0;
    }
#endif
    //move the motor
    analogWrite(motors[i].pwm_pin, motors[i].pwm);
  }
}

int main () {
  bool touchpad_enabled = false;

  init();
  setup();

  while (1) {
    readKeyboard(touchpad_enabled);
/*
    if (touchpad_enabled) {
      motors[MOTOR_X].command_position = MOTOR_SCALER * xout;
      motors[MOTOR_X2].command_position = MOTOR_SCALER * yout;
    }
*/

    Serial.print("x: ");
    Serial.print(encoder_counts[MOTOR_X]);

    Serial.print(" x2: ");
    Serial.print(encoder_counts[MOTOR_X2]);

    Serial.print(" y: ");
    Serial.print(encoder_counts[MOTOR_Y]);

    Serial.print(" w: ");
    Serial.print(encoder_counts[MOTOR_WINCH]);
    Serial.print("\n");
  }
}

void setup() {
  noInterrupts();

  //set motor pins and interrupts
  //motor1
  motors[MOTOR_X].pwm_pin = MOTOR_X_PWM;
  motors[MOTOR_X].directionb = MOTOR_X_PIN1;
  motors[MOTOR_X].directiona = MOTOR_X_PIN2;

  pinMode(motors[MOTOR_X].pwm_pin, OUTPUT);
  pinMode(motors[MOTOR_X].directionb, OUTPUT);
  pinMode(motors[MOTOR_X].directiona, OUTPUT);

  motors[MOTOR_X].command_position = 0;

  setPIDConstants(motor_pid[MOTOR_X], KP, KI, KD, 1000);

  //motor2
  motors[MOTOR_X2].pwm_pin = MOTOR_X2_PWM;
  motors[MOTOR_X2].directionb = MOTOR_X2_PIN1;
  motors[MOTOR_X2].directiona = MOTOR_X2_PIN2;

  pinMode(motors[MOTOR_X2].pwm_pin, OUTPUT);
  pinMode(motors[MOTOR_X2].directionb, OUTPUT);
  pinMode(motors[MOTOR_X2].directiona, OUTPUT);

  motors[MOTOR_X2].command_position = 0;

  setPIDConstants(motor_pid[MOTOR_X2], KP, KI, KD, 1000);

  //motor3
  motors[MOTOR_Y].pwm_pin = MOTOR_Y_PWM;
  motors[MOTOR_Y].directionb = MOTOR_Y_PIN1;
  motors[MOTOR_Y].directiona = MOTOR_Y_PIN2;

  pinMode(motors[MOTOR_Y].pwm_pin, OUTPUT);
  pinMode(motors[MOTOR_Y].directionb, OUTPUT);
  pinMode(motors[MOTOR_Y].directiona, OUTPUT);

  motors[MOTOR_Y].command_position = 0;

  setPIDConstants(motor_pid[MOTOR_Y], KP, KI, KD, 1000);

  //motor4
  motors[MOTOR_WINCH].pwm_pin = MOTOR_WINCH_PWM;
  motors[MOTOR_WINCH].directionb = MOTOR_WINCH_PIN1;
  motors[MOTOR_WINCH].directiona = MOTOR_WINCH_PIN2;

  pinMode(motors[MOTOR_WINCH].pwm_pin, OUTPUT);
  pinMode(motors[MOTOR_WINCH].directionb, OUTPUT);
  pinMode(motors[MOTOR_WINCH].directiona, OUTPUT);

  motors[MOTOR_WINCH].command_position = 0;

  setPIDConstants(motor_pid[MOTOR_WINCH], KP, KI, KD, 1000);

  //claw pin
  pinMode(CLAW_MOTOR_PIN1, OUTPUT);
  pinMode(CLAW_MOTOR_PIN2, OUTPUT);

  analogReference(DEFAULT);
  Serial.begin(9600);

  //set up timer interrupt
  TCCR1A = 0;
  TCCR1B = PRESCALE_8; //sets the prescaler to 1
  TCNT1  = 0;             //resets the timer

  OCR1A = CTC_MATCH; //set target number to count to
  TCCR1B |= (0x01 << WGM12);  //enables CTC mode
  TIMSK1 |= (0x01 << OCIE1A); //enables the interrupt CTC interrupt

  //set encoder pins to input
  DDRK = 0x00;

  // enable pullup for encoder pins
  PORTK |= _BV(PORTK7) | _BV(PORTK6) | _BV(PORTK5) |
    _BV(PORTK4) | _BV(PORTK3) | _BV(PORTK2) |
    _BV(PORTK1) | _BV(PORTK0);


  // enable button pin change interrupt
  PCMSK2 = _BV(PCINT16) | _BV(PCINT17) | _BV(PCINT18) | _BV(PCINT19) | _BV(PCINT20)
    | _BV(PCINT21) | _BV(PCINT22) | _BV(PCINT23);
  PCICR = _BV(PCIE2);  // F-port interrupt enable

  interrupts();
}

void readKeyboard(bool &touchpad_enabled) {
  char incoming_byte;
  if (Serial.available()) {
    incoming_byte = Serial.read();
    switch(incoming_byte) {
      case 'w':
        motors[MOTOR_X].command_position += 200;
        break;

      case 's':
        motors[MOTOR_X].command_position -= 200;
        break;

      case 'a':
        motors[MOTOR_X2].command_position -= 200;
        break;

      case 'q':
        motors[MOTOR_X2].command_position += 200;
        break;

      case 'y':
        motors[MOTOR_X].command_velocity += 200;
        motors[MOTOR_X2].command_velocity += 200;
        break;

      case 'h':
        motors[MOTOR_X].command_velocity += 200;
        motors[MOTOR_X2].command_velocity += 200;
        break;

      case 'i':
        motors[MOTOR_X].command_velocity += 800;
        motors[MOTOR_X2].command_velocity += 800;
        break;

      case 'k':
        motors[MOTOR_X].command_velocity -= 800;
        motors[MOTOR_X2].command_velocity -= 800;
        break;

      case 'l':
        motors[MOTOR_Y].command_velocity -= 800;
        break;

      case 'j':
        motors[MOTOR_Y].command_velocity += 800;
        break;

      case '>':
        motors[MOTOR_Y].command_position -= 200;
        break;

      case '<':
        motors[MOTOR_Y].command_position += 200;
        break;

      case 'D':
        motors[MOTOR_WINCH].command_velocity += 3200;
        break;

      case 'U':
        motors[MOTOR_WINCH].command_velocity -= 3200;
        break;

      case 'C':
        analogWrite(CLAW_MOTOR_PIN1, 255);
        analogWrite(CLAW_MOTOR_PIN2,  0);
        break;

      case 'O':
        analogWrite(CLAW_MOTOR_PIN2, 255);
        analogWrite(CLAW_MOTOR_PIN1,  0);
        break;

      case ' ':
        for (int i = 0; i < NUM_MOTORS; ++i) {
          motors[i].command_position = 0;
          motors[i].command_velocity = 0;
          encoder_counts[i] = 0;
        }
        break;

      case '.':
        for (int i = 0; i < NUM_MOTORS; ++i) {
          motors[i].command_position = encoder_counts[i];
          motors[i].command_velocity = 0;
        }
        break;

      case 't':
        analogWrite(CLAW_MOTOR_PIN2, 0);
        analogWrite(CLAW_MOTOR_PIN1, 0);
        touchpad_enabled = !touchpad_enabled;
        break;
    }
  }
}

coordinates_t readTouchpad() {
  static int moving_average_index = 0;
  int a, b, c, d, x, y, xout = 0, yout = 0;// prev_x, prev_y;
  static int xbuffer[NUM_SAMPLES];
  static int ybuffer[NUM_SAMPLES];
  coordinates_t gantry_coords;

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

  for (int i = 0; i < NUM_SAMPLES; ++i) {
    xout += xbuffer[i];
    yout += ybuffer[i];
  }

  xout /= NUM_SAMPLES;
  yout /= NUM_SAMPLES;
  //end of smoothing
  gantry_coords.x = xout;
  gantry_coords.y = yout;
}
