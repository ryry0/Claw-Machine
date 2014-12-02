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
#include <defs.h> //look here for all define/constant values

//data structure definitions
//struct that represents coordinate values
struct coordinates_t {
  int x;
  int y;
};

//struct that represents motos limit values
struct limits_t {
  int min;
  int max;
} motor_limits[NUM_MOTORS];

enum states_t {
  WAIT_FOR_START,
  INIT,
  PLAY,
  RETRIEVE_PRIZE,
  RETURN_HOME,
  DEBUG
};

//global variables
motor motors[NUM_MOTORS]; //motor structs
pid_data motor_pid[NUM_MOTORS]; //pid structs
volatile int encoder_counts[NUM_MOTORS]; //separate encoder structs
states_t game_state = WAIT_FOR_START;

//function prototypes
#include <kbd_debug.h>
void setup();
coordinates_t readTouchpad();
bool readClawButton();
void readDirectionButtons();

//port B interrupt vector used to read all encoder lines
ISR(PCINT2_vect) {
  static const int8_t rot_states[] = //lookup table of rotation states
  {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
  static uint8_t AB[NUM_MOTORS] = {0x03, 0x03, 0x03, 0x03};
  uint8_t t = PINK;  // read port status

  for (int i = 0; i < NUM_MOTORS; ++i) {
    // check for rotary state change button1
    AB[i] <<= 2;                  // save previous state
    AB[i] |= (t >> 2*i) & 0x03;     // add current state
    encoder_counts[i] += rot_states[AB[i] & 0x0f];
  }
} //end ISR(PCINT2_vect)

//Timer interrupt
ISR(TIMER1_COMPA_vect) { //computes the PID for each motors @200Hz
  float current_error = 0;
  for (int i = 0; i < NUM_MOTORS; ++i) { //for num motors
    motors[i].encoder_value = encoder_counts[i]; //set motor encoder count

    motors[i].command_position += motors[i].command_velocity * SAMPLE_TIME;

    //ensure position commanded is within bounds in every mode except debug
    if (game_state != DEBUG) {
      motors[i].command_position = constrain (motors[i].command_position,
          motor_limits[i].min,
          motor_limits[i].max);
    }

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

//MAIN FUNCTION///////////////////
int main () {
  bool touchpad_enabled = false;
  char incoming_byte;
  coordinates_t gantry_coordinates;

  init();
  setup();

  while (1) { //main loop
    switch(game_state) { //game state machine
      case WAIT_FOR_START: //wait for the start button to be pressed
        analogWrite(CLAW_MOTOR_PIN1,  0);
        analogWrite(CLAW_MOTOR_PIN2, 0);

        if (Serial.available()) { //check keyboard for init
          incoming_byte = Serial.read();
          if (incoming_byte == ' ')
            game_state = INIT;
        }

        //read claw button and spin till person releases
        while (readClawButton())
          game_state = INIT;

        break; //end WAIT_FOR_START

      case INIT: //zero out everything
        for (int i = 0; i < NUM_MOTORS; ++i)
          motors[i].command_velocity = 0;

        game_state = PLAY;
        break; //end INIT

      case PLAY: //take input from touchpad and keyboard
        if (touchpad_enabled) {
          gantry_coordinates = readTouchpad();
          //motors[MOTOR_X].command_position = MOTOR_SCALER * xout;
          //motors[MOTOR_X2].command_position = MOTOR_SCALER * yout;
        }

        readKeyboardInput();
        readDirectionButtons();

        if (readClawButton())
          game_state = RETRIEVE_PRIZE;

        Serial.print("x vel: ");
        Serial.print(motors[MOTOR_X].command_velocity, 4);
        Serial.print("\n");
        break; //end PLAY

      case RETRIEVE_PRIZE:
        //stop x and y movement
        for (int i = 0; i < 3; ++i) {
          motors[i].command_velocity = 0;
        }

        //open the claw
        analogWrite(CLAW_MOTOR_PIN1,  0);
        analogWrite(CLAW_MOTOR_PIN2, 255);

        motors[MOTOR_WINCH].command_velocity = 3200;

        //spin and wait till we get to bottom
        while( (abs(encoder_counts[MOTOR_WINCH] - WINCH_MAX) > 100) );

        //close the claw
        analogWrite(CLAW_MOTOR_PIN1,  255);
        analogWrite(CLAW_MOTOR_PIN2, 0);

        motors[MOTOR_WINCH].command_velocity = -3200;


        //go over hole
        motors[MOTOR_X].command_velocity = 800;
        motors[MOTOR_X2].command_velocity = 800;
        motors[MOTOR_Y].command_velocity = -1600;

        //spin till we get attain x and y coords, and the reel is up
        //stays in while loop until we're about 100 enc ticks away

        //while ( x and x2 != xmax or y != y_max or winch != winch_min)
        while( ((abs(encoder_counts[MOTOR_X] < SQUARE_X))  &&
            (abs(encoder_counts[MOTOR_X2]    < SQUARE_X))) ||
            (abs(encoder_counts[MOTOR_Y]     > SQUARE_Y))  ||
            (abs(encoder_counts[MOTOR_WINCH] > WINCH_END)) ) {

          //shut off x motors when needed
          if ((abs(encoder_counts[MOTOR_X] > SQUARE_X)) &&
              (abs(encoder_counts[MOTOR_X2]  > SQUARE_X))) {
            motors[MOTOR_X].command_velocity = 0;
            motors[MOTOR_X2].command_velocity = 0;
          }

          if (abs(encoder_counts[MOTOR_Y] < SQUARE_Y)) {
            motors[MOTOR_Y].command_velocity = 0;
          }

          if (abs(encoder_counts[MOTOR_WINCH] < WINCH_END)) {
            motors[MOTOR_WINCH].command_velocity = 0;
          }
        } //end while spin

        //open the claw
        analogWrite(CLAW_MOTOR_PIN1,  0);
        analogWrite(CLAW_MOTOR_PIN2, 255);

        game_state = RETURN_HOME;
        break; //end RETRIEVE_PRIZE

      case RETURN_HOME: //goes back to back left
        motors[MOTOR_X].command_velocity = -800;
        motors[MOTOR_X2].command_velocity = -800;

        motors[MOTOR_Y].command_velocity = -1600;

        while(abs(encoder_counts[MOTOR_X] > X_MIN + POS_TOLERANCE));

        while(abs(encoder_counts[MOTOR_Y] > Y_MIN + POS_TOLERANCE));

        //set everything to zero
        analogWrite(CLAW_MOTOR_PIN1,  0);
        analogWrite(CLAW_MOTOR_PIN2, 0);
        for (int i = 0; i < NUM_MOTORS; ++i) {
          motors[i].command_velocity = 0;
          motors[i].command_position = encoder_counts[i];
        }
        game_state = WAIT_FOR_START;
        break; //end RETURN_HOME

      case DEBUG: //for debugging purposes, removes software limiters
        readKeyboardDebug(touchpad_enabled);
        Serial.print("x: ");
        Serial.print(encoder_counts[MOTOR_X]);

        Serial.print(" x2: ");
        Serial.print(encoder_counts[MOTOR_X2]);

        Serial.print(" y: ");
        Serial.print(encoder_counts[MOTOR_Y]);

        Serial.print(" w: ");
        Serial.print(encoder_counts[MOTOR_WINCH]);
        Serial.print("\n");
        break; // END DEBUG

      default:
        break;
    } //end switch
    //silence motors
    for (int i = 0; i < NUM_MOTORS; ++i) {
      if (motors[i].command_velocity == 0) {
        motors[i].command_position = encoder_counts[i];
      }
    } //end for
  } //end while
} //end main

void setup() {
  noInterrupts();

  //set motor pins and interrupts
  //motor1
  motors[MOTOR_X].pwm_pin = MOTOR_X_PWM;
  motors[MOTOR_X].directionb = MOTOR_X_PIN1;
  motors[MOTOR_X].directiona = MOTOR_X_PIN2;
  motor_limits[MOTOR_X].min = X_MIN;
  motor_limits[MOTOR_X].max = X_MAX;

  //motor2
  motors[MOTOR_X2].pwm_pin = MOTOR_X2_PWM;
  motors[MOTOR_X2].directionb = MOTOR_X2_PIN1;
  motors[MOTOR_X2].directiona = MOTOR_X2_PIN2;
  motor_limits[MOTOR_X2].min = X_MIN;
  motor_limits[MOTOR_X2].max = X_MAX;

  //motor3
  motors[MOTOR_Y].pwm_pin = MOTOR_Y_PWM;
  motors[MOTOR_Y].directionb = MOTOR_Y_PIN1;
  motors[MOTOR_Y].directiona = MOTOR_Y_PIN2;
  motor_limits[MOTOR_Y].min = Y_MIN;
  motor_limits[MOTOR_Y].max = Y_MAX;

  //motor4
  motors[MOTOR_WINCH].pwm_pin = MOTOR_WINCH_PWM;
  motors[MOTOR_WINCH].directionb = MOTOR_WINCH_PIN1;
  motors[MOTOR_WINCH].directiona = MOTOR_WINCH_PIN2;
  motor_limits[MOTOR_WINCH].min = WINCH_MIN;
  motor_limits[MOTOR_WINCH].max = WINCH_MAX;

  for (int i = 0; i < NUM_MOTORS; ++i) {
    pinMode(motors[i].pwm_pin, OUTPUT);
    pinMode(motors[i].directionb, OUTPUT);
    pinMode(motors[i].directiona, OUTPUT);
    motors[i].command_position = 0;
    setPIDConstants(motor_pid[i], KP, KI, KD, 1000);
  }

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
  PCMSK2 = _BV(PCINT16) | _BV(PCINT17) | _BV(PCINT18) | _BV(PCINT19) |
    _BV(PCINT20) | _BV(PCINT21) | _BV(PCINT22) | _BV(PCINT23);
  PCICR = _BV(PCIE2);  // F-port interrupt enable

  //setup directional buttons
  DDRD &= 0xF0; //set as input
  PORTD |= 0x0F;//set internal pullup resistors

  //setup start/claw button
  DDRH &= 0xFE; //set as input
  PORTH |= 0x01; //set internal pullup resistor

  interrupts();
}

//this function reads the touchpad.
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

  return gantry_coords;
} //end readTouchpad

//reads the button that begins the game/indicates prize retrieval
bool readClawButton() {
  static float temp = 0;
  float next_val = 0;

  if ((PINH & 0x01) == 0x00) //pin is pressed
#ifdef DEBOUNCE_BUTTON
    next_val = 10;
  //software exponential moving avg (lowpass) filter
  temp = DEBOUNCE_CONSTANT*next_val + (1 - DEBOUNCE_CONSTANT)*temp;
  if (temp > 5.0)
#endif
    return true;

  return false;
}

//reads the buttons that control the direction
void readDirectionButtons() {
  if ((PIND & 0x01) == 0x00) //left is pressed
    motors[MOTOR_Y].command_velocity = MAX_Y_SPEED;
  else if ((PIND & 0x02) == 0x00) //right is pressed
    motors[MOTOR_Y].command_velocity = -MAX_Y_SPEED;
  else
    motors[MOTOR_Y].command_velocity = 0;

  if ((PIND & 0x04) == 0x00) {//up is pressed
    motors[MOTOR_X].command_velocity = MAX_X_SPEED;
    motors[MOTOR_X2].command_velocity = MAX_X_SPEED;
  }
  else if ((PIND & 0x08) == 0x00) {//down is pressed
    motors[MOTOR_X].command_velocity = -MAX_X_SPEED;
    motors[MOTOR_X2].command_velocity = -MAX_X_SPEED;
  }
  else {
    motors[MOTOR_X].command_velocity =0;
    motors[MOTOR_X2].command_velocity = 0;
  }
}
