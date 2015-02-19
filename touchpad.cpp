/*
 * This is the main file for the program. It contains the PID calculation,
 * input detection and interpretation, and the game state machine.
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
  float x;
  float y;
};

//struct that represents motos limit values
struct limits_t {
  int min;
  int max;
} motor_limits[NUM_MOTORS];

//states for the state machine
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
volatile long encoder_counts[NUM_MOTORS]; //separate encoder structs
states_t game_state = WAIT_FOR_START;

//function prototypes
#include <kbd_debug.h>
void setup();
coordinates_t readTouchpad();
void readDirectionButtons();

bool readClawButton();
void openClaw();
void closeClaw();
void relaxClaw();

void lightGreenLED();
void lightRedLED();

//port B interrupt vector used to read all encoder lines
ISR(PCINT2_vect) {
  static const int8_t rot_states[] = //lookup table of rotation states
  {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
  static uint8_t AB[NUM_MOTORS] = {0x03, 0x03, 0x03, 0x03}; //encoder states
  uint8_t t = PINK;  // read port status

  for (int i = 0; i < NUM_MOTORS; ++i) {
    // check for rotary state change
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
    //check which game state we're in
    switch(game_state) {
      case DEBUG: //remove limiters on debug
        break;

      default:
        motors[i].command_position = constrain (motors[i].command_position,
            motor_limits[i].min,
            motor_limits[i].max);
        break;
    }

    //calc err
    current_error = motors[i].command_position - motors[i].encoder_value;
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
  coordinates_t prev_coordinates; //prev coordinates for computing velocity

  gantry_coordinates.x = 0;
  gantry_coordinates.y = 0;
  prev_coordinates.x = 0;
  prev_coordinates.y = 0;

  float x_vel = 0, y_vel = 0;

  //variables for filtering touchpad velocity commands
#ifdef FILTER_VELOCITY
  float filtered_x_vel = 0, filtered_y_vel = 0;
#endif

  init();
  setup();

  while (1) { //main loop
    switch(game_state) { //game state machine
      case WAIT_FOR_START: //wait for the start button to be pressed
        relaxClaw();

        lightRedLED();

        if (Serial.available()) { //check keyboard for init
          incoming_byte = Serial.read();
          if (incoming_byte == ' ')
            game_state = INIT;
        }

        //read claw button and spin till person releases
        while (readClawButton())
          game_state = INIT;

        break; //end WAIT_FOR_START

      case INIT: //zero out velocites
        for (int i = 0; i < NUM_MOTORS; ++i)
          motors[i].command_velocity = 0;

        lightGreenLED();

        Serial.print("PLAY GAME\n");

        game_state = PLAY;
        break; //end INIT

        ///////PLAY///////////////////////////////
      case PLAY: //take input from touchpad or buttons
        if (touchpad_enabled) {

          //analogRead is equivalent to ad0_conv();
          if (analogRead(STYLUS_PIN) > 0) { //check if stylus is touching
            gantry_coordinates = readTouchpad();

            //command velocity is proportional to pen velocity.
            x_vel = constrain(X_VEL_SCALER *
                (gantry_coordinates.x - prev_coordinates.x), -MAX_X_SPEED,
                MAX_X_SPEED);

            y_vel = constrain(Y_VEL_SCALER *
                (gantry_coordinates.y - prev_coordinates.y), -MAX_Y_SPEED,
                MAX_Y_SPEED);

            //assign the velocities
            motors[MOTOR_X2].command_velocity =
              motors[MOTOR_X].command_velocity =
#ifdef FILTER_VELOCITY //switch to filter the velocity or not
              VEL_FILTER_CONSTANT*x_vel +
              (1-VEL_FILTER_CONSTANT) * motors[MOTOR_X].command_velocity;
#else
            x_vel;
#endif
            motors[MOTOR_Y].command_velocity =
#ifdef FILTER_VELOCITY
              VEL_FILTER_CONSTANT*y_vel +
              (1-VEL_FILTER_CONSTANT) * motors[MOTOR_Y].command_velocity;
#else
            y_vel;
#endif

            //remember prev state
            prev_coordinates = gantry_coordinates;
          } //end if (analogRead(STYLUS_PIN) > 0)

          else {
            for (int i = 0; i < NUM_MOTORS; ++i) {
              motors[i].command_velocity = 0;
            }
          } //end else (analogRead(STYLUS_PIN) > 0)

        } //end if (touchpad_enabled)
        else {
          readDirectionButtons();
        }

        readKeyboardInput();

        if (readClawButton())
          game_state = RETRIEVE_PRIZE;

        break; //end PLAY

        /////////////RETRIEVE_PRIZE/////////////////
      case RETRIEVE_PRIZE:
        //stop x and y movement
        for (int i = 0; i < 3; ++i) {
          motors[i].command_velocity = 0;
        }

        //open the claw
        openClaw();

        motors[MOTOR_WINCH].command_position = WINCH_MAX; //drop claw

        //spin and wait till we get to bottom
        while(encoder_counts[MOTOR_WINCH] < WINCH_BOTTOM) {
          //Serial.print(motors[MOTOR_WINCH].encoder_value);
        }

        delay(1500);
        //close the claw
        closeClaw();

        motors[MOTOR_WINCH].command_position = WINCH_MIN; //reel claw

        //spin and wait till we get back to top
        while (encoder_counts[MOTOR_WINCH] > WINCH_END);

        //go over hole
        motors[MOTOR_X].command_velocity = 800;
        motors[MOTOR_X2].command_velocity = 800;
        motors[MOTOR_Y].command_velocity = -1600;

        //spin till we get attain x and y coords, and the reel is up
        //stays in while loop until we're about 100 enc ticks away

        //while ( x and x2 != xmax or y != y_max or winch != winch_min)
        while( ((encoder_counts[MOTOR_X] < SQUARE_X)  &&
              (encoder_counts[MOTOR_X2] < SQUARE_X)) ||
            (encoder_counts[MOTOR_Y]  > SQUARE_Y)) {

          //shut off the motors when needed
          if ((encoder_counts[MOTOR_X] > SQUARE_X) &&
              (encoder_counts[MOTOR_X2]  > SQUARE_X)) {
            motors[MOTOR_X].command_velocity = 0;
            motors[MOTOR_X2].command_velocity = 0;
          }

          if (encoder_counts[MOTOR_Y] < SQUARE_Y) {
            motors[MOTOR_Y].command_velocity = 0;
          }
        } //end while spin

        //open the claw
        openClaw();

        game_state = RETURN_HOME;
        break; //end RETRIEVE_PRIZE

        /////////RETURN HOME/////////////////////
      case RETURN_HOME: //goes back to back left
        //command motors to return to 0,0
        motors[MOTOR_X].command_velocity = -MAX_X_SPEED/2;
        motors[MOTOR_X2].command_velocity = -MAX_X_SPEED/2;

        motors[MOTOR_Y].command_velocity = -MAX_Y_SPEED;

        //spin till we get there
        //while ( x and x2 > x_min or y > y_min )
        while((encoder_counts[MOTOR_X] > X_MIN + POS_TOLERANCE) ||
            (encoder_counts[MOTOR_Y] > Y_MIN + POS_TOLERANCE)) {

          //shut off motors when needed
          if(!(encoder_counts[MOTOR_X] > X_MIN + POS_TOLERANCE)) {
            motors[MOTOR_X].command_velocity = 0;
            motors[MOTOR_X2].command_velocity = 0;
          }

          if(!(encoder_counts[MOTOR_Y] > Y_MIN + POS_TOLERANCE)) {
            motors[MOTOR_Y].command_velocity = 0;
          }

          //debug information
          for (int i = 0; i < NUM_MOTORS; ++i) {
            Serial.print(encoder_counts[i]);
            Serial.print(" ");
          }
          Serial.print("\n");
        }

        //set everything to zero
        relaxClaw();
        for (int i = 0; i < NUM_MOTORS; ++i) {
          motors[i].command_velocity = 0;
          motors[i].command_position = encoder_counts[i];
        }
        game_state = WAIT_FOR_START;
        break; //end RETURN_HOME

        //////////DEBUG/////////////////////////////
      case DEBUG: //for debugging purposes, removes software limiters
        readKeyboardDebug(touchpad_enabled);
        Serial.print("DEBUG : W: ");
        Serial.print(encoder_counts[MOTOR_WINCH]);
        Serial.print(" X: ");
        Serial.print(encoder_counts[MOTOR_X]);
        Serial.print(" Y: ");
        Serial.print(encoder_counts[MOTOR_Y]);

        Serial.print(" S1: ");
        Serial.print(digitalRead(CART_STOP));
        Serial.print(" S2: ");
        Serial.print(digitalRead(SHELF_STOP_L));
        Serial.print(" S3: ");
        Serial.print(digitalRead(SHELF_STOP_R));
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

  //set motor pins and interrupts and limits
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

  //set PID constants
  for (int i = 0; i < NUM_MOTORS; ++i) {
    motors[i].command_position = 0;
    setPIDConstants(motor_pid[i], KP, KI, KD, 1000);
  }


  //set all PWM pins to output
  DDRH |= 0x78;
  DDRE |= 0x08;
  DDRG |= 0x20;

  //configure directional pins to output
  DDRG |= 0x03;
  DDRA |= 0x03;
  DDRC |= 0x0C;
  DDRB |= 0x0C;

  //configure hardware stops
  pinMode(CART_STOP, INPUT);
  pinMode(SHELF_STOP_L, INPUT);
  pinMode(SHELF_STOP_R, INPUT);

  //turn on internal pullup resistors
  digitalWrite(CART_STOP, HIGH);
  digitalWrite(SHELF_STOP_L, HIGH);
  digitalWrite(SHELF_STOP_R, HIGH);

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

  //setup LED Pins
  DDRC |= 0xC0;

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
  a = analogRead(CORNER_0); //equivalent of ad0_conv
  b = analogRead(CORNER_1);
  c = analogRead(CORNER_2);
  d = analogRead(CORNER_3);

  //do the x magic
  x = c + a - b - d;

  //do the y magic
  y = b + a - c - d;

  //FIR moving average to smooth motion
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

//sets Green LED port high and red low
void lightGreenLED() {
  PORTC |= 0x80;
  PORTC &= 0xBF;
}

//opposite of above
void lightRedLED() {
  PORTC |= 0x40;
  PORTC &= 0x7F;
}

//writes a PWM to one of the claw input lines
void openClaw() {
  analogWrite(CLAW_MOTOR_PIN1,  0);
  analogWrite(CLAW_MOTOR_PIN2, 255);
}

//writes a PWM to the opposite line as above
void closeClaw() {
  analogWrite(CLAW_MOTOR_PIN1,  255);
  analogWrite(CLAW_MOTOR_PIN2, 0);
}

void relaxClaw() {
  analogWrite(CLAW_MOTOR_PIN1,  0);
  analogWrite(CLAW_MOTOR_PIN2, 0);
}
