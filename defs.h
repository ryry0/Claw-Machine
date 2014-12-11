//THIS FILE CONTAINS ALL THE DEFINITIONS IN ONE HANDY PLACE
#ifndef DEFS_H_
#define DEFS_H_

//analog pins used for corners
#define CORNER_0 0
#define CORNER_1 1
#define CORNER_2 2
#define CORNER_3 3
#define STYLUS_PIN 4

#define TOUCH_TOP -100
#define TOUCH_BOTTOM 50
#define TOUCH_LEFT 100
#define TOUCH_RIGHT -50

//#define PWM_REMOVAL
#define MOTOR_SCALER -100
#define X_VEL_SCALER 600
#define Y_VEL_SCALER -100000

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

#define TICKS_PER_REVOLUTION 1680

//this is the number of ticks for CTC mode
#define SAMPLE_RATE 200 //Hz
#define CTC_MATCH 10000 //*should* run the interrupt at 200Hz
#define SAMPLE_TIME 0.005

#define PRESCALE_8    0x02

//motor constants
#define KP 0.9 //7 //5
#define KI 0
#define KD 4.5

#define ERR_TOLERANCE 1.0
#define ENC_ERROR 10.0

//Software boundaries
#define X_MIN 0
#define X_MAX 8400
#define MAX_X_SPEED 1600 //ticks/s

#define Y_MIN 0
#define Y_END 200
#define Y_MAX 9600
#define MAX_Y_SPEED 1600 //ticks/s

#define WINCH_MIN 0
#define WINCH_MAX 20000

#define POS_TOLERANCE 300 //position tolerance "must be within 100 ticks of x"
#define SQUARE_X 7000 //x position of square
#define SQUARE_Y 200 //Y position of square
#define WINCH_END 2000

#define NUM_SAMPLES 5 //number of samples for touchpad moving avg

#define FILTER_VELOCITY
#define VELOCITY_SAMPLES 10
const float VEL_FILTER_CONSTANT = 1/float(VELOCITY_SAMPLES);

#define DEBOUNCE_BUTTON //enable software filtering of button

#define DEBOUNCE_SAMPLES 50
//constants for software debouncing
const float DEBOUNCE_CONSTANT = 1/float(DEBOUNCE_SAMPLES);

#define GREEN_LED 30
#define RED_LED 31

#endif
