#include <Arduino.h>
#include <AFMotor.h>

#define NUM_SAMPLES 10
#define CORNER_0 0
#define CORNER_1 1
#define CORNER_2 2
#define CORNER_3 3

#define MOTOR_PWM 5

AF_DCMotor motor(1, MOTOR12_64KHZ); // create motor #2, 64KHz pwm

void setup() {
  pinMode(MOTOR_PWM, OUTPUT);
  analogReference(DEFAULT);
  Serial.begin(9600);
  motor.setSpeed(250);
}

int main () {
  int moving_average_index = 0;
  int a, b, c, d, x, y, xout, yout, i;
  int xbuffer[NUM_SAMPLES];
  int ybuffer[NUM_SAMPLES];

  init();
  setup();

  while (1) {
    //read the four corners
    a = analogRead(CORNER_0);
    b = analogRead(CORNER_1);
    c = analogRead(CORNER_2);
    d = analogRead(CORNER_3);

    //do the x magic
    //x = (2.0663707*c + 2.0544114*a - 1.1423724*b - 1.5299211*d - 1266.8639)/1.5;
    x = (c + a - b - d - 0)/1.5;

    //do the y magic
    //y = (2.6366761*b + 3.5104709*a - 0.41156325*c - 1.3292001*d - 3814.8667)/1.5;
    y = (b + a - c - d - 0)/1.5;
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
    if (xout < -50) {
      motor.run(FORWARD);
    }

    else if (xout == 0) {
      motor.run(RELEASE);
    }
    else if (xout > 30) {
      motor.run(BACKWARD);
    }
    //send x y positions to computer. A program of your own is required to receive this and interpret it into mouse movement.
    Serial.print("x ");
    Serial.print(xout);
    delay(25);
    Serial.print(" y ");
    Serial.print(yout);
    Serial.print("\n");
    delay(25);
  }
}
