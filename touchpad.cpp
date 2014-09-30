#include <>
int a, b, c, d, x, y, xout, yout, i;
int xbuffer[10];
int ybuffer[10];

void setup() {
  analogReference(DEFAULT);
  Serial.begin(9600);
}

void loop() {
  //read the four corners
  a = analogRead(0);
  b = analogRead(1);
  c = analogRead(2);
  d = analogRead(3);
  //

  x = (2.0663707*c + 2.0544114*a - 1.1423724*b - 1.5299211*d - 1266.8639) / 1.5; //do the x magic
  y = (2.6366761*b + 3.5104709*a - 0.41156325*c - 1.3292001*d - 3814.8667) / 1.5; //do the y magic
  //(the above formulas are derived from the properties of MY piece of paper + graphite and probably won't work well with any other)

  //buffering stuff to smooth mouse motion
  xbuffer[0] = x;
  ybuffer[0] = y;

  xout = 0;
  yout = 0;

  for(i=0; i<9; i++)
  {
    xout = xout + xbuffer[i];
    yout = yout + ybuffer[i];
  }

  xout = xout / 10;
  yout = yout / 10;

  for(i=1; i<9; i++)
  {
    xbuffer[i] = xbuffer[i-1];
    ybuffer[i] = ybuffer[i-1];
  }
  //end of buffering

  //send x y positions to computer. A program of your own is required to receive this and interpret it into mouse movement.
  Serial.print("x");
  Serial.print(xout);
  delay(25);
  Serial.print("y");
  Serial.print(yout);
  Serial.print("e");
  delay(25);
}
