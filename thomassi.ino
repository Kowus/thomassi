#include <MPU6050_tockn.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

// ESP
// const int in1 = 16, in2 = 17, in3 = 18, in4 = 19, led = 13;
const int in1 = 9, in2 = 5, in3 = 4, in4 = 3, led = 13;

float thetaDesired = 0.0f;

float x[] = {0, 0, 0};
float y[] = {0, 0, 0};

float b[] = {0.00024132, 0.00048264, 0.00024132};
float a[] = {1.95558189, -0.95654717};
float error_tolerance = 2.80f;
int k = 0, dataCmd, buf;
long prevT = 0, tripTime = 0;
char cmd, dir;

int nominalSpeed = 255;
String commandData = "";
bool gotData = false;
int provisionalDuration = 0;
float tripDistance = 0.0f;

MPU6050 gyro(Wire);

int initAngleZ = 0;
float currentTheta = 0.0f;
void setup()
{
  // put your setup code here, to run once:
  delay(1000);
  Serial.begin(9600);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(led, OUTPUT);
  Wire.begin();
  gyro.begin();
  gyro.calcGyroOffsets(true);
  digitalWrite(led, HIGH);
  delay(500);
  digitalWrite(led, LOW);
  delay(300);
  digitalWrite(led, HIGH);
  delay(300);
  digitalWrite(led, LOW);
  delay(500);
  digitalWrite(led, HIGH);
  prevT = micros();
}

void loop()
{
  // put your main code here, to run repeatedly:
  while (Serial.available() > 0)
  {
    commandData = Serial.readStringUntil('\n');
    gotData = true;

    yield();
  }

  if (gotData)
  {
    if (commandData[0])
    {
      Serial.println("Got Data");
      cmd = commandData[0];
      dir = commandData[1];
      String subCmd = commandData.substring(2, 5);
      buf = subCmd.toInt();
      if (cmd == 'f' || cmd == 'b')
      {
        thetaDesired = 0;
      }
      else if (cmd == 'l' || cmd == 'r')
      {
        thetaDesired = buf;
      }
      tripTime = 0;
      tripDistance = 0.0f;
    }
    // Serial.print(commandData);
    // Serial.print(", ");
    // Serial.println(buf);

    gotData = false;
  }

  gyro.update();
  float thetaX = gyro.getGyroAngleX();
  float thetaY = gyro.getGyroAngleY();
  float thetaZ = gyro.getGyroAngleZ();
  float accX = gyro.getAccX();
  float accY = gyro.getAccY();
  float theta = atanf(accY / accX) * RAD_TO_DEG;
  float argL = sqrt(accX * accX + accY * accY);
  long currT = micros();
  float t = currT / 1.0e6;
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  prevT = currT;

  float numResolved = argL * cos(theta * DEG_TO_RAD);
  float denResolved = argL * sin(theta * DEG_TO_RAD);
  float accelerationResolved = numResolved + denResolved;
  float velocityResolved = accelerationResolved * deltaT;
  float distanceResolved = velocityResolved * deltaT;
  tripDistance += distanceResolved;

  // PID Cooefficients
  float kp = 0.025;
  float ki = 0.12;

  double thetaError = thetaDesired - thetaZ;
  double teRad = thetaError * DEG_TO_RAD;
  double sinTRad = sin(teRad);

  float uSpeed = (kp * sinTRad * nominalSpeed * RAD_TO_DEG);
  // float uAc = nominalSpeed

  int vSpeed = (int)fabs(uSpeed);
  if (vSpeed > 255)
    vSpeed = 255;
  // float sinError = sinh(thetaError);
  /*
  Serial.print(thetaZ);
  Serial.print(",\t");
  Serial.print(thetaError);
  Serial.print(",\t");
  Serial.print(teRad);
  Serial.print(",\t");
  Serial.print(sinTRad);
  Serial.print(",\t");
  Serial.print(uSpeed);
  Serial.print(",\t");
  Serial.println(vSpeed);
  */

  if (cmd && cmd != 's' && (cmd == 'f' || cmd == 'b'))
  {
    int isFwd = cmd == 'f';
    if (thetaError < (-1 * error_tolerance))
    {
      // LEFT
      Serial.println("LEFT");
      analogWrite(in1, vSpeed * isFwd);
      analogWrite(in2, vSpeed * !isFwd);
      analogWrite(in3, vSpeed * !isFwd);
      analogWrite(in4, vSpeed * isFwd);
    }
    else if (thetaError > error_tolerance)
    {
      // RIGHT
      Serial.println("RIGHT");
      analogWrite(in1, vSpeed * !isFwd);
      analogWrite(in2, vSpeed * isFwd);
      analogWrite(in3, vSpeed * isFwd);
      analogWrite(in4, vSpeed * !isFwd);
    }
    else
    {
      // FORWARD
      vSpeed = 60;
      Serial.println("FORWARD");
      analogWrite(in1, vSpeed * !isFwd);
      analogWrite(in2, vSpeed * isFwd);
      analogWrite(in3, vSpeed * !isFwd);
      analogWrite(in4, vSpeed * isFwd);
    }
  }
  else if (cmd && cmd != 's' && (cmd == 'l' || cmd == 'r'))
  {
    if (!initAngleZ)
    {
      currentTheta = gyro.getGyroAngleZ();
      initAngleZ = 1;
    }
    // TURN
    int turnDirection = (cmd == 'l');
    int needsTurn = fabs(currentTheta - thetaZ) < (thetaDesired + error_tolerance);
    // Serial.println("TURN");
    analogWrite(in1, vSpeed * turnDirection * needsTurn);
    analogWrite(in2, vSpeed * !turnDirection * needsTurn);
    analogWrite(in3, vSpeed * turnDirection * needsTurn);
    analogWrite(in4, vSpeed * !turnDirection * needsTurn);
    if (!needsTurn)
      initAngleZ = 0; // reset init angle if turn is done
    /*
    if (cmd == 'l')
    {
      // LEFT
      analogWrite(in1, vSpeed);
      digitalWrite(in2, LOW);
      analogWrite(in3, vSpeed);
      digitalWrite(in4, LOW);
    }
    else if (dir == 'r')
    {
      // RIGHT
      digitalWrite(in1, LOW);
      analogWrite(in2, vSpeed);
      digitalWrite(in3, LOW);
      analogWrite(in4, vSpeed);
    }
    */
  }
  else
  {
    // STOP
    Serial.println("STOPPED");
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
  // Serial.println(tripDistance);

  // forward
  // analogWrite(in1, vSpeed);
  // digitalWrite(in2, LOW);
  // analogWrite(in3, vSpeed);
  // digitalWrite(in4, LOW);
}
