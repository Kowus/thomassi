#include <MPU6050_tockn.h>
#include <Wire.h>
#include <Math.h>
#include "helpers.h"

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
float prevAccX = 0, prevAccY = 0;

MPU6050 gyro(Wire);
Command command_u;

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
  // send ack to indicate ready to accept data
  prevT = micros();
}

void loop()
{
  // put your main code here, to run repeatedly:
  while (Serial.available() > 0)
  {
    commandData = Serial.readStringUntil('\n');
    command_u = parseCommand(commandData);

    yield();
  }
  /*
   * According to available spec, process only the following commands:
   * <Command><SubCommand> where Command is one of:
   * f - forward
   * b - backward
   * l - left
   * r - right
   * and SubCommand is an integer value representing the distance in cm
   * or angle in degrees for turns.
   * For example:
   * f100 - move forward 100 cm
   * b50 - move backward 50 cm
   * l90 - turn left 90 degrees
   * r45 - turn right 45 degrees
   * System should not respond to any other commands.
   * If no command is received, the system should not do anything.
   * If a command is received, it should be processed immediately.
   * If a command is received while another command is being processed,
   * the new command should not override the previous one.
   * The system should wait for the previous command to finish.
   */
  if (command_u.desiredAction != CommandType::NONE || command_u.desiredAction != CommandType::ERROR)
  {
    switch (command_u.desiredAction)
    {
    case CommandType::FORWARD:
      Serial.print("Moving forward ");
      Serial.print(command_u.SubCommand);
      Serial.println(" cm");
      // moveDistance(command_u.SubCommand);
      break;

    case CommandType::BACKWARD:
      Serial.print("Moving backward ");
      Serial.print(command_u.SubCommand);
      Serial.println(" cm");
      // moveDistance(-command_u.SubCommand);
      break;

    case CommandType::LEFT:
      Serial.print("Turning left ");
      Serial.print(command_u.SubCommand);
      Serial.println(" degrees");
      // turnLeft(command_u.SubCommand);
      break;

    case CommandType::RIGHT:
      Serial.print("Turning right ");
      Serial.print(command_u.SubCommand);
      Serial.println(" degrees");
      // turnRight(command_u.SubCommand);
      break;

    case CommandType::ERROR:
    default:
      Serial.println("Unknown command received");
      break;
    }
  }
  {
  }

  // Serial.print(commandData);
  // Serial.print(", ");
  // Serial.println(buf);

  gotData = false;
}

void stopMotors()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void doMove(CommandType action, float orientation)
{
  // gyro.
  float thetaX = gyro.getGyroAngleX();
  float thetaY = gyro.getGyroAngleY();
  float thetaZ = gyro.getGyroAngleZ();

  float accX = gyro.getAccX();
  float accY = gyro.getAccY();
  float theta = atanf(accY / accX) * RAD_TO_DEG;
  float r = sqrt(accX * accX + accY * accY);

  // better to keep working in micros till final
  // conversion to reduce quantization error
  long currT = micros();
  float deltaT = (float)(currT - prevT);
  float xResolved = r * cos(theta * DEG_TO_RAD);
  float yResolved = r * cos(theta * DEG_TO_RAD);

  float accelerationResolved = xResolved + yResolved;
  float velocityResolved = accelerationResolved * deltaT;
  float distanceResolved = velocityResolved * deltaT;
  tripDistance += distanceResolved;

  float kp = 0.025;
  float ki = 0.12;

  double thetaError = orientation - thetaZ;
}