//Sample Mini Sumo bot behavior.
//This will run to the edges of the board and then 
//turn around.
//Then if the front sensor detects the other robot it will drive forward
// full speed to push them off the arena.

///Pin Settings for the robot's sensors.
/*
A0 Line follower, light/dark detector New Sensor White surface = >800 Black surface = < 50
A3 Sharp IR Range Finder Object detected raw analog reading > 300
D0 RX Serial com to USB
D1 TX Serial com to USB
D3 PWM2B
D4 DIR_CLK
D5 PWM0B
D6 PWM0A
D7 DIR_EN
D8 DIR_SER
D9 PWM1A
D10 PWM1B
D11 PWM2A
D12 DIR_LATCH
*/

#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #if defined(__AVR__)
    #include <avr/io.h>
  #endif
  #include "WProgram.h"
#endif

#include "AFMotor.h"

//Configuration Section
int LINESENSOR = A0;
int FRONTSENSOR = A3;
int LINESENSE = 300; //value when over white edge line
int MIN_DISTANCE = 400; //4 cm to enemy bot
int MINSPEED = 80;
int NORMALSPEED = 90;
int MAXSPEED = 255;

AF_DCMotor motor1(1, MOTOR12_64KHZ);
AF_DCMotor motor2(2, MOTOR12_64KHZ);

void setup()
{
  pinMode(LINESENSOR, INPUT);
  pinMode(FRONTSENSOR, INPUT);
  //start with the motors stopped.
  fullStop();
  Serial.begin(9600);
  Serial.println("Mini Sumo Bot test start!");
  Serial.println("Waiting five seconds...");
  waitFiveSeconds();
}

void loop()
{
  //Read all the sensors
  int lineValue = analogRead(LINESENSOR);
  int frontValue = analogRead(FRONTSENSOR);
  Serial.print("line value: ");
  Serial.println(lineValue);
  Serial.print("front value: ");
  Serial.println(frontValue);
  
  //Make descision based on sensor readings
  if (lineValue > LINESENSE)
  {
    //then we've detected the white line on the edge. Turn around!
    fullStop();
    delay(100);
    goReverse(NORMALSPEED);
    delay(1000);
    turnRight(NORMALSPEED);
    delay(1000);
  }
  else if (frontValue > MIN_DISTANCE)
  {
    //then we've detected the opponent in front of us. Full speed ahead!
    goForward(MAXSPEED);
  }
  else
  {
    //not on the edge, no robots around, just move forward.
    goForward(NORMALSPEED);
  }
}

void fullStop()
{
  //stops both motors.
  motor1.run(RELEASE);
  motor2.run(RELEASE);
}

void goForward(int speed)
{
  //speed is 70-255
  //Note that the motors are wired up backwards so we need to tell it
  //BACKWARD for forward and vice-versa.
  motor1.setSpeed(speed);
  motor2.setSpeed(speed);
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
}

void goReverse(int speed)
{
  //speed is 70-255
  //Note that the motors are wired up backwards so we need to tell it
  //BACKWARD for forward and vice-versa.
  motor1.setSpeed(speed);
  motor2.setSpeed(speed);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
}

void turnRight(int speed)
{
  //speed is 70-255
  //Note that the motors are wired up backwards so we need to tell it
  //BACKWARD for forward and vice-versa.
  motor1.setSpeed(speed);
  motor2.setSpeed(speed);
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
}

void waitFiveSeconds()
{
  //the mini sumo bot competition rules require your bot to wait
  //five seconds before it starts to move to give humans time to move
  //back away from the arena.
  delay(5000);
}
