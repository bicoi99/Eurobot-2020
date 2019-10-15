#include <TM1637Display.h>
#include <Wire.h>
#include <Enums.MD25.h>
#include <MD25.h>
#include <Ultrasonic.h>
#include <Servo.h>
#include <Arduino.h>

#define CLK 12
#define DIO 13

#define TEST_DELAY  2000

TM1637Display display(CLK, DIO);
/*
 * Demonstrates the use of interrupts.
 * This will pause execution whilst an IR
 * range finder detects something within
 * 10cm, and end the program once 20seconds
 * has passed.
 * 
 * The first interrupt function passed to
 * an Advanced function will only pause
 * execution until as long as it is true.
 * 
 * The second interrupt function will end
 * the function, and will return TRUE, so
 * that you can then use that information
 * as required in your program.
 */

 const int MicroSwitch = 6;
 int SwitchStateMicro = 0;
 const int ToggleSwitch = 10;
 int SwitchStateToggle = 0;
 const int CourseSwitch = 11;
 int SwitchStateCourse= 0;

Servo leftservo;
Servo rightservo;
Servo grabberservo;

int pos = 0;    // variable to store the servo position

using namespace RD02;  // everything is kept in here to prevent possible clashes
                // if you don't understand namespaces, just put this in then forget about it <3

MD25 md25;  // make and empty MD25 object

long timer;  // to record the start time of the program.

Ultrasonic ultrafront(2,4);   // (Trig PIN,Echo PIN)
Ultrasonic ultraback(3,5);  // (Trig PIN,Echo PIN)

void setup()
{
md25 = MD25(99 * PI, 266.6/2, 20, Motor::_SWAPPED, 1);  // define the MD25 object
                // first parameter is the wheel circumference used (wheel diameter * PI)
                // second parameter is the robot's turn radius (half the distance between each wheel)
                // third parameter is the speed
                // fourth parameter says whether the motors are swapped
                // fifth parameter determines the acceleration
  leftservo.attach(7);  // attaches the left servo on pin 8 to the servo object
  rightservo.attach(8); // attaches the right servo on pin 9 to the servo object
  grabberservo.attach(9); // attaches the grabber servo on pin 10 to the servo object
  leftservo.write(150); // sets the left servo to 90 degrees (parallel to the side of the robot)
  rightservo.write(180); // sets the right servo to 160 degrees (parallel to the side of the robot)
  grabberservo.write(165); // resets the grabber

  display.setBrightness(0x0f);

  uint8_t blank[] = { 0x0,  0x0,  0x0,  0x0 };
  display.setSegments(blank);
  
}

void loop ()
{
  SwitchStateToggle = digitalRead(ToggleSwitch);
  if (SwitchStateToggle == LOW) //YELLOW SIDE
{
  SwitchStateCourse = digitalRead(CourseSwitch);
  if (SwitchStateCourse == LOW) //AGGRESSIVE
{
  SwitchStateMicro = digitalRead(MicroSwitch);
  if (SwitchStateMicro == LOW)
{
  timer = millis();
  if(md25.AdvancedForward(340, noCheck, checkTime, 25, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  display.showNumberDec(40, false, 2, 2); 
  if(md25.AdvancedForward(-70, checkBack, checkTime, 25, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;
  if(md25.AdvancedTurn(90, noCheck, checkTime, 10, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(900, checkFront, checkTime,30 , falloffFunctions::Final10percent_linear_10percent)) goto timeUp;
  if(md25.AdvancedTurn(90, checkFront, checkTime, 25, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(-135, noCheck, checkTime, 20)) goto timeUp;   // move forwards 155mm
  if(md25.AdvancedForward(91, checkFront, checkTime, 10, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedTurn(-90, checkFront, checkTime, 10, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(380, checkFront, checkTime,20 , falloffFunctions::Final10percent_linear_10percent)) goto timeUp;
  leftservo.write(68);
  delay(1000);
  if(md25.AdvancedForward(100, checkFront, checkTime, 25, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;
  display.showNumberDec(60, false, 2, 2); 
  if(md25.AdvancedForward(-100, checkBack, checkTime, 25, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;
  leftservo.write(150);
  if(md25.AdvancedTurn(90, checkFront, checkTime, 25, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(-115, noCheck, checkTime, 20)) goto timeUp;   // move forwards 155mm
  if(md25.AdvancedForward(200, checkFront, checkTime, 20, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;
  if(md25.AdvancedTurn(-90, checkFrontandBack, checkTime, 8, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(642.5, checkFront, checkTime, 20, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;
  if(md25.AdvancedTurn(-90, checkFrontandBack, checkTime, 8, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  delay(500);
  if(md25.AdvancedForward(45, noCheck, checkTime, 5)) goto timeUp;
  delay(1000);
  grabberservo.write(35);
  delay(2000);
  if(md25.AdvancedTurn(2.5, noCheck, checkTime, 10)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program en
  delay(1000);
  if(md25.AdvancedForward(-70, checkBack, checkTime, 20, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;
  display.showNumberDec(80, false, 2, 2); 
  if(md25.AdvancedTurn(-92.5, checkFrontGrabberandBack, checkTime, 10, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(1075, checkFrontGrabber, checkTime, 30, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;
  if(md25.AdvancedTurn(-90, checkFrontGrabber, checkTime, 25, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(-260, noCheck, checkTime, 18)) goto timeUp;   // move forwards 155mm
  if(md25.AdvancedForward(950, checkFrontGrabber, checkTime, 18, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;
  if(md25.AdvancedTurn(-12.5, checkFrontGrabberandBack, checkTime, 10, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(120, checkFrontGrabber, checkTime, 20)) goto timeUp;
  if(md25.AdvancedForward(180, noCheck, checkTime, 20)) goto timeUp;
  delay(1000);
  grabberservo.write(165);
  display.showNumberDec(104, false, 3, 1); 
  if(md25.AdvancedForward(-100, checkBack, checkTime, 20, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;
  if(md25.AdvancedTurn(147.5, noCheck, checkTime, 10, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(1075, checkFront, checkTime, 25)) goto timeUp;
  display.showNumberDec(117, false, 3, 1);
  if(md25.AdvancedForward(-280, checkBack, checkTime, 35)) goto timeUp;
  if(md25.AdvancedTurn(30, checkFrontandBack, checkTime, 25, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(225, checkFront, checkTime, 35)) goto timeUp;
  if(md25.AdvancedArc(133, -26, checkFront, checkTime, 25)) goto timeUp;
  display.showNumberDec(118, false, 3, 1);
  while(true);

  for(int i = 0; i < 50; i++)  // delays must be done like this if you don't want to exceed a time limit.
  {
    if (checkTime()) goto timeUp;
    delay (10);
  }
  timeUp:
  while(true);  // ends the program
}}
  SwitchStateCourse = digitalRead(CourseSwitch);
  if (SwitchStateCourse == HIGH) //YELLOW CONSERVATIVE
{
  SwitchStateMicro = digitalRead(MicroSwitch);
  if (SwitchStateMicro == LOW)
{
  timer = millis();
  if(md25.AdvancedForward(340, noCheck, checkTime, 20, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  display.showNumberDec(40, false, 2, 2);
  if(md25.AdvancedForward(-1650, checkBack, checkTime, 35, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedForward(-100, noCheck, checkTime, 20)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedForward(95, checkFront, checkTime, 20, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedTurn(-90, noCheck, checkTime, 20, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(-400, noCheck, checkTime, 25)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedForward(-700, noCheck, checkTime, 120)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  display.showNumberDec(48, false, 2, 2);
  if(md25.AdvancedForward(975, noCheck, checkTime, 40)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedForward(-60, checkBack, checkTime, 20, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedTurn(90, noCheck, checkTime, 20, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(-140, noCheck, checkTime, 20)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedForward(565, checkFront, checkTime, 25, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedTurn(90, checkFront, checkTime, 20, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(245, checkFront, checkTime, 20, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedTurn(-90, checkFront, checkTime, 20, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(-100, noCheck, checkTime, 25)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedForward(800, checkFront, checkTime, 35, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedForward(-100, checkBack, checkTime, 25, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedTurn(90, checkFrontandBack, checkTime, 10, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(325, checkFront, checkTime, 25, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedTurn(-90, checkFrontandBack, checkTime, 10, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(305, checkFront, checkTime, 15, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedTurn(-90, checkFrontandBack, checkTime, 10, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(525, checkFront, checkTime, 25, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  display.showNumberDec(61, false, 2, 2);
  if(md25.AdvancedForward(-325, checkBack, checkTime, 25, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedTurn(-90, checkFrontandBack, checkTime, 20, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(800, checkFront, checkTime, 30)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedForward(102.5, noCheck, checkTime, 20)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedForward(-75, checkBack, checkTime, 20, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedTurn(-82.5, noCheck, checkTime, 10, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(575, checkFront, checkTime, 25)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedTurn(-132.5, noCheck, checkTime, 15, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(1000, checkFront, checkTime, 25)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedArc(280, -55, checkFront, checkTime, 5)) goto timeUp;
  display.showNumberDec(69, false, 2, 2);
  while(true);

  for(int i = 0; i < 50; i++)  // delays must be done like this if you don't want to exceed a time limit.
  {
    if (checkTime()) goto timeUp;
    delay (10);
  }
}}}
  SwitchStateToggle = digitalRead(ToggleSwitch);
  if (SwitchStateToggle == HIGH) //PURPLE SIDE
{
  SwitchStateCourse = digitalRead(CourseSwitch);
  if (SwitchStateCourse == LOW) //AGGRESSIVE 
{
  SwitchStateMicro = digitalRead(MicroSwitch);
  if (SwitchStateMicro == LOW)
{
  timer = millis();
  if(md25.AdvancedForward(340, noCheck, checkTime, 25, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  display.showNumberDec(40, false, 2, 2);
  if(md25.AdvancedForward(-70, checkBack, checkTime, 25, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;
  if(md25.AdvancedTurn(-90, noCheck, checkTime, 10, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(900, checkFront, checkTime, 30, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;
  if(md25.AdvancedTurn(-90, checkFront, checkTime, 25, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(-135, noCheck, checkTime, 20)) goto timeUp;   // move forwards 155mm
  if(md25.AdvancedForward(92.5, checkFront, checkTime, 10, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedTurn(90, checkFront, checkTime, 10, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(380, checkFront, checkTime,20 , falloffFunctions::Final10percent_linear_10percent)) goto timeUp;
  rightservo.write(90);
  delay(1000);
  if(md25.AdvancedForward(110, checkFront, checkTime, 25, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;
  display.showNumberDec(60, false, 2, 2);
  if(md25.AdvancedForward(-110, checkBack, checkTime, 25, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;
  rightservo.write(170);
  if(md25.AdvancedTurn(-90, checkFront, checkTime, 25, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(-115, noCheck, checkTime, 20)) goto timeUp;   // move forwards 155mm
  if(md25.AdvancedForward(200, checkFront, checkTime,20, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;
  if(md25.AdvancedTurn(90, checkFrontandBack, checkTime, 8, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(645, checkFront, checkTime, 20, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;
  if(md25.AdvancedTurn(90, checkFrontandBack, checkTime, 8, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  delay(500);
  if(md25.AdvancedForward(52.5, noCheck, checkTime, 5)) goto timeUp;
  delay(1000);
  grabberservo.write(40);
  delay(2000);
  if(md25.AdvancedTurn(-2.5, noCheck, checkTime, 10)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program en
  delay(1000);
  if(md25.AdvancedForward(-70, checkBack, checkTime, 20, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;
  display.showNumberDec(80, false, 2, 2);
  if(md25.AdvancedTurn(92.5, checkFrontGrabberandBack, checkTime, 10, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(1075, checkFrontGrabber, checkTime, 30, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;
  if(md25.AdvancedTurn(90, checkFrontGrabber, checkTime, 25, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(-235, noCheck, checkTime, 18)) goto timeUp;   // move forwards 155mm
  if(md25.AdvancedForward(950, checkFrontGrabber, checkTime, 18, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;
  if(md25.AdvancedTurn(25.5, checkFrontGrabberandBack, checkTime, 10, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(120, checkFrontGrabber, checkTime, 20)) goto timeUp;
  if(md25.AdvancedForward(200, noCheck, checkTime, 20)) goto timeUp;
  delay(1000);
  grabberservo.write(165);
  display.showNumberDec(104, false, 3, 1);
  if(md25.AdvancedForward(-100, checkBack, checkTime, 20, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;
  if(md25.AdvancedTurn(-150, noCheck, checkTime, 10, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(1100, checkFront, checkTime, 25)) goto timeUp;
  display.showNumberDec(117, false, 3, 1);
  if(md25.AdvancedForward(-330, checkBack, checkTime, 35)) goto timeUp;
  if(md25.AdvancedTurn(-30, checkFrontandBack, checkTime, 25, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(225, checkFront, checkTime, 35)) goto timeUp;
  if(md25.AdvancedArc(133, 30, checkFront, checkTime, 25)) goto timeUp;
  display.showNumberDec(118, false, 3, 1);
  while(true);

  for(int i = 0; i < 50; i++)  // delays must be done like this if you don't want to exceed a time limit.
{
    if (checkTime()) goto timeUp;
    delay (10);
}
}}
  SwitchStateCourse = digitalRead(CourseSwitch);
  if (SwitchStateCourse == HIGH) //PURPLE CONSERVATIVE
{
  SwitchStateMicro = digitalRead(MicroSwitch);
  if (SwitchStateMicro == LOW)
{
  timer = millis();
  if(md25.AdvancedForward(340, noCheck, checkTime, 20, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  display.showNumberDec(40, false, 2, 2);
  if(md25.AdvancedForward(-1650, checkBack, checkTime, 35, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedForward(-100, noCheck, checkTime, 20)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedForward(105, checkFront, checkTime, 10, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedTurn(90, noCheck, checkTime, 20, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(-400, noCheck, checkTime, 25)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedForward(-700, noCheck, checkTime, 120)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  display.showNumberDec(48, false, 2, 2);
  if(md25.AdvancedForward(975, noCheck, checkTime, 40)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedForward(-60, checkBack, checkTime, 20, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedTurn(-90, noCheck, checkTime, 20, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(-170, noCheck, checkTime, 15)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedForward(575, checkFront, checkTime, 25, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedTurn(-90, checkFront, checkTime, 20, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(245, checkFront, checkTime, 20, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedTurn(90, checkFront, checkTime, 20, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(-100, noCheck, checkTime, 25)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedForward(800, checkFront, checkTime, 35, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedForward(-100, checkBack, checkTime, 25, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedTurn(-90, checkFrontandBack, checkTime, 10, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(325, checkFront, checkTime, 25, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedTurn(90, checkFrontandBack, checkTime, 10, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(305, checkFront, checkTime, 15, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedTurn(90, checkFrontandBack, checkTime, 10, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(525, checkFront, checkTime, 25, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  display.showNumberDec(61, false, 2, 2);
  if(md25.AdvancedForward(-325, checkBack, checkTime, 25, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedTurn(90, checkFrontandBack, checkTime, 20, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(800, checkFront, checkTime, 30)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedForward(102.5, noCheck, checkTime, 20)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedForward(-75, checkBack, checkTime, 25, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedTurn(82.5, noCheck, checkTime, 10, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(575, checkFront, checkTime, 25)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedTurn(130, noCheck, checkTime, 15, falloffFunctions::Final10percent_linear_10percent)) goto timeUp;  // if the time limit has been exceeded, this will take us to the program end
  if(md25.AdvancedForward(1000, checkFront, checkTime, 25)) goto timeUp;  // move forwards 5m, checking there's nothing in front of us we may crash into
  if(md25.AdvancedArc(280, 70, checkFront, checkTime, 5)) goto timeUp;
  display.showNumberDec(69, false, 2, 2);
  while(true);

  for(int i = 0; i < 50; i++)  // delays must be done like this if you don't want to exceed a time limit.
  {
    if (checkTime()) goto timeUp;
    delay (10);
  }
}}}}


//INTERRUPT FUNCTIONS//
bool checkFront()
{
  delay(35);
  if (((ultrafront.Ranging(CM)) < 9) and ((ultrafront.Ranging(CM)) > 4)) return true;
  return false;
}
bool checkBack()
{
  delay(35);
  if ((ultraback.Ranging(CM)) < 5) return true;
  return false; 
}
bool checkTime()
{
  if (millis() - timer > 100000) return true;
  return false;
}
bool noCheck()
{
  return false;
}
bool checkFrontGrabber()
{
  delay(35);
  if (((ultrafront.Ranging(CM)) < 17) and ((ultrafront.Ranging(CM)) > 12)) return true;
  return false;
}
bool checkFrontandBack()
{
  delay(35);
  if  ((((ultrafront.Ranging(CM)) < 9) and ((ultrafront.Ranging(CM)) > 4)) or ((ultraback.Ranging(CM)) < 5)) return true;
  return false;
}
bool checkFrontGrabberandBack()
{
   delay(35);
   if  ((((ultrafront.Ranging(CM)) < 17) and ((ultrafront.Ranging(CM)) > 12)) or ((ultraback.Ranging(CM)) < 5)) return true;
   return false; 
}
