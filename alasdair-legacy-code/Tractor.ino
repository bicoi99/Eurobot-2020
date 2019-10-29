/*
  Sketch Name: Tractor

  Description: Sketch contains functions to drive forward and rotate a forward facing robot using MD25 motor drive. Should turn functions into a library soon.

  MD25 documentation: https://www.robot-electronics.co.uk/htm/md25i2c.htm

  Wire MD25 as shown in https://blackboard.soton.ac.uk/bbcswebdav/pid-4040104-dt-content-rid-3986367_1/courses/FEEG2001-31292-18-19/MD25%20Circuit%20Guide.pdf
  Wire buttons as shown in http://ardx.org/src/circ/CIRC07-sheet-OOML.pdf with buttons connected to pins 2 and 7
  2200 ohm pullup resistors used for MD25 and buttons

  Created 01/02/2019
  By Alasdair Mann and Will Birch-Tomlinson
  Modified 30/03/2019
  By Will Birch-Tomlinson

*/

/** Imported libraries **/
#include <Wire.h>                                             // Imports I2C bus library
#include <NewPing.h>                                          // Imports Ultrasonic Sensor library
#include <Servo.h>
#include <IRremote.h>

/** Constants **/
#define MD25ADDRESS         0x58                              // Address of the MD25
#define SPEED1              0x00                              // Byte to send speed to motor 1(left): -128 is full reverse, 0 is stop, 127 is full forward
#define SPEED2              0x01                              // Byte to send speed to motor 2(right): -128 is full reverse, 0 is stop, 127 is full forward
#define ENCODERONE          0x02                              // Byte to read motor encoder 1
#define ENCODERTWO          0x06                              // Byte to read motor encoder 2
#define ACCELERATION        0xE                               // Byte to define motor acceleration. Acceleration time = 25ms * |new speed-old speed|/(register)
#define CMD                 0x10                              // Byte to reset encoder values
#define MODE_SELECTOR       0xF                               // Byte to change between control MODES
#define BATTERY             0x0A                              // Byte to read battery voltage     
#define BOT_WIDTH           315                               // Robot's width in mm
#define WHEEL_TO_FRONT      86                               // Distance from centre of wheels to front of robot
#define WHEEL_TO_BACK       167                               // Distance from centre of wheels to back of robot
#define WHEEL_TO_WHEEL      285                               // Distance between two wheels in mm
#define PULL_SERVO          13                                // Servo 1 control pin
#define TRAY_SERVO          11                               // Servo 2 control pin
#define DOOR_SERVO          12                               // Servo 3 control pin
#define US_0                9                                 // Set trigger and echo pin for sensor 0
#define US_1                8                                 // Set trigger and echo pin for sensor 1
#define US_2                7                                 // Set trigger and echo pin for sensor 2
#define US_3                6                                 // Set trigger and echo pin for sensor 3
#define US_4                5                                 // Set trigger and echo pin for sensor 4
#define US_5                4                                 // Set trigger and echo pin for sensor 5
#define TRIGGER             2                                 // Microswitch control pin
#define IR_LED              3                                 // IR LED Control Pin
#define TOGGLE              10                                 // Toggle switch control pin
#define MAX_DISTANCE 400                                      // Maximum range for ultrasonic sensors

// Initialise an array of 6 NewPing objects for sensors
NewPing sonar[6] = {NewPing(US_0, US_0, MAX_DISTANCE), NewPing(US_1, US_1, MAX_DISTANCE), NewPing(US_2, US_2, MAX_DISTANCE),
                    NewPing(US_3, US_3, MAX_DISTANCE), NewPing(US_4, US_4, MAX_DISTANCE), NewPing(US_5, US_5, MAX_DISTANCE)
                   };

// Initialise servos
Servo tray;
Servo door;
Servo atom_pull;

// Initialise infrared sensor and decoder
IRsend irsend;

/* Test values hidden in code */
const unsigned long maxTime = 100000;                                   // Maximum time in milliseconds
const byte straightTolerance = 0;                             // Accepted tolerance of driving straight in mm without correcting
const byte rotationTolerance = 0;                             // Accepted tolerance of rotation in mm without correcting
const double conversionFactor = 0.8726;                // Value that converts encoder readings to mm, theoretically = Diameter/(360*pi)
float fudge = 0.05;

// Constants for servo positions
int tray_horizontal_pos = 120;
int tray_dispenser_pos = 98 ;  //was 50
int tray_down_pos = 45;

int door_up_pos = 150;
int door_down_pos = 50;

int atom_pull_up_pos = 0;
int atom_pull_down_pos = 80;

/** Universal variables **/
boolean isHomologating = false;                                //true if homologating
boolean isOrangeSide = false;                                  // True if starting on orange side of game board
unsigned long startTime;                                       // Time at which the robot is triggered

/** Program set up **/
void setup() {
  // Begin communications
  Wire.begin();                                               // I2C bus
  Serial.begin(9600);                                         // Serial at 9600 baud
  delay(100);                                                 // Wait for everything to power up

  // Attach servos to pins
  atom_pull.attach(PULL_SERVO);
  tray.attach(TRAY_SERVO);
  door.attach(DOOR_SERVO);

  // Set MD25 operation MODE
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(MODE_SELECTOR);
  Wire.write(1);                                              // Set MD25 to mode 1, mode described in constants
  Wire.endTransmission();

  // Initialise pins as inputs to receive HIGH or LOW signal
  pinMode(TOGGLE, INPUT);                                          // Pin connected to button or toggle switch for choosing side of game board
  pinMode(TRIGGER, INPUT);                                          // Pin connected to micro switch for triggering

  // Resets encoders to 0 incase wheels moved during set up
  encoderReset();

  // Makes sure mechanisms are all inside
  tray.write(tray_down_pos);
  door.write(door_down_pos);
  atom_pull.write(atom_pull_up_pos);
}

/** Program loop **/
void loop() {
  //atom_pull.write(atom_pull_down_pos);
  complete_path();
  //purple_path();
  //orange_path();

  /**rotation calibration **/
  //rotate(90, 50, 2, 1, true);

  /*Video Tracking*/    //doing a left handed square
  //  delay(500);
  //  straight(500, 30, 2, 2, true, false);
  //  rotate(90, 50, 2, 1, true);
  //  straight(500, 30, 2, 2, true, false);
  //  rotate(90, 50, 2, 1, true);
  //  straight(500, 30, 2, 2, true, false);
  //  rotate(90, 50, 2, 1, true);
  //  straight(500, 30, 2, 2, true, false);
  //  rotate(90, 50, 2, 1, true);
}

/* Linear drive function that can predict when to start decelerating */
/** float distance: distance robot travels linearly in mm, a negative disntance means backwards movement */
/** byte speedValue: speed from 0 to 127 at which the wheels move at */
/** byte accelerationRegister: deceleration rate from 1-10, 1 being slowest, 10 being fastest. */
/** byte decelerationRegister: deceleration rate from 1-10, 1 being slowest, 10 being fastest. */
/** boolean ifCorrecting: true if correcting over/undershoots */
/** boolean ifSensing: true if collision with another robot is probable */
void straight(float distance, short speedValue, byte accelerationRegister, byte decelerationRegister, boolean ifCorrecting, boolean ifSensing) {
  // Speed is inverted if driving in reverse
  if (distance < 0) {
    speedValue = -1 * speedValue;
  }

  // Stops motors accounting for deceleration
  drive(distance, speedValue, speedValue, accelerationRegister, decelerationRegister, ifSensing);

  // Corrects linear movement if error is bigger than tolerance in mm
  if (ifCorrecting && abs(encoder(1) - distance) > straightTolerance && abs(encoder(2) - distance) > straightTolerance) {
    correctStraight(distance);
  }
  // Need to rest encoders after each movement because distance accumulates
  encoderReset();
}

void leftWheelFasterStraight(float distance, short speedValue, byte accelerationRegister, byte decelerationRegister, boolean ifSensing) {
  // Speed is inverted if driving in reverse
  fudge = 0.2;
  if (distance < 0) {
    speedValue = -1 * speedValue;
  }

  short speedValue2 = speedValue * 0.2;

  // Stops motors accounting for deceleration
  drive(distance, speedValue, speedValue2, accelerationRegister, decelerationRegister, ifSensing);

  // Need to rest encoders after each movement because distance accumulates
  encoderReset();

  fudge = 0.05;
}

void rightWheelFasterStraight(float distance, short speedValue2, byte accelerationRegister, byte decelerationRegister, boolean ifSensing) {
  // Speed is inverted if driving in reverse

  fudge = 0.2;

  if (distance < 0) {
    speedValue2 = -1 * speedValue2;
  }

  short speedValue = speedValue2 * 0.2;

  // Stops motors accounting for deceleration
  drive(distance, speedValue, speedValue2, accelerationRegister, decelerationRegister, ifSensing);

  // Need to rest encoders after each movement because distance accumulates
  encoderReset();

  fudge = 0.05;
}

/* Stationary spinning drive function */
/** float angle: spinning angle in degrees, a negative angle gives an anticlockwise rotation */
/** byte speedValue: Value from 0 to 127 at which wheels move. Wheels move in opposite directions, should always be positive */
/** byte accelerationRegister: deceleration rate from 1-10, 1 being slowest, 10 being fastest. */
/** byte decelerationRegister: deceleration rate from 1-10, 1 being slowest, 10 being fastest. */
/** boolean ifCorrecting: true if correcting over/undershoots */
void rotate(float angle, short speedValue, byte accelerationRegister, byte decelerationRegister, boolean ifCorrecting) {

  // Speed is inverted if turning anticlockwise
  if (angle < 0) {
    speedValue = -speedValue;
  }

  // Variables calculated before rotating based on input
  angle = (angle / 180) * PI;                                                     // Converts angle to radians
  float distance = 0.5 * (WHEEL_TO_WHEEL) * angle;                               // s=r*theta, distance moved for each wheel is distance along "circumference" of bot

  // Stops motors accounting for deceleration
  drive(distance, speedValue, -speedValue, accelerationRegister, decelerationRegister, 0);

  printEncoders("Before correction", "rotate", distance, 0, 0);

  // Corrects if error is bigger than tolerance in %
  if (ifCorrecting && abs(encoder(1) - distance) > rotationTolerance && abs(encoder(2) - distance) > rotationTolerance) {
    correctRotation(distance);
  }

  // Need to rest encoders after each movement because distance accumulates
  encoderReset();
}

/*void arc(float radius, float angle, short outerSpeedValue, byte accelerationRegister, byte deceleraetionRegister, boolean ifSensing){
  angle = (angle/180)*PI;
  float outerDistance = angle*(radius+(WHEEL_TO_WHEEL/2));
  float innerDistance = angle*(radius-(WHEEL_TO_WHEEL/2));
  short innerSpeedValue = (innerDistance/outerDistance)*outerSpeedValue;

  }*/

/* Corrects errors after linear motion */
/** float distance: target distance, same value as initial linear motion **/
void correctStraight(float distance) {

  // Variables may be determined through testing
  short speedValue1 = 8;                                                               // Byte speed for correcting, test this value
  byte accelerationRegister = 3;                                                      // Acceleration rate from 1-10 for correcting, test this value
  byte decelerationRegister = 3;                                                     // Deceleration rate from 1-10 for correcting, test this value

  // Variables calculated before correcting based on input
  float distance1 = distance - encoder(1);                                                 // Distance to cover in correcting
  float distance2 = distance - encoder(2);                                                 // Each wheel may have travelled a different distance
  short speedValue2 = speedValue1 * abs(encoder(2) - distance) / abs(encoder(1) - distance); // Wheels will need to travel at different speeds if different distances were covered

  // Speeds are inverted if moving in reverse,there may be a situation where one wheel might have overshot while the other didn't
  if (distance1 < 0) {
    speedValue1 = -speedValue1;
  }
  if (distance2 < 0) {
    speedValue2 = -speedValue2;
  }

  // Hacky fix, required for testing, correction method requires values of encoders to reset, and arduino can't return an array
  float encoderA = encoder(1);
  float encoderB = encoder(2);

  // Resets encoders for new path
  encoderReset();

  drive(min(abs(distance1), abs(distance2)), speedValue1, speedValue2, accelerationRegister, decelerationRegister, false);

  printEncoders("After correction", "straight", distance, encoderA, encoderB);
}

/* Corrects errors after rotation */
/** float target: target distance, same value as calculated distance **/
void correctRotation(float target) {

  // Variables may be determined through testing
  short speedValue1 = 5;                                                               // Byte speed for correcting, test this value
  byte accelerationRegister = 1;                                                      // Acceleration rate from 1-10 for correcting, test this value
  byte decelerationRegister = 3;                                                      // Deceleration rate from 1-10 for correcting, test this value

  // Variables calculated before correcting based on input
  float distance1 = abs(target) - abs(encoder(1));                                   //distance to cover in correcting
  float distance2 = abs(target) - abs(encoder(2));                                   //each wheel may have travelled a different distance

  // Inverts speeds if overshot
  if (distance1 < 0 && encoder(1) > 0) {
    speedValue1 = speedValue1 * -1;
  } else if (distance1 > 0 && encoder(1) < 0) {
    speedValue1 = speedValue1 * -1;
  }

  //Wheels will need to travel at proportionally different speeds if different distance is covered
  short speedValue2 = -speedValue1 * abs(distance2) / abs(distance1);

  // Hacky fix, required for testing, correction method requires values of encoders to reset, and arduino can't return an array
  float encoderA = encoder(1);
  float encoderB = encoder(2);

  // Resets encoders for new path
  encoderReset();

  drive(min(abs(distance1), abs(distance2)), speedValue1, speedValue2, accelerationRegister, decelerationRegister, false);

  printEncoders("After correction", "rotate", target, encoderA, encoderB);
}

void drive(float distance, short speed1, short speed2, byte accelerationRegister, byte decelerationRegister, float scanningTo) {
  float encoderHistory[2][3] = {{0, 0, 0}, {0, 0, 0}};                                         // 2x3 matrix, row 1 for encoder 1, row 2 for encoder 2. Columns are for displacement at time stamps.
  float currentVelocities[2] = {0, 0};                                                         // Array for holding current velocities of each encoder
  unsigned long encoderTime[3] = {millis(), millis(), millis()};                                       // Array for holding time stamps
  float currentDecelerationDistance[2] = {0, 0};                                               // Array for holding current deceleration distances of each encoder
  float remainingDistance[2] = {0, 0};                                                         // Array for holding current remaining distance to cover of each encoder
  boolean dec = false;
  float scanDistance = 0;
  setMotors(accelerationRegister, speed1, speed2);
  while (dec == false) {
    checkTime();

    // Reads ultrasonic sensors before writing the next speed value, stops motors for as long as ultrasonics detect a surprising object

    // Currently collision distance is a constant, could possibly work by accounting for decelerationDistance. But I wasn't able to make it reliable.
    short collisionDistance = 250; //was 150
    bool stopped = false;
    // Calls scanning function and times for duration
    unsigned long startOfScan = millis();
    if (scanningTo != 0 && abs(encoder(1)) < abs(scanningTo)*abs(distance)) {
      if (scan(accelerationRegister, speed1, speed2, collisionDistance, distance, 10)) {
        scanDistance = abs(encoder(1));
      }
    }
    unsigned long endOfScan = millis();

    // Shifts previous displacement values backwards before new reading is taken
    encoderHistory[0][0] = encoderHistory[0][1];
    encoderHistory[0][1] = encoderHistory[0][2];
    encoderHistory[1][0] = encoderHistory[1][1];
    encoderHistory[1][1] = encoderHistory[1][2];

    // Reads encoder values and stores as the most recent value in array, [2], always positive
    encoderHistory[0][2] = abs(encoder(1));
    encoderHistory[1][2] = abs(encoder(2));

    // Shift existing time stamps backwards
    encoderTime[0] = encoderTime[1];
    encoderTime[1] = encoderTime[2];
    // Current time stamp stored as most recent value, [2]
    encoderTime[2] = millis();

    // Linearly estimates velocity at time value [1] using encoder and time values at [2] and [0]. Maximum sensible speed is 1 m/s.
    currentVelocities[0] = min(((encoderHistory[0][2] - encoderHistory[0][0]) / (encoderTime[2] - encoderTime[0])), 1);
    currentVelocities[1] = min(((encoderHistory[1][2] - encoderHistory[1][0]) / (encoderTime[2] - encoderTime[0])), 1);

    // Integrates linear velocity curve to get displacement while decelerating
    currentDecelerationDistance[0] = abs(currentVelocities[0] * (max(millis(), endOfScan) - startOfScan)) + abs(0.5 * currentVelocities[0] * 25 * speed1 / decelerationRegister);
    currentDecelerationDistance[1] = abs(currentVelocities[1] * (max(millis(), endOfScan) - startOfScan)) + abs(0.5 * currentVelocities[1] * 25 * speed2 / decelerationRegister);

    // Calculates remaining distance each wheel needs to cover
    remainingDistance[0] = abs(distance) - encoderHistory[0][2];
    remainingDistance[1] = abs(distance) - encoderHistory[1][2];

    Serial.println(currentVelocities[0]);

    // If statement prevents a overflow bug that occurs sometimes at the start of paths
    if (encoderHistory[0][2] > abs(distance) * 0.2 && encoderHistory[1][2] > abs(distance) * 0.2) {
      // dec becomes true if the deceleration distance exceeds the remaining distance
      if (abs(currentDecelerationDistance[0]) >= remainingDistance[0] || abs(currentDecelerationDistance[1]) >= remainingDistance[1] || encoderHistory[0][2] > (abs(distance)) || encoderHistory[1][2] > (abs(distance))) {
        dec = true;
      }
    }
    if (encoderHistory[0][2] > (abs(distance) * fudge + scanDistance) && currentVelocities[0] < 0.0001) {
      dec = true;
      Serial.println("TREUTRUETUEUREURUTEURUERUREURUERHAEIURHUIEHRIAHEIURHPIAEGUIAGEFUIGAHSCAKSDVLHJAVSDLYGASYDIGAILUSDGLUIASGDIUOGASIUDGIOUSDGUIASGDIUOAGSIUODGIUOASGCUIOGASIUDAIUSGDIUAGSDIGASDIUOASD");
    }

  }

  printEncoders("Before decelerating", "straight", distance, 0, 0);
  Serial.println("Distance to stop:");
  Serial.println(speed1);
  Serial.println(speed2);

  // Stops motors
  stopMotor(decelerationRegister);

  printEncoders("After decelerating", "straight", distance, 0, 0);
}

/* Stops motors at given rate */
/** byte rate: acceleration register from 1-10 MD25 takes **/
void stopMotor(byte rate) {

  // Set MD25 acceleration register and motor speeds
  setMotors(rate, 0, 0);

  // Initialise equal variable as false
  boolean equal = false;

  // Initialise comparison values as current encoder values
  float check1 = encoder(1);
  float check2 = encoder(2);

  // While loop to ensure encoders are at full stop
  while (equal == false) {
    // Delay between checks that may be tested
    delay(10);
    if (check1 == encoder(1) && check2 == encoder(2)) {
      // Exits loop if comparison values are the same after 10 ms
      equal = true;
    } else {
      // Updates comparison values if they are unequal to current encoder values
      check1 = encoder(1);
      check2 = encoder(2);
    }
  }
}

/* Calibrate robot speed with distance */
float callibrateSpeed(float newTarget) {
  return (max(min(abs(newTarget) / 1000, 1), 0.2));
}

/* Obstacle avoidance */
/** byte s: original speed of path */
/** float maxDistance: maximum distance before colliding */
/** float target: original intended path length */
/** byte dec: input deceleration register */
bool scan(byte a, short s1, short s2, float maxDistance, float target, byte dec) {
  // Starts loop and ends at different numbers depending on front or backwards facing
  // Front facing sensors are 0, 1, 2, back facing sensors are 3, 4, 5
  byte loopStart = 0;
  byte loopEnd = 2;
  if (s1 < 0) {
    loopStart = 3;
    loopEnd = 5;
  }

  // Sensor scans
  unsigned int distance[6];
  for (byte i = loopStart; i <= loopEnd; i = i + 1) {
    distance[i] = sonar[i].convert_cm(sonar[i].ping_median(3)) * 10;
  }

  // Robot stops while there is something in the way
  bool change = false;
  while ((distance[loopStart]   > 20 && distance[loopStart] < maxDistance)
         || (distance[loopStart + 1] > 20 && distance[loopStart + 1] < maxDistance)
         || (distance[loopStart + 2] > 20 && distance[loopStart + 2] < maxDistance)) {
    stopMotor(dec);
    // Sensors keep checking
    for (byte i = loopStart; i <= loopEnd; i = i + 1) {
      distance[i] = sonar[i].convert_cm(sonar[i].ping_median(3)) * 10;
    }
    change = true;
  }
  if (change == true) {
    setMotors(a, s1, s2);
    Serial.println("Sensor:");
    Serial.println(distance[0]);
  }
  return (change);
}

/* Function to reset encoders to 0*/
void encoderReset() {
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(CMD);
  Wire.write(0x20);                         // Command given by manufacturer
  Wire.endTransmission();
  delay(50);                                // Wait for reset
}

/* Read encoder bytes as a 32 bit long */
/** int value: encoder number, 1 or 2 **/
float encoder(byte value) {                                          // Read Encoder bytes into 32 bit long 8 bits at a time and bitshift up

  // Send byte to get a reading from encoder
  if (value = 1) {
    Wire.beginTransmission(MD25ADDRESS);
    Wire.write(ENCODERONE);
    Wire.endTransmission();
  } else if (value = 2) {
    Wire.beginTransmission(MD25ADDRESS);
    Wire.write(ENCODERTWO);
    Wire.endTransmission();
  }

  // Request 4 bytes from MD25
  Wire.requestFrom(MD25ADDRESS, 4);
  while (Wire.available() < 4);                           // Wait for 4 bytes to arrive
  long pos = Wire.read();                                 // First byte for encoder 1, HH.
  pos <<= 8;                                              // Bitshift up
  pos += Wire.read();                                     // Second byte for encoder 1, HL
  pos <<= 8;                                              // Bitshift up
  pos += Wire.read();                                     // Third byte for encoder 1, LH
  pos <<= 8;                                              // Bitshift up
  pos  += Wire.read();                                    // Fourth byte for encoder 1, LLalue
  delay(5);                                               // Wait for everything to make sure everything is sent

  return (pos * conversionFactor);
}

/* Read battery voltage in V */
float readBatteryVolts() {
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(BATTERY);
  Wire.endTransmission();

  Wire.requestFrom(MD25ADDRESS, 4);                          //Request 4 bytes from MD25
  byte voltage = Wire.read();
  return voltage / 10;                                       //Divide by 10 as values given in tenths of volts
}

/* Print encoder values */
/** String statement: String before printing values **/
/** String movement: straight or rotate **/
/** float distance: target distance in mm**/
/** float encoderA: value to add to encoder value if there was a reset **/
/** float encoderB: same as encoderA **/
void printEncoders(String statement, String movement, float distance, float encoderA, float encoderB) {

  Serial.println(statement);

  if (movement == "straight") {

    Serial.println("Encoder 1:");
    Serial.println(encoder(1) + encoderA);
    Serial.println("Encoder 2:");
    Serial.println(encoder(2) + encoderB);
  } else if (movement == "rotate") {
    // Rotation testing is in % overshoot
    Serial.println("Encoder 1:");
    Serial.println(abs(encoderA + encoder(1)) / abs(distance));
    Serial.println("Encoder 2:");
    Serial.println(abs(encoderB + encoder(2)) / abs(distance));
  }
}

/* Change acceleration and speed of motors */
/** int rate: acceleration register from 1-10 */
/** int speed1: speed for wheel 1 */
/** int speed2: speed for wheel 2 */
void setMotors(byte rate, short speed1, short speed2) {
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(ACCELERATION);
  Wire.write(rate);
  Wire.endTransmission();

  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(SPEED1);
  Wire.write(speed1);
  Wire.endTransmission();

  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(SPEED2);
  Wire.write(speed2);
  Wire.endTransmission();
}

/* Check if its time to stop */
void checkTime() {
  boolean neverFalse = true;                //boolean to put arduino in infinite loop
  if (millis() - startTime > maxTime) {
    while (neverFalse) {
      stopMotor(10);
    }
  }
}

// this is the function to keep the tractor stopped until the input amount of seconds out of 100
void waitUntil(unsigned long milli) {
  while ((millis() - startTime) < milli) {
    stopMotor(10);
  }
}

/* Servo functions */

//Back door up and down functions
void door_up() {
  for (int pos = door_down_pos; pos <= door_up_pos; pos += 1) {
    door.write(pos);
  }
}

void door_down() {
  for (int pos = door_up_pos; pos >= door_down_pos; pos -= 1) {
    door.write(pos);
  }
}


//Tray up and down functions
void tray_up_horizontal() {
  for (int pos = tray_down_pos; pos <= tray_horizontal_pos; pos += 1) {
    tray.write(pos);
    delay(10);
  }
}

void tray_up_dispenser() {
  for (int pos = tray_down_pos; pos <= tray_dispenser_pos; pos += 1) {
    tray.write(pos);
    delay(10);
  }
}

void tray_up_dispenser_horizontal() {
  for (int pos = tray_dispenser_pos; pos <= tray_horizontal_pos; pos += 1) {
    tray.write(pos);
    delay(10);
  }
}

void tray_down_horizontal() {
  for (int pos = tray_horizontal_pos; pos >= tray_down_pos; pos -= 1) {
    tray.write(pos);
    delay(10);
  }
}


//Atom Pull up and down fucntions
void atom_pull_up() {
  for (int pos = atom_pull_down_pos; pos >= atom_pull_up_pos; pos -= 1) {
    atom_pull.write(pos);
  }
}

void atom_pull_down() {
  for (int pos = atom_pull_up_pos; pos <= atom_pull_down_pos; pos += 1) {
    atom_pull.write(pos);
  }
}


void complete_path() {
  // Loops until robot is triggered with by microswitch
  if (digitalRead(TOGGLE) == LOW) {
    Serial.println("orangeorangeorange");
    isOrangeSide = true;
  }
  while (digitalRead(TRIGGER) == HIGH) {
    Serial.println("waiting");
    Serial.println(readBatteryVolts());
    if (digitalRead(TOGGLE) == LOW) {
      Serial.println("orangeorangeorange");
      isOrangeSide = true;
    } else if (digitalRead(TOGGLE) == HIGH) {
      Serial.println("purplepurplepurple");
      isOrangeSide = false;
    }
  }
  startTime = millis();
  for (int i = 0; i < 100; i = i + 1) {
    irsend.sendNEC(0x12341234, 32);
  }
  if (isOrangeSide) {
    if (isHomologating) {
      homologation_orange();
    } else {
      orange_path();
    }
  } else {
    if (isHomologating) {
      homologation_purple();
    } else {
      purple_path();
    }
  }
}

/**Individual Paths**/
void purple_path() {
  /** Box for chaos area and periodic table **/
  tray.write(tray_dispenser_pos);
  straight(580 + WHEEL_TO_FRONT, 60, 1, 2, true, true);
  for (int i = 0; i < 100; i = i + 1) {
    irsend.sendNEC(0x12341234, 32);
  }
  rotate(-90, 50, 2, 1, true);
  straight(550 - (BOT_WIDTH / 2), 60, 1, 2, true, true);
  rotate(20, 20, 2, 1, true);
  straight(120, 40, 1, 2, true, true);
  rotate(-50, 20, 2, 1, true);
  straight(180, 40, 1, 2, true, true);
  rotate(-60, 40, 2, 1, true);
  straight(500, 40, 1, 2, true, true);
  rotate(-90, 40, 2, 1, true);
  straight(650, 40, 1, 2, true, true); //was 620
  /**Depositing Atoms in red square**/
  rotate(85, 50, 2, 1, true);
  tray.write(tray_horizontal_pos);
  straight(420 + WHEEL_TO_BACK, 40, 1, 3, false, false);
  straight(-460 - WHEEL_TO_BACK + 32 + 80, 60, 2, 2, true, true);
  tray.write(tray_down_pos);
  rotate(90, 50, 2, 1, true);
  /**Driving to small Atom dispenser**/
  straight(1193 - WHEEL_TO_FRONT - 30, 60, 2, 2 , false, false);
  //straight(100, 20, 1, 3, false, false);
  straight(-200, 40, 2, 2, true, false);
  rotate(-90, 50, 2, 1, true);
  straight(820 - WHEEL_TO_FRONT, 40, 2, 3, false, false);
  straight(-110, 20, 2, 2, true, false);
  rotate(90, 30, 2, 1, true);
  straight(845 - WHEEL_TO_FRONT, 60 , 2, 2, false, false);
//  delay(500);
//  /** Retrieve green and blue **/
//  straight(-70, 9, 1, 2, true, false);
//  delay(500);
//  tray.write(tray_dispenser_pos);
//  delay(500);
//  straight(75, 10, 2, 2, false, false);
//  atom_pull.write(atom_pull_down_pos);
//  delay(500);
//  straight(-25, 20, 3, 2, true, false);
//  atom_pull.write(atom_pull_up_pos);
//  straight(55, 10, 1, 2, false, false);
//  tray_up_dispenser_horizontal();
//  straight(-25, 10, 1, 2, true, false);
//  straight(-330 + WHEEL_TO_FRONT, 20, 1, 2, true, true);
//  /**Arc to deposit on scale **/
//  leftWheelFasterStraight(640, 20, 2, 2, false); //was 650
//  tray.write(tray_down_pos);
//  straight(-50, 20, 2, 2, false, false);
//  delay(1000);
//  rotate(-3, 20, 2, 1, false);
//  leftWheelFasterStraight(-580, 20, 2, 2, false);
//  rotate(10, 20, 2, 2, true);
//  straight(380, 40, 2, 2, false, false);
  straight(-WHEEL_TO_FRONT, 60, 1, 3, true, false); //was 200
  rotate(-90, 40, 2, 1, true);
  /**Reverse up ramp**/
  straight(250, 30, 2, 2, false, false);
  straight(-(1100 - WHEEL_TO_FRONT), 75, 1, 10, false, false);
  /**Last push**/
  delay(10000000000000000000);
  waitUntil(90000);
  rotate(15, 40, 2, 1, false);
  straight(-150, 30, 2, 2, false, false);
  door.write(door_up_pos);
  tray.write(tray_dispenser_pos);
  straight(1100 - WHEEL_TO_FRONT, 60, 2, 1, false , true);
  tray.write(tray_down_pos);
  door.write(door_down_pos);
  straight(150, 40, 2, 2, false, false);
  straight(-(1150 - WHEEL_TO_FRONT), 75, 2, 10, false, false);
  delay(10000000000000000);
}

void orange_path() {
  //  /** Box for chaos area and periodic table **/
  tray.write(tray_dispenser_pos);
  straight(580 + WHEEL_TO_FRONT, 60, 1, 2, true, true);
  for (int i = 0; i < 100; i = i + 1) {
    irsend.sendNEC(0x12341234, 32);
  }
  rotate(90, 50, 2, 1, true);
  straight(550 - (BOT_WIDTH / 2), 60, 1, 2, true, true);
  rotate(-20, 20, 2, 1, true);
  straight(120, 40, 1, 2, true, true);
  rotate(50, 20, 2, 1, true);
  straight(190, 40, 1, 2, true, true);
  rotate(60, 40, 2, 1, true);
  straight(500, 40, 1, 2, true, true); //was 510
  rotate(90, 40, 2, 1, true);
  straight(650, 40, 1, 2, true, true);
  /**Depositing Atoms in red square**/
  rotate(-90, 50, 2, 1, true);
  tray.write(tray_horizontal_pos);
  straight(420 + WHEEL_TO_BACK, 40, 1, 3, false, false);
  straight(-460 - WHEEL_TO_BACK + 32 + 80, 60, 2, 2, true, true);
  tray.write(tray_down_pos);
  rotate(-85, 50, 2, 1, true);
  /**Driving to small Atom dispenser**/
  straight(1093 - WHEEL_TO_FRONT, 60, 2, 2 , false, false);
  straight(100, 20, 1, 3, false, false);
  straight(-200, 40, 2, 2, true, false);
  rotate(90, 50, 2, 1, true);
  straight(720 - WHEEL_TO_FRONT, 40, 2, 3, false, false);
  straight(-110, 20, 2, 2, true, false);
  rotate(-90, 30, 2, 1, true);
  straight(845 - WHEEL_TO_FRONT, 60 , 2, 2, false, false);
//  delay(500);
//  /** Retrieve green and blue **/
////  straight(-70, 10, 2, 1, true, false);
////  delay(500);
////  tray.write(tray_dispenser_pos);
////  delay(500);
////  straight(55, 10, 2, 2, false, false);
//  fudge = 0.04;
//  atom_pull.write(atom_pull_down_pos);
//  delay(500);
//  straight(-100, 50, 4, 2, true, false); //new
//  //straight(-25, 15, 4, 2, true, false);
//  delay(400);
//  atom_pull.write(atom_pull_up_pos);
//  delay(400);
//  tray_up_dispenser_horizontal(); //new
//  delay(400); //new
//  straight(70, 40, 4, 2, false, false); //new
//  //straight(65, 10, 1, 2, false, false);
//  tray.write(tray_down_pos);
//  straight(70, 40, 4, 2, false, false);
//  //straight(-20, 10, 1, 2, false, false);
//  straight(-360 + WHEEL_TO_FRONT, 20, 1, 2, true, false);
//  fudge = 0.05;
//  /**Arc to deposit on scale **/
//  rightWheelFasterStraight(690, 10, 5, 2, false);  //was 690
//  rotate(-10, 10, 2, 1, false);
//  straight(100+300, 20, 2, 2, false, false);
//  tray.write(tray_horizontal_pos);
//  straight(-60-300, 20, 2, 2, false, false);
//  delay(1000);
//  tray.write(tray_down_pos); //new
//  straight(-490, 40, 2, 2, false, false);
//  straight(70, 20, 2, 2, true, false);
  straight(-WHEEL_TO_FRONT, 40, 2, 2, true, false);
  rotate(90, 30, 2, 1, true); //new
  //rotate(190, 40, 2, 1, true);
  straight(300, 50, 4, 2, false, false);
  straight(-(1090 - WHEEL_TO_FRONT), 75, 2, 10, false, false);
  delay(10000000000000000000); //new
  waitUntil(92000 - 30000);
  straight(-100, 50, 4, 2, false, false);
  rotate(30, 30, 2, 1, false);
  straight(400, 50, 2, 2, false, false);
  door.write(door_up_pos);
  tray.write(tray_dispenser_pos);
  straight(1100 - WHEEL_TO_FRONT, 60, 2, 1, false , true);
  tray.write(tray_down_pos);
  door.write(door_down_pos);
  straight(150, 40, 2, 2, false, false);
  straight(-150, 40, 2, 2, false, false);
  //rotate(8, 30, 4, 2, false);
  straight(-(1150 - WHEEL_TO_FRONT), 75, 2, 10, false, false);
  delay(10000000000000000000);







  //  straight(-50, 20, 2, 2, false, false);
  //  delay(1000);
  //  rotate(3, 20, 2, 1, false);
  //  rightWheelFasterStraight(-500, 10, 5, 2, false);
  //  straight(400, 40, 2, 2, false, false);
  //  straight(-170 + WHEEL_TO_FRONT, 60, 1, 3, true, false);
  //  rotate(90, 40, 2, 1, true);
  //  /**Reverse up ramp**/
  //  straight(190, 30, 2, 2, false, false);
  //  straight(-(1090 - WHEEL_TO_FRONT), 75, 5, 10, false, false);
  //  delay(100000);
}

void homologation_purple() {
  /** Box for chaos area and periodic table **/
  tray.write(tray_dispenser_pos);
  straight(580 + WHEEL_TO_FRONT, 20, 1, 2, true, true);
  rotate(-90, 50, 2, 1, true);
  straight(550 - (BOT_WIDTH / 2), 20, 1, 2, true, true);
  rotate(20, 20, 2, 1, true);
  straight(120, 20, 1, 2, true, true);
  rotate(-50, 20, 2, 1, true);
  straight(180, 20, 1, 2, true, true);
  rotate(-60, 40, 2, 1, true);
  straight(510, 20, 1, 2, true, true);
  rotate(-90, 40, 2, 1, true);
  straight(620 + 50, 20, 1, 2, true, true);
  /**Dipositing Atoms in red square**/
  rotate(90, 50, 2, 1, true);
  tray.write(tray_horizontal_pos);
  straight(420 + WHEEL_TO_BACK, 20, 1, 3, false, false);
  straight(-460 - WHEEL_TO_BACK + 32 + 80, 20, 2, 2, true, true);
  tray.write(tray_down_pos);
  rotate(90, 50, 2, 1, true);
  delay(100000);
}

void homologation_orange() {
  /** Box for chaos area and periodic table **/
  tray.write(tray_dispenser_pos);
  straight(580 + WHEEL_TO_FRONT, 30, 1, 2, true, true);
  rotate(90, 50, 2, 1, true);
  straight(550 - (BOT_WIDTH / 2), 30, 1, 2, true, true);
  rotate(-20, 20, 2, 1, true);
  straight(120, 20, 1, 2, true, true);
  rotate(50, 20, 2, 1, true);
  straight(180, 20, 1, 2, true, true);
  rotate(60, 40, 2, 1, true);
  straight(510, 20, 1, 2, true, true);
  rotate(90, 40, 2, 1, true);
  straight(600 + 50, 20, 1, 2, true, true);
  /**Dipositing Atoms in red square**/
  rotate(-90, 50, 2, 1, true);
  tray.write(tray_horizontal_pos);
  straight(420 + WHEEL_TO_BACK, 20, 1, 3, false, false);
  straight(-460 - WHEEL_TO_BACK + 32 + 80, 20, 2, 2, true, true);
  tray.write(tray_down_pos);
  rotate(-90, 50, 2, 1, true);
  delay(100000);
}
