/*
  Sketch Name: R2D2

  Description: Sketch contains functions to drive forward and rotate a forward facing robot using MD25 motor drive. Should turn functions into a library soon.

  MD25 documentation: https://www.robot-electronics.co.uk/htm/md25i2c.htm

  Wire MD25 as shown in https://blackboard.soton.ac.uk/bbcswebdav/pid-4040104-dt-content-rid-3986367_1/courses/FEEG2001-31292-18-19/MD25%20Circuit%20Guide.pdf
  Wire buttons as shown in http://ardx.org/src/circ/CIRC07-sheet-OOML.pdf with buttons connected to pins 2 and 7
  2200 ohm pullup resistors used for MD25 and buttons

  Created 01/02/2019
  By Alasdair Mann and Will Birch-Tomlinson
  Modified 02/03/2019
  By Alasdair Mann

*/

/** Imported libraries, make sure these are downloaded **/
#include <IRremote.h>
#include <Wire.h>                                             // Imports I2C bus library
#include <NewPing.h>                                          // Imports Ultrasonic Sensor library
#include <Servo.h>



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
#define BOT_WIDTH           189                               // Robot's width in mm
#define BOT_LENGTH          199                               // Robot's length in mm
#define WHEEL_TO_WHEEL      180                               // Distance between centre of two wheels in mm
#define KNOCK_SERVO         11                                // Servo 1 control pin
#define CLAW_SERVO          10                                // Servo 2 control pin
#define DEPLOY_SERVO        9                                 // Servo 3 control pin
#define PUSH_SERVO          8                                 // Servo 4 control pin
#define US_0                13                                 // Set trigger and echo pin for sensor 0
#define US_1                12                                 // Set trigger and echo pin for sensor 1
#define US_2                2                                 // Set trigger and echo pin for sensor 2
#define US_3                4                                 // Set trigger and echo pin for sensor 3
#define TRIGGER             3                                 // Infrared receiver control pin
#define TRIGGER2            6                                 // Limit switch trigger
#define TOGGLE              7                                 // Toggle switch control pin
#define MAX_DISTANCE 400                                      // Maximum range for ultrasonic sensors

// Initialise an array of 6 NewPing objects for sensors
NewPing sonar[4] = {NewPing(US_0, US_0, MAX_DISTANCE),
                    NewPing(US_1, US_1, MAX_DISTANCE),
                    NewPing(US_2, US_2, MAX_DISTANCE),
                    NewPing(US_3, US_3, MAX_DISTANCE)
                   };

// Initialise servos
Servo atom_push;
Servo knock;
Servo claw;
Servo claw_deploy;

// Initialise infrared sensor and decoder
IRrecv irrecv(TRIGGER);
decode_results results;

/* Test values hidden in code */
const unsigned long maxTime = 100000;                                   // Maximum time in milliseconds
const byte straightTolerance = 0;                             // Accepted tolerance of driving straight in mm without correcting
const byte rotationTolerance = 0;                             // Accepted tolerance of rotation in mm without correcting
const double conversionFactor = 0.5263913024;                 // Value that converts encoder readings to mm, theoretically = Diameter/(360*pi)

// Constants for servo positions
int atom_push_in_pos = 4;
int atom_push_out_pos = 180;

int claw_open_pos = 110;
int claw_close_pos = 15;

int claw_deploy_out_pos = 25;
int claw_deploy_in_pos = 160;

int knock_up_pos = 15;
int knock_down_pos = 115;

/** Universal variables **/
boolean isPurpleSide = false;                                         // True if starting on purple side of game board
bool usingAsPrimary = true;
bool isHomologating = false;
bool tryingToAlternate = false;
bool actuallyAlternate = false;
float makeUpDistance = 0;
unsigned long startTime;                                      // Time at which the robot is triggered

/** Program set up **/
void setup() {
  // Begin communications
  Wire.begin();                                               // I2C bus
  Serial.begin(9600);                                         // Serial at 9600 baud
  delay(100);                                                 // Wait for everything to power up

  // Attach servos to pins
  claw.attach(CLAW_SERVO);
  knock.attach(KNOCK_SERVO);
  claw_deploy.attach(DEPLOY_SERVO);
  atom_push.attach(PUSH_SERVO);

  // Set MD25 operation MODE
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(MODE_SELECTOR);
  Wire.write(1);                                              // Set MD25 to mode 1, mode described in constants
  Wire.endTransmission();

  // Initialise pins as inputs to receive HIGH or LOW signal
  pinMode(TOGGLE, INPUT);                                          // Pin connected to button or toggle switch for choosing side of game board
  pinMode(TRIGGER, INPUT);                                          // Pin connected to micro switch for triggering
  pinMode(TRIGGER2, INPUT);

  // Resets encoders to 0 incase wheels moved during set up
  encoderReset();

  // Makes sure mechanisms are all inside
  claw.write(claw_open_pos);
  atom_push.write(atom_push_out_pos);
  knock.write(knock_down_pos);
  claw_deploy.write(claw_deploy_in_pos);

  irrecv.enableIRIn();                        // Start the receiver
  Serial.println("starting");
}

/** Program loop **/
void loop() {
  complete_path();
  //homologation_orange();
  //homologation_purple();
  //straight(1500, 60, 5, 2, true, true);
  //straight(-1500, 60, 5, 2, true, true);
  // Loops until robot is triggered with infrared
  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:

  //complete_path();
  //orangePath();
  //purplePath();
}

/* Linear drive function that can predict when to start decelerating */
/** float distance: distance robot travels linearly in mm, a negative disntance means backwards movement */
/** byte speedValue: speed from 0 to 127 at which the wheels move at */
/** byte accelerationRegister: deceleration rate from 1-10, 1 being slowest, 10 being fastest. */
/** byte decelerationRegister: deceleration rate from 1-10, 1 being slowest, 10 being fastest. */
/** boolean ifCorrecting: true if correcting over/undershoots */
/** boolean ifSensing: true if collision with another robot is probable */
void straight(float distance, short speedValue, byte accelerationRegister, byte decelerationRegister, boolean ifCorrecting, boolean ifSensing) {
  distance = -distance;                                                               // Hacky fix, can't change orientation of wheels

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
void waitUntill(unsigned long milli) {
  while ((millis() - startTime) < milli) {
    stopMotor(10);
  }
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
  short speedValue1 = 3;                                                               // Byte speed for correcting, test this value
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

void drive(float distance, short speed1, short speed2, byte accelerationRegister, byte decelerationRegister, bool ifSensing) {
  float encoderHistory[2][3] = {{0, 0, 0}, {0, 0, 0}};                                         // 2x3 matrix, row 1 for encoder 1, row 2 for encoder 2. Columns are for displacement at time stamps.
  float currentVelocities[2] = {0, 0};                                                         // Array for holding current velocities of each encoder
  unsigned long encoderTime[3] = {millis(), millis(), millis()};                                       // Array for holding time stamps
  float currentDecelerationDistance[2] = {0, 0};                                               // Array for holding current deceleration distances of each encoder
  float remainingDistance[2] = {0, 0};                                                         // Array for holding current remaining distance to cover of each encoder
  short originalSpeed = speed1;                                                     // Saves original input for speed incase the speed needs to change after obstacle avoidance
  boolean dec = false;

  setMotors(accelerationRegister, speed1, speed2);
  while (dec == false && actuallyAlternate == false) {
    checkTime();

    // Reads ultrasonic sensors before writing the next speed value, stops motors for as long as ultrasonics detect a surprising object

    // Currently collision distance is a constant, could possibly work by accounting for decelerationDistance. But I wasn't able to make it reliable.
    short collisionDistance = 250;

    // Calls scanning function and times for duration
    unsigned long startOfScan = millis();
    if (ifSensing) {
      scan(originalSpeed, collisionDistance, distance, 10);
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
    //Serial.println("Decelereation distance");
    //Serial.println(currentDecelerationDistance[1]);

    // Calculates remaining distance each wheel needs to cover
    remainingDistance[0] = abs(distance) - abs(encoder(1));
    remainingDistance[1] = abs(distance) - abs(encoder(2));

    // If statement prevents a overflow bug that occurs sometimes at the start of paths
    if (abs(encoder(1)) > abs(distance) * 0.2 && abs(encoder(2)) > abs(distance) * 0.2) {
      // dec becomes true if the deceleration distance exceeds the remaining distance
      if (abs(currentDecelerationDistance[0]) >= remainingDistance[0] || abs(currentDecelerationDistance[1]) >= remainingDistance[1]) {
        dec = true;
      }
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
void scan(short s, float maxDistance, float target, byte dec) {
  // Starts loop and ends at different numbers depending on front or backwards facing
  // Front facing sensors are 0, 1, 2, back facing sensors are 3, 4, 5
  byte loopStart = 0;
  byte loopEnd = 1;
  if (s < 0) {
    loopStart = 2;
    loopEnd = 3;
  }

  // Sensor scans
  unsigned int distance[4] = {0, 0, 0, 0};
  for (int i = loopStart; i <= loopEnd; i = i + 1) {
    distance[i] = sonar[i].convert_cm(sonar[i].ping_median(3)) * 10;
  }

  // Robot stops while there is something in the way
  bool change = false;
  unsigned long alternateTimerStart = millis();
  while ((distance[loopStart]   > 20 && distance[loopStart] < maxDistance) || (distance[loopStart + 1]   > 20 && distance[loopStart + 1] < maxDistance)) {
    stopMotor(dec);
    // Sensors keep checking
    for (byte i = loopStart; i <= loopEnd; i = i + 1) {
      distance[i] = sonar[i].convert_cm(sonar[i].ping_median(3)) * 10;
    }
    change = true;
    delay(100);
    if (tryingToAlternate && (millis() - alternateTimerStart) > 15000) {
      makeUpDistance = abs(target) - abs(encoder(1));
      actuallyAlternate = true;
      break;
    }
  }
  if (change == true && actuallyAlternate == false) {
    setMotors(1, s, s);
    Serial.println("Sensor:");
    Serial.println(distance[0]);
  }
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
  if (millis() - startTime > maxTime) {                 //puts arduino in infinite loop after 98 seconds, test
    while (neverFalse) {
      stopMotor(10);
    }
  }
}

/* Servo functions */
void claw_out() {
  for (short pos = claw_deploy_in_pos; pos >= claw_deploy_out_pos; pos -= 1) {
    claw_deploy.write(pos);
    delay(10);
  }
}

void claw_in() {
  for (short pos = claw_deploy_out_pos; pos <= claw_deploy_in_pos; pos += 1) {
    claw_deploy.write(pos);
    delay(10);
  }
}


//Knock up and down fucntions
void knock_up() {
  for (short pos = knock_down_pos; pos >= knock_up_pos; pos -= 1) {
    knock.write(pos);
  }
}

void knock_down() {
  for (short pos = knock_up_pos; pos <= knock_down_pos; pos += 1) {
    knock.write(pos);
  }
}

//Atom push in and out functions
void atom_push_in() {
  for (short pos = atom_push_out_pos; pos >= atom_push_in_pos; pos -= 1) {
    atom_push.write(pos);
    //delay(10);
  }
}

void atom_push_out() {
  for (short pos = atom_push_in_pos; pos <= atom_push_out_pos; pos += 1) {
    atom_push.write(pos);
    //delay(10);
  }
}

//Claw open and close functions
void claw_open() {
  for (short pos = claw_close_pos; pos <= claw_open_pos; pos += 1) {
    claw.write(pos);
  }
}

void claw_close() {
  for (short pos = claw_open_pos; pos >= claw_close_pos; pos -= 1) {
    claw.write(pos);
  }
}

void complete_path() {
  // Loops until robot is triggered with infrared
  bool starting = false;
  if (digitalRead(TOGGLE) == HIGH) {
    Serial.println("purplepurplepurple");
    isPurpleSide = true;
  } else if (digitalRead(TOGGLE) == LOW) {
    Serial.println("orangeorangeorange");
    isPurpleSide = false;
  }
  while (starting == false) {
    Serial.println(readBatteryVolts());
    if (digitalRead(TOGGLE) == HIGH) {
      Serial.println("purplepurplepurple");
      isPurpleSide = true;
    } else if (digitalRead(TOGGLE) == LOW) {
      Serial.println("orangeorangeorange");
      isPurpleSide = false;
    }
    if (irrecv.decode(&results)) {
      Serial.println(results.value);
      /* 4294967295 is the decoded signal read from the signal sent by irsend.sendNEC(0x12341234, 32); */
      if (results.value == 305402420) {
        starting = true;
      }
      irrecv.resume(); // Receive the next value
    }
    if (usingAsPrimary && digitalRead(TRIGGER2) == LOW) {
      starting = true;
    }

  }
  startTime = millis();
  delay(2000);
  // Need to test path for both sides as robot is not symmetrical about any axis
  Serial.println("out");
  if (isPurpleSide) {
    if (isHomologating) {
      homologation_purple();
    } else {
      purplePath();
    }
  } else if (!isPurpleSide) {
    if (isHomologating) {
      homologation_orange();
    } else {
      orangePath();
    }
  }
}

void orangePath() {
  /** Driving to accelerator **/
  straight(-140 - 50, 60, 5, 2, true, true);
  rotate(-90, 60, 5, 2, true);
  straight(-100, 40, 2, 2, false, false);
  delay(2000);

  tryingToAlternate = true;
  straight(1530, 100, 4, 2, true, true);
  if (actuallyAlternate) {
    float clearanceDistance = 200;
    actuallyAlternate = false;
    rotate(90, 60, 5, 2, true);
    straight(clearanceDistance, 80, 4, 2, true, true);
    rotate(-90, 60, 5, 2, true);
    straight(makeUpDistance, 40, 3, 2, true, true);
    makeUpDistance = clearanceDistance;
  }
  actuallyAlternate = false;
  rotate(90, 60, 5, 2, true);
  /** Knocking accelerator **/
  straight(-250 - makeUpDistance, 80, 3, 2, false, true);
  straight(-380, 80, 5, 2, false, false);
  straight(67, 40, 2, 3, true, false);
  knock.write(55);
  rotate(90, 50, 5, 2, true);
  straight(-230, 40, 1, 2, true, true);
  knock.write(knock_up_pos);
  /** Driving and grabbing gold **/
  straight(-80, 40, 2, 2, true, true);
  rotate(-90, 50, 5, 2, true);
  straight(-90, 40, 2, 2, true, false);
  straight(BOT_LENGTH + 50, 50, 3, 2, true, true);
  rotate(-90, 60, 5, 2, true);
  knock_down();
  straight(383, 60, 5, 2, true, true);
  rotate(-90, 60, 5, 2, true);
  straight(BOT_LENGTH - 30, 50, 2, 3, false, false);
  claw_out();
  delay(200);
  claw_close();
  delay(400);
  rotate(10, 10, 2, 1, false);
  /** Driving from gold to scale **/
  straight(-70, 40, 3, 2, false, true);
  rotate(-180, 70, 5, 2, true);
  straight(-BOT_LENGTH - 40, 50, 4, 2, true, false);
  straight(250, 50, 4, 3, true, true);
  rotate(63, 40, 3, 2, true);
  straight(1200, 90, 5, 2, true, true);
  rotate(-80, 40, 4, 2, true);
  straight(650, 70, 5, 2, false, false);
  claw_open();
  delay(500);
  claw_in();
  /** Realigning with distributor **/
  straight(-20, 30, 2, 1, true, true);
  rotate(118, 60, 5, 2, true);
  /** Knocking down green and blue **/
  straight(-230, 60, 5, 2, false, false);
  straight(400, 60, 5, 2, false, true);
  rotate(174, 70, 5, 2, true);
  knock.write(45);
  straight(-385, 80, 5, 2, true, false);
  rotate(6, 5, 2, 2, false);
  knock.write(95);
  delay(300);
  knock.write(45);
  atom_push_in();
  straight(-200, 50, 5, 2, true, true);
  knock.write(95);
  delay(300);
  knock.write(45);
  waitUntill(88000);
  straight(-260, 30, 5, 2, true, true);
  rotate(95, 60, 5, 2, true);
  straight(-450, 80, 5, 2, false, true);
  atom_push_out();
  delay(700);
  atom_push_in();
  straight(-250, 80, 5, 2, false, true);
  atom_push_out();
  delay(500);
  rotate(-20, 20, 2, 2, false);
  delay(1000000000);


//  knock.write(knock_up_pos);
//  straight(-190, 60, 5, 2, true, true);
//  rotate(-92, 50, 5, 2, true);
//  straight(-350, 70, 4, 1, true, true);
//  atom_push_out();
//  delay(700);
//  atom_push_in();
//  straight(-120, 40, 3, 1, false, false);
//  atom_push_out();
//  delay(700);
//  rotate(-3, 40, 5, 2, false);
//  straight(600, 70, 5, 2, true, true);
//  rotate(5, 40, 5, 2, false);
//  straight(110, 70, 5, 2, false, true);
//  rotate(20, 40, 5, 2, false);
//  straight(50, 70, 5, 2, false, false);
//  delay(100000000);
}

void purplePath() {
  /** Driving to accelerator **/
  straight(-140 - 50, 60, 5, 2, true, true);
  rotate(90, 60, 5, 2, true);
  straight(-100, 40, 2, 2, false, false);
  tryingToAlternate = true;
  straight(1450, 100, 4, 2, true, true);
  if (actuallyAlternate) {
    float clearanceDistance = 200;
    actuallyAlternate = false;
    rotate(-90, 60, 5, 2, true);
    straight(clearanceDistance, 80, 4, 2, true, true);
    rotate(90, 60, 5, 2, true);
    straight(makeUpDistance, 40, 3, 2, true, true);
    makeUpDistance = clearanceDistance;
  }
  actuallyAlternate = false;
  rotate(-90, 60, 5, 2, true);
  /** Knocking accelerator **/
  straight(-250 - makeUpDistance, 80, 5, 2, false, true);
  makeUpDistance = 0;
  straight(-350 + 50, 80, 5, 2, false, false);
  straight(73, 40, 2, 3, true, false);
  knock.write(55);
  rotate(90, 60, 5, 2, true);
  straight(140, 40, 1, 2, true, true);
  knock.write(knock_up_pos);
  /** Driving and grabbing gold **/
  straight(235, 40, 2, 2, true, true);
  rotate(-90, 60, 5, 2, true);
  straight(-90, 40, 2, 2, true, false);
  straight(BOT_LENGTH + 50, 50, 3, 2, true, true);
  rotate(90, 60, 5, 2, true);
  knock_down();
  straight(331, 60, 5, 2, true, true);
  rotate(90, 60, 5, 2, true);
  straight(BOT_LENGTH - 70, 50, 2, 3, false, false);
  claw_out();
  delay(200);
  claw_close();
  delay(500);
  straight(50, 40, 2, 2, false, false);
  //rotate(7, 10, 2, 1, false);
  rotate(-3, 10, 2, 2, false);
  straight(-70, 40, 1, 3, false, true);
  /** Driving from gold to scale **/
  rotate(180, 60, 4, 2, true);
  straight(-BOT_LENGTH - 40, 40, 1, 3, true, false);
  straight(250, 40, 4, 3, true, true);
  rotate(-50.5, 40, 2, 1, true);
  straight(1220, 80, 4, 3, true, true);
  rotate(60, 40, 4, 2, true);
  straight(600, 50, 4, 1, false, false);
  claw_open();
  delay(500);
  claw_in();
  delay(500);
  /** Realigning with distributor **/
  straight(-10, 30, 2, 1, true, true);
  rotate(-120, 50, 5, 2, true);
  /** Knocking down green and blue **/
  straight(-230, 60, 5, 2, false, false);
  straight(230, 60, 5, 2, true, false);
  rotate(5, 10, 2, 2, false);
  straight(50, 20, 2, 1, true, false);
  knock.write(50);
  rotate(-2, 5, 2, 2, false); //new
  straight(265, 60, 4, 2, true, false);
  //rotate(-2, 5, 2, 2, false);
  knock.write(130);
  delay(400);
  knock.write(45);
  atom_push_in();
  straight(190, 50, 5, 2, true, false);
  //rotate(-1, 10, 1, 2, false);
  knock.write(130);
  delay(300);
  knock.write(45);
  //waitUntill(88000);
  straight(200, 30, 5, 2, true, true);
  rotate(-95, 60, 5, 2, true);
  straight(450, 80, 5, 2, false, true);
  atom_push_out();
  delay(700);
  atom_push_in();
  straight(250, 80, 5, 2, false, true);
  atom_push_out();
  delay(500);
  rotate(20, 20, 2, 2, false);
  delay(1000000000);
//  
//  knock.write(knock_up_pos);
//  straight(400, 60, 5, 2, true, true);
//  rotate(94, 50, 5, 2, true);
//  straight(250, 50, 4, 1, true, true);
//  atom_push_out();
//  delay(700);
//  atom_push_in();
//  straight(50, 40, 3, 1, false, false);
//  atom_push_out();
//  delay(700);
//  rotate(4, 40, 5, 2, false);
//  straight(-300, 70, 5, 2, true, true);
//  rotate(-20, 40, 5, 2, false);
//  straight(-2000, 70, 5, 2, false, true);
//  delay(100000);
}

void homologation_orange() {
  /** Driving to accelerator **/
  straight(-140 - 50, 60, 5, 2, true, true);
  rotate(-90, 60, 5, 2, true);
  straight(-100, 40, 2, 2, false, false);
  delay(2000);

  tryingToAlternate = true;
  straight(1530, 70, 4, 2, true, true);
  if (actuallyAlternate) {
    float clearanceDistance = 200;
    actuallyAlternate = false;
    rotate(90, 60, 5, 2, true);
    straight(clearanceDistance, 80, 4, 2, true, true);
    rotate(-90, 60, 5, 2, true);
    straight(makeUpDistance, 40, 3, 2, true, true);
    makeUpDistance = clearanceDistance;
  }
  actuallyAlternate = false;
  rotate(90, 60, 5, 2, true);
  /** Knocking accelerator **/
  straight(-250 - makeUpDistance, 80, 3, 2, false, true);
  straight(-380, 80, 5, 2, false, false);
  straight(65, 40, 2, 3, true, false);
  knock.write(55);
  rotate(90, 50, 5, 2, true);
  straight(-230, 40, 1, 2, true, true);
  knock.write(knock_up_pos);
  /** Driving and grabbing gold **/
  straight(-80, 40, 2, 2, true, true);
  rotate(-90, 50, 5, 2, true);
  straight(-90, 40, 2, 2, true, false);
  straight(BOT_LENGTH + 50, 50, 3, 2, true, true);
  rotate(-90, 60, 5, 2, true);
  knock_down();
  straight(375, 60, 5, 2, true, true);
  rotate(-90, 60, 5, 2, true);
  straight(BOT_LENGTH - 20, 50, 2, 3, false, false);
  claw_out();
  delay(200);
  claw_close();
  delay(400);
  rotate(10, 10, 2, 1, false);
  /** Driving from gold to scale **/
  straight(-70, 40, 3, 2, false, true);
  delay(10000000);
}

void homologation_purple() {
  /** Driving to accelerator **/
  straight(-140 - 50, 60, 5, 2, true, true);
  rotate(90, 60, 5, 2, true);
  straight(-100, 40, 2, 2, false, false);
  tryingToAlternate = true;
  straight(1450, 80, 4, 2, true, true);
  if (actuallyAlternate) {
    float clearanceDistance = 200;
    actuallyAlternate = false;
    rotate(-90, 60, 5, 2, true);
    straight(clearanceDistance, 80, 4, 2, true, true);
    rotate(90, 60, 5, 2, true);
    straight(makeUpDistance, 40, 3, 2, true, true);
    makeUpDistance = clearanceDistance;
  }
  actuallyAlternate = false;
  rotate(-90, 60, 5, 2, true);
  /** Knocking accelerator **/
  straight(-250 - makeUpDistance, 80, 5, 2, false, true);
  makeUpDistance = 0;
  straight(-350 + 50, 80, 5, 2, false, false);
  straight(70, 40, 2, 3, true, false);
  knock.write(55);
  rotate(90, 60, 5, 2, true);
  straight(145, 40, 1, 2, true, true);
  knock.write(knock_up_pos);
  /** Driving and grabbing gold **/
  straight(235, 40, 2, 2, true, true);
  rotate(-90, 60, 5, 2, true);
  straight(-90, 40, 2, 2, true, false);
  straight(BOT_LENGTH + 50, 50, 3, 2, true, true);
  rotate(90, 60, 5, 2, true);
  knock_down();
  straight(337, 60, 5, 2, true, true);
  rotate(90, 60, 5, 2, true);
  straight(BOT_LENGTH - 50, 50, 2, 3, false, false);
  claw_out();
  delay(200);
  claw_close();
  delay(500);
  straight(50, 40, 2, 2, false, false);
  //rotate(7, 10, 2, 1, false);
  rotate(-3, 10, 2, 2, false);
  straight(-70, 40, 1, 3, false, true);
  delay(1000000);
}
