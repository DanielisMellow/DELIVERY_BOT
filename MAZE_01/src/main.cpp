#include <Arduino.h>
#include <QTRSensors.h>
#include <Romi32U4.h>
#include <Chassis.h>
#include <Rangefinder.h>
#include <servo32u4.h>

// Control Delivery Sequence
#define r1 'a'
#define r2 'a'
#define r3 'a'

// Amount of Routes
#define numDest 1

#define k_p 0.09
#define k_d 0.225
#define k_i 0.000

// Values Required For PID Calculations
const float kp = k_p, kd = k_d, ki = k_i;

// SETUP QTR SENSOR
QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

// Chasis Functionality
Chassis chassis(7, 1440, 14.7);

// Ultrasonic sensor
Rangefinder rangefinder(11, 4);

// Motor Parameters
Romi32U4Motors motors;
static const int16_t MAX_SPEED = 420;
static const int16_t MIN_SPEED = 150;
static const uint16_t BASE_SPEED = 250;

// BUTTORN PARAMETERS
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;

// SERVO
Servo32U4 servo;
static const uint16_t SERVO_DOWN = 500;
static const uint16_t SERVO_UP = 5000;

enum ROBOT_STATE
{
  ROBOT_UNCALIBRATED,
  ROBOT_CALIBRATED,
  ROBOT_ORDERS,
  ROBOT_FOLLOW,
  ROBOT_TURNING,
  ROBOT_PICKUP,
  ROBOT_DELIVERY
};
static ROBOT_STATE robotState = ROBOT_UNCALIBRATED;

typedef struct
{
  char routeName;        // A B C D E F G H
  char turnSequence[12]; // How the robot handles intersection events
  uint16_t height;
} Directions;

Directions ROUTE[3];

// add other nodes
Directions TestRoute = {'T', {'R', 'L', 'L', 'S', 'L', 'S', 'T', 'E'}, 0};

// ROUTES
Directions ARoute = {'A', {'P', 'R', 'R', 'D', 'L', 'L', 'T', 'E'}, 0};
Directions BRoute = {'B', {'T', 'R', 'S', 'T', 'S', 'L', 'T', 'E'}, 0};

void calibration();
void checkIntersectionEvent(int left, int right);
void inline TurnEffort(int16_t *lastError, float *turnEffort);
void inline SpeedIncrementFUNC(int16_t *sIncrement, int16_t *samples, int16_t *samples1);
void inline motorSpeedLimitBound(int *m1, int *m2);
void inline lineFollowingHandler(void);

void turn(float ang, float speed)
{
  chassis.turnFor(ang, speed, true);
  robotState = ROBOT_FOLLOW;
}
void drive(float distance, float speed)
{
  chassis.driveFor(distance, speed, true);
  robotState = ROBOT_FOLLOW;
}

void handleDelivery()
{
  chassis.driveFor(-3, 90, true);
  servo.writeMicroseconds(SERVO_DOWN);
  chassis.driveFor(-3, 90, true);
  servo.writeMicroseconds(SERVO_UP);
  turn(180, 360);
}

void handlePickUP(float distance)
{
  Serial.print(distance);
  Serial.print(" cm\n");
  if (distance < 3)
  {
    servo.writeMicroseconds(SERVO_UP);
    chassis.driveFor(-10, 90, true);
    turn(180, 360);
  }
  else
  {
    servo.writeMicroseconds(SERVO_DOWN);
    chassis.driveFor(1, 30, true);
  }
}

void handleTurnEvent(Directions route[], int n)
{

  static uint16_t InterCount = 0;
  static uint8_t RouteCount = 0;

  Serial.println("ROBOT STATE: TURNING");
  Serial.print("\nInterCount: ");
  Serial.println(InterCount);
  if (RouteCount < n)
  {
    Serial.println(route[RouteCount].turnSequence[InterCount]);
    switch (route[RouteCount].turnSequence[InterCount])
    {
    case 'T': // Turn around
      turn(180, 360);
      InterCount++;
      break;
    case 'S':
      drive(3, 30); // Straight
      InterCount++;
      break;
    case 'R': // Turn right
      chassis.driveFor(6, 90, true);
      turn(-95, 360);
      InterCount++;
      break;
    case 'L': // Turn left
      chassis.driveFor(6, 90, true);
      turn(95, 360);
      InterCount++;
      break;
    case 'D': // drop off sequence
      // Call function to handle deliveries for each individual track
      // dropOffSequence(&route[RouteCount]);
      robotState = ROBOT_DELIVERY;
      InterCount++;
      break;
    case 'P': // pick up off sequence
      // Call function to handle Package Pick Up
      robotState = ROBOT_PICKUP;
      InterCount++;
      break;
    case 'E':
      chassis.turnFor(-180, 360, true);
      chassis.driveFor(1, 90, true);
      motors.setSpeeds(0, 0);
      InterCount = 0;
      RouteCount++;
      if (RouteCount < n)
        robotState = ROBOT_FOLLOW;
      else
      {
        // RouteCount = 0;
        robotState = ROBOT_ORDERS;
      }
      break;

    default:
      break;
    }
  }
  else
  {
    robotState = ROBOT_ORDERS;
    RouteCount = 0;
  }
}

// Eventually change this into array of structures
void checkIntersectionEvent(int left, int right)
{
  if ((left > 350) && (right > 350))
  {
    chassis.idle();
    robotState = ROBOT_TURNING;
  }
}

// Function that creates a dynamic array array of structures that have every route
void pathSequence(char Des[], int n)
{
  for (int i = 0; i < n; i++)
  {
    switch (Des[i])
    {
    case 't':
      ROUTE[i] = TestRoute;
      Serial.println("ROUTE A WAS MARKED");
      break;
    case 'a':
      ROUTE[i] = ARoute;
      Serial.println("ROUTE A WAS MARKED");
      break;
    case 'b':
      ROUTE[i] = BRoute;
      Serial.println("ROUTE B WAS MARKED");
      break;
    // Add other cases
    default:
      break;
    }
  }
}

void handleButton(int16_t keyPress)
{
  Serial.println("Key: " + String(keyPress));
  static char route[3] = {r1, r2, r3};

  if (robotState == ROBOT_CALIBRATED || robotState == ROBOT_ORDERS)
  {
    switch (keyPress)
    {
    case 0:
      Serial.println("ROBOT STATE: ROBOT_FOLLOW");
      for (int i = 0; i < 3; i++)
        Serial.println(route[i]);

      pathSequence(route, numDest);
      robotState = ROBOT_FOLLOW;

      break;
    case 1:
      Serial.println("SELECTED ROUTES");
      break;
    default:
      break;
    }
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Initialize Chasis
  chassis.init();

  // Initialize QTR sensor
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A7, A2, A3, A4, A0, A11}, SensorCount);

  // RangeFinder Init
  rangefinder.init();

  // Call attach() to set up the servo
  servo.attach();
  servo.setMinMaxMicroseconds(SERVO_DOWN, SERVO_UP);
  servo.writeMicroseconds(SERVO_DOWN);

  motors.allowTurbo(true);
  motors.setSpeeds(0, 0);
}

void loop()
{
  static uint16_t cPressedCount = 0b0;
  // int16_t keyPress = decoder.getKeyCode();
  if (buttonA.getSingleDebouncedPress())
  {
    Serial.println("ROBOT STATE: UNCALIBRATED");
    ledRed(1);
    calibration();
  }
  else if (buttonB.getSingleDebouncedPress())
  {
    cPressedCount ^= 0b1;
    handleButton(cPressedCount);
  }
  static float distance;
  switch (robotState)
  {
  case ROBOT_FOLLOW:
    lineFollowingHandler();
    break;
  case ROBOT_TURNING:
    chassis.idle();
    handleTurnEvent(ROUTE, numDest); // Array that stores destinations and number of destinations
    break;
  case ROBOT_ORDERS:
    chassis.idle();
    break;
  case ROBOT_PICKUP:
    distance = rangefinder.getDistance();
    handlePickUP(distance);
    break;
  case ROBOT_DELIVERY:
    handleDelivery();
    break;
  default:
    break;
  }
}

void calibration()
{
  Serial.println("CALIBRATING: Please Wait...");
  for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate();
  }

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  robotState = ROBOT_CALIBRATED;
  Serial.println("\nROBOT STATE: CALIBRATED");
  ledRed(0);
  ledGreen(1);
}
void inline TurnEffort(int16_t *lastError, float *turnEffort)
{
  int position = qtr.readLineBlack(sensorValues);
  int16_t error = position - 2500;

  // I = I + error;
  // if (I > 10000)
  //   I = 10000;
  // float turnEffort = (kp * error) + (ki * I) + (kd * (error - lastError));
  float tempTurnEffort = (kp * error) + (kd * (error - (*lastError)));

  *lastError = error;
  *turnEffort = tempTurnEffort;
}
void inline SpeedIncrementFUNC(int16_t *sIncrement, int16_t *samples, int16_t *samples1)
{
  int16_t tempSI = *sIncrement;
  int16_t tempS = *samples;
  int16_t tempS1 = *samples1;
  qtr.read(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }

  if ((sensorValues[2] > (sensorValues[0] + sensorValues[1])) || (sensorValues[3] > (sensorValues[4] + sensorValues[5])))
  {
    tempS++;
    tempS1 = 0;
    if (tempS > 8)
    {
      tempSI += 1;
      tempS = 0;
    }
  }
  else
  {
    tempS1++;
    tempS = 0;
    if (tempS1 > 4)
    {
      tempSI--;
      if (BASE_SPEED + tempS1 < MIN_SPEED)
      {
        tempS1 = MIN_SPEED - BASE_SPEED;
      }
      tempS1 = 0;
    }
  }

  *sIncrement = tempSI;
  *samples = tempS;
  *samples1 = tempS1;
  checkIntersectionEvent(sensorValues[0], sensorValues[5]);
}
void inline motorSpeedLimitBound(int16_t *m1, int16_t *m2)
{
  if (*m1 < 0)
    *m1 = 0;
  if (*m2 < 0)
    *m2 = 0;
  if (*m1 > MAX_SPEED)
    *m1 = MAX_SPEED;
  if (*m2 > MAX_SPEED)
    *m2 = MAX_SPEED;
}
void inline lineFollowingHandler(void)
{
  float turnEffort = 0;
  static int16_t sIncrement = 0;
  static int16_t samples = 0;
  static int16_t samples1 = 0;

  static int16_t lastError = 0;

  // Final Motor Speeds
  int16_t m1Speed;
  int16_t m2Speed;

  TurnEffort(&lastError, &turnEffort);
  SpeedIncrementFUNC(&sIncrement, &samples, &samples1);

  m1Speed = (BASE_SPEED + sIncrement) - turnEffort;
  m2Speed = (BASE_SPEED + sIncrement) + turnEffort;

  Serial.print("Error: ");
  Serial.print(lastError);
  Serial.print("\t");

  Serial.print("Turn Effort: ");
  Serial.print(turnEffort);
  Serial.print("\t");

  motorSpeedLimitBound(&m1Speed, &m2Speed);
  // Serial.print("M1: ");
  Serial.print(m1Speed);
  Serial.print("\t");

  Serial.print("M2: ");
  Serial.print(m1Speed);
  Serial.println("\t");

  // motorSpeedLimitBound(&m1Speed, &m2Speed);

  motors.setSpeeds(m1Speed, m2Speed);
}
