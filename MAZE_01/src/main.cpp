#include <Arduino.h>
#include <QTRSensors.h>
#include <Romi32U4.h>
#include <Chassis.h>
#include <IRdecoder.h>
#include <ir_codes.h>
#include <Rangefinder.h>

// #define k_p 0.085
// #define k_d 0.2
// #define k_i 0.0001

#define k_p 0.09
#define k_d 0.225
#define k_i 0.000

// Values Required For PID Calculations
const float kp = k_p, kd = k_d, ki = k_i;

// static int32_t I = 0;

// SETUP QTR SENSOR
QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

// IR Decoder
const uint8_t IR_DETECTOR_PIN = 1;
IRDecoder decoder(IR_DETECTOR_PIN);

// Chasis Functionality
Chassis chassis(7, 1440, 14.7);

// Ultrasonic sensor
Rangefinder rangefinder(11, 4);

// Motor Parameters
Romi32U4Motors motors;
static const int16_t MAX_SPEED = 420;
static const int16_t MIN_SPEED = 50;
static const uint16_t BASE_SPEED = 300;

// BUTTORN PARAMETERS
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;

enum ROBOT_STATE
{
  ROBOT_UNCALIBRATED,
  ROBOT_CALIBRATED,
  ROBOT_ORDERS,
  ROBOT_FOLLOW,
  ROBOT_TURNING,
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
// Directions ARoute = {'A', {'T', 'R', 'D', 'L', 'L', 'T', 'E'}, 0};
//  Directions ARoute = {'A', {'D', 'D', 'D', 'D', 'D', 'D', 'E'}, 0}; //Test Range Finder
Directions ARoute = {'A', {'T', 'R', 'R', 'T', 'L', 'L', 'T', 'E'}, 0};

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

void handleDropOff()
{
  // Call a function to Put the arm down Dropping off the package
  turn(180, 360);
}
void handleDelivery(float distance)
{
  Serial.print(distance);
  Serial.print(" cm\n");
  if (distance < 5)
  {
    handleDropOff();
  }
  else
  {
    chassis.driveFor(1, 30, true);
  }
}
// Function used to determined drop off sequence based on ROUTE
void dropOffSequence(Directions *dir)
{

  switch (dir->routeName)
  {
  case 'T':
    /* code */
    break;
  case 'A':
    chassis.driveFor(6, 90, true);
    turn(90, 90);
    robotState = ROBOT_DELIVERY;
    Serial.println("ROBOT STATE: DELIVERY");

    break;

  default:
    break;
  }
}

void drive(float distance, float speed)
{
  chassis.driveFor(distance, speed, true);
  robotState = ROBOT_FOLLOW;
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
      turn(-90, 90);
      InterCount++;
      break;
    case 'L': // Turn left
      chassis.driveFor(6, 90, true);
      turn(90, 90);
      InterCount++;
      break;
    case 'D': // drop off sequence
      // Call function to handle deliveries for each individual track
      dropOffSequence(&route[RouteCount]);
      InterCount++;
      break;
    case 'P': // pick up off sequence
      // Call function to handle Package Pick Up
      InterCount++;
      break;
    case 'E':
      chassis.turnFor(-180, 360, true);
      motors.setSpeeds(0, 0);
      InterCount = 0;
      RouteCount++;
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
  if ((left > 650) && (right > 650))
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
    // Add other cases
    default:
      break;
    }
  }
}

void handleKeyPress(int16_t keyPress)
{
  Serial.println("Key: " + String(keyPress));
  static char route[3];
  static uint8_t deliveries = 0;
  if (deliveries > 2)
  {
    deliveries = 2;
  }

  if (robotState == ROBOT_CALIBRATED)
  {
    // robotState = ROBOT_ORDERS;
    switch (keyPress)
    {
    case ENTER_SAVE:
      /// Directions were selected
      // BUILD PATH Sequence
      //  Switch State To Robot Follow
      Serial.println("ROBOT STATE: ROBOT_FOLLOW");
      for (int i = 0; i < 3; i++)
        Serial.println(route[i]);

      deliveries = 0;
      robotState = ROBOT_FOLLOW;
      pathSequence(route, 1);

      break;
    case NUM_1:
      route[deliveries] = 'a';
      Serial.println("Path A Selected");
      deliveries++;
      break;
    case NUM_2:
      route[deliveries] = 'b';
      Serial.println("Path B Selected");
      deliveries++;
      break;
    case NUM_3:
      route[deliveries] = 'c';
      Serial.println("Path C Selected");
      deliveries++;
      break;
    default:
      break;
    }
  }
}

void handleButton(int16_t keyPress)
{
  Serial.println("Key: " + String(keyPress));
  static char route[3] = {'a', 'a', 'a'};

  if (robotState == ROBOT_CALIBRATED || robotState == ROBOT_ORDERS)
  {
    switch (keyPress)
    {
    case 0:
      Serial.println("ROBOT STATE: ROBOT_FOLLOW");
      for (int i = 0; i < 3; i++)
        Serial.println(route[i]);

      pathSequence(route, 1);
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

  // Initializes the IR decoder
  decoder.init();

  // Initialize QTR sensor
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A7, A2, A3, A4, A0, A11}, SensorCount);

  // RangeFinder Init
  rangefinder.init();

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
  // else if (keyPress >= 0)
  //   handleKeyPress(keyPress);
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
    // motors.setSpeeds(0, 0);
    chassis.idle();
    handleTurnEvent(ROUTE, 1); // Array that stores destinations and number of destinations
    break;
  case ROBOT_ORDERS:
    // motors.setSpeeds(0, 0);
    chassis.idle();
    // Serial.println("ROBOT STATE: TURNING"); // Call a function that turns at appropriate direction and changes state back to appropriate state
    break;
  case ROBOT_DELIVERY:
    distance = rangefinder.getDistance();
    handleDelivery(distance);
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
  checkIntersectionEvent(sensorValues[1], sensorValues[4]);
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
  static float turnEffort = 0;
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
  Serial.print("\t\n");

  motorSpeedLimitBound(&m1Speed, &m2Speed);
  // Serial.print("M1: ");
  // Serial.print(m1Speed);
  // Serial.print("\t");

  // Serial.print("M2: ");
  // Serial.print(m1Speed);
  // Serial.println("\t");

  // motorSpeedLimitBound(&m1Speed, &m2Speed);

  motors.setSpeeds(m1Speed, m2Speed);
}
