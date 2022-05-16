#include <Arduino.h>
#include <QTRSensors.h>
#include <Romi32U4.h>
#include <Chassis.h>
#include <Rangefinder.h>
#include <servo32u4.h>

#define DEBUG 1
#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

// Control Delivery Sequence
#define r1 't'
#define r2 't'
#define r3 't'

// Amount of Routes
#define numDest 1

#define k_p 0.08
#define k_d 0.375
#define k_i 0.0001

#define InterSens 0.45
#define CenterSens 0.38

// Value to detect intersections at home
//#define InterSens 0.54

// Value to detect intersections at school
// #define InterSens 0.45
// #define CenterSens 0.38

// Values Required For PID Calculations
const float kp = k_p, kd = k_d, ki = k_i;

// SETUP QTR SENSOR
QTRSensors qtr;
const uint8_t SensorCount = 6;
// const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];
uint16_t MEAN_RIGHT;
uint16_t MEAN_LEFT;
uint16_t CENTER_RIGHT;
uint16_t CENTER_LEFT;

// Chasis Functionality
Chassis chassis(7, 1440, 14.7);

// Ultrasonic sensor
Rangefinder rangefinder(11, 4);

// Motor Parameters
Romi32U4Motors motors;
static const int16_t MAX_SPEED = 420;
static const int16_t MIN_SPEED = 1;
static const uint16_t BASE_SPEED = 300;

// BUTTORN PARAMETERS
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;

// SERVO
Servo32U4 servo;
static const int16_t SERVO_DOWN = 1900;
static const int16_t SERVO_UP = 500;

static const int MAX_MESSAGE_LENGTH = 16;
static uint8_t routeCount = numDest;

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
  char turnSequence[16]; // How the robot handles intersection events
  uint16_t height;
} Directions;

static Directions ROUTE[numDest];

// add other nodes
Directions TestRoute = {'T', {'T', 'R', 'L', 'L', 'S', 'L', 'S', 'T', 'E'}, 0};

// ROUTES
Directions ARoute = {'A', {'P', 'R', 'R', 'D', 'L', 'L', 'T', 'E'}, 0};
Directions BRoute = {'B', {'P', 'R', 'S', 'D', 'S', 'L', 'T', 'E'}, 0};
Directions CRoute = {'C', {'P', 'R', 'L', 'R', 'D', 'L', 'R', 'L', 'T', 'E'}, 0};
Directions DRoute = {'D', {'P', 'R', 'L', 'S', 'D', 'S', 'R', 'L', 'T', 'E'}, 0};
Directions ERoute = {'E', {'P', 'R', 'L', 'L', 'R', 'D', 'L', 'R', 'R', 'L', 'T', 'E'}, 0};
Directions FRoute = {'F', {'P', 'R', 'L', 'L', 'L', 'D', 'R', 'R', 'R', 'L', 'T', 'E'}, 0};
Directions GRoute = {'G', {'P', 'R', 'L', 'L', 'S', 'R', 'D', 'L', 'S', 'R', 'R', 'L', 'T', 'E'}, 0};
Directions HRoute = {'H', {'P', 'R', 'L', 'L', 'S', 'S', 'D', 'S', 'S', 'R', 'R', 'L', 'T', 'E'}, 0};
Directions IRoute = {'I', {'P', 'L', 'D', 'R', 'T', 'E'}, 0};

void calibration();
void checkIntersectionEvent(int left, int right);
void inline TurnEffort(int16_t *lastError, float *turnEffort);
void inline SpeedIncrementFUNC(int16_t *sIncrement, int16_t *samples, int16_t *samples1);
void inline motorSpeedLimitBound(int *m1, int *m2);
void inline lineFollowingHandlerV1(uint16_t baseSpeed, bool acceleration, float distance);

void turn(float ang, float speed)
{
  chassis.turnFor(ang, speed, true);
  robotState = ROBOT_FOLLOW;
  // Serial1.println(robotState);
}
void drive(float distance, float speed)
{
  chassis.driveFor(distance, speed, true);
  robotState = ROBOT_FOLLOW;
}

void moveArmUp()
{
  for (int x = SERVO_DOWN; x > SERVO_UP; x -= 5)
  {
    servo.writeMicroseconds(x);
    delay(5);
  }
}

void moveArmDown()
{
  for (int x = SERVO_UP; x < SERVO_DOWN; x += 5)
  {
    servo.writeMicroseconds(x);
    delay(5);
  }
}

int handleDropOff(int height)
{

  int Microseconds = 0;
  switch (height)
  {
  case 0:
    // Case for 0 blocks
    for (int x = SERVO_UP; x < SERVO_DOWN; x++)
    {
      Microseconds = x;
      servo.writeMicroseconds(Microseconds);
      delay(5);
    }
    break;
  case 1:
    // Case for 1 blocks
    for (int x = SERVO_UP; x < 1580; x++)
    {

      Microseconds = x;
      servo.writeMicroseconds(Microseconds);
      delay(5);
    }
    break;
  case 2:
    // Case for 2 Blocks
    for (int x = SERVO_UP; x < 1300; x++)
    {
      Microseconds = x;
      servo.writeMicroseconds(Microseconds);
      delay(5);
    }
    break;

  default:
    for (int x = SERVO_UP; x < SERVO_DOWN; x++)
    {
      Microseconds = x;
      servo.writeMicroseconds(Microseconds);
      delay(5);
    }
    break;
  }

  return Microseconds;
}

void remainderDown(int microseconds)
{
  for (int x = microseconds; x < SERVO_DOWN; x += 5)
  {
    servo.writeMicroseconds(x);
    delay(5);
  }
}

void handleDelivery(int height)
{
  debugln(height);
  chassis.driveFor(-2, 60, true);
  int ms = handleDropOff(height);
  Serial1.println(robotState);
  delay(1000);
  chassis.driveFor(-2, 90, true);
  turn(180, 360);
  remainderDown(ms);
}

void handlePickUP(float distance)
{
  debugln(distance);
  debugln(" cm\n");
  if (distance < 8)
  {
    chassis.driveFor(-1, 30, true);
    moveArmUp();
    chassis.driveFor(-1, 90, true);
    turn(180, 180);
  }
  else
  {
    chassis.driveFor(1, 30, true);
  }
}

void dynamicTurnL(float ang, float speed)
{
  Serial1.println(robotState);
  int i = 0;
  chassis.turnFor(ang - 45, speed, true);
  delay(5);
  qtr.read(sensorValues);
  while (sensorValues[2] < 600 && sensorValues[3] < 600)
  {
    chassis.turnFor(ang - 45 + i, speed, false);
    // delay(1);
    i += 2;
    qtr.read(sensorValues);
    debug("LEFT: ");
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      debug(sensorValues[i]);
      debug('\t');
    }
    debugln();
  }
  chassis.driveFor(1, 90, true);
  i = 0;
  robotState = ROBOT_FOLLOW;
}

void dynamicTurnR(float ang, float speed)
{
  Serial1.println(robotState);
  int i = 0;
  chassis.turnFor(ang + 45, speed, true);
  delay(5);
  qtr.read(sensorValues);

  uint16_t meanR = (sensorValues[0] + sensorValues[1]) / 2;
  uint16_t meanL = (sensorValues[4] + sensorValues[5]) / 2;

  while (sensorValues[2] < meanR && sensorValues[3] < meanL)
  {
    qtr.read(sensorValues);
    chassis.turnFor(45 + ang + i, speed, false);
    // delay(1);
    i -= 2;
    debug("RIGHT: ");
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      debug(sensorValues[i]);
      debug('\t');
    }
    debugln();
  }

  chassis.driveFor(1, 90, true);
  i = 0;
  robotState = ROBOT_FOLLOW;
}

void dynamicTurn(float ang, float speed)
{
  Serial1.println(robotState);
  // qtr.read(sensorValues);
  int i = 0;

  if (ang < 0)
  {
    chassis.turnFor(ang, speed, true);
    qtr.read(sensorValues);
    while (sensorValues[2] < CENTER_RIGHT && sensorValues[3] < CENTER_LEFT)
    {
      qtr.read(sensorValues);
      chassis.turnFor(ang + i, speed, false);
      // delay(1);
      i--;
      debug("RIGHT: ");
      for (uint8_t i = 0; i < SensorCount; i++)
      {
        debug(sensorValues[i]);
        debug('\t');
      }
      debugln();
    }
  }
  else
  {
    chassis.turnFor(ang, speed, true);
    // delay(10);
    qtr.read(sensorValues);
    // while (sensorValues[1] < CENTER_RIGHT && sensorValues[2] < CENTER_LEFT)
    while (sensorValues[2] < CENTER_RIGHT && sensorValues[3] < CENTER_LEFT)
    {
      chassis.turnFor(ang + i, speed, false);
      // delay(1);
      i++;
      qtr.read(sensorValues);
      debug("LEFT: ");
      for (uint8_t i = 0; i < SensorCount; i++)
      {
        debug(sensorValues[i]);
        debug('\t');
      }
      debugln();
    }
  }

  // chassis.idle();
  chassis.driveFor(1, 90, true);
  i = 0;
  robotState = ROBOT_FOLLOW;
}

void handleTurnEvent(Directions route[], int n)
{

  static uint16_t InterCount = 0;
  static uint8_t RouteCount = 0;

  // debugln("ROBOT STATE: TURNING");
  // Serial1.println(robotState);
  //  debugln("\nInterCount: ");
  //  debugln(InterCount);

  if (RouteCount < n)
  {
    debugln(route[RouteCount].turnSequence[InterCount]);
    switch (route[RouteCount].turnSequence[InterCount])
    {
    case 'T': // Turn around
      // dynamicTurnL(180, 360);
      // turn(180, 360);
      dynamicTurn(180, 360);
      Serial1.println(robotState);
      InterCount++;
      break;
    case 'S':
      drive(3, 30); // Straight
      InterCount++;
      break;
    case 'R': // Turn right
      chassis.driveFor(6, 180, true);
      // dynamicTurnR(-90, 180);
      // turn(-90, 180);
      dynamicTurn(-90, 180);
      Serial1.println(robotState);
      InterCount++;
      break;
    case 'L': // Turn left
      chassis.driveFor(6, 180, true);
      // dynamicTurnL(90, 180);
      // turn(90, 180);
      dynamicTurn(90, 180);
      Serial1.println(robotState);
      InterCount++;
      break;
    case 'D': // drop off sequence
      // Transition to a different State temporarily
      robotState = ROBOT_DELIVERY;
      // chassis.idle();
      // Serial1.println(robotState);
      handleDelivery(route[RouteCount].height);

      InterCount++;
      break;
    case 'P': // pick up off sequence
      // Transition to a different State temporarily
      robotState = ROBOT_PICKUP;
      // chassis.driveFor(-1, 30, true);
      Serial1.println(robotState);
      InterCount++;
      break;
    case 'E':
      // servo.writeMicroseconds(SERVO_DOWN);
      dynamicTurn(-180, 360);
      // robotState = ROBOT_TURNING;

      // chassis.turnFor(-180, 360, true);
      chassis.driveFor(1, 90, true);
      motors.setSpeeds(0, 0);
      InterCount = 0;
      RouteCount++;
      if (RouteCount < n)
      {
        robotState = ROBOT_FOLLOW;
        Serial1.println(robotState);
      }
      else
      {
        RouteCount = 0;
        robotState = ROBOT_ORDERS;
        Serial1.println(robotState);
      }
      break;
    default:
      break;
    }
  }
}

void checkIntersectionEventV1(uint16_t meanR, uint16_t meanL)
{
  if ((meanL > MEAN_LEFT) && (meanR > MEAN_RIGHT))
  {
    chassis.idle();
    robotState = ROBOT_TURNING;
  }
}

// Eventually change this into array of structures
void checkIntersectionEvent(int left, int right)
{
  if ((left > 1500) && (right > 1500))
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
      debugln("ROUTE A WAS MARKED");
      break;
    case 'a':
      ROUTE[i] = ARoute;
      debugln("ROUTE A WAS MARKED");
      break;
    case 'b':
      ROUTE[i] = BRoute;
      debugln("ROUTE B WAS MARKED");
      break;
    case 'c':
      ROUTE[i] = CRoute;
      debugln("ROUTE C WAS MARKED");
      break;
    case 'd':
      ROUTE[i] = DRoute;
      debugln("ROUTE D WAS MARKED");
      break;
    // Add other cases
    default:
      break;
    }
  }
}

uint8_t handleButton(int16_t keyPress)
{
  debugln("Key: " + String(keyPress));
  static char route[3] = {r1, r2, r3};

  if (robotState == ROBOT_CALIBRATED || robotState == ROBOT_ORDERS)
  {
    switch (keyPress)
    {
    case 0:
      debugln("ROBOT STATE: ROBOT_FOLLOW");
      for (int i = 0; i < 3; i++)
        debugln(route[i]);

      pathSequence(route, numDest);
      robotState = ROBOT_FOLLOW;

      break;
    case 1:
      debugln("SELECTED ROUTES");
      break;
    default:
      break;
    }
  }
  return numDest;
}

uint8_t pathSequenceUART(char Des[], Directions *routes)
{

  debugln(Des);
  int n = Des[0] - '0';
  if (n > numDest)
    n = numDest;

  if (isDigit(Des[0]) && n > 0)
  {
    robotState = ROBOT_FOLLOW;

    int j = 0;
    for (int i = 1; i < n + 1; i++)
    {
      // debugln(Des[i]);
      //  debugln(Des[i + n]);
      switch (Des[i])
      {
      case 't':
        routes[j] = TestRoute;
        routes[j].height = Des[i + n] - '0';
        debugln("ROUTE T WAS MARKED");
        debugln(routes[j].height);
        break;
      case 'a':
        routes[j] = ARoute;
        routes[j].height = Des[i + n] - '0';
        debugln("ROUTE A WAS MARKED");
        debugln(routes[j].height);
        break;
      case 'b':
        routes[j] = BRoute;
        routes[j].height = Des[i + n] - '0';
        debugln("ROUTE B WAS MARKED");
        debugln(routes[j].height);
        break;
      case 'c':
        routes[j] = CRoute;
        routes[j].height = Des[i + n] - '0';
        debugln("ROUTE C WAS MARKED");
        debugln(routes[j].height);
        break;
      case 'd':
        routes[j] = DRoute;
        routes[j].height = Des[i + n] - '0';
        debugln("ROUTE D WAS MARKED");
        debugln(routes[j].height);
        break;
      case 'e':
        routes[j] = ERoute;
        routes[j].height = Des[i + n] - '0';
        debugln("ROUTE E WAS MARKED");
        debugln(routes[j].height);
        break;
      case 'f':
        routes[j] = FRoute;
        routes[j].height = Des[i + n] - '0';
        debugln("ROUTE F WAS MARKED");
        debugln(routes[j].height);
        break;
      case 'g':
        routes[j] = GRoute;
        routes[j].height = Des[i + n] - '0';
        debugln("ROUTE G WAS MARKED");
        debugln(routes[j].height);
        break;
      case 'h':
        routes[j] = HRoute;
        routes[j].height = Des[i + n] - '0';
        debugln("ROUTE H WAS MARKED");
        debugln(routes[j].height);
        break;
      case 'i':
        routes[j] = IRoute;
        routes[j].height = Des[i + n] - '0';
        debugln("ROUTE I WAS MARKED");
        debugln(routes[j].height);
        break;
      // Add other cases
      default:
        break;
      }

      j++;
    }
  }

  // Serial.println(Des);
  return n;
}

uint8_t handleUARTCOM(void)
{
  uint8_t routeCount = 0;
  while (Serial1.available() > 0)
  {
    static char MESSAGE[MAX_MESSAGE_LENGTH];
    static uint8_t pos = 0;

    char inByte = Serial1.read();
    if (inByte != '\n' && (pos < MAX_MESSAGE_LENGTH - 1))
    {
      MESSAGE[pos] = inByte;
      pos++;
    }
    else
    {
      MESSAGE[pos] = '\0';
      pos = 0;
      // debugln(MESSAGE);
      routeCount = pathSequenceUART(MESSAGE, ROUTE);
    }
  }

  return routeCount;
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(57600);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  delay(100);
  digitalWrite(13, HIGH);

  // Initialize Chasis
  chassis.init();

  // Initialize QTR sensor
  qtr.setTypeAnalog();

  qtr.setSensorPins((const uint8_t[]){A7, A2, A3, A4, A0, A11}, SensorCount);
  // qtr.setSensorPins((const uint8_t[]){A7, A3, A4, A11}, SensorCount);

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
    Serial1.println(robotState);
    qtr.resetCalibration();

    debugln("ROBOT STATE: UNCALIBRATED");
    ledRed(1);
    calibration();
    Serial1.println(robotState);
  }
  else if (Serial1.available())
  {
    if ((robotState == ROBOT_CALIBRATED || robotState == ROBOT_ORDERS))
      routeCount = handleUARTCOM();
  }

  else if (buttonB.getSingleDebouncedPress())
  {
    servo.writeMicroseconds(SERVO_DOWN);
    cPressedCount ^= 0b1;
    routeCount = handleButton(cPressedCount);
  }

  static float distance;
  switch (robotState)
  {
  case ROBOT_FOLLOW:

    distance = rangefinder.getDistance();

    debug("Distance: ");
    debug(distance); // return curent distance in serial
    debug("\t\t");

    if (distance < 25)
    {
      lineFollowingHandlerV1(100, false, distance);
    }
    else
    {
      lineFollowingHandlerV1(BASE_SPEED, true, distance);
      // lineFollowingHandler();
    }

    break;
  case ROBOT_TURNING:
    chassis.idle();
    handleTurnEvent(ROUTE, routeCount); // Array that stores destinations and number of destinations
    break;
  case ROBOT_ORDERS:
    chassis.idle();
    break;
  case ROBOT_PICKUP:
    distance = rangefinder.getDistance();
    handlePickUP(distance);
    break;
  case ROBOT_DELIVERY:
    chassis.idle();
    break;
  default:
    break;
  }
}

void calibration()
{
  MEAN_RIGHT = 0;
  MEAN_LEFT = 0;
  debugln("CALIBRATING: Please Wait...");
  for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate();
  }

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    debug(qtr.calibrationOn.minimum[i]);
    debug(' ');
  }
  debugln();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0, j = SensorCount - 1; i < SensorCount / 2; i++, j--)
  {
    debug(qtr.calibrationOn.maximum[i]);
    MEAN_RIGHT += qtr.calibrationOn.maximum[i];
    MEAN_LEFT += qtr.calibrationOn.maximum[j];
    debug(' ');
  }
  for (uint8_t i = SensorCount / 2; i < SensorCount; i++)
  {
    debug(qtr.calibrationOn.maximum[i]);
    // MEAN_LEFT += qtr.calibrationOn.maximum[i];
    debug(' ');
  }

  CENTER_RIGHT = qtr.calibrationOn.maximum[(SensorCount / 2 - 1)];
  CENTER_LEFT = qtr.calibrationOn.maximum[SensorCount / 2];

  CENTER_RIGHT = CENTER_RIGHT - ((float)CENTER_RIGHT * CenterSens);
  CENTER_LEFT = CENTER_LEFT - ((float)CENTER_LEFT * CenterSens);

  debug("\nCENTER_R: ");
  debugln(CENTER_RIGHT);
  debug("CENTER_L: ");
  debugln(CENTER_LEFT);

  // MEAN_RIGHT /= 3;
  // MEAN_LEFT /= 3;
  debug("\nMEAN_R: ");
  debugln(MEAN_RIGHT);
  debug("MEAN_L: ");
  debugln(MEAN_LEFT);

  MEAN_RIGHT = MEAN_RIGHT - ((float)MEAN_RIGHT * InterSens);
  MEAN_LEFT = MEAN_LEFT - ((float)MEAN_LEFT * InterSens);

  debugln("\nAdjusted with sensitivity: ");
  debug(InterSens);
  debug("\nMEAN_R: ");
  debugln(MEAN_RIGHT);
  debug("MEAN_L: ");
  debugln(MEAN_LEFT);
  robotState = ROBOT_CALIBRATED;
  debugln("\nROBOT STATE: CALIBRATED");

  ledRed(0);
  ledGreen(1);
}

void inline TurnEffort(int16_t *lastError, float *turnEffort)
{
  int position = qtr.readLineBlack(sensorValues);
  // int16_t error = position - 2500;
  int16_t error = position - 1500;
  static int I = 0;
  I += error;
  if (I > 10000)
    I = 0;
  // float turnEffort = (kp * error) + (ki * I) + (kd * (error - lastError));
  float tempTurnEffort = (kp * error) + (ki * I) + (kd * (error - (*lastError)));

  *lastError = error;
  *turnEffort = tempTurnEffort;
}
void inline SpeedIncrementFUNC(int16_t *sIncrement, int16_t *samples, int16_t *samples1)
{
  int16_t tempSI = *sIncrement;
  int16_t tempS = *samples;
  int16_t tempS1 = *samples1;
  qtr.read(sensorValues);

  // checkIntersectionEvent((sensorValues[0] + sensorValues[1] + sensorValues[2]), (sensorValues[3] + sensorValues[4] + sensorValues[5]));
  checkIntersectionEventV1((sensorValues[0] + sensorValues[1] + sensorValues[2]), (sensorValues[3] + sensorValues[4] + sensorValues[5]));

  // checkIntersectionEventV1((sensorValues[0] + sensorValues[1]), (sensorValues[3] + sensorValues[4]));
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    debug(sensorValues[i]);
    debug('\t');
  }

  // if ((sensorValues[1] > (sensorValues[0])) || (sensorValues[2] > (sensorValues[4])))
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

void inline lineFollowingHandlerV1(uint16_t baseSpeed, bool acceleration, float distance)
{
  float turnEffort = 0;
  static int16_t sIncrement = 0;
  static int16_t samples = 0;
  static int16_t samples1 = 0;
  static int16_t lastError = 0;

  // Final Motor Speeds
  int16_t m1Speed;
  int16_t m2Speed;

  if (acceleration)
  {
    SpeedIncrementFUNC(&sIncrement, &samples, &samples1);
    TurnEffort(&lastError, &turnEffort);
  }
  else
  {
    SpeedIncrementFUNC(&sIncrement, &samples, &samples1);
    // qtr.read(sensorValues);
    const int16_t MAX_DISTANCE = 25;
    // const int16_t STOP_DISTANCE = 1;
    // const float DISTANCE_FACTOR = MAX_DISTANCE / 1000;
    // const float MOTOR_FACTOR = baseSpeed / 1000;

    TurnEffort(&lastError, &turnEffort);
    sIncrement = 0;
    float magnitude = (float)(MAX_DISTANCE - distance) / (float)MAX_DISTANCE;
    float cruiseRate = magnitude * (float)baseSpeed;
    // Serial.println(cruiseRate);

    if (cruiseRate > baseSpeed)
    {
      m1Speed = (MIN_SPEED)-turnEffort;
      m2Speed = (MIN_SPEED) + turnEffort;
    }
    else
    {
      m1Speed = (int)(baseSpeed - cruiseRate) - turnEffort;
      m2Speed = (int)(baseSpeed - cruiseRate) + turnEffort;
    }
  }

  m1Speed = (baseSpeed + sIncrement) - turnEffort;
  m2Speed = (baseSpeed + sIncrement) + turnEffort;

  debug("Error: ");
  debug(lastError);
  debug("\t");

  debug("Turn Effort: ");
  debug(turnEffort);
  debug("\t");

  motorSpeedLimitBound(&m1Speed, &m2Speed);
  // debug("M1: ");
  debug(m1Speed);
  debug("\t");

  debug("M2: ");
  debug(m1Speed);
  debugln("\t");

  // motorSpeedLimitBound(&m1Speed, &m2Speed);

  motors.setSpeeds(m1Speed, m2Speed);
}
