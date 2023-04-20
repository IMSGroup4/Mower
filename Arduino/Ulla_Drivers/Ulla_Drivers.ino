#include <avr/wdt.h>
#include <MeAuriga.h>
#include "MeEEPROM.h"
#include <Wire.h>
#include <SoftwareSerial.h>

//#define DEBUG_INFO
//#define DEBUG_INFO1

Servo servos[12];  
MeDCMotor dc;
MeTemperature ts;
MeRGBLed led;
MeUltrasonicSensor ultraSensor(PORT_7); //PORT_10
Me7SegmentDisplay seg;
MePort generalDevice;
MeLEDMatrix ledMx;
MeInfraredReceiver *ir = NULL;  //PORT_8
MeGyro gyro_ext(0,0x68);  //external gryo sensor
MeGyro gyro(1,0x69);      //On Board external gryo sensor
MeCompass Compass;
MeJoystick joystick;
MeStepper steppers[4];
MeBuzzer buzzer;
MeHumiture humiture;
MeFlameSensor FlameSensor;
MeGasSensor GasSensor;
MeTouchSensor touchSensor;
Me4Button buttonSensor;
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeLineFollower line(PORT_6);
MeEncoderMotor encoders[2];
MePm25Sensor *pm25sensor = NULL;
MeSmartServo *mysmartservo = NULL;

typedef struct MeModule
{
  int16_t device;
  int16_t port;
  int16_t slot;
  int16_t pin;
  int16_t index;
  float values[3];
} MeModule;

/*
struct Driver
{
  enum Mode = {
    Autonomous,
    Controller,
    Toddler
  }Mode;
  enum STATE = {
    Driving,
    Scanning,
    
  }State;

}Driver;
*/
enum Direction {
  forward,
  backward,
  left,
  right,
  backwardAndLeft,
  backwardAndRight,
  spinCW,
  spinCCW,
  spinCWHalfSpeed,
  spinCCWHalfSpeed,
  stop
}direction;

union
{
  uint8_t byteVal[4];
  float floatVal;
  long longVal;
}val;

union
{
  uint8_t byteVal[8];
  double doubleVal;
}valDouble;

union
{
  uint8_t byteVal[2];
  int16_t shortVal;
}valShort;
MeModule modules[12];
#if defined(__AVR_ATmega32U4__) 
  int16_t analogs[12]={A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11};
#endif
#if defined(__AVR_ATmega328P__) or defined(__AVR_ATmega168__)
  int16_t analogs[8]={A0,A1,A2,A3,A4,A5,A6,A7};
#endif
#if defined(__AVR_ATmega1280__)|| defined(__AVR_ATmega2560__)
  int16_t analogs[16]={A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15};
#endif

int16_t len = 52;
int16_t servo_pins[12]={0,0,0,0,0,0,0,0,0,0,0,0};
//Just for Auriga
int16_t moveSpeed = 180;
int16_t turnSpeed = 180;
int16_t minSpeed = 45;
int16_t factor = 23;
int16_t distance=0;
int16_t randnum = 0;                                                                               
int16_t auriga_power = 0;
int16_t LineFollowFlag=0;
#define MOVE_STOP       0x00
#define MOVE_FORWARD    0x01
#define MOVE_BACKWARD   0x02

#define BLUETOOTH_MODE                       0x00
#define AUTOMATIC_OBSTACLE_AVOIDANCE_MODE    0x01
#define BALANCED_MODE                        0x02
#define IR_REMOTE_MODE                       0x03
#define LINE_FOLLOW_MODE                     0x04
#define MAX_MODE                             0x05

#define POWER_PORT                           A4
#define BUZZER_PORT                          45
#define RGBLED_PORT                          44

uint8_t command_index = 0;
uint8_t auriga_mode = BLUETOOTH_MODE;
uint8_t index = 0;
uint8_t dataLen;
uint8_t modulesLen=0;
uint8_t irRead = 0;
uint8_t prevc=0;
uint8_t keyPressed = KEY_NULL;
uint8_t serialRead;
uint8_t buffer[52];

double  lastTime = 0.0;
double  currentTime = 0.0;
double  CompAngleY, CompAngleX, GyroXangle;
double  LastCompAngleY, LastCompAngleX, LastGyroXangle;
double  last_turn_setpoint_filter = 0.0;
double  last_speed_setpoint_filter = 0.0;
double  last_speed_error_filter = 0.0;
double  speed_Integral_average = 0.0;
double  angle_speed = 0.0;
double  balance_car_speed_offsets = 0.0;

double max_crash_distance_cm = 10;

float angleServo = 90.0;
float dt;

long lasttime_angle = 0;
long lasttime_speed = 0;
long update_sensor = 0;
long blink_time = 0;
long rxruntime = 0;
long lasttime_receive_cmd = 0;
long last_Pulse_pos_encoder1 = 0;
long last_Pulse_pos_encoder2 = 0;

boolean isStart = false;
boolean isAvailable = false;
boolean leftflag;
boolean rightflag;
boolean start_flag = false;
boolean move_flag = false;
boolean boot_show_flag = true;
boolean blink_flag = false;

String mVersion = "09.01.016";


//////////////////////////////////////////////////////////////////////////////////////
float RELAX_ANGLE = -1;                    //Natural balance angle,should be adjustment according to your own car
#define PWM_MIN_OFFSET_BIAS   5

#define VERSION                0
#define ULTRASONIC_SENSOR      1
#define TEMPERATURE_SENSOR     2
#define LIGHT_SENSOR           3
#define POTENTIONMETER         4
#define JOYSTICK               5
#define GYRO                   6
#define SOUND_SENSOR           7
#define RGBLED                 8
#define SEVSEG                 9
#define MOTOR                  10
#define SERVO                  11
#define ENCODER                12
#define IR                     13
#define IRREMOTE               14
#define PIRMOTION              15
#define INFRARED               16
#define LINEFOLLOWER           17
#define IRREMOTECODE           18
#define SHUTTER                20
#define LIMITSWITCH            21
#define BUTTON                 22
#define HUMITURE               23
#define FLAMESENSOR            24
#define GASSENSOR              25
#define COMPASS                26
#define TEMPERATURE_SENSOR_1   27
#define DIGITAL                30
#define ANALOG                 31
#define PWM                    32
#define SERVO_PIN              33
#define TONE                   34
#define BUTTON_INNER           35
#define ULTRASONIC_ARDUINO     36
#define PULSEIN                37
#define STEPPER                40
#define LEDMATRIX              41
#define TIMER                  50
#define TOUCH_SENSOR           51
#define JOYSTICK_MOVE          52
#define COMMON_COMMONCMD       60
  //Secondary command
  #define SET_STARTER_MODE     0x10
  #define SET_AURIGA_MODE      0x11
  #define SET_MEGAPI_MODE      0x12
  #define GET_BATTERY_POWER    0x70
  #define GET_AURIGA_MODE      0x71
  #define GET_MEGAPI_MODE      0x72
#define ENCODER_BOARD          61
  //Read type
  #define ENCODER_BOARD_POS    0x01
  #define ENCODER_BOARD_SPEED  0x02

#define ENCODER_PID_MOTION     62
  //Secondary command
  #define ENCODER_BOARD_POS_MOTION         0x01
  #define ENCODER_BOARD_SPEED_MOTION       0x02
  #define ENCODER_BOARD_PWM_MOTION         0x03
  #define ENCODER_BOARD_SET_CUR_POS_ZERO   0x04
  #define ENCODER_BOARD_CAR_POS_MOTION     0x05


struct DriveInformation{
  enum STATE = {SET_MOTOR, SPIN};
  leftMotor = 0;
  rightMotor = 0;
}driveInformation;

void setMotorPwm(int16_t pwm);

void updateSpeed(void);

double getDistanceCm(){
  double distance = ultraSensor.distanceCm();
  return distance;
}
bool crashDetection(double limit){
  
  if( getDistanceCm()<= limit){
    //C is a placeholder for a crash symbol
    //Serial.write("C")
    //LOCK DRIVING HERE TO AWAIT FURTHER ENVIRONMENTAL INFORMATION
    return true;
  }
  else{
    return false;
  }
}
void crashAvoidance(){
  Backward();
  TurnLeft();

}
void toddlerTime(){
  if(crashDetection(max_crash_distance_cm)){
    Backward();
    Stop();
    RAM();
    
  }
}

void setup() {
  Serial.begin(115200);
  
  encoders[0] = MeEncoderMotor(SLOT_1);
  encoders[1] = MeEncoderMotor(SLOT_2);
  encoders[0].begin();
  encoders[1].begin();

  
  wdt_reset();
  encoders[0].runSpeed(0);
  encoders[1].runSpeed(0);

  //Set Pwm 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  while (!Serial) {
    ; // wait for serial port to connect via USB
  }
  direction = Direction::stop;
  buzzer.setpin(BUZZER_PORT);
  gyro.begin();
  Serial.println(Compass.testConnection());
  //direction = Direction::forward;
}

int leftCompensation = 10;
void Forward(void)
{
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed + leftCompensation);
}
void Backward(void)
{
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed);
}
void TurnLeft(void)
{
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed/2);
}

void TurnRight(void)
{
  Encoder_1.setMotorPwm(-moveSpeed/2);
  Encoder_2.setMotorPwm(moveSpeed);
}
void BackwardAndTurnLeft(void)
{
  Encoder_1.setMotorPwm(moveSpeed/4);
  Encoder_2.setMotorPwm(-moveSpeed);
}
void BackwardAndTurnRight(void)
{
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed/4);
}

void SpinCW(){
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed);
}
void SpinCCW(){
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed);
}
void SpinCWHalfSpeed(){
  Encoder_1.setMotorPwm(moveSpeed/2);
  Encoder_2.setMotorPwm(moveSpeed/2);
}
void SpinCCWHalfSpeed(){
  Encoder_1.setMotorPwm(-moveSpeed/2);
  Encoder_2.setMotorPwm(-moveSpeed/2);
}

void Stop(void){
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);
}
void RAM(void){
  Encoder_1.setMotorPwm(-255);
  Encoder_2.setMotorPwm(255);
}
void changeDirection(Direction dir){
  if(dir == direction){
    //DO NOTHING
  }
  else{
    switch(dir){
      case Direction::forward:
      Forward();
      break;
      case Direction::backward:
      Backward();
      break;
      case Direction::left:
      TurnLeft();
      break;
      case Direction::right:
      TurnRight();
      break;
      case Direction::backwardAndLeft:
      BackwardAndTurnLeft();
      break;
      case Direction::backwardAndRight:
      BackwardAndTurnRight();
      break;
      case Direction::stop:
      Stop();
      break;
      case Direction::spinCW:
      SpinCW();
      break;
      case Direction::spinCCW:
      SpinCCW();
      break;
      case Direction::spinCWHalfSpeed:
      SpinCWHalfSpeed();
      break;
      case Direction::spinCCWHalfSpeed:
      SpinCCWHalfSpeed();
      break;
    }
    direction = dir;
  }
}

void SpinCWDeg(int deg){
  gyro.update();
  double initial_deg = gyro.getAngleZ();
  double target_deg = initial_deg + deg;
  if(target_deg>= 180){
    double offset = target_deg - 180;
    target_deg = offset - 180;
  }
  double current_deg = gyro.getAngleZ();
  while((int)target_deg != (int)current_deg){
    gyro.update();
    current_deg = gyro.getAngleZ();
    changeDirection(Direction::spinCW);

}
changeDirection(Direction::stop);
}
void SpinCCWDeg(int deg){
  gyro.update();
  double initial_deg = gyro.getAngleZ();
  double target_deg = initial_deg - deg;
  if(target_deg<= -180){
    double offset = target_deg + 180;
    target_deg = offset - 180;
  }
  double current_deg = gyro.getAngleZ();
  while((int)target_deg != (int)current_deg){
    gyro.update();
    current_deg = gyro.getAngleZ();
    changeDirection(Direction::spinCCW);

}
changeDirection(Direction::stop);
}

/**
 * @brief This function reads a char = {w,s,a,d} and responds by driving in a direction ={Front,Back,Left,Right}
 * it also responds using UART with the command that was run as feedback
 * 
 */

void grayscaleTest(){
  int sensorState = line.readSensors();
  switch (sensorState)
  {
  case S1_IN_S2_IN:
    Serial.println("BOTH SENSORS ON LINE");
    break;
  case S1_IN_S2_OUT:
    Serial.println("SENSOR 2 IS OUTSIDE OF BLACK LINE");
    break;
  case S1_OUT_S2_IN:
    Serial.println("SENSOR 1 IS OUTSIDE OF BLACK LINE");
    break;
  case S1_OUT_S2_OUT:
    Serial.println("BOTH SENSORS ARE OUTSIDE");
    break;

  default:
    Serial.println("ERROR");
    break;
  }
}
void readSerialBus()
{
  char data;
  
  if (Serial.available() > 0) {
    data = Serial.read();
    switch (data) {
    case 'w':
      Serial.write("W", 1);
      Forward();
      break;
    case 's':
      Serial.write("S", 1);
      Backward();
      break;
    case 'a':
      Serial.write("A", 1);
      TurnLeft();
      break;
    case 'd':
      Serial.write("D", 1);
      TurnRight();
      break;
    default:
      Serial.write("No commands", 12);
      break;    
    }
  }
}
void runOnSerialBus()
{
  char data;
  if(Serial.available() > 0){
    data = Serial.read();
    switch(data){
      case 'w':
      changeDirection(Direction::forward);
      break;
      case 's':
      changeDirection(Direction::backward);
      break;
      case 'a':
      changeDirection(Direction::left);
      break;
      case 'd':
      changeDirection(Direction::right);
      break;
      default:
      break;
    }

  }
}

void scanForObstacles(){
  changeDirection(Direction::backward);
  delay(1000);
  //TODO: TUNE THESE DELAYS
  changeDirection(Direction::spinCW);
  delay(1000);
  double rightVal = getDistanceCm();
  changeDirection(Direction::spinCCW);
  delay(2000);
  double leftVal = getDistanceCm();
  if(rightVal >= leftVal){
    changeDirection(Direction::right);
    delay(2000);
  }
  changeDirection(Direction::forward);
}



void loop() {

  double distance = getDistanceCm();
  Serial.println(distance);
  /*
  gyro.update();
  double z_ang = gyro.getAngleZ();
  Serial.println(z_ang);
  */
  /*
  changeDirection(Direction::forward);

  if(crashDetection(max_crash_distance_cm * 2)){
    scanForObstacles();
  

  }
  */

  /*
  do{
  Forward(); 
  }
  while (crashDetection(max_crash_distance_cm));

  do{
    TurnRight(); 
  }
  while(!crashDetection(max_crash_distance_cm*2));
  */
  } 

