#include <avr/wdt.h>
#include <MeAuriga.h>
#include "MeEEPROM.h"
#include <Wire.h>
#include <SoftwareSerial.h>
#include <string.h>

//#define DEBUG_INFO
//#define DEBUG_INFO1
#define AUTONOMOUS  10
#define JOYSTICK  15
#define MOTOR_CONTROL 1
#define OBJECT_DETECTION  2

Servo servos[12];  
MeDCMotor dc;
MeTemperature ts;
MeRGBLed led(PORT_3);
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
MeEncoderOnBoard LeftMotor(SLOT1);
MeEncoderOnBoard RightMotor(SLOT2);
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

int leftSpeed = 0;
int rightSpeed = 0;

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
int16_t moveSpeed = 100;

double max_crash_distance_cm = 10;

float angleServo = 90.0;
float dt;

void setMotorPwm(int16_t pwm);

void updateSpeed(void);



bool has_motor_control = false;
bool has_autonamous = false;
bool has_find_object = false;


String readData(){
  String data;
  if(Serial.available() > 0){
    data = Serial.readString();
    data.trim(); //Remove access symbols at end, might cause issues so be careful
    return data;
  }
  else{
    return "";
  }
}


int getDataAmount(String data){
  int count = 0;
  for(char c : data){
    if(c == ','){
      count++;
    }
  }
  return count +1;
}

String* parseDataString(String data, int length){
  String buff = "";
  String* dataArray = new String[length];
  int i = 0;
  for(char c : data){
    if(c == ','){
      dataArray[i] = buff;
      i++;
      buff = "";
    }
    else{
      buff += c;
    }
  }
  dataArray[length-1] = buff;
  return dataArray;
}
int* dataToIntArray(String* parsedData, int length){
  int* array = new int[length];
  for(int i = 0; i < length; i++){
    array[i] = parsedData[i].toInt();
  }
  delete[] parsedData;
  return array;
}
int* getInformation(){
    String data = readData();
    int length = getDataAmount(data);
    String* parsedData = parseDataString(data, length);
    int* intArray = dataToIntArray(parsedData, length);
    return intArray;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);
  
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
  gyro.begin();
  //Serial.println(Compass.testConnection());
  //direction = Direction::forward;
}

int leftCompensation = 10;
void Forward(void)
{
  leftSpeed = -moveSpeed;
  rightSpeed = moveSpeed;
  LeftMotor.setMotorPwm(leftSpeed);
  RightMotor.setMotorPwm(rightSpeed);
}
void Backward(void)
{
  leftSpeed = moveSpeed;
  rightSpeed = -moveSpeed;
  LeftMotor.setMotorPwm(leftSpeed);
  RightMotor.setMotorPwm(rightSpeed);
}
void TurnLeft(void)
{
  leftSpeed = -moveSpeed;
  rightSpeed = round(moveSpeed/2);
  LeftMotor.setMotorPwm(leftSpeed);
  RightMotor.setMotorPwm(rightSpeed);
}

void TurnRight(void)
{
  leftSpeed = round(-moveSpeed/2);
  rightSpeed = moveSpeed;
  LeftMotor.setMotorPwm(leftSpeed);
  RightMotor.setMotorPwm(rightSpeed);
}
void BackwardAndTurnLeft(void)
{
  leftSpeed = round(moveSpeed/4);
  rightSpeed = -moveSpeed,
  LeftMotor.setMotorPwm(leftSpeed);
  RightMotor.setMotorPwm(rightSpeed);
}
void BackwardAndTurnRight(void)
{
  leftSpeed = moveSpeed;
  rightSpeed = round(-moveSpeed / 4);
  LeftMotor.setMotorPwm(leftSpeed);
  RightMotor.setMotorPwm(rightSpeed);
}

void SpinCW(){
  leftSpeed = int(moveSpeed * 1.5);
  rightSpeed = int(moveSpeed * 1.5);
  LeftMotor.setMotorPwm(leftSpeed);
  RightMotor.setMotorPwm(rightSpeed);
}
void SpinCCW(){
  leftSpeed = int(-moveSpeed * 1.5);
  rightSpeed = int(-moveSpeed * 1.5);
  LeftMotor.setMotorPwm(leftSpeed);
  RightMotor.setMotorPwm(rightSpeed);
}
void SpinCWHalfSpeed(){
  leftSpeed = round(moveSpeed/2);
  rightSpeed = round(moveSpeed/2);
  LeftMotor.setMotorPwm(leftSpeed);
  RightMotor.setMotorPwm(rightSpeed);
}
void SpinCCWHalfSpeed(){
  leftSpeed = round(-moveSpeed / 2);
  rightSpeed = round(-moveSpeed / 2);
  LeftMotor.setMotorPwm(leftSpeed);
  RightMotor.setMotorPwm(rightSpeed);
}

void Stop(void){
  leftSpeed = 0;
  rightSpeed = 0;
  LeftMotor.setMotorPwm(leftSpeed);
  RightMotor.setMotorPwm(rightSpeed);
}
void RAM(void){
  leftSpeed = -255;
  rightSpeed = 255;
  LeftMotor.setMotorPwm(leftSpeed);
  RightMotor.setMotorPwm(rightSpeed);
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


void updatePosition(){
  //double* pos = new double[3];
  float wheelCirc  = M_PI * 0.045;
  gyro.fast_update();
  double Sangle = gyro.getAngleZ() +180;
  double LRPMS = leftSpeed;
  double RRPMS = rightSpeed;
  double   = wheelCirc * LRPMS;
  double rightMotorSpeed = wheelCirc * RRPMS;
  //Serial.println("LEFT MOTOR SPEED ");
  //Serial.println(leftSpeed);
  //Serial.println("RIGHT MOTOR SPEED ");
  //Serial.println(rightSpeed);
  double avgSpeed = (leftMotorSpeed + rightMotorSpeed)/2;
  //int newX = oldX + (cos(radians(Sangle)) * avgSpeed);
  //double newY = oldY + (sin(radians(Sangle)) * avgSpeed);
  String sendData = String(-leftSpeed) + "," + String(rightSpeed) + "," + Sangle;
  
  Serial.println(sendData);
  //Serial.write("hej",4);

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
  if(target_deg< -180){
    target_deg += 360;
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
    //Serial.println("BOTH SENSORS ON LINE");
    changeDirection(Direction::backward);
    delay(150);
    SpinCWDeg(100-5);
    break;
  case S1_IN_S2_OUT:
    //Serial.println("SENSOR 2 IS OUTSIDE OF BLACK LINE");
    changeDirection(Direction::backward);
    delay(150);
    SpinCWDeg(45-5);
    break;
  case S1_OUT_S2_IN:
    //Serial.println("SENSOR 1 IS OUTSIDE OF BLACK LINE");
    changeDirection(Direction::backward);
    delay(150);
    SpinCCWDeg(45-5);
    break;
  case S1_OUT_S2_OUT:
    //Serial.println("BOTH SENSORS ARE OUTSIDE");
    changeDirection(Direction::forward);
    break;

  default:
    //Serial.println("ERROR");
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


bool shouldChangeMotorSpeed(int oldL, int oldR, int newL, int newR){
//Serial.print("OLD L");
//Serial.println(oldL);
//Serial.print("OLD R");
//Serial.println(oldR);
//Serial.print("NEW L");
//Serial.println(newL);
//Serial.print("NEW R");
//Serial.println(newR);
int leftMotorDiff = abs(oldL-newL);
int rightMotorDiff = abs(oldR-newR);
int totalDiff = leftMotorDiff + rightMotorDiff;
if(totalDiff > 20){
  return true;
}
else{
  return false;
}
}
int compensateSpinMotor(int angle, int compensation){
  if((angle-compensation) < 0){
    return abs(angle-compensation);
  }
  else{
    return (angle-compensation);
  }
}

int readHeader = 0, spinDeg = 0;
int* intArray;
int posX, posY = 0;
void loop() {
  //readHeader = 10;
  if(Serial.available() > 0 ){
    intArray = getInformation();
    readHeader = intArray[0];
    //Serial.println("GOT INFO");
    //Serial. (harald);
    //DO OTHER STUFF
  }
  switch (readHeader) {
      case 10:
          has_motor_control = false;
          has_find_object = false;
          if(has_autonamous == false){
            //Serial.println("autonomous");
            has_autonamous = true;
          }
        while(has_autonamous){
          if(Serial.available() > 0 ){
              intArray = getInformation();
              readHeader = intArray[0];
              if(readHeader != 10){
                has_autonamous = false;
                leftSpeed = 0;
                rightSpeed = 0;
                LeftMotor.setMotorPwm(leftSpeed);
                RightMotor.setMotorPwm(rightSpeed);
              }
          }
          int autonomousHeader = intArray[1];
          //Serial.println(autonomousHeader);
          updatePosition();
          switch(autonomousHeader){
            case 0:
              //Serial.println("Starting GrayscaleTest");
              grayscaleTest();
              break;
            case 1:
              //Serial.println("STOPSTOP");
              changeDirection(Direction::stop);
              break;
            
            case 3:
              updatePosition();
              intArray[1] = 0;
              break;
            
            case 2:
              int spinDegrees = intArray[2];
              //Serial.println(spinDegrees);
              if(spinDegrees > 0){
                if(spinDegrees > 10){
                SpinCWDeg(spinDegrees - 5);
                }
                else{
                  SpinCWDeg(spinDegrees);
                }
              }
              else{
                if(spinDegrees < -10){
                SpinCCWDeg(-(spinDegrees+5));
                }
                else{
                  SpinCCWDeg(-spinDegrees);
                }
              }
              intArray[1] = 1;
              intArray[2] = 0;
              break;

            default:
              //Serial.print(autonomousHeader);
              break;
          }
        }
        break;
      case 1:
        //Serial.println("ENTERED MOTOR CONTROL");
        has_find_object = false;
        has_autonamous = false;
        if(has_motor_control == false){
          has_motor_control = true;
          }
          bool state_changed = false;
          int oldR = 0;
          int oldL = 0;
          while(has_motor_control){
            updatePosition();
            if(Serial.available() > 0 ){
              intArray = getInformation();
              readHeader = intArray[0];
              if(readHeader != 1){
                has_motor_control = false;
                changeDirection(Direction::stop);
              }
            }
          if(intArray[1] == 3){
            updatePosition();
              
          }
          else{
          leftSpeed = -intArray[1];
          rightSpeed = intArray[2];
          //Serial.println(leftMotor);
          //Serial.println(rightMotor);
          if((oldL != leftSpeed) && (oldR != rightSpeed)){
          LeftMotor.setMotorPwm(leftSpeed);
          RightMotor.setMotorPwm(rightSpeed);
          oldR = rightSpeed;
          oldL = leftSpeed;
          } 
          }
          }
        break;
      
      default:
        changeDirection(Direction::stop);
          //Serial.write("No commands", 1);
        break;
  } 
  } 

