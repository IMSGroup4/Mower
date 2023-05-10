#include <avr/wdt.h>
#include <MeAuriga.h>
#include "MeEEPROM.h"
#include <Wire.h>
#include <SoftwareSerial.h>

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
int16_t moveSpeed = 100;

double max_crash_distance_cm = 10;

float angleServo = 90.0;
float dt;

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
    double offset = target_deg + 360;
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
    changeDirection(Direction::backward);
    delay(150);
    SpinCWDeg(100-5);
    break;
  case S1_IN_S2_OUT:
    Serial.println("SENSOR 2 IS OUTSIDE OF BLACK LINE");
    changeDirection(Direction::backward);
    delay(150);
    SpinCWDeg(45-5);
    break;
  case S1_OUT_S2_IN:
    Serial.println("SENSOR 1 IS OUTSIDE OF BLACK LINE");
    changeDirection(Direction::backward);
    delay(150);
    SpinCCWDeg(45-5);
    break;
  case S1_OUT_S2_OUT:
    Serial.println("BOTH SENSORS ARE OUTSIDE");
    changeDirection(Direction::forward);
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

int readHeader = 0, leftMotor = 0, rightMotor= 0, spinDeg = 0;
int* intArray;

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
            Serial.println("autonomous");
            has_autonamous = true;
          }
        bool stateChanged = false;
        while(!stateChanged){
          if(Serial.available() > 0 ){
              intArray = getInformation();
              readHeader = intArray[0];
              if(readHeader != 1){
                stateChanged = true;
                Encoder_1.setMotorPwm(0);
                Encoder_2.setMotorPwm(0);
              }
          }
          int autonomousHeader = intArray[1];
          switch(autonomousHeader){
            case 0:
              grayscaleTest();
              break;
            case 1:
              changeDirection(Direction::stop);
              break;
            case 2:
              int spinDegrees = intArray[2];
              Serial.println(spinDegrees);
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
          }
        }
        /*
        bool stateChanged = false;
        while(!stateChanged){
          if(Serial.available() > 0 ){
              intArray = getInformation();
              //Serial.println(intArray[0]);
              //Serial.println(intArray[1]);
              //Serial.println(intArray[2]);
              readHeader = intArray[1];
              if(readHeader != 0){
                state_changed = false;
                Encoder_1.setMotorPwm(0);
                Encoder_2.setMotorPwm(0);
              }
          grayscaleTest();
        }
        while(stateChanged){
          //DO TASK
          //THEN GO INTO LINE MODE
        }
        //autonamous_flag = true;
        //start_autonamous();
        */
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
          while(!state_changed){
            if(Serial.available() > 0 ){
              intArray = getInformation();
              //Serial.println(intArray[0]);
              //Serial.println(intArray[1]);
              //Serial.println(intArray[2]);
              readHeader = intArray[0];
              if(readHeader != 1){
                state_changed = false;
                Encoder_1.setMotorPwm(0);
                Encoder_2.setMotorPwm(0);
              }
              //Serial.println("GOT INFO");
              //Serial. (harald);
              //DO OTHER STUFF
            }
          leftMotor = intArray[1];
          rightMotor = intArray[2];
          //Serial.println(leftMotor);
          //Serial.println(rightMotor);
          if((oldL != leftMotor) && (oldR != rightMotor)){
          Encoder_1.setMotorPwm(-leftMotor);
          Encoder_2.setMotorPwm(rightMotor);
          oldR = rightMotor;
          oldL = leftMotor;
          } 
          }
        //has_motor_control = false;
        //motor_control_flag = true;
        //start_motor_control()
        //if(shouldChangeMotorSpeed(leftMotor, rightMotor, intArray[1], intArray[2])){
          //Serial.println(leftMotor);
          //Serial.println(rightMotor);
        //}
        //else{
            //DO NOTHING
        //}
        break;
      case OBJECT_DETECTION:
        led.setColor(0,255,0);
        led.show();
        has_motor_control = false;
        has_autonamous = false;
        if(has_find_object == false){
        Serial.println("find object");
        has_find_object = true;
        }
        Serial.println(spinDeg);
        spinDeg = intArray[1];
        if(spinDeg == 0){
          break;
        }
        else if(spinDeg < 0){
          SpinCCWDeg(compensateSpinMotor((-spinDeg), 5)); 
        }
        else{
          SpinCWDeg(compensateSpinMotor(spinDeg, 5));
        }
        readHeader = 0;
        spinDeg = 0;
        has_find_object = false;
        //find_object_flag = true;
        //find_object(spinDeg, motorspeed);

        break;
      default:
        Encoder_1.setMotorPwm(0);
        Encoder_2.setMotorPwm(0);
          //Serial.write("No commands", 1);
        break;    
  } 
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

