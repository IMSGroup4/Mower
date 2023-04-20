

#define MOTOR_CONTROL_HEADER 1
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
  while (!Serial) {
    ; // wait for serial port to connect via USB
  }
}
int* intArray;
int header; 
int motorL;
int motorR;
void loop() {
  if(Serial.available() > 0 ){
    intArray = getInformation();
    header = intArray[0];
    motorL = intArray[1];
    motorR = intArray[2];
    //DO OTHER STUFF
  }
//[SetupIdentifier, Mode]
//[CommandType, PARAMS, ...]

  //69 autonomous, 420 motorcontrol, 1337 find object
    switch (header) {
      case 69:
        has_motor_control = false;
        has_find_object = false;
        if(has_autonamous == false){
          Serial.println("autonomous");
          has_autonamous = true;
        }
        //autonamous_flag = true;
        //start_autonamous();
        break;
      case 420:
        has_find_object = false;
        has_autonamous = false;
        if(has_motor_control == false){
          Serial.println("motorcontrol");
          Serial.println("LEFT MOTOR:   ");
          Serial.print(motorL);
          Serial.println("RIGHT MOTOR:  ");
          Serial.print(motorR);
          has_motor_control = true;
          }
        //has_motor_control = false;
        //motor_control_flag = true;
        //start_motor_control()
        break;
      case 1337:
      
        if(has_find_object == false){
        has_motor_control = false;
        has_autonamous = false;
        Serial.println("find object");
        has_find_object = true;
        }
        //find_object_flag = true;
        //find_object(spinDeg, motorspeed);

        break;
      default:
          //Serial.write("No commands", 1);
        break;    
  } 


}
