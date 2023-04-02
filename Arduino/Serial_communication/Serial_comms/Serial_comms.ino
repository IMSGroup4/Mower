

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect via USB
  }
}

void loop() {
  char data;
  
  if (Serial.available() > 0) {
    data = Serial.read();
    switch (data) {
    case 'w':
      Serial.write("W", 1);
      break;
    case 's':
      Serial.write("S", 1);
      break;
    case 'a':
      Serial.write("A", 1);
      break;
    case 'd':
      Serial.write("D", 1);
      break;
    default:
      Serial.write("No commands", 1);
      break;    
    }
  } 
}
