// programme pour tester le codage du message

float _rain = 24;
float _windDir = 135;
float _windSpeed = 32;
float _tempDHT = 23.5;
float _humidity = 80;
float _tempBMP = 24.9;
float _pressure = 1032;
float _altitude = 300;
float _light = 10;
float _tempRTC = 30;
 
float _batteryVoltage = 3.7;
float _batteryTemp = 50;

float valeurTest = 1000;

char radioBuffer[62];
char bufferTest[4];

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  
  
  Serial.println("*****************************************************");
  /*
  value2BuffTest(valeurTest);
  Serial.println("CodageTest");
  for(int j=0; j<4; j++){
    Serial.print(bufferTest[j], HEX);
  }
  Serial.println("");
  Serial.println("Decodage : ");
  Serial.println(buff2ValueTest());
  */

  
  Serial.println("Codage Message Radio");
  codingMessage();
  displayBuffer();
  testDecodage();
  
}



void value2Buff(float value, int start, byte tempTest = false)
{
  // this fonction transforme the value in int
  // then write the value in the 4 byte starting at start

  int valueInt;
  if (tempTest) {
    // if it's a temp value, add an offset of +40°C to avoid negative number then multiply by 10
    valueInt = int(roundf(((value + 40) * 10)));
  }
  else
  { // else only multiply by 10 to keep one digit
    valueInt = int(roundf(value * 10));
  }

  radioBuffer[start + 3] = valueInt & 0xff;
  radioBuffer[start + 2] = (valueInt >> 8) & 0xff;
  radioBuffer[start + 1] = (valueInt >> 16) & 0xff;
  radioBuffer[start] = (valueInt >> 24) & 0xff;
}

float buff2Value(int start, byte tempTest = false)
{
  int valueInt;
  byte byte1 = radioBuffer[start]; // compter de gauche à droite
  byte byte2 = radioBuffer[start + 1];
  byte byte3 = radioBuffer[start + 2];
  byte byte4 = radioBuffer[start + 3];
  valueInt = (int)(byte1 << 24 | byte2 << 16 | byte3 << 8 | byte4);

  float value;
  if (tempTest) {
    // if it's a temp value, divise by 10 then remove the +40°C offset
    value = (float(valueInt) / 10) - 40;
  }
  else
  { // else only divise by 10
    value = float(valueInt) / 10;
  }

  return (value);
}

void codingMessage()
{
  // this function create the message for the radio
  value2Buff(_rain, 0);
  value2Buff(_windDir, 4);
  value2Buff(_windSpeed, 8);
  value2Buff(_tempDHT, 12, true);
  value2Buff(_humidity, 16);
  value2Buff(_tempBMP, 20, true);
  value2Buff(_pressure, 24);
  value2Buff(_altitude, 28);
  value2Buff(_light, 32);
  value2Buff(_tempRTC, 36, true);

  value2Buff(_batteryVoltage, 52);
  value2Buff(_batteryTemp, 56, true);
}


void displayBuffer(){
  
  Serial.println("Buffer radio :");
  for( int i =0; i<62; i++){
    Serial.print(radioBuffer[i], HEX);
  }
  Serial.println("");

  /*
  for( int i =0; i<62; i++){
    Serial.print("Byte ");
    Serial.print(i);
    Serial.print(" : ");
    Serial.println(radioBuffer[i], HEX);
  }
  Serial.println("");
  */
}

float value2BuffTest(float value){
  //convertit un float sur 4 bytes
  int valueInt = int(value);
  
  bufferTest[0 + 3] = valueInt & 0xff;
  bufferTest[0 + 2] = (valueInt >> 8) & 0xff;
  bufferTest[0 + 1] = (valueInt >> 16) & 0xff;
  bufferTest[0] = (valueInt >> 24) & 0xff;
  
}

float buff2ValueTest(){

  int valueInt;
  byte byte1 = bufferTest[0]; // compter de gauche à droite
  byte byte2 = bufferTest[0 + 1];
  byte byte3 = bufferTest[0 + 2];
  byte byte4 = bufferTest[0 + 3];
  Serial.println(byte1, BIN);
  Serial.println(byte2, BIN);
  Serial.println(byte3, BIN);
  Serial.println(byte4, BIN);
  valueInt = (int)(byte1 << 24 | byte2 << 16 | byte3 << 8 | byte4);

  return (float(valueInt));
}


void testDecodage(){

  Serial.println(buff2Value(0));
  Serial.println(buff2Value(4));
  Serial.println(buff2Value(8));
  Serial.println(buff2Value(12, true));
  Serial.println(buff2Value(16));
  Serial.println(buff2Value(20, true));
  Serial.println(buff2Value(24));
  Serial.println(buff2Value(28));
  Serial.println(buff2Value(32));

  Serial.println(buff2Value(52));
  Serial.println(buff2Value(56, true));
}





void loop() {
  // put your main code here, to run repeatedly:

}
