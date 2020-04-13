#include <SlaveSPIClass.h>
#include <SPI.h>
#define MO 22
#define MI 23
#define MCLK 19
#define MS 18
#define SO 32
#define SI 25
#define SCLK 27
#define SS 34
SlaveSPI slave;
SPIClass master(VSPI);
void test();
void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
master.begin(MCLK,MI,MO);
slave.begin((gpio_num_t)SO,(gpio_num_t)SI,(gpio_num_t)SCLK,(gpio_num_t)SS,8,test);//seems to work with groups of 4 bytes
SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
pinMode(MS,OUTPUT);
}
String txt = "";
String cmd ="";
void loop() {
  // put your main code here, to run repeatedly:
if(slave.getBuff()->length()&&digitalRead(SS)==HIGH)
{
  while(slave.getBuff()->length())
    txt+=slave.read();
  Serial.println("slave input:");
  Serial.println(txt);
}
while(Serial.available())
{
 cmd +=(char) Serial.read();
}
while(txt.length()>0)
{
  slave.trans_queue(txt);
  Serial.println("slave output:");
  Serial.println(txt);
  txt ="";
}
while(cmd.length()>0)
{
  Serial.println("input:");
  Serial.println(cmd);
  Serial.println("Master input:");
  digitalWrite(MS,LOW);
  for(int i=0;i<cmd.length();i++)
  {
    cmd[i] = master.transfer(cmd[i]); //ERROR : gives the transmitted data <<1 
  }
  digitalWrite(MS,HIGH);
  for(int i=0;i<cmd.length();i++)
    Serial.print(cmd[i],HEX);
  cmd ="";
}
}
void test()
{
  //Serial.println("test");
  //Serial.println(slave[0]);
}



