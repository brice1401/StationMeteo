/* this program is used to send the first data to the OVH server
 *  token : uFsX3b2hqmqpTYp3xbolE9eaDkpsmDwIlijtxyBsqS7qdtRZK3XBX9UaYH9GajwiSXq0JiPiXIBUvOmOcAJIOcr6OrW8zRbMx5bAGQsuwxtDBmVf61ITgJGXp.soBWzt
 *  opentsdb : https://opentsdb.gra1.metrics.ovh.net
 *  fingerprint : 84 8F 74 12 8E 0D 25 E3 8C C7 47 8F 75 EF 30 15 70 03 A1 07 62 EB 75 AB BE 09 9B FF B2 66 4E F8
 */

#include <SPI.h>
#include <Ethernet.h>

// Mac address
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

// if you don't want to use DNS (and reduce your sketch size)
// use the numeric IP instead of the name for the server:
//IPAddress server(74,125,232,128);  // numeric IP for Google (no DNS)
char server[] = "www.google.com";    // name address for Google (using DNS)

// Set the static IP address to use if the DHCP fails to assign
IPAddress ip(192, 168, 0, 177);
IPAddress myDns(192, 168, 0, 1);

// Initialize the Ethernet client library
// with the IP address and port of the server
// that you want to connect to (port 80 is default for HTTP):
EthernetClient client;

// Variables to measure the speed
unsigned long beginMicros, endMicros;
unsigned long byteCount = 0;
bool printWebData = true;  // set to false for better speed measurement


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
