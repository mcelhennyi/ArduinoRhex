#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);

String NMEA1;
String NMEA2;
char c;
String lattitude;
String longitude;
String knots;
String altitude;
String fixQualAV;
String fixQual01;
String satLock;

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  GPS.begin(9600);
  
  GPS.sendCommand("$PGCMD, 33, 0*6D");//Turn off antenna update data
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); //Set uprate to 1 hz
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //request RMC and GGA sentences only
  delay(1000);
  
}

void loop() 
{
  // put your main code here, to run repeatedly:
  readGPS();
  
//  delay(2000);
  
}

void parseGPS()
{
  
  while(
    if(string.indexOf(val) != null)
      {
        index = string.indexOf(",")
      }
    
  while(
}

void readGPS()
{
  clearOldData();
  clearOldData();

  while(!GPS.newNMEAreceived())
  {
    c=GPS.read();
  } 
  GPS.parse(GPS.lastNMEA());
  NMEA1 = GPS.lastNMEA();

  while(!GPS.newNMEAreceived())
  {
    c=GPS.read();
  } 
  GPS.parse(GPS.lastNMEA());
  NMEA2 = GPS.lastNMEA();  
  
  Serial.println(NMEA1);
  Serial.println(NMEA2);
  
}

void clearOldData()
{
  while(!GPS.newNMEAreceived())
  {
    c=GPS.read();
  } 
  GPS.parse(GPS.lastNMEA());

  while(!GPS.newNMEAreceived())
  {
    c=GPS.read();
  } 
  GPS.parse(GPS.lastNMEA());
}
