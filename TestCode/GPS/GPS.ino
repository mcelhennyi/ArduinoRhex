#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);

String NMEA1;
String NMEA2;
char c;
String time;
String lattitude;
String longitude;
String knots;
String date;
String altitude;
String fixQualAV;
String fixQual01;
//mc is 12 commas, ga is 14 commas
int indexmc[11]; 
int indexga[13];

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
  parseString(NMEA1);
//  parseString(NMEA2);
  Serial.println(time);
//  Serial.println(fixQualAV);
  Serial.println(lattitude);
  Serial.println(longitude);
  Serial.println("Knts");
  Serial.println(knots);
//  Serial.println(date);
  Serial.println(fixQual01);
  Serial.println(altitude);
}

//This will take A NMEA string and return values of concern:
  //Lat, Long, Speed in knots, Altitude, time UTC
void parseString(String NMEA)
{
  indexCommas(NMEA1);
  indexCommas(NMEA2);

  if(NMEA.substring(5,7).equalsIgnoreCase("GA"))//Contains 
    {  
      time = NMEA.substring(indexga[0] + 1, indexga[1]); //time in UTC--Time can be faulty if gotten from MC not GA
      fixQual01 = NMEA.substring(indexga[5] + 1, indexga[6]);//fix quality 0 bad 1 is good
      altitude = NMEA.substring(indexga[8] + 1, indexga[9]);//Altitude from mean sealevel
    }
  else if(NMEA.substring(5,7).equalsIgnoreCase("MC"))//12 commas
    {
//      fixQualAV = NMEA.substring(indexmc[1] + 1, indexmc[2]);//Active or void fix
      lattitude = NMEA.substring(indexmc[2] + 1, indexmc[3]) + NMEA.substring(indexmc[3] + 1, indexmc[4]);//Lattitude with hemisphere
      longitude = NMEA.substring(indexmc[4] + 1, indexmc[5]) + NMEA.substring(indexmc[5] + 1, indexmc[6]);//logitude with hemisphere
      knots = NMEA.substring(indexmc[6] + 1, indexmc[7]); //speed in knots
//      date = NMEA.substring(indexmc[8] + 1, indexmc[9]); //date DD/MM/YY
    }
}

//Goes through a NMEA string and finds the index of all the commas
void indexCommas(String NMEA)
{
  if(NMEA.substring(5,7).equalsIgnoreCase("GA")) //14 commas
    {  
      indexga[0] = NMEA.indexOf(",");
      for( int x = 1; x <= 13; x++)
        {
          indexga[x] = NMEA.indexOf(",", indexga[x-1] + 1);
        }
    }
  else if(NMEA.substring(5,7).equalsIgnoreCase("MC"))//12 commas
    {
      indexmc[0] = NMEA.indexOf(",");
      for( int x = 1; x <= 11; x++)
        {
          indexmc[x] = NMEA.indexOf(",", indexmc[x-1] + 1);
        }
    }
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
