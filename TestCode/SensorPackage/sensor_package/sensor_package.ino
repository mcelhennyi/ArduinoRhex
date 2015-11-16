//Add the SPI library so we can communicate with the ADXL345 sensor
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
//////////////////////////////////////////
/////////////Accel////////////////////////
//////////////////////////////////////////
//Assign the Chip Select signal to pin 10.
int CS=10;
int button = 9;

//This is a list of some of the registers available on the ADXL345.
//To learn more about these and the rest of the registers on the ADXL345, read the datasheet!
char POWER_CTL = 0x2D;	//Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32;	//X-Axis Data 0
char DATAX1 = 0x33;	//X-Axis Data 1
char DATAY0 = 0x34;	//Y-Axis Data 0
char DATAY1 = 0x35;	//Y-Axis Data 1
char DATAZ0 = 0x36;	//Z-Axis Data 0
char DATAZ1 = 0x37;	//Z-Axis Data 1

//This buffer will hold values read from the ADXL345 registers.
byte values[10];
//These variables will be used to hold the x,y and z axis accelerometer values.
int x,y,z;
int baseX;
int baseY;
int baseZ;
float angle;
float anglePerAccel = 0.6946564885;
int count = 1;
int avgX, avgY, avgZ = 0;

//////////////////////////////////////////
/////////////GPS//////////////////////////
//////////////////////////////////////////
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
  Serial.begin(9600);  
  ////////
  //GPS///
  ////////
  GPS.begin(9600);
  GPS.sendCommand("$PGCMD, 33, 0*6D");//Turn off antenna update data
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); //Set uprate to 1 hz
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //request RMC and GGA sentences only
  
  /////////
  //Acell//
  /////////
  SPI.begin();  //Initiate an SPI communication instance.
  SPI.setDataMode(SPI_MODE3);  //Configure the SPI connection for the ADXL345.
  pinMode(CS, OUTPUT);  //Set up the Chip Select pin to be an output from the Arduino.
  digitalWrite(CS, HIGH);  //Before communication starts, the Chip Select pin needs to be set high.
  //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, 0x01);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);  //Measurement mode  
}

void loop()
{
  /////////
  //Accel//
  /////////
  //Reading 6 bytes of data starting at register DATAX0 will retrieve the x,y and z acceleration values from the ADXL345.
  //The results of the read operation will get stored to the values[] buffer.
  readRegister(DATAX0, 6, values);

  //The X value is stored in values[0] and values[1].
  x = ((int)values[1]<<8)|(int)values[0];
  //The Y value is stored in values[2] and values[3].
  y = ((int)values[3]<<8)|(int)values[2];
  //The Z value is stored in values[4] and values[5].
  z = ((int)values[5]<<8)|(int)values[4];
   
  if(count == 1)//makes the next if statement not have a half the first x bias for average
  {
   avgX = x;
   avgY = y;
   avgZ = z;
   count++; 
  }
  else if(count < 100)
  {
    avgX = (x + avgX)/2;
    avgY = (y + avgY)/2;
    avgZ = (z + avgZ)/2;
    count++;
  }
  else if(count == 100)//Should happen about every second
  {
//    Serial.println("publish");
    publishData(); 
    count = 1;
  }  
  

  delay(0); 
}
///////////////////
//General Methods//
///////////////////
//This method is called on and publishes current data
void publishData()
{
  ///////
  //GPS//
  ///////
  readGPS();
  parseString(NMEA1);
  parseString(NMEA2);

  ///////////
  //Publish//
  ///////////
  Serial.println( "$$$" + 
                  String(getAngle(avgX)) + ";" + 
                  String(getAngle(avgY)) + ";" +
                  String(getAngle(avgZ)) + ";" +
                  fixQualAV + ";" +
                  time + ";" +
                  date + ";" +
                  lattitude + ";" +
                  longitude + ";" +
                  altitude + ";" +
                  knots + "$$$" );
                  
}

/////////////////
//Accel Methods//
/////////////////
//This function gets an angle given a standard, stationary input from the accelerometer
float getAngle(int leg)
{
  angle = float(leg) * anglePerAccel;
  return angle;
}

//This function will write a value to a register on the ADXL345.
//Parameters:
//  char registerAddress - The register to write a value to
//  char value - The value to be written to the specified register.
void writeRegister(char registerAddress, byte value)
{
  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(CS, HIGH);
}

//This function will read a certain number of registers starting from a specified address and store their values in a buffer.
//Parameters:
//  char registerAddress - The register addresse to start the read sequence from.
//  int numBytes - The number of registers that should be read.
//  char * values - A pointer to a buffer where the results of the operation should be stored.
void readRegister(char registerAddress, int numBytes, byte * values)
{
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if(numBytes > 1)address = address | 0x40;
  
  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for(int i=0; i<numBytes; i++){
    values[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(CS, HIGH);
}

///////////////
//GPS methods//
///////////////
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
      fixQualAV = NMEA.substring(indexmc[1] + 1, indexmc[2]);//Active or void fix
      lattitude = NMEA.substring(indexmc[2] + 1, indexmc[3]) + NMEA.substring(indexmc[3] + 1, indexmc[4]);//Lattitude with hemisphere
      longitude = NMEA.substring(indexmc[4] + 1, indexmc[5]) + NMEA.substring(indexmc[5] + 1, indexmc[6]);//logitude with hemisphere
      knots = NMEA.substring(indexmc[6] + 1, indexmc[7]); //speed in knots
      date = NMEA.substring(indexmc[8] + 1, indexmc[9]); //date DD/MM/YY
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
//  clearOldData();

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
  
//  Serial.println(NMEA1);
//  Serial.println(NMEA2);
  
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
