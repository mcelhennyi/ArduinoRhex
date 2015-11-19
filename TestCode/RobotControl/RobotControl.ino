#include <Servo.h>
#include <SPI.h>

//Assign the Chip Select signal to pin 10.
int CS=10;
int button = 9;

//This is a list of some of the registers available on the ADXL345.
//To learn more about these and the rest of the registers on the ADXL345, read the datasheet!
char POWER_CTL = 0x2D;  //Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32; //X-Axis Data 0
char DATAX1 = 0x33; //X-Axis Data 1
char DATAY0 = 0x34; //Y-Axis Data 0
char DATAY1 = 0x35; //Y-Axis Data 1
char DATAZ0 = 0x36; //Z-Axis Data 0
char DATAZ1 = 0x37; //Z-Axis Data 1

//This buffer will hold values read from the ADXL345 registers.
byte values[10];
//These variables will be used to hold the x,y and z axis accelerometer values.
int x,y,z;
int baseX;
int baseY;
int baseZ;
float angle;
float anglePerAccel = 0.6946564885;

// Constant Definitions
#define LEDPIN 8 // Pin on Arduino board to which status LED is connected
#define READ_RATE 100 // How often the serial link is read, in milliseconds
#define FLASH_RATE 100 // The on/off period in milliseconds, for the LED Flash status feedback

// Declarations
byte cmd;  // Stores the next byte of incoming data, which is a "command" to do something 
byte param; // Stores the 2nd byte, which is the command parameter

Servo leftMotor1;
Servo leftMotor2;

Servo rightMotor1; 
Servo rightMotor2; 

const int serialPeriod = 250;       // only print to the serial console every 1/4 second
unsigned long timeSerialDelay = 0;

const int loopPeriod = 20;          // a period of 20ms = a frequency of 50Hz
unsigned long timeLoopDelay   = 0;

//rig & echo pins used for the ultrasonic sensors
const int ultrasonic2TrigPin = 8;
const int ultrasonic2EchoPin = 9;

int ultrasonic2Distance;
int ultrasonic2Duration;

// define the state
#define DRIVE_FORWARD   0
#define TURN_LEFT       1

int state = DRIVE_FORWARD; // 0 = drive forward (DEFAULT), 1 = turn left

void stateMachine()
{
    if(state == DRIVE_FORWARD) 
    {
        if(ultrasonic2Distance > 6 || ultrasonic2Distance < 0) //Nothing in front 
        {
          leftMotor1.write(100);
          leftMotor2.write(100); 
          rightMotor1.write(100); 
          rightMotor2.write(100); 
        }
        else // there's an object in front of us
        {
            state = TURN_LEFT;
        }
    }
    else if(state == TURN_LEFT) // obstacle detected -- turn left
    {
        unsigned long timeToTurnLeft = 1100; // it takes around 1.1 seconds to turn 90 degrees
        
        unsigned long turnStartTime = millis(); // save the time that we started turning

        while((millis()-turnStartTime) < timeToTurnLeft) // stay in this loop until timeToTurnLefthas elapsed
        {
          leftMotor1.write(100);
          leftMotor2.write(100); 
          rightMotor1.write(150); 
          rightMotor2.write(150); 

        }
        
        state = DRIVE_FORWARD;
    }
}


void readUltrasonicSensors()
{
    digitalWrite(ultrasonic2TrigPin, HIGH);
    delayMicroseconds(10);                  
    digitalWrite(ultrasonic2TrigPin, LOW);
 
    ultrasonic2Duration = pulseIn(ultrasonic2EchoPin, HIGH);
    ultrasonic2Distance = (ultrasonic2Duration/2)/29;
}


void debugOutput()
{
    if((millis() - timeSerialDelay) > serialPeriod)
    {
        Serial.print("ultrasonic2Distance: ");
        Serial.print(ultrasonic2Distance);
        Serial.print("cm");
        Serial.println();
        
        timeSerialDelay = millis();
    }
}

void selfDrive()
{
      if(millis() - timeLoopDelay >= loopPeriod)
    {
        readUltrasonicSensors(); // read and store the measured distances
        
        stateMachine(); 
        
        timeLoopDelay = millis();
    }
}
void setBase()
{
  baseX = x;
  baseY = y;
  baseZ = z; 
  
  Serial.println(getAngle(baseY));
  
  Serial.print(baseX, DEC);
  Serial.print(',');
  Serial.print(baseY, DEC);
  Serial.print(',');
  Serial.println(baseZ, DEC); 
}

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
void writeRegister(char registerAddress, byte value){
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
void readRegister(char registerAddress, int numBytes, byte * values){
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


void setup()
{
    Serial.begin(9600);
  
    // ultrasonic sensor pin config
    pinMode(ultrasonic2TrigPin, OUTPUT);
    pinMode(ultrasonic2EchoPin, INPUT);
    
    leftMotor1.attach(3);
    leftMotor2.attach(12); 
    rightMotor1.attach(5);
    rightMotor2.attach(10); 


    //Initiate an SPI communication instance.
    SPI.begin();
    //Configure the SPI connection for the ADXL345.
    SPI.setDataMode(SPI_MODE3);
    //Create a serial connection to display the data on the terminal.
    Serial.begin(9600);
    
    //Set up the Chip Select pin to be an output from the Arduino.
    pinMode(CS, OUTPUT);
    pinMode(button, INPUT);
    //Before communication starts, the Chip Select pin needs to be set high.
    digitalWrite(CS, HIGH);
    
    //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
    writeRegister(DATA_FORMAT, 0x01);
    //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
    writeRegister(POWER_CTL, 0x08);  //Measurement mode  
  
 
    
}


void loop()
{
      //Reading 6 bytes of data starting at register DATAX0 will retrieve the x,y and z acceleration values from the ADXL345.
      //The results of the read operation will get stored to the values[] buffer.
      readRegister(DATAX0, 6, values);
    
      //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
      //The X value is stored in values[0] and values[1].
      x = ((int)values[1]<<8)|(int)values[0];
      //The Y value is stored in values[2] and values[3].
      y = ((int)values[3]<<8)|(int)values[2];
      //The Z value is stored in values[4] and values[5].
      z = ((int)values[5]<<8)|(int)values[4];
      
      //Print the results to the terminal.
      //  Serial.print(x, DEC);
      //  Serial.print(',');
      //  Serial.print(y, DEC);
      //  Serial.print(',');
      //  Serial.println(z, DEC); 
      if(digitalRead(button) == HIGH)
      setBase();  
      delay(100); 
      
      debugOutput(); // prints debugging messages to console 
   
      switch ( cmd ) {
      case 1:
      // First byte contains a generic "command" byte. We arbitrarily defined '1' as the command to then check the 2nd parameter byte
      // User can additional commands by adding case 2, 3, 4, etc
      {
         // read the parameter byte
         param = Serial.read();
         
         switch (param)
         {
           case 1: //go forward
          leftMotor1.write(100);
          leftMotor2.write(100); 
          rightMotor1.write(100); 
          rightMotor2.write(100); 
           break;
          
           case 2: //go backward
           //code to move bot backwards
           break;
          
           case 3:  //go left
          leftMotor1.write(100);
          leftMotor2.write(100); 
          rightMotor1.write(150); 
          rightMotor2.write(150);
          break;
           
           case 4: //go right
          leftMotor1.write(150);
          leftMotor2.write(150); 
          rightMotor1.write(100); 
          rightMotor2.write(100); 
          break;
           case 5: //stop
          leftMotor1.write(0);
          leftMotor2.write(0); 
          rightMotor1.write(0); 
          rightMotor2.write(0);
           break;
           case 6: //execute routine1
           selfDrive(); 
           break;
             
           default: break; // do nothing
         } // switch (param)
      } // switch (cmd) case 1
      default: break; // do nothing
  } // switch (cmd)
  
  delay(READ_RATE);                    // wait 100ms for next reading
}





