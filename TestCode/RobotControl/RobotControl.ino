#include <Servo.h>

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

void setup()
{
    Serial.begin(9600);
  
    // ultrasonic sensor pin config
    pinMode(ultrasonic2TrigPin, OUTPUT);
    pinMode(ultrasonic2EchoPin, INPUT);
    
    leftMotor1.attach(13);
    leftMotor2.attach(12); 
    rightMotor1.attach(11);
    rightMotor2.attach(10); 
}


void loop()
{
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
           //code to cut servos, no movement
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





