// Constant Definitions
#define LEDPIN 8 // Pin on Arduino board to which status LED is connected
#define READ_RATE 100 // How often the serial link is read, in milliseconds
#define FLASH_RATE 100 // The on/off period in milliseconds, for the LED Flash status feedback

// Declarations
byte cmd;  // Stores the next byte of incoming data, which is a "command" to do something 
byte param; // Stores the 2nd byte, which is the command parameter



#include <Servo.h>

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


int autonomous = 0; 

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
        //drive forward 
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
            // turn left

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
void blinkLed(int nr){
    for (int i=0; i< nr; i++)
    {
      digitalWrite(LEDPIN, LOW); 
      delay(250); 
      digitalWrite(LEDPIN, HIGH);
      delay(250); 
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


    if(autonomous == 1){
    
    if(millis() - timeLoopDelay >= loopPeriod)
    {
        readUltrasonicSensors(); // read and store the measured distances
        
        stateMachine(); 
        
        timeLoopDelay = millis();
    }
    }

    else
    {

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
             //code to move bot forward
             blinkLed(1);
             break;
           case 2: //go backward
             //code to move bot backwards
             blinkLed(2);
             break;
           case 3:  //go left
             //code to turn bot left
             blinkLed(3);
             break;
           case 4: //go right
             //code to turn bot right
             blinkLed(4);
             break;
           case 5: //stop
              //code to cut servos, no movement
              blinkLed(5);
             break;
           case 6: //execute routine1
              //movement code
              blinkLed(6);
              //note: make this something that can loop, the server will end the routine when you lift your finger
             break;
           case 7: //execute routine2
              blinkLed(7);
              break;
           case 8: //execute routine3
              blinkLed(8);
              break;
           default: break; // do nothing
         } // switch (param)
      } // switch (cmd) case 1
      default: break; // do nothing
  } // switch (cmd)
  
  delay(READ_RATE);                    // wait 100ms for next reading
}
}




