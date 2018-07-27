/*
 * Arduino Sketch for Odometry exercise
 * By Aero Group 13 2016-17
 * 
 * Functions Forward(), rightTurn(), and housekeeping functions encoderReset(), encoder1(), encoder2() and stopMotor()
 * are based on sample MD25 code provided to team at blackboard.soton.ac.uk
 */

#include <Wire.h>                                             // Calls for I2C bus library
#include <Servo.h>                                            // Calls the Servo library

#define MD25ADDRESS         0x58                              // Address of MD25
#define SPEED1              0x00                              // Hex to send Motor 1 Speed (MODE 0, 1) or Both Motor Speed (MODE 2, 3)
#define SPEED2              0x01                              // Hex to send Motor 2 Speed (MODE 0, 1) or Turn Speed (MODE 2, 3) 
#define ENCODERONE          0x02                              // Hex to read motor encoder 1
#define ENCODERTWO          0x06                              // Hex to read motor encoder 2
#define ACCELERATION        0x0E                              // Hex to define motor acceleration
#define MODESELECT          0x0F                              // Hex to change between control modes
#define CMD                 0x10                              // Hex to reset encoder values

void botMove(float, float, char);                             // Function prototypes declared (for safe-mode config)
void botArc(int, int, float, float);
void arcMove(int, int, float, float);
void angledTurn(float, char);
void Forward();
void rightTurn();
void leftTurn();
void encoderReset();
float encoder1();
float encoder2();
void stopMotor();
void servoRun();
void wyptTrig();

int Mode = 2;                                                 // Mode in which the MD25 will operate
int DualSpeedValue = 0;                                       // Combined motor speed variable
float Wheel1mm = 0;                                           // Wheel 1 travel distance variable
float Wheel2mm = 0;                                           // Wheel 2 travel distance variable

Servo myservo;                                                // Servo object


void setup(){
  Wire.begin();                                               // Begin I2C bus
  Serial.begin(9600);                                         // Begin serial at 9600 baud (for diagnostics)
  delay(100);                                                 // Wait for everything to power up

  Wire.beginTransmission(MD25ADDRESS);                        // Set MD25 operation mode to Mode 2
  Wire.write(MODESELECT);
  Wire.write(Mode);
  Wire.endTransmission();

  encoderReset();                                             // Calls a function that resets the encoder values to 0 
  myservo.attach(9);                                          // Initialises servo object
  myservo.write(0);
  pinMode(13, OUTPUT);                                        // Initialises platform LED pin
  pinMode(11, OUTPUT);                                        // Initialises buzzer pin

//  Begin run

  botMove(310, 310, 'f');                                  //Calls a function to move the platform forward

  //Waypoint 1
  wyptTrig();                                              //Calls function which triggers outputs at the waypoint
  
  botMove(182, -182, 'r');                                 //Calls a function to move the platform right 90 degrees

  botMove(235, 235, 'f');                                  //Calls a function to move the platform forward

  //Waypoint 2 (M&M 1)
  wyptTrig();                                              //Calls function which triggers outputs at the waypoint
  servoRun();                                              //Calls function which runs the M&M servo

  botMove(-180, 180, 'l');                                 //Calls a function to move the platform left 90 degrees

  botMove(470, 470, 'f');                                  //Calls a function to move the platform forward

  //Waypoint 3
  wyptTrig();                                              //Calls function which triggers outputs at the waypoint

  botMove(-177, 177, 'l');                                 //Calls a function to move the platform left 90 degrees
  
  //arcSpin(23, 5);                               //Calls a function which moves the platform forward in an arc
  botArc(30, 10, 550, 224);
  
  //Waypoint 4 (M&M 2)
  wyptTrig();                                              //Calls function which triggers outputs at the waypoint
  servoRun();                                              //Calls function which runs the M&M servo

  botMove(180, -180, 'r');                                  //Calls a function to move the platform right 90 degrees
  
  botMove(595, 595, 'f');                                  //Calls a function to move the platform forward

  //Waypoint 5
  wyptTrig();                                              //Calls function which triggers outputs at the waypoint
  
  botMove(-178, 178, 'l');                                 //Calls a function to move the platform left 90 degrees

  botMove(360, 360, 'f');                                  //Calls a function to move the platform forward

  //Waypoint 6 (M&M 3)
  wyptTrig();                                              //Calls function which triggers outputs at the waypoint
  servoRun();                                              //Calls function which runs the M&M servo

  botMove(-180, 180, 'l');                                 //Calls a function to move the platform left 90 degrees

  botMove(370, 370, 'f');                                  //Calls a function to move the platform forward

  //Waypoint 7
  wyptTrig();                                              //Calls function which triggers outputs at the waypoint
  
  botMove(-184, 184, 'l');                                 //Calls a function to move the platform left 90 degrees

  botMove(350, 350, 'f');                                  //Calls a function to move the platform forward

  //Waypoint 8 (M&M 4)
  wyptTrig();                                              //Calls function which triggers outputs at the waypoint
  servoRun();                                              //Calls function which runs the M&M servo

  //angledTurn(40, 'l');                                     //Calls a function to move the platform at a specified angle in the specified direction
  botMove(-70, 70, 'l');
  
  botMove(600, 600, 'f');                              //Calls a function to move the platform forward

  //Waypoint 9
  wyptTrig();                                              //Calls function which triggers outputs at the waypoint

  //angledTurn(140, 'l');                                     //Calls a function to move the platform at a specified angle in the specified direction
  botMove(-290, 290, 'l');
  
  botMove(170, 170, 'f');                                  //Calls a function to move the platform forward

  //Waypoint 10 (M&M 5)
  wyptTrig();                                              //Calls function which triggers outputs at the waypoint
  servoRun();                                              //Calls function which runs the M&M servo

  botMove(188, -188, 'r');                                  //Calls a function to move the platform right 90 degrees
  
  botArc(62, 10, 1310, 236);                              //Calls a function which moves the platform forward in an arc

  //Waypoint 11
  wyptTrig();                                              //Calls function which triggers outputs at the waypoint
  
  //angledTurn(143, 'r');                                     //Calls a function to move the platform at a specified angle in the specified direction
  botMove(-290, 290, 'l');
  
  botMove(363, 363, 'f');                                  //Calls a function to move the platform forward

  //Waypoint 12
  wyptTrig();                                              //Calls function which triggers outputs at the waypoint
  
  botMove(380, 380, 'f');                                  //Calls a function to move the platform forward

  //Waypoint 13 (and end of run)
  wyptTrig();                                              //Calls function which triggers outputs at the waypoint
}


void loop(){                                              // Dummy loop function as no repeating code is desired
}


void botMove(float Wheel1, float Wheel2, char dir){      //This function moves the platform by the specified amount in the specified direction

  Wheel1mm = Wheel1;                                          // Sets wheel 1 travel distance value - mm
  Wheel2mm = Wheel2;                                          // Sets wheel 2 travel distance value - mm

  switch (dir){                                               //Selects function based on required movement
    case 'f':
      DualSpeedValue = 150;
      Forward();
      break;
    case 'l':
      DualSpeedValue = 106;
      leftTurn();
      break;
    case 'r':
      DualSpeedValue = 150;
      rightTurn();
      break;
  }
  
  stopMotor();                                                // Calls a function that stops the platform
//  delay(100);
//  switch (dir){                                               //Selects function based on required movement
//    case 'f':
//      DualSpeedValue = 106;
//      Backward('f');
//      break;
//    case 'l':
//      DualSpeedValue = 150;
//      Backward('l');
//      break;
//    case 'r':
//      DualSpeedValue = 106;
//      Backward('r');
//      break;
//  }
//  stopMotor();
  encoderReset();                                             // Calls a function that resets the encoder values to 0 
  delay(100);

}


void botArc(int Speed1, int Speed2, float Wheel1, float Wheel2) {      //This function moves the platform forward in a curved path in the specified direction
  Wire.beginTransmission(MD25ADDRESS);                        // Set MD25 operation mode to Mode 1 for individual wheel operations
  Wire.write(MODESELECT);
  Wire.write(1);
  Wire.endTransmission();

  arcMove(Speed1, Speed2, Wheel1, Wheel2);                    // Calls the low level function which sends the I2C commands
     
  Wire.beginTransmission(MD25ADDRESS);                        // Set MD25 operation back to Mode 2
  Wire.write(MODESELECT);
  Wire.write(2);
  Wire.endTransmission();
  
  stopMotor();                                                // Calls a function that stops the platform 
  encoderReset();                                             // Calls a function that resets the encoder values to 0 
  delay(100);
}


void arcMove(int Speed1, int Speed2, float Wheel1, float Wheel2){      // This function does the low-level I2C commands for moving the platform in an arc
  
  encoder1();                                                  // Calls a function that reads value of encoder 1
  encoder2();                                                  // Calls a function that reads value of encoder 2

  if (encoder1() <= Wheel1 && encoder2() <= Wheel2){           // If statement to check the status of the travelled distance
    Wire.beginTransmission(MD25ADDRESS);                       // Sets the acceleration to register 3 (0.675s)
    Wire.write(ACCELERATION);
    Wire.write(3);
    Wire.endTransmission();
  
    Wire.beginTransmission(MD25ADDRESS);                       // Sets motor 1 speed value
    Wire.write(SPEED1);
    Wire.write(Speed1);
    Wire.endTransmission();
  
    Wire.beginTransmission(MD25ADDRESS);                       // Sets motor 2 speed value
    Wire.write(SPEED2);
    Wire.write(Speed2);
    Wire.endTransmission();

    arcMove(Speed1, Speed2, Wheel1, Wheel2);                  // Calls recursively until the distance target is met
  }
  
}

//
//void angledTurn(float deg, char dir) {                      // This function moves the platform in a precise turn in the specified angle
//  Wheel1mm, Wheel2mm = (deg * 3.1415 / 180.0) * 130.0;          // Calculates the wheel distance and assigns it to the variables
//
//  switch (dir){                                             //Selects direction based on dir parameter and calls the appropriate function
//    case 'l':
//      DualSpeedValue = 106;
//      leftTurn();
//      break;
//    case 'r':
//      DualSpeedValue = 150;
//      rightTurn();
//      break;
//  }
//  stopMotor();                                                // Calls a function that stops the platform 
//  encoderReset();                                             // Calls a function that resets the encoder values to 0 
//  delay(100);
//}


void Forward(){                                               // This function moves the platform forward by a previously set distance
  
  encoder1();                                                 // Calls a function that reads value of encoder 1
  encoder2();                                                 // Calls a function that reads value of encoder 2

  if (encoder1() <= Wheel1mm && encoder2() <= Wheel2mm){      // If statement to check the status of the travelled distance
    Wire.beginTransmission(MD25ADDRESS);                      // Sets the acceleration to register 5 (1.275s)
    Wire.write(ACCELERATION);
    Wire.write(1);
    Wire.endTransmission();

    
    Wire.beginTransmission(MD25ADDRESS);                      // Sets a combined motor speed value
    Wire.write(SPEED1);
    Wire.write(DualSpeedValue);
    Wire.endTransmission();
      
    Forward();                                                // Calls recursively until encoders match required distance
  }
//    if (encoder1() >= Wheel1mm / 4.0 && encoder2() >= Wheel2mm / 4.0) {
//      Wire.beginTransmission(MD25ADDRESS);                      // Sets a combined motor speed value
//      Wire.write(SPEED1);
//      Wire.write(DualSpeedValue / 2);
//      Wire.endTransmission();
//      Forward();
////    } else if (encoder1() >= Wheel1mm / 4.0 && encoder2() >= Wheel2mm / 4.0) {
////      Wire.write(SPEED1);
////      Wire.write(DualSpeedValue / 4);
//    } else {
//
//      Wire.beginTransmission(MD25ADDRESS);                      // Sets a combined motor speed value
//      Wire.write(SPEED1);
//      Wire.write(DualSpeedValue);
//      Wire.endTransmission();
//      Forward();
//    }
////    Forward();                                                // Calls recursively until encoders match required distance
//  }
  
}


void rightTurn(){                                           // This function turns the platform right
  encoder1();                                                // Calls a function that reads value of encoder 1
  encoder2();                                                // Calls a function that reads value of encoder 2

  if (encoder1() <= Wheel1mm && encoder2() >= Wheel2mm){     // If statement to check the status of the travelled distance
    
    Wire.beginTransmission(MD25ADDRESS);                     // Sets the acceleration to register 5 (1.275s)
    Wire.write(ACCELERATION);
    Wire.write(1);
    Wire.endTransmission();

    Wire.beginTransmission(MD25ADDRESS);                     // Sets a combined motor speed value
    Wire.write(SPEED2);
    Wire.write(DualSpeedValue);
    Wire.endTransmission();
    
    rightTurn();                                            // Calls recursively until encoders match required distance
  }
}


void leftTurn(){                                            // This function turns the platform left
  encoder1();                                                // Calls a function that reads value of encoder 1
  encoder2();                                                // Calls a function that reads value of encoder 2

  if (encoder1() >= Wheel1mm && encoder2() <= Wheel2mm){     // If statement to check the status of the travelled distance
    
    Wire.beginTransmission(MD25ADDRESS);                     // Sets the acceleration to register 5 (1.275s)
    Wire.write(ACCELERATION);
    Wire.write(1);
    Wire.endTransmission();

    Wire.beginTransmission(MD25ADDRESS);                     // Sets a combined motor speed value
    Wire.write(SPEED2);
    Wire.write(DualSpeedValue);
    Wire.endTransmission();
    
    leftTurn();                                             // Calls recursively until encoders match required distance
  }
}


//void Backward(char dir){
//  
//  encoder1();                                                 // Calls a function that reads value of encoder 1
//  encoder2();                                                 // Calls a function that reads value of encoder 2
//
//  switch (dir) {
//    case 'f':
//      if (encoder1() >= Wheel1mm && encoder2() >= Wheel2mm){      // If statement to check the status of the travelled distance
//        
//        Wire.beginTransmission(MD25ADDRESS);                      // Sets the acceleration to register 5 (1.275s)
//        Wire.write(ACCELERATION);
//        Wire.write(1);
//        Wire.endTransmission();
//    
//        Wire.beginTransmission(MD25ADDRESS);                      // Sets a combined motor speed value
//        Wire.write(SPEED1);
//        Wire.write(DualSpeedValue);
//        Wire.endTransmission();
//        
//        Backward('f');                                                // Calls recursively until encoders match required distance
//      }
//      break;
//    case 'l':
//      if (encoder1() <= Wheel1mm && encoder2() >= Wheel2mm){      // If statement to check the status of the travelled distance
//        
//        Wire.beginTransmission(MD25ADDRESS);                      // Sets the acceleration to register 5 (1.275s)
//        Wire.write(ACCELERATION);
//        Wire.write(1);
//        Wire.endTransmission();
//    
//        Wire.beginTransmission(MD25ADDRESS);                      // Sets a combined motor speed value
//        Wire.write(SPEED1);
//        Wire.write(DualSpeedValue);
//        Wire.endTransmission();
//        
//        Backward('l');                                                // Calls recursively until encoders match required distance
//      }
//      break;
//    case 'r':
//      if (encoder1() >= Wheel1mm && encoder2() <= Wheel2mm){      // If statement to check the status of the travelled distance
//        
//        Wire.beginTransmission(MD25ADDRESS);                      // Sets the acceleration to register 5 (1.275s)
//        Wire.write(ACCELERATION);
//        Wire.write(1);
//        Wire.endTransmission();
//    
//        Wire.beginTransmission(MD25ADDRESS);                      // Sets a combined motor speed value
//        Wire.write(SPEED1);
//        Wire.write(DualSpeedValue);
//        Wire.endTransmission();
//        
//        Backward('r');                                                // Calls recursively until encoders match required distance
//      }
//      break;
//  }
//  
//}


void encoderReset(){                                         // This function resets the encoder values to 0
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(CMD);
  Wire.write(0x20);                                         
  Wire.endTransmission(); 
  delay(50);
}


float encoder1(){                                           // Function to read and display value of encoder 1 as a long
  Wire.beginTransmission(MD25ADDRESS);                      // Send hex to get a reading from encoder 1
  Wire.write(ENCODERONE);
  Wire.endTransmission();

  Wire.requestFrom(MD25ADDRESS, 4);                         // Request 4 bytes from MD25
  while(Wire.available() < 4);                              // Wait for 4 bytes to arrive
  long poss1 = Wire.read();                                 // First byte for encoder 1, HH
  poss1 <<= 8;
  poss1 += Wire.read();                                     // Second byte for encoder 1, HL
  poss1 <<= 8;
  poss1 += Wire.read();                                     // Third byte for encoder 1, LH
  poss1 <<= 8;
  poss1  +=Wire.read();                                     // Fourth byte for encoder 1, LL
  delay(5);                                                 // Wait to make sure everything is sent
  return(poss1*0.8727);                                     // Convert encoder value to mm 9599 9162 8814 8901
}


float encoder2(){                                            // Function to read and display value of encoder 2 as a long
  Wire.beginTransmission(MD25ADDRESS);                       // Send hex to get a reading from encoder 2
  Wire.write(ENCODERTWO);
  Wire.endTransmission();

  Wire.requestFrom(MD25ADDRESS, 4);                         // Request 4 bytes from MD25
  while(Wire.available() < 4);                              // Wait for 4 bytes to arrive
  long poss2 = Wire.read();                                 // First byte for encoder 2, HH
  poss2 <<= 8;
  poss2 += Wire.read();                                     // Second byte for encoder 2, HL             
  poss2 <<= 8;
  poss2 += Wire.read();                                     // Third byte for encoder 2, LH             
  poss2 <<= 8;
  poss2  +=Wire.read();                                     // Fourth byte for encoder 2, LL
  delay(5);                                                 // Wait to make sure everything is sent
  return(poss2*0.8727);                                     // Convert encoder value to mm
}


void stopMotor(){                                           // Function to stop motors
  
  Wire.beginTransmission(MD25ADDRESS);                      // Sets the acceleration to register 10 (0.65s)
  Wire.write(ACCELERATION);
  Wire.write(10);
  Wire.endTransmission();

  Wire.beginTransmission(MD25ADDRESS);                      // Stops Motor 1 (Mode 0, 1) or Both Motors (Mode 2, 3)
  Wire.write(SPEED1);
  Wire.write(128);
  Wire.endTransmission();

  Wire.beginTransmission(MD25ADDRESS);                      // Stops Motor 2 (Mode 0, 1) or Both Motors While Turning (Mode 2, 3)
  Wire.write(SPEED2);
  Wire.write(128);
  Wire.endTransmission();
  
  delay(50);

  encoder1();                                                // Calls a function that reads value of encoder 1
  encoder2();                                                // Calls a function that reads value of encoder 2

  Serial.print("Encoder 1 Distance CM - ");                  // Displays last recorded travelled distance
  Serial.print(encoder1());
  Serial.print("   ");
  Serial.print("Encoder 2 Distance CM - ");
  Serial.print(encoder2());
  Serial.println(" ");
}


void servoRun() {                                            // This function runs the M&M servo
  for (int pos = 0; pos <= 67; pos += 1) {                   // Runs the servo arm from full retract to full extend position
    myservo.write(pos);
    delay(9);
  }
  for (int pos = 67; pos >= 0; pos -= 1) {                   // Runs the servo arm back from full extend to full retract position
    myservo.write(pos);
    delay(5);
  }  
}


void wyptTrig() {                                           // This function runs the events which trigger once the platform reaches a waypoint
  digitalWrite(13, HIGH);                                   // Turn the LED on and then off
  delay(250);
  digitalWrite(13, LOW);
  for (int i = 0; i < 300; i++){                                // Play the buzzer
    digitalWrite(11, HIGH);
    delayMicroseconds(100);
    digitalWrite(11, LOW);
    delayMicroseconds(100);
  }
  for (int i = 0; i < 600; i++){                                // Play the buzzer
    digitalWrite(11, HIGH);
    delayMicroseconds(50);
    digitalWrite(11, LOW);
    delayMicroseconds(50);  
  }
}

