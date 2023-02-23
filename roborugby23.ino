#include <LCD16x2.h> //allows us to use the little LCD at the top
#include <Wire.h>
#include <NineAxesMotion.h>
#include <math.h>

LCD16x2 lcd; // Setup for LCD

NineAxesMotion mySensor; // Setup for 9 axis sensor

int E1 = 5;     //Port for M1 Speed Control
int E2 = 6;     //Port for M2 Speed Control

int M1 = 4;    //Port for M1 Direction Control
int M2 = 7;    //Port for M2 Direction Control

int b;


void setup() {
  Wire.begin();
    
  pinMode(E1, OUTPUT);    //These pins control the motor H-bridge circuitry, so we need to set them up as outputs
  pinMode(E2, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(A0, INPUT_PULLUP);  //Sets up optical sensor with correct pullup resistor
  pinMode(A3, INPUT);  //Sets up the distance sensor
    
  lcd.lcdClear();
       
  lcd.lcdGoToXY(1,1);
  lcd.lcdWrite("Press white key");
  
  int b= lcd.readButtons();    //Sample the state of the large white buttons
    
  while(b == 15)    // A value of 15 will only be seen if no buttons are pressed
  {
      b = lcd.readButtons();    //Sample the state of the large white buttons
  }
    
  //Peripheral Initialization
  Serial.begin(9600);           //Initialize the Serial Port to view information on the Serial Monitor
  I2C.begin();                    //Initialize I2C communication to the let the library communicate with the sensor.
  Wire.begin();
  
  //Sensor Initialization
  mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   //NDOF = 9 Degrees of Freedom Sensor (Other operatin modes are available)
  mySensor.setUpdateMode(AUTO);  //The default is AUTO. Changing to MANUAL requires calling the relevant update functions prior to calling the read functions
  //Setting to MANUAL requires fewer reads to the sensor
}

void forward(int speed){    // Function for forward travel
  analogWrite (E1,speed);     //Set speed
  digitalWrite(M1,HIGH);    //Set direction 
 
  analogWrite (E2,speed);   
  digitalWrite(M2,HIGH) ;   
}
  
void reverse(int speed){     // Function for reverse travel
  analogWrite (E1,speed);    //Set speed
  digitalWrite(M1,LOW);     //Set direction 
 
  analogWrite (E2,speed);    
  digitalWrite(M2,LOW);     
}

void stop(){ // Stops the motors entirely by utilizing the forward function and setting it to zero
  forward(0);
}

void turnRight(int speed){
  analogWrite (E1,speed);    //Set speed
  digitalWrite(M1,HIGH);     //Set direction 
 
  analogWrite (E2,speed);    
  digitalWrite(M2,HIGH);   
}

void turnLeft(int speed){
  analogWrite (E1,speed);    //Set speed
  digitalWrite(M1,LOW);     //Set direction 
 
  analogWrite (E2,speed);  
  digitalWrite(M2,HIGH);     
}

int XPos, YPos,robLength,robWidth, tableLength = 2440,tableWidth = 1225; // X and Y co-ordinates measured from geometric centre of robot

int StartXPos = robLength/2.0; // Start X position is half the length of the robot (length positioned parallel to shorter table edge)
int StartYPos = (tableLength/2.0) - 20; // Robot positioned 20mm away from midway of table

int distChange;

void head2CoOrd(){
  int XMult, YMult;  // Define a multiplier for X and Y Co-ordinates to be used later
  int xcoord = distChange*cos(heading);
  int ycoord = distChange*sin(heading);
  deltax = xcoord*XMult;
  deltay = ycoord*YMult;
  
  }

void coordAdj(){
 XPos = StartXPos + deltax; //Ensures that the XPos variable always holds the correct starting value for XPos
  }
  
void loop() {


}
