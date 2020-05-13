#include <VarSpeedServo.h>

/*
This program controls a robot arm like that discussed at the link below.
http://projectsfromtech.blogspot.com/2013/09/simple-arduino-robot-arm-from-9-gram.html

Position is calculated using simple inverse kinematics based on high school trig

Setup:
* Change arm constants to match physical setup (distance between pivot points in cm)
* Change servo correction factors to match physical setup
* Change origin correction factors to match desired origin

Use:
* Enter (x,y) coordinates in FixCoordinate(x,y). Distances are in cm and are measured from origin
* Call CalculateServoAngles()
* Call MoveArm()

GAITPOS      X   X   X   X   X   X   X   X   X   X   X   X
             |   |   |   |   |   |   |   |   |   |   |   |     
FL           1   2   3---4---5---6---7---8---9---10--11--12
RR           10--11--12  1   2   3---4---5---6---7---8---9
FR           7---8---9---10--11--12  1   2   3---4---5---6
RL           4---5---6---7---8---9---10--11--12  1   2   3
___________________________________________________________ 
LEGS-ON-GND    3   3   3   3   3   3   3   3   3   3   3
 

*/
//Default standing position
float defaultPosX = 10;
float defaultPosY = 2;

//MAKE THE SERVO INSTANCES
VarSpeedServo ServoS_0;       // Coxa left rear
VarSpeedServo ServoS_1;      // Tibia left rear
VarSpeedServo ServoS_2;      // Femur left rear

VarSpeedServo ServoS_3;       // Coxa right rear
VarSpeedServo ServoS_4;      // Tibia right rear
VarSpeedServo ServoS_5;      // Femur right rear

VarSpeedServo ServoS_6;       // Coxa right front
VarSpeedServo ServoS_7;      // Tibia right front
VarSpeedServo ServoS_8;      // Femur right front

VarSpeedServo ServoS_9;       // Coxa left front
VarSpeedServo ServoS_10;      // Tibia left front
VarSpeedServo ServoS_11;      // Femur left front

int i=1;
int gait="S";
int inByte;

// Servo Angles
float ServoS_1_Angle = 90;
float ServoS_2_Angle = 90;

// Define arm Constants
const float a = 13.0;      // lower joint length (cm)
const float b = 11.0;      // upper joint length (cm)

// Correction factors to align servo values with their respective axis
const float S_1_CorrectionFactor = 7;     // Align arm "a" with the horizontal when at 0 degrees
const float S_2_CorrectionFactor = -70;     // Align arm "b" with arm "a" when at 0 degrees

// Correction factor to shift origin out to edge of the mount
const float X_CorrectionFactor = 6.5;       // X direction correction factor (cm)
const float Y_CorrectionFactor = -6;       // Y direction correction factor (cm)

// Angle Variables
float A;            //Angle oppposite side a (between b and c)
float B;            //Angle oppposite side b
float C;            //Angle oppposite side c
float theta;        //Angle formed between line from origin to (x,y) and the horizontal

// Distance variables
float x;            // x position (cm)
float y;            // y position (cm)
float c;            // Hypotenuse legngth in cm
const float pi = M_PI;  //Store pi in a less annoying format

//GAIT DATA
float FLArrayX[12]={5,10,10,10,10,10,10,10,10,10,10,5};       // CUT OFF THE LAST 3 NUMBERS AND MOVE THEM TO THE FRONT OF THE ARRAY FOR THE NEXT LEG (X)
float FLArrayY[12]={2,2,3.5,5,6.5,8,9.5,11,12.5,14,16,16};    // CUT OFF THE LAST 3 NUMBERS AND MOVE THEM TO THE FRONT OF THE ARRAY FOR THE NEXT LEG (Y)    

float RRArrayX[12]={10,10,5,5,10,10,10,10,10,10,10,10};
float RRArrayY[12]={14,16,16,2,2,3.5,5,6.5,8,9.5,11,12.5};

float RLArrayX[12]={10,10,10,10,10,5,5,10,10,10,10,10};
float RLArrayY[12]={9.5,11,12.5,14,16,16,2,2,3.5,5,6.5,8};

float FRArrayX[12]={10,10,10,10,10,10,10,10,5,5,10,10};
float FRArrayY[12]={5,6.5,8,9.5,11,12.5,14,16,16,2,2,3.5};

//===================================================================================

void setup()
{
  ServoS_0.attach(26);//coxa
  delay(500);
  ServoS_1.attach(28);//femur      // Attach RL servos
  delay(500);
  ServoS_2.attach(30);//tibia
  delay(500);
  ServoS_3.attach(32);
  delay(500);
  ServoS_4.attach(34);             // Attach RR servos
  delay(500);
  ServoS_5.attach(36);
  delay(500);
  ServoS_6.attach(38);
  delay(500);
  ServoS_7.attach(40);             // Attach FR servos
  delay(500);
  ServoS_8.attach(42);
  delay(500);
  ServoS_9.attach(44);
  delay(500);
  ServoS_10.attach(46);             // Attach FL servos
  delay(500);
  ServoS_11.attach(48);

  //Set paws to default pos
  FixCoordinates(defaultPosX,defaultPosY);   
  CalculateServoAngles();          
  FLMoveArm();

  FixCoordinates(defaultPosX,defaultPosY);   
  CalculateServoAngles();          
  RRMoveArm();

  FixCoordinates(defaultPosX,defaultPosY);   
  CalculateServoAngles();          
  RLMoveArm();

  FixCoordinates(defaultPosX,defaultPosY);   
  CalculateServoAngles();          
  FRMoveArm();
  delay(500);
  
  Serial.begin(115200);             // -For debugging
}
//--------------------------------------------------------------


void loop()
{  
  if (Serial.available() > 0) {
    inByte = Serial.read();
    Serial.print(inByte);
    switch (inByte) {
      case 'F':
        gait = "F";   //Forward
        break;
      case 'B':
        gait = "B";   //Backwards
        break;
      case 'L':
        gait = "L";   //Turn Left
        break;
      case 'R':
        gait = "R";   //Turn Right
        break;
      case 'U':
        gait = "U";   //Up
        break;
      case 'D':
        gait = "D";   //Down
        break;
      case 'S':
        gait="S";
        //Set paws to default pos (STOP)
        FixCoordinates(defaultPosX,defaultPosY);   
        CalculateServoAngles();          
        FLMoveArm();
      
        FixCoordinates(defaultPosX,defaultPosY);   
        CalculateServoAngles();          
        RRMoveArm();
      
        FixCoordinates(defaultPosX,defaultPosY);   
        CalculateServoAngles();          
        RLMoveArm();
      
        FixCoordinates(defaultPosX,defaultPosY);   
        CalculateServoAngles();          
        FRMoveArm();
        delay(500);
        break;
      case 'p':
        ServoS_0.detach();    // Power servos down (test)
        ServoS_1.detach();             // Detach FL servos
        ServoS_2.detach();
      
        ServoS_3.detach();
        ServoS_4.detach();             // Detach RL servos
        ServoS_5.detach();
      
        ServoS_6.detach();
        ServoS_7.detach();             // Detach servos
        ServoS_8.detach();
      
        ServoS_9.detach();
        ServoS_10.detach();             // Detach servos
        ServoS_11.detach();   
        break;
      case 'P':
        ServoS_0.attach(26);//coxa
        delay(500);
        ServoS_1.attach(28);//femur             // Attach RL servos
        delay(500);
        ServoS_2.attach(30);//tibia
        delay(500);
        ServoS_3.attach(32);
        delay(500);
        ServoS_4.attach(34);             // Attach RR servos
        delay(500);
        ServoS_5.attach(36);
        delay(500);
        ServoS_6.attach(38);
        delay(500);
        ServoS_7.attach(40);             // Attach FR servos
        delay(500);
        ServoS_8.attach(42);
        delay(500);
        ServoS_9.attach(44);
        delay(500);
        ServoS_10.attach(46);             // Attach FL servos
        delay(500);
        ServoS_11.attach(48);
        break;
      }
  }
  while(gait=="F"){
    for(int i=0;i<11;i++){
      FixCoordinates(FLArrayX[i], FLArrayY[i]);   
      CalculateServoAngles();          
      FLMoveArm();

      FixCoordinates(RRArrayX[i], RRArrayY[i]);   
      CalculateServoAngles();          
      RRMoveArm();

      FixCoordinates(RLArrayX[i], RLArrayY[i]);   
      CalculateServoAngles();          
      RLMoveArm();

      FixCoordinates(FRArrayX[i], FRArrayY[i]);   
      CalculateServoAngles();          
      FRMoveArm();
      delay(100);
    }
    if(Serial.available()) {
      break;
    }
  }
  if(gait=="S"){
    
  }
}

//====================================================================================

// 2D INVERSE KINEMATICS (UNUSABLE, WAS MEANT FOR ROBOT ARM)
// Get x and y measured from the bottom of the base. Function corrects for offset
void FixCoordinates(float x_input, float y_input)
{
 x = x_input + X_CorrectionFactor;
 y = y_input + Y_CorrectionFactor;
}

// Calculate necessary servo angles to move arm to desired points
void CalculateServoAngles()
{
  c = sqrt( sq(x) + sq(y) );                                            // pythagorean theorem
  B = (acos( (sq(b) - sq(a) - sq(c))/(-2*a*c) )) * (180/pi);            // Law of cosines: Angle opposite upper arm section
  C = (acos( (sq(c) - sq(b) - sq(a))/(-2*a*b) )) * (180/pi);            // Law of cosines: Angle opposite hypotenuse
  theta = (asin( y / c )) * (180/pi);                                   // Solve for theta to correct for lower joint's impact on upper joint's angle
  ServoS_1_Angle = B + theta + S_1_CorrectionFactor;                    // Find necessary angle. Add Correction
  ServoS_2_Angle = C + S_2_CorrectionFactor;                            // Find neceesary angle. Add Correction
}

// Update the servos
void RLMoveArm()
{
  ServoS_0.slowmove(90, 20);
  ServoS_1.slowmove(90 - ServoS_1_Angle, 160);              // Move joint to desired position
  ServoS_2.slowmove(90 - ServoS_2_Angle, 160);              // Move joint to desired position
}

void FRMoveArm()
{
  ServoS_6.slowmove(90, 20);
  ServoS_7.slowmove(90 + ServoS_1_Angle, 160);              // Move joint to desired position
  ServoS_8.slowmove(90 + ServoS_2_Angle, 160);              // Move joint to desired position
}

void RRMoveArm()
{
  ServoS_3.slowmove(90, 20);
  ServoS_4.slowmove(90 + ServoS_1_Angle, 160);              // Move joint to desired position
  ServoS_5.slowmove(90 + ServoS_2_Angle, 160);              // Move joint to desired position
}

void FLMoveArm()
{
  ServoS_9.slowmove(90, 20);
  ServoS_10.slowmove(90 - ServoS_1_Angle, 160);              // Move joint to desired position
  ServoS_11.slowmove(90 - ServoS_2_Angle, 160);              // Move joint to desired position
}
