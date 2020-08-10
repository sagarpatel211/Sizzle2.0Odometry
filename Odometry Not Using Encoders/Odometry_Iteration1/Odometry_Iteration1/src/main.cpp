/*----------------------------------------------------------------------------*/
/*    Module:       main.cpp                                                  */
/*    Author:       sagarpatel and saurinpatel                                */
/*    Created:      April 26, 2020                                            */
/*    Description:  Odometry For Precise Autonomous Motion                    */
/*    Credits:      5225A For Pilons POS Document and QUEENS for odom Alg.    */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RightFront           motor         18              
// RightBack            motor         17              
// LeftFront            motor         20              
// LeftBack             motor         14              
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <math.h>
#include <iostream> 
using namespace vex; 
vex::competition    Competition;

//Variables are defined here for use in the Odometry calculations
/*-------------------------------Variables-----------------------------------*/
#define Pi 3.14159265358979323846
#define SL 5.2
#define SR 5.2
#define SS 7.75
#define WheelDiameter 4.125
#define tpr 360
double Alpha, Theta, angledegrees, MainX, MainY, X, Y;
double angle = Theta;

//Creates variables for the integrated motor encoders
double CurrentRightFrontValue = RightFront.position(degrees);
double CurrentRightBackValue = RightBack.position(degrees);
double CurrentLeftFrontValue = LeftFront.position(degrees);
double CurrentLeftBackValue = LeftBack.position(degrees);

//Averages the encoder values for more accuracy
double CurrentRightValue = (CurrentRightFrontValue + CurrentRightBackValue) / 2;
double CurrentLeftValue = (CurrentLeftFrontValue + CurrentLeftBackValue) / 2;

//Sets the previous encoder values for use in Delta calculations
double PreviousRightValue;
double PreviousLeftValue;
//double PreviousBackValue = 0;

//Created these variables in case of a need for a reset function
double XAtReset = 0;
double YAtReset = 0;
double ThetaAtReset = 0;

//Creates Delta variables for use in calculation
double DeltaLeft;
double DeltaRight;
//double DeltaBack;

//Creates Delta inches variables for use in calculation
double DeltaLeftInches;
double DeltaRightInches;
//double DeltaBackInches;

//Creates change in X,Y variables for use in end of algorithm
double DeltaX;
double DeltaY;
//double DeltaX2;
//double DeltaY2;

//Made a displacement variable to use for trig calculations
double Displacement1;
//double Displacement2;
/*---------------------------------------------------------------------------*/
/*                            Odometry Functions                             */
/*---------------------------------------------------------------------------*/
void TrackPOS() {
  //Updates the encoder variables
  CurrentRightFrontValue = RightFront.position(degrees);
  CurrentRightBackValue = RightBack.position(degrees);
  CurrentLeftFrontValue = LeftFront.position(degrees);
  CurrentLeftBackValue = LeftBack.position(degrees);

  //Averages the encoder values for more accuracy
  double CurrentRightValue = (CurrentRightFrontValue + CurrentRightBackValue) / 2;
  double CurrentLeftValue = (CurrentLeftFrontValue + CurrentLeftBackValue) / 2;

  //Updates Delta variables using the current and previous values
  DeltaLeft = CurrentLeftValue - PreviousLeftValue;
  DeltaRight = CurrentRightValue - PreviousRightValue;
  //DeltaBack = CurrentBackValue - PreviousBackValue;

  //Converts the Delta values into inches
  DeltaLeftInches = DeltaLeft/tpr * WheelDiameter * Pi;
  DeltaRightInches = DeltaRight/tpr * WheelDiameter * Pi;
  //DeltaBackInches = DeltaBack/TicksPerRevolution * WheelDiameter * Pi;

  //Calculates the change in the angle
  Alpha = (DeltaLeftInches - DeltaRightInches) / (SL + SR);

  //Displacement calculation
  Displacement1 = 2 * (DeltaLeftInches / Alpha + SL) * sin (Alpha / 2);
  // Displacement2 = 2 * (DeltaBackInches / Alpha + SS) * sin(Alpha / 2);
  
  //Converts it to an X,Y calculation with trig
  DeltaX = Displacement1 * cos (Theta + (Alpha / 2));
  DeltaY = Displacement1 * sin (Theta + (Alpha / 2));
  // DeltaX2 = Displacement2 * cos(Theta + Alpha / 2);
  // DeltaY2 = Displacement2 * sin(Theta + Alpha / 2);

  //Updates robot's position
  Theta = Theta + Alpha;
  X += DeltaX;
  Y += DeltaY;
  angle = Theta;

  //Converts to degrees to allow for younger members to understand
  angledegrees = Theta * (180 / Pi);

  //This is for printing to the terminal and brain for debugging
  Brain.Screen.printAt(100,20, "X: %f",X);
  Brain.Screen.printAt(100,40, "Y: %f",Y);
  Brain.Screen.printAt(100,60, "Theta: %f",Theta);
  Brain.Screen.printAt(100,80, "Angle: %f",angledegrees);
  Brain.Screen.printAt(100,100, "Displacement1: %f",Displacement1);
  Brain.Screen.printAt(100,120, "DeltaLeftInches: %f",DeltaLeftInches);
  Brain.Screen.printAt(100,140, "DeltaRightInches: %f",DeltaRightInches);
  Brain.Screen.printAt(100,160, "DeltaX: %f",DeltaX);
  Brain.Screen.printAt(100,180, "DeltaY: %f",DeltaY);

  //Updates/Resets the values to reuse in next cycle of the calculation
  DeltaX = 0;
  DeltaY = 0;
  PreviousRightValue = CurrentRightValue;
  PreviousLeftValue = CurrentLeftValue;
  Alpha = 0;
}
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/
void pre_auton( void ) {
  vexcodeInit(); //Initializing Robot Configuration - Required!!!
  Brain.resetTimer(); //Resets The Brain Timer
  RightFront.resetRotation(); //Resets Motor Values For Accuracy In Autonomous
  RightBack.resetRotation(); //Resets Motor Values For Accuracy In Autonomous
  LeftFront.resetRotation(); //Resets Motor Values For Accuracy In Autonomous
  LeftBack.resetRotation(); //Resets Motor Values For Accuracy In Autonomous
}
/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/
void autonomous( void ) {
  //Autonomous code goes here
}
/*----------------------------------------------------------------------------*/
/*                              User Control Task                             */
/*----------------------------------------------------------------------------*/
void usercontrol( void ) {
  while (1){
    Brain.Screen.clearScreen(); //Clears screen to continuosly display new odom values

    //provides power to the motors to allow for movement of robot for testing using controller
    LeftBack.spin(vex::directionType::fwd, ((Controller1.Axis3.value() + (Controller1.Axis1.value())*0.1)), vex::velocityUnits::pct);
    LeftFront.spin(vex::directionType::fwd, ((Controller1.Axis3.value() + (Controller1.Axis1.value())*0.1)), vex::velocityUnits::pct);
    RightBack.spin(vex::directionType::fwd, ((Controller1.Axis3.value() - (Controller1.Axis1.value())*0.1)), vex::velocityUnits::pct);
    RightFront.spin(vex::directionType::fwd, ((Controller1.Axis3.value() - (Controller1.Axis1.value())*0.1)), vex::velocityUnits::pct);
    
    //Calls function
    TrackPOS();
    Brain.Screen.render(); //push data to the LCD all at once to prevent image flickering
    vex::task::sleep(10); //Slight delay so the Brain doesn't overprocess
  }
}
int main() {
    pre_auton(); //Calls the pre-autonomous function
    Competition.autonomous( autonomous ); //Calls the autonomous function
    Competition.drivercontrol( usercontrol ); //Calls the driver control function
    while(1) {
      vex::task::sleep(5); //Slight delay so the Brain doesn't overprocess
    }
}