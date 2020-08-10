/*----------------------------------------------------------------------------*/
/*    Module:       main.cpp                                                  */
/*    Author:       sagarpatel and saurinpatel                                */
/*    Description:  Odometry For Precise Autonomous Motion                    */
/*    Credits:      5225A For Pilons POS Document and QUEENS for Odom Alg.    */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RightFront           motor         18              
// RightBack            motor         17              
// LeftFront            motor         20              
// LeftBack             motor         14              
// Controller1          controller                    
// BackEncoder          encoder       A, B            
// LeftEncoder          encoder       C, D            
// RightEncoder         encoder       E, F            
// ---- END VEXCODE CONFIGURED DEVICES ----

/*-------------------------------Includes------------------------------------*/
#include "vex.h"
#include <math.h>
#include <iostream> 
using namespace vex;
vex::competition    Competition;

//Variables are defined here for use in the Odometry calculations
/*-------------------------------Variables-----------------------------------*/
#define Pi 3.14159265358979323846
#define SL 5 //distance from tracking center to middle of left wheel
#define SR 5 //distance from tracking center to middle of right wheel
#define SS 7.75 //distance from tracking center to middle of the tracking wheel
#define WheelDiam 4.125 //diameter of all the wheels being used for tracking
#define tpr 360  //Degrees per single encoder rotation
double DeltaL,DeltaR,DeltaB,currentL,currentR,currentB,PreviousL,PreviousR,PreviousB,DeltaTheta;
double X,Y,Theta,DeltaXSide,DeltaYSide,DeltaXBack,DeltaYBack,SideChord,BackChord,OdomHeading;
/*---------------------------------------------------------------------------*/
/*                            Odometry Functions                             */
/*---------------------------------------------------------------------------*/
void TrackPOS() {
// 2 cases could be occuring in odometry
// 1: Going in a straight line
// 2: Going in an arc motion
// If the bot is on an angle and going straight the displacement would be linear at angle Theta, meaning a right triangle is formed (Trig ratios to calc movement)
// Since it is a linear motion, the Left and right will move the same amount so we can just pick a side and do our movement calculation
// Since this calculation is working based of very infinitely small arcs, the displacement of the robot will be a chord
  currentR = (RightEncoder.position(degrees)); //Tracks right encoder value
  currentL = (LeftEncoder.position(degrees));  //Tracks left encoder value
  currentB = (BackEncoder.position(degrees));  //Tracks back encoder value

  //Creates variables for change in each side info in inches (12.9590697 is circumference of wheel)
  DeltaL = ((currentL - PreviousL) * 12.9590697) / tpr;
  DeltaR = ((currentR - PreviousR) * 12.9590697) / tpr;
  DeltaB = ((currentB - PreviousB) * 12.9590697) / tpr;

  //Determines the change in angle of the robot using the rotational change in each side
  DeltaTheta = (DeltaR - DeltaL) / (SL + SR);

  //Creates an if/else statement to prevent NaN values from appearing and causing issues with calculation
  if(DeltaTheta == 0) {  //If there is no change in angle
    X += DeltaL * sin (Theta);
    Y += DeltaL * cos (Theta);
    X += DeltaB * cos (Theta + 1.57079633);
    Y += DeltaB * sin (Theta + 1.57079633);

  //If there is a change in angle, it will calculate the changes in X,Y from chords of an arc/circle.
  } else {  //If there is a change in angle
      SideChord = 2 * ((DeltaL / DeltaTheta) + SL) * sin (DeltaTheta / 2);
      BackChord = 2 * ((DeltaB / DeltaTheta) + SS) * sin (DeltaTheta / 2);
      DeltaYSide = SideChord * cos (Theta + (DeltaTheta / 2));
      DeltaXSide = SideChord * sin (Theta + (DeltaTheta / 2));
      DeltaXBack = BackChord * sin (Theta + (DeltaTheta / 2));
      DeltaYBack = -BackChord * cos (Theta + (DeltaTheta / 2));
      Theta += DeltaTheta;
      X += DeltaXSide;
      Y += DeltaYSide;
    }

    //Odom heading is converting the radian value of Theta into degrees
    OdomHeading = Theta * 57.295779513;

    //Converts values into newer values to allow for code to effectively work in next cycle
    PreviousL = currentL;
    PreviousR = currentR;
    PreviousB = currentB;
    DeltaTheta = 0;

    //This is for printing to the brain for debugging (printing to Terminal coming soon)
    Brain.Screen.printAt(100,20, "X: %f",X);
    Brain.Screen.printAt(100,40, "Y: %f",Y);
    Brain.Screen.printAt(100,60, "Theta: %f",Theta);
    Brain.Screen.printAt(100,80, "Angle: %f",OdomHeading);
    Brain.Screen.printAt(100,100, "Displacement1: %f",SideChord);
    Brain.Screen.printAt(100,120, "DeltaLeftInches: %f",DeltaL);
    Brain.Screen.printAt(100,140, "DeltaRightInches: %f",DeltaR);
    Brain.Screen.printAt(100,160, "DeltaX: %f",DeltaXSide);
    Brain.Screen.printAt(100,180, "DeltaY: %f",DeltaYSide);
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
  //Autonomous goes here
}
/*----------------------------------------------------------------------------*/
/*                              User Control Task                             */
/*----------------------------------------------------------------------------*/
void usercontrol( void ) {
  while (1){
    Brain.Screen.clearScreen(); //clears the screen to continuously display the odometry info    
    //provides power to the motors to allow for movement of robot for testing using controller
    LeftBack.spin(vex::directionType::fwd, ((Controller1.Axis3.value() + (Controller1.Axis1.value())*0.1)), vex::velocityUnits::pct);
    LeftFront.spin(vex::directionType::fwd, ((Controller1.Axis3.value() + (Controller1.Axis1.value())*0.1)), vex::velocityUnits::pct);
    RightBack.spin(vex::directionType::fwd, ((Controller1.Axis3.value() - (Controller1.Axis1.value())*0.1)), vex::velocityUnits::pct);
    RightFront.spin(vex::directionType::fwd, ((Controller1.Axis3.value() - (Controller1.Axis1.value())*0.1)), vex::velocityUnits::pct);
    TrackPOS(); //Calls the TrackPosition function
    Brain.Screen.render(); //push data to the LCD all at once to prevent image flickering
    vex::task::sleep(10); //Slight delay so the Brain doesn't overprocess
  }
}
int main() {
    pre_auton();  //Calls the pre-autonomous function
    Competition.autonomous( autonomous ); //Calls the autonomous function
    Competition.drivercontrol( usercontrol ); //Calls the driver control function
    while(1) {
      vex::task::sleep(15); //Slight delay so the Brain doesn't overprocess
    }
}