#include "vex.h"
#include <math.h>

using namespace vex;

controller Controller1 = controller(primary);
brain  Brain;

//motors are notated by position.  For example, BL = back left
motor DriveBL = motor(PORT5, ratio6_1, true);
motor DriveFL = motor(PORT6, ratio6_1, true);
motor DriveBR = motor(PORT7, ratio6_1, true);
motor DriveFR = motor(PORT8, ratio6_1, true);

motor TurnBL = motor(PORT1, ratio6_1, true);
motor TurnFL = motor(PORT2, ratio6_1, true);
//port 3 is broken
//sad
motor TurnBR = motor(PORT14, ratio6_1, true);
motor TurnFR = motor(PORT4, ratio6_1, true);

motor_group DriveTrain = motor_group(DriveBL, DriveFL, DriveBR, DriveFR);
motor_group TurnTrain = motor_group(TurnBL, TurnFL, TurnBR, TurnFR);

rotation RotationBL = rotation(PORT9, false);
rotation RotationFL = rotation(PORT10, false);
rotation RotationBR = rotation(PORT11, false);
//port 12 is broken
//sad
rotation RotationFR = rotation(PORT15, false);

inertial Inertial = inertial(PORT16);

int rc_auto_loop_function_Controller1();

competition Competition;

float targetDirection;
float rotationOffset;
float magnitude;
float turnMagnitude;
float avgDif;
directionType DirecBL;
directionType DirecFL;
directionType DirecBR;
directionType DirecFR;

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Brain.Screen.print(color::red);
  RotationBL.setPosition(0, rev);
  RotationFL.setPosition(0, rev);
  RotationBR.setPosition(0, rev);
  RotationFR.setPosition(0, rev);
  Inertial.setRotation(0, rev);
  Inertial.calibrate();
  while(Inertial.isCalibrating()){
    wait(100, msec);
  }
  Brain.Screen.print(color::cyan);
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

void autonomous(void) {
  Brain.Screen.clearScreen(color::cyan);

  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

float clamp(float target){
  if(target > 180){
    target -= 360;
  } else if(target < -180){
    target += 360;
  }
  return target;
}

void correctDrive(motor currentMotor, rotation currentRot, int orient, int turnOffset){
  float currentDirection = fmod((currentRot.position(rev)/7 * 3) * 360,360);
  float difDirection;
  currentDirection = clamp(currentDirection);
  if(magnitude > 10 && turnMagnitude < 10){
    difDirection = (currentDirection - clamp((targetDirection + clamp(fmod(Inertial.rotation(degrees),360)))));
  } else if(magnitude < 10 && turnMagnitude > 10){
    difDirection = (currentDirection - turnOffset);
  } else {
    difDirection = 0;
  }
  
  if(fabs(clamp(difDirection)) > fabs(clamp(clamp(difDirection) - 180))){
    difDirection -= 180;
    //Jonah if you leave in four nested if statements I will crucify you
    if(orient == 0){
      DirecBL = reverse;
    } else if(orient == 1){
      DirecFL = reverse;
    } else if(orient == 2){
      DirecBR = reverse;
    } else if(orient == 3){
      DirecFR = reverse;
    }
  } else {
    if(orient == 0){
      DirecBL = forward;
    } else if(orient == 1){
      DirecFL = forward;
    } else if(orient == 2){
      DirecBR = forward;
    } else if(orient == 3){
      DirecFR = forward;
    }
  }
  difDirection = clamp(difDirection);
  avgDif += fabs(difDirection);
  if(fabs(difDirection) > 1){
    currentMotor.setVelocity(fabs(difDirection), percent);
    if(difDirection > 0){
      currentMotor.spin(forward);
    } else {
      currentMotor.spin(reverse);
    }
  } else {
    currentMotor.stop();
  }
}

void usercontrol(void) {
  Brain.Screen.clearScreen(color::green);
  // User control code here, inside the loop
  while(1) {
    //x and y are swapped for a reason
    float xInput = Controller1.Axis4.position();
    float yInput = -Controller1.Axis3.position();
    turnMagnitude = Controller1.Axis1.position();
    magnitude = sqrt((xInput * xInput + yInput * yInput));
    //FIX THIS MATH ON SITE JONAH DO IT OR ELSE
    //actually it's a problem for later
    targetDirection = atan2(xInput,yInput)* 180.0 / 3.14159265 + 180;
    Brain.Screen.clearLine(0);
    Brain.Screen.print(turnMagnitude);
    avgDif = 0;
    correctDrive(TurnBL, RotationBL, 0, 225);
    correctDrive(TurnFL, RotationFL, 1, 135);
    correctDrive(TurnBR, RotationBR, 2, 315);
    correctDrive(TurnFR, RotationFR, 3, 45);
    avgDif /= 4;
    if(magnitude > 10 && turnMagnitude < 10){
      //move motors
      float speed = magnitude/(1 + avgDif/100);
      DriveBL.setVelocity(speed, percent);
      DriveFL.setVelocity(speed, percent);
      DriveBR.setVelocity(speed, percent);
      DriveFR.setVelocity(speed, percent);
      //magnitude - avgDif is potentially problematic because the cutoff changes based on magnitude
      DriveBL.spin(DirecBL);
      DriveFL.spin(DirecFL);
      DriveBR.spin(DirecBR);
      DriveFR.spin(DirecFR);
    } else if(magnitude < 10 && turnMagnitude > 10){
      float speed = turnMagnitude - avgDif * 10;
      DriveBL.setVelocity(speed, percent);
      DriveFL.setVelocity(speed, percent);
      DriveBR.setVelocity(speed, percent);
      DriveFR.setVelocity(speed, percent);
      //magnitude - avgDif is potentially problematic because the cutoff changes based on magnitude
      DriveBL.spin(DirecBL);
      DriveFL.spin(DirecFL);
      DriveBR.spin(DirecBR);
      DriveFR.spin(DirecFR);
    } else {
      DriveTrain.stop();
      TurnTrain.stop();
    }
    wait(20, msec); // 1000 divided by wait duration = refresh rate of the code
                    // for example, 20 msec = 50hz refresh rate
  }
}

// Main will set up the competition functions and callbacks.

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}