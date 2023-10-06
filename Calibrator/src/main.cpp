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

float clamp(float target){
  if(target > 180){
    target -= 360;
  } else if(target < -180){
    target += 360;
  }
  return target;
}

void correctDrive(motor currentMotor, rotation currentRot){
  //get wheel rotation
  float currentDirection = currentRot.position(rev);
  //check if the robot should be in turn mode
  currentMotor.setVelocity(fabs(currentDirection * 25), percent);
  if(currentDirection > 0){
    currentMotor.spin(forward);
  } else {
    currentMotor.spin(reverse);
  }
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Brain.Screen.print(color::red);
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

void usercontrol(void) {
  Brain.Screen.clearScreen(color::green);
  while(1){
    Brain.Screen.clearScreen(color::green);
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print(RotationBL.position(rev));
    //correctDrive(TurnBL, RotationBL);
    correctDrive(TurnFL, RotationFL);
    correctDrive(TurnBR, RotationBR);
    correctDrive(TurnFR, RotationFR);
    wait(10, msec);
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