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

void correctDrive(motor currentMotor, rotation currentRot, int orient, int turnOffset, int turnOffset2){
  //get wheel rotation
  float currentDirection = fmod((currentRot.position(rev)/7 * 3) * 360,360);
  float difDirection;
  currentDirection = clamp(currentDirection);
  //check if the robot should be in turn mode
  if(magnitude < 10 && fabs(turnMagnitude) > 10){
    //in turn mode, the wheels always face a set direction.  No target directions are needed
    difDirection = (currentDirection - turnOffset);
  } else if(magnitude > 10 && fabs(turnMagnitude) < 10){
    //in drive mode, use both the rotation sensor and the joystick input to find the right direction to face
    //this is what makes the robot "field-centric"
    difDirection = (currentDirection - clamp(targetDirection));
    // + clamp(fmod(Inertial.heading(degrees),360))
  } else if(magnitude > 10 && fabs(turnMagnitude) > 10) {
    difDirection = (currentDirection - clamp(targetDirection + ((turnMagnitude * 0.45) * (cos((targetDirection + turnOffset2) * (3.14159/180))))));
  } else {
    difDirection = 0;
  }
  //Determine the proper drive directions
  //This code is bad. Like really bad. Leave it out of the notebook until it's better
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
  //Determine which way the drive needs to rotate to reach the target direction
  difDirection = clamp(difDirection);
  //find the average difference in drive rotations
  //this makes sure the drive will not move unless everything is facing the right direction
  //without it, the drive would damage itself
  avgDif += fabs(difDirection);
  if(fabs(difDirection) > 1){
    //set drive speed based on the distance the drive needs to turn
    //this way, it will slow down before reaching the target and not overshoot
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
    //x and y had to be swapped because of how vex handles axes
    float xInput = Controller1.Axis4.position();
    float yInput = -Controller1.Axis3.position();
    turnMagnitude = -Controller1.Axis1.position();
    //find the magnitude of the left stick
    magnitude = sqrt((xInput * xInput + yInput * yInput));
    //find the direction the stick is pointed in
    targetDirection = atan2(xInput,yInput)* 180.0 / 3.14159265 + 180;
    avgDif = 0;
    //run the function to correct each drive
    //since each wheel is handled separately, it can correct for being knocked around and desynced
    correctDrive(TurnBL, RotationBL, 0, 225, 225);
    correctDrive(TurnFL, RotationFL, 1, 135, 315);
    correctDrive(TurnBR, RotationBR, 2, 315, 135);
    correctDrive(TurnFR, RotationFR, 3, 45, 45);
    avgDif /= 4;
    //check if the drive should be in turn mode
    //each drive mode has a different drive function, to account for little differences
    //in the end, if they're still identical I'll merge them to cut down on spaghetti code
    if(magnitude > 10 && fabs(turnMagnitude) < 10){
      //move motors
      float speed = magnitude;
      DriveBL.setVelocity(speed, percent);
      DriveFL.setVelocity(speed, percent);
      DriveBR.setVelocity(speed, percent);
      DriveFR.setVelocity(speed, percent);
      DriveBL.spin(DirecBL);
      DriveFL.spin(DirecFL);
      DriveBR.spin(DirecBR);
      DriveFR.spin(DirecFR);
    } else if(magnitude < 10 && fabs(turnMagnitude) > 10){
      float speed = turnMagnitude/(1 + avgDif/25);
      DriveBL.setVelocity(speed, percent);
      DriveFL.setVelocity(speed, percent);
      DriveBR.setVelocity(speed, percent);
      DriveFR.setVelocity(speed, percent);
      DriveBL.spin(DirecBL);
      DriveFL.spin(DirecFL);
      DriveBR.spin(DirecBR);
      DriveFR.spin(DirecFR);
    } else if(magnitude > 10 && fabs(turnMagnitude) > 10){
      float speed = magnitude/1.5;
      DriveBL.setVelocity(speed * (1 - 0.5 * (cos((targetDirection + 90 * (turnMagnitude/100) + 225) * (3.14159/180)))), percent);
      DriveFL.setVelocity(speed * (1 - 0.5 * (cos((targetDirection + 90 * (turnMagnitude/100) + 315) * (3.14159/180)))), percent);
      DriveBR.setVelocity(speed * (1 - 0.5 * (cos((targetDirection + 90 * (turnMagnitude/100) + 135) * (3.14159/180)))), percent);
      DriveFR.setVelocity(speed * (1 - 0.5 * (cos((targetDirection + 90 * (turnMagnitude/100) + 45) * (3.14159/180)))), percent);
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