#include "vex.h"
#include <math.h>

using namespace vex;

controller Controller1 = controller(primary);
brain  Brain;

//motors are notated by position.  For example, BL = back left
motor DriveBL = motor(PORT6, ratio6_1, true);
motor DriveFL = motor(PORT5, ratio6_1, true);
motor DriveBR = motor(PORT7, ratio6_1, true);
motor DriveFR = motor(PORT8, ratio6_1, true);

motor TurnBL = motor(PORT1, ratio6_1, true);
motor TurnFL = motor(PORT2, ratio6_1, true);
//port 3 is broken
//sad
motor TurnBR = motor(PORT14, ratio6_1, true);
motor TurnFR = motor(PORT4, ratio6_1, true);
//unassigned motors are placeholdered at 20
motor Intake = motor(PORT16, ratio6_1, false);
motor Flywheel = motor(PORT19, ratio6_1, true);
digital_out PnuIntake = digital_out(Brain.ThreeWirePort.A);

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
float xRotPoint;
float yRotPoint;
directionType DirecBL;
directionType DirecFL;
directionType DirecBR;
directionType DirecFR;
int intakeState;
int flywheelState;

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

float clamp(float target){
  if(target > 180){
    target -= 360;
  } else if(target < -180){
    target += 360;
  }
  return target;
}

float sign(float target){
  if(target > 0){
    target = 1;
  } else if(target < 0){
    target = -1;
  } else {
    target = 0;
  }
  return target;
}

void toggleIntake(){
  if(intakeState == false){
    Intake.spin(forward);
    Intake.setVelocity(100, percent);
    intakeState = true;
  } else {
    Intake.stop();
    intakeState = false;
  }
}

void toggleFlywheel(){
  if(flywheelState == false){
    Flywheel.spin(forward);
    Flywheel.setVelocity(100, percent);
    flywheelState = true;
  } else {
    Flywheel.stop();
    flywheelState = false;
  }
}

void correctDrive(motor currentMotor, rotation currentRot, int orient, int turnOffset, int turnOffset2, int x, int y){
  //get wheel rotation
  PnuIntake = true;
  float currentDirection = fmod((currentRot.position(rev)/7 * 3) * 360,360);
  float difDirection;
  currentDirection = clamp(currentDirection);
  //check if the robot should be in turn mode
  if(magnitude > 10 && fabs(turnMagnitude) < 10){
    //in drive mode, use both the rotation sensor and the joystick input to find the right direction to face
    //this is what makes the robot "field-centric"
    difDirection = (currentDirection - clamp(targetDirection));
    // + clamp(fmod(Inertial.heading(degrees),360))
  } else if(fabs(turnMagnitude) > 10) {
    // difDirection = (currentDirection - clamp(targetDirection + ((turnMagnitude * 0.45) * (cos((targetDirection + turnOffset2) * (3.14159/180))))));
    difDirection = clamp(currentDirection + clamp((atan2((xRotPoint - x),(yRotPoint - y))) * 180/3.14159265) + 90);
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

void autonomous(void) {
  Brain.Screen.clearScreen(color::cyan);
  PnuIntake = true;
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

void usercontrol(void) {
  Brain.Screen.clearScreen(color::green);
  //Controller1.ButtonR2.pressed(toggleIntake);
  //Adrian doesn't want an intake toggle
  Controller1.ButtonUp.pressed(toggleFlywheel);
  //Flywheel should start in the on state
  //But not during development.  Uncomment this for comp
  // Flywheel.spin(forward);
  // Flywheel.setVelocity(65, percent);
  // flywheelState = true;
  PnuIntake = false;
  wait(20, msec);
  PnuIntake = true;
  // User control code here, inside the loop
  while(1) {
    //x and y had to be swapped because of how vex handles axes
    float xInput = -Controller1.Axis4.position();
    float yInput = Controller1.Axis3.position();
    turnMagnitude = Controller1.Axis1.position();
    //find the direction the stick is pointed in
    targetDirection = atan2(xInput,yInput)* 180.0 / 3.14159265 + 180;
    //find the magnitude of the left stick
    magnitude = sqrt((xInput * xInput + yInput * yInput));
    xRotPoint = -sin(fabs(targetDirection) * 3.14159265/180 + 1.570796325) * (magnitude/20) * sign(turnMagnitude);
    yRotPoint = cos(fabs(targetDirection) * 3.14159265/180 + 1.570796325) * (magnitude/20) * sign(turnMagnitude);
    //coefficient goes up to 8
    avgDif = 0;
    //run the function to correct each drive
    //since each wheel is handled separately, it can correct for being knocked around and desynced
    correctDrive(TurnBL, RotationBL, 0, 225, 225, -1, -1);
    correctDrive(TurnFL, RotationFL, 1, 135, 315, -1, 1);
    correctDrive(TurnBR, RotationBR, 2, 315, 135, 1, -1);
    correctDrive(TurnFR, RotationFR, 3, 45, 45, 1, 1);
    avgDif /= 4;
    
    //check if the drive should be in turn mode
    //each drive mode has a different drive function, to account for little differences
    //in the end, if they're still identical I'll merge them to cut down on spaghetti code
    if(magnitude > 10 && fabs(turnMagnitude) < 10){
      //move motors
      float speed = magnitude;
      DriveTrain.setVelocity(speed, percent);
      DriveBL.spin(DirecBL);
      DriveFL.spin(DirecFL);
      DriveBR.spin(DirecBR);
      DriveFR.spin(DirecFR);
    } else if(fabs(turnMagnitude) > 10){
      DriveBL.setVelocity((2.5 * (6.28318530 * (sqrt(pow(xRotPoint + 1,2)+ pow(yRotPoint + 1,2)))) * sign(magnitude) * sign(turnMagnitude)) + turnMagnitude/(magnitude + 1), percent);
      DriveFL.setVelocity((2.5 * (6.28318530 * (sqrt(pow(xRotPoint + 1,2)+ pow(yRotPoint - 1,2)))) * sign(magnitude) * sign(turnMagnitude)) + turnMagnitude/(magnitude + 1), percent);
      DriveBR.setVelocity((2.5 * (6.28318530 * (sqrt(pow(xRotPoint - 1,2)+ pow(yRotPoint + 1,2)))) * sign(magnitude) * sign(turnMagnitude)) + turnMagnitude/(magnitude + 1), percent);
      DriveFR.setVelocity((2.5 * (6.28318530 * (sqrt(pow(xRotPoint - 1,2)+ pow(yRotPoint - 1,2)))) * sign(magnitude) * sign(turnMagnitude)) + turnMagnitude/(magnitude + 1), percent);
      DriveBL.spin(DirecBL);
      DriveFL.spin(DirecFL);
      DriveBR.spin(DirecBR);
      DriveFR.spin(DirecFR);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
    } else {
      DriveTrain.stop();
      TurnTrain.stop();
    }
    if(Controller1.ButtonR2.pressing()){
      Intake.setVelocity(100, percent);
      Intake.spin(forward);
      intakeState = true;
    } else if(Controller1.ButtonR1.pressing()){
      Intake.setVelocity(100, percent);
      Intake.spin(reverse);
      intakeState = true;
    } else {
      Intake.stop();
      intakeState = false;
    }
    //button state measures whether the button is pressed, so that the function doesn't trigger every frame
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