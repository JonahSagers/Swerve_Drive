using namespace vex;

extern brain Brain;

// VEXcode devices

extern competition Competition;
extern controller Controller1;
extern motor DriveBL;
extern motor DriveFL;
extern motor DriveBR;
extern motor DriveFR;
extern motor TurnBL;
extern motor TurnFL;
extern motor TurnBR;
extern motor TurnFR;
extern motor_group DriveTrain;
extern motor_group TurnTrain;
extern rotation RotationBL;
extern rotation RotationFL;
extern rotation RotationBR;
extern rotation RotationFR;
extern inertial Inertial;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );