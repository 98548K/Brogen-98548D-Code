using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern smartdrive Drivetrain;
extern motor_group RightDriveSmart;
extern motor_group LeftDriveSmart;
extern motor armMotor;
extern motor_group conveyorMotorGroup;
extern digital_out clampPneum;
extern digital_out intakePneum;
extern digital_out cornerPneum;
extern digital_out clawPnuem;
extern inertial DrivetrainInertial;
extern rotation rotationSens;
extern rotation rightDriveRot;
extern rotation leftDriveRot;
extern optical colorSens;
extern bool ArmReset;
extern bool elevationActive;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );