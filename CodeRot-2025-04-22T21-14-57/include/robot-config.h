using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern smartdrive Drivetrain;
extern motor_group RightDriveSmart;
extern motor_group LeftDriveSmart;
extern motor armMotor;
extern motor intakeMotor;
extern motor_group conveyorMotorGroup;
extern digital_out clampPneum;
extern digital_out intakePneum;
extern digital_out cornerPneum;
extern digital_out clawPneum;
extern digital_out ringRush;
extern inertial DrivetrainInertial;
extern rotation rotationSens;
extern rotation rightDriveRot;
extern rotation leftDriveRot;
extern optical colorSens;

// Tasks
extern task task_ringSorter;
extern task task_brainScreenPrint;

// Functions
extern event autonSelector_pressed;
extern event autonSelector_2_pressed;
extern event delaySelector;
extern event armSelector;
extern void newHeading(double newHeading__Degrees);
//extern void velociDrive(directionType direct, double distance_num);
extern void arcDrive(directionType direct, double directionDegrees, double distanceMillimeters);
extern void blueLeftMAIN();
extern void blueLeftALT();
extern void blueRightMAIN();
extern void blueRightALT();
extern void redLeftMAIN();
extern void redLeftALT();
extern void redRightMAIN();
extern void redRightALT();

// Variables
extern bool ArmReset;
extern bool ArmSet;
extern bool conveyorStop;
extern bool runArm;
extern int autonSelector;
extern int autonSelector_2;
extern int delay;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );