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
extern digital_in sorterSwitch;
extern inertial DrivetrainInertial;
extern rotation rotationSens;
extern rotation rightDriveRot;
extern rotation leftDriveRot;
extern optical colorSens;

// Tasks
extern task task_ringSorter;
extern task task_brainScreenPrint;

// Functions
extern event armPosition;
extern event armRotationReset;
extern void newHeading(double newHeading__Degrees);
extern void velociDrive(directionType direct, double distance_num);
extern void arcDrive(directionType direct, double directionDegrees, double distanceMillimeters, double drive_velocity, double turn_velocity);
extern void onevent_Controller1ButtonB_pressed();
extern void auton();

// Variables
extern bool ArmReset;
extern bool ArmSet;
extern bool conveyorStop;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );