using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern smartdrive Drivetrain;
extern digital_out Pneumatic1;
extern digital_out Pneumatic2;
extern motor IntakeMotor;
extern motor_group PuncherMotors;
extern distance Distance21;
extern optical Optical11;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );