#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor leftMotorA = motor(PORT4, ratio6_1, false);
// Motor on port 4 reversed
motor leftMotorB = motor(PORT3, ratio6_1, true);
motor leftMotorC = motor(PORT5, ratio6_1, true);
// Left Drive: Blue Zipties
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB, leftMotorC);
motor rightMotorA = motor(PORT16, ratio6_1, true);
// Motor on port 16 reversed
motor rightMotorB = motor(PORT17, ratio6_1, false);
motor rightMotorC = motor(PORT18, ratio6_1, false);
// Right Drive: Purple Zipties
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB, rightMotorC);
// Inertial: Black Zipties
inertial DrivetrainInertial = inertial(PORT19);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, DrivetrainInertial, 319.19, 320, 40, mm, 0.6666666666666666);
// Pneumatics: Red Zipties
digital_out clampPneum = digital_out(Brain.ThreeWirePort.B);
digital_out intakePneum = digital_out(Brain.ThreeWirePort.C);
digital_out cornerPneum = digital_out(Brain.ThreeWirePort.A);
digital_out clawPneum = digital_out(Brain.ThreeWirePort.D);
digital_out ringRush = digital_out(Brain.ThreeWirePort.E);
// Conveyor: Green Zipties
motor conveyorMotor = motor(PORT2, ratio18_1, true);
// Intake: 
motor intakeMotor = motor(PORT10, ratio18_1, false);
motor_group conveyorMotorGroup = motor_group(conveyorMotor, intakeMotor);
// Arm: 
motor armMotor = motor(PORT20, ratio18_1, false);
// Rotation: 
rotation rotationSens = rotation(PORT9, false);
rotation rightDriveRot = rotation(PORT8, true);
rotation leftDriveRot = rotation(PORT7, true);
// Color Sensor
optical colorSens = optical(PORT1);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool Controller1LeftShoulderControlMotorsStopped = true;
bool Controller1RightShoulderControlMotorsStopped = true;
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;
bool ArmSet = false;
bool ArmReset = true;

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3
      // right = Axis2
      int drivetrainLeftSideSpeed = Controller1.Axis3.position();
      int drivetrainRightSideSpeed = Controller1.Axis2.position();
      
      // check if the value is inside of the deadband range
      if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
        // check if the left motor has already been stopped
        if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor
          LeftDriveSmart.stop();
          // tell the code that the left motor has been stopped
          DrivetrainLNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range
        DrivetrainLNeedsToBeStopped_Controller1 = true;
      }
      // check if the value is inside of the deadband range
      if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
        // check if the right motor has already been stopped
        if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor
          RightDriveSmart.stop();
          // tell the code that the right motor has been stopped
          DrivetrainRNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
        DrivetrainRNeedsToBeStopped_Controller1 = true;
      }
      
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }
      // check the ButtonL1/ButtonL2 status to control Intake Motor & Conveyor Motor
      if (Controller1.ButtonL1.pressing() && !conveyorStop) {
        // This is intaking a ring
        conveyorMotorGroup.setVelocity(100, percent);
        conveyorMotorGroup.spin(forward);
        Controller1LeftShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonL2.pressing() && !conveyorStop) {
        // This is outaking a ring
        conveyorMotorGroup.setVelocity(75, percent);
        conveyorMotorGroup.spin(reverse);
        Controller1LeftShoulderControlMotorsStopped = false;
      } else if (!Controller1LeftShoulderControlMotorsStopped || conveyorStop) {
        conveyorMotorGroup.stop();
        // set the toggle so that we don't constantly tell the motors to stop when the buttons are released
        Controller1LeftShoulderControlMotorsStopped = true;
      }
      // check the ButtonR1/ButtonR2 status to control Arm Motors
      if (!ArmReset) {
        if (Controller1.ButtonR1.pressing() && rotationSens.position(degrees) > 75) {
          // Moves arm forward
          armMotor.setVelocity(100, percent);
          armMotor.spin(reverse);
          ArmSet = false;
          Controller1RightShoulderControlMotorsStopped = false;
        } else if (!Controller1.ButtonR1.pressing() && ((rotationSens.position(degrees) < 350 && ArmSet == false))) {
          // Moves arm backward automatically
          armMotor.setVelocity(100, percent);
          armMotor.spin(forward);
          Controller1RightShoulderControlMotorsStopped = false;
        } else if (!Controller1RightShoulderControlMotorsStopped) {
          armMotor.stop();
          // Set the toggle so that we don't constantly tell the motors to stop when the buttons are released
          Controller1RightShoulderControlMotorsStopped = true;
        }
      }
    }
    // Wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  // calibrate the drivetrain Inertial
  wait(200, msec);
  DrivetrainInertial.calibrate();
  Brain.Screen.print("Calibrating Inertial for Drivetrain");
  // wait for the Inertial calibration process to finish
  while (DrivetrainInertial.isCalibrating()) {
    wait(25, msec);
  }
  // reset the screen now that the calibration is complete
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
  wait(50, msec);
  Brain.Screen.clearScreen();
}


