#include "vex.h"

using namespace vex;

// Blue Goalside Auton Win Point
void blueLeftMAIN() {
   Drivetrain.setHeading(295, deg);
  // Put ring on alliance stake
  // Get first ring
  arcDrive(forward, 222, 0);
  intakePneum.set(true);
  intakeMotor.spin(forward);
  arcDrive(forward, 222, 200);
  intakePneum.set(false);
  wait(100, msec);
  // Clamp goal
  arcDrive(reverse, 347, 950);
  Drivetrain.setDriveVelocity(40, percent);
  Drivetrain.drive(reverse);
  wait(300, msec);
  clampPneum.set(false);
  wait(50, msec);
  Drivetrain.stop();
  // Drive towards second ring and put it on goal
  arcDrive(forward, 94, 0);
  conveyorMotorGroup.spin(forward);
  arcDrive(forward, 94, 100);
  // Drive to corner
  double Deg[] = {30,1,35,-1};
  double Dist[] = {50,700,300,-1};
  arcDriveCont(forward, Deg, Dist);
  Drivetrain.setDriveVelocity(20, percent);
  Drivetrain.drive(forward);
  wait(900, msec);
  for (int i = 0; i < 2; i++) {
    Drivetrain.drive(reverse);
    wait(150, msec);
    Drivetrain.drive(forward);
    wait(150, msec);
  }
  Drivetrain.drive(forward);
  wait(1000, msec);
  Drivetrain.drive(reverse);
  wait(400, msec);
  Drivetrain.stop();
  intakePneum.set(true);
  wait(200, msec);
  Drivetrain.drive(forward);
  wait(1100, msec);
  intakePneum.set(false);
  wait(200, msec);
  Drivetrain.stop();
  // Drive to ladder
  arcDrive(reverse, 315, 450);
  clampPneum.set(true);
  conveyorMotorGroup.stop();
  armMotor.spin(reverse);
  arcDrive(forward, 220, 1250);
}

// Blue Goalside Auton Goal Rush
void blueLeftALT() {
  Drivetrain.setDriveVelocity(45, percent);
  // Drive to goal and grab it with the claw while picking up ring
  cornerPneum.set(true);
  intakeMotor.spin(forward);
  Drivetrain.driveFor(forward, 825, mm);
  clawPneum.set(true);
  // Pull goal back
  arcDrive(reverse, 0, 500);
  clawPneum.set(false);
  // Turn to lift corner arm
  Drivetrain.turn(right);
  wait(300, msec);
  cornerPneum.set(false);
  Drivetrain.stop();
  // Turn to get second goal
  arcDrive(forward, 238, 0);
  arcDrive(reverse, 238, 675);
  intakeMotor.stop();
  clampPneum.set(false);
  // Place two rings on goal
  conveyorMotorGroup.spin(forward);
  arcDrive(forward, 350, 0);
  // Put goal near positive corner
  arcDrive(reverse, 350, 600);
  conveyorMotorGroup.stop();
  clampPneum.set(true);
  // Clamp first goal
  arcDrive(forward, 130, 0);
  arcDrive(reverse, 130, 900);
  clampPneum.set(false);
}

// Blue Stackside Auton Win Point
void blueRightMAIN() {
  
}

// Blue Stackside Auton Goal Fill
void blueRightALT() {
 
}

// Blue Solo Sig Win Point
void blueSig() {
  Drivetrain.setHeading(63, deg);
  // Put ring on alliance stake
  arcDrive(forward, 135, 0);
  intakePneum.set(true);
  intakeMotor.spin(forward);
  arcDrive(forward, 135, 200);
  intakePneum.set(false);
  arcDrive(reverse, 350, 1000);
  clampPneum.set(false);
  wait(100, msec);
  conveyorMotorGroup.spin(forward);
  arcDrive(forward, 230, 0);
  double Deg[] = {221.2,250.1,253.1,263.3,272.9,277.2,267.5,286.9,356.6,11.7,11,-1};
  double Dist[] = {35.7,37.6,36.7,36.1,46.9,40.1,39.9,35.2,38.4,35.7,35,-1};
  arcDriveCont(forward, Deg, Dist);
  wait(200, msec);
  conveyorMotorGroup.stop();
  intakeMotor.spin(forward);
  arcDrive(forward, 65, 1400);
  clampPneum.set(true);
  arcDrive(forward, 300, 0);
  arcDrive(reverse, 300, 900);
  clampPneum.set(false);
  arcDrive(forward, 90, 0);
  conveyorMotorGroup.spin(forward);
  arcDrive(forward, 90, 400);
}