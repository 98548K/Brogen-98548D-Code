#include "vex.h"

using namespace vex;

// Red Leftside Auton Win Point
void redLeftMAIN() {
  Drivetrain.setDriveVelocity(15, percent);
  Drivetrain.setTurnVelocity(10, percent);
  // Put ring on alliance stake
  // Get first ring
  newHeading(298);
  intakePneum.set(true);
  intakeMotor.spin(forward);
  Drivetrain.driveFor(forward, 10, inches, true);
  wait(100, msec);
  Drivetrain.driveFor(reverse, 8, inches, false);
  wait(100, msec);
  intakePneum.set(false);
  wait(150, msec);
  armMotor.stop();
  // Drive toward and clamp goal
  newHeading(35);
  Drivetrain.setDriveVelocity(25, percent);
  Drivetrain.driveFor(reverse, 40, inches, false);
  wait(1300, msec);
  clampPneum.set(false);
  wait(200, msec);
  // Drive towards second ring and put it on goal
  Drivetrain.setDriveVelocity(30, percent);
  newHeading(204);
  conveyorMotorGroup.spin(forward);
  Drivetrain.drive(forward);
  wait(150, msec);
  // Drive to second ring in an arc
  RightDriveSmart.setVelocity(35, percent);
  LeftDriveSmart.setVelocity(10, percent);
  RightDriveSmart.spin(forward);
  LeftDriveSmart.spin(forward);
  wait(700, msec);
  RightDriveSmart.setVelocity(30, percent);
  LeftDriveSmart.setVelocity(30, percent);
  wait(400, msec);
  RightDriveSmart.setVelocity(35, percent);
  LeftDriveSmart.setVelocity(10, percent);
  wait(200, msec);
  RightDriveSmart.stop();
  LeftDriveSmart.stop();
  RightDriveSmart.setVelocity(30, percent);
  LeftDriveSmart.setVelocity(30, percent);
  Drivetrain.driveFor(forward, 8, inches, true);
  wait(700, msec);
  // Drive towards third ring and put it on goal
  newHeading(44);
  conveyorMotorGroup.spin(forward);
  Drivetrain.driveFor(forward, 16, inches, true);
  wait(200, msec);
  newHeading(344);
  intakePneum.set(true);
  Drivetrain.setDriveVelocity(40, percent);
  // Drive towards ladder
  Drivetrain.driveFor(forward, 40, inches, false);
  wait(500, msec);
  conveyorMotorGroup.stop();
  wait(900, msec);
  intakePneum.set(false);
}

// Red Leftside Auton Goal Fill
void redLeftALT() {
  Drivetrain.setDriveVelocity(15, percent);
  Drivetrain.setTurnVelocity(12, percent);
  // Put ring on alliance stake
  // Get first ring
  newHeading(298);
  intakePneum.set(true);
  intakeMotor.spin(forward);
  Drivetrain.driveFor(forward, 10, inches, true);
  wait(100, msec);
  Drivetrain.driveFor(reverse, 8, inches, false);
  wait(100, msec);
  intakePneum.set(false);
  wait(150, msec);
  // Drive toward and clamp goal
  newHeading(35);
  armMotor.stop();
  Drivetrain.setDriveVelocity(25, percent);
  Drivetrain.driveFor(reverse, 40, inches, false);
  wait(1300, msec);
  clampPneum.set(false);
  wait(200, msec);
  // Drive towards second ring and put it on goal
  Drivetrain.setDriveVelocity(30, percent);
  newHeading(200);
  conveyorMotorGroup.spin(forward);
  Drivetrain.drive(forward);
  wait(175, msec);
  // Drive to third ring in an arc
  RightDriveSmart.setVelocity(35, percent);
  LeftDriveSmart.setVelocity(8, percent);
  RightDriveSmart.spin(forward);
  LeftDriveSmart.spin(forward);
  wait(700, msec);
  RightDriveSmart.setVelocity(30, percent);
  LeftDriveSmart.setVelocity(30, percent);
  wait(400, msec);
  RightDriveSmart.setVelocity(35, percent);
  LeftDriveSmart.setVelocity(10, percent);
  wait(200, msec);
  RightDriveSmart.stop();
  LeftDriveSmart.stop();
  RightDriveSmart.setVelocity(30, percent);
  LeftDriveSmart.setVelocity(30, percent);
  Drivetrain.driveFor(forward, 8, inches, true);
  wait(200, msec);
  // Drive towards fourth ring and put it on goal
  newHeading(44);
  Drivetrain.setDriveVelocity(30, percent);
  conveyorMotorGroup.spin(forward);
  Drivetrain.driveFor(forward, 18, inches, true);
  wait(200, msec);
  newHeading(65);
  // Drive towards corner rings
  RightDriveSmart.setVelocity(50, percent);
  LeftDriveSmart.setVelocity(50, percent);
  RightDriveSmart.spin(forward);
  LeftDriveSmart.spin(forward);
  wait(900, msec);
  RightDriveSmart.setVelocity(15, percent);
  LeftDriveSmart.setVelocity(30, percent);
  wait(200, msec);
  RightDriveSmart.stop();
  LeftDriveSmart.stop();
  RightDriveSmart.setVelocity(30, percent);
  LeftDriveSmart.setVelocity(30, percent);
  Drivetrain.setDriveVelocity(30, percent);
  Drivetrain.driveFor(forward, 16, inches, false);
  wait(800, msec);
  Drivetrain.driveFor(reverse, 2, inches, true);
  Drivetrain.driveFor(forward, 10, inches, false);
  wait(600, msec);
  Drivetrain.setDriveVelocity(20, percent);
  Drivetrain.driveFor(reverse, 18, inches, true);
}

// Red Rightside Auton Win Point
void redRightMAIN() {
  Drivetrain.setDriveVelocity(15, percent);
  Drivetrain.setTurnVelocity(10, percent);
  // Put ring on alliance stake
  // Get first ring
  newHeading(62);
  intakePneum.set(true);
  intakeMotor.spin(forward);
  Drivetrain.driveFor(forward, 10, inches, true);
  wait(100, msec);
  Drivetrain.driveFor(reverse, 8, inches, false);
  wait(100, msec);
  intakePneum.set(false);
  wait(100, msec);
  armMotor.stop();
  // Drive toward and clamp goal
  newHeading(326);
  Drivetrain.setDriveVelocity(25, percent);
  Drivetrain.driveFor(reverse, 40, inches, false);
  wait(1200, msec);
  clampPneum.set(false);
  wait(200, msec);
  // Drive towards second ring and put it on goal
  Drivetrain.setDriveVelocity(30, percent);
  newHeading(215);
  conveyorMotorGroup.spin(forward);
  Drivetrain.driveFor(forward, 23, inches, true);
  wait(800, msec);
  // Drive to ladder
  newHeading(20);
  wait(200, msec);
  conveyorMotorGroup.stop();
  Drivetrain.driveFor(forward, 39, inches, true);
}

// Red Rightside Auton Two Ring
void redRightALT() {
  Drivetrain.setDriveVelocity(25, percent);
  Drivetrain.setTurnVelocity(10, percent);
  // Put ring on alliance stake
  // Drive toward and clamp goal
  Drivetrain.driveFor(reverse, 4, inches, true);
  newHeading(320);
  Drivetrain.driveFor(reverse, 32, inches, false);
  wait(1100, msec);
  clampPneum.set(false);
  wait(200, msec);
  newHeading(210);
  conveyorMotorGroup.spin(forward);
  Drivetrain.driveFor(forward, 24, inches, true);
  wait(1100, msec);
  newHeading(17);
  Drivetrain.driveFor(forward, 40, inches, true);
  conveyorMotorGroup.stop();
}