#include "vex.h"

using namespace vex;

// Blue Leftside Auton Win Point
void blueLeftMAIN() {
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
  wait(100, msec);
  armMotor.stop();
  // Drive toward and clamp goal
  newHeading(37);
  Drivetrain.setDriveVelocity(25, percent);
  Drivetrain.driveFor(reverse, 40, inches, false);
  wait(1200, msec);
  clampPneum.set(false);
  wait(200, msec);
  // Drive towards second ring and put it on goal
  Drivetrain.setDriveVelocity(30, percent);
  newHeading(145);
  conveyorMotorGroup.spin(forward);
  Drivetrain.driveFor(forward, 23, inches, true);
  wait(800, msec);
  newHeading(335);
  wait(200, msec);
  conveyorMotorGroup.stop();
  Drivetrain.driveFor(forward, 39, inches, true);
}

// Blue Leftside Auton Two Rush
void blueLeftALT() {
  Drivetrain.setDriveVelocity(25, percent);
  Drivetrain.setTurnVelocity(10, percent);
  // Put ring on alliance stake
  // Drive toward and clamp goal
  Drivetrain.driveFor(reverse, 4, inches, true);
  newHeading(40);
  Drivetrain.driveFor(reverse, 32, inches, false);
  wait(1100, msec);
  clampPneum.set(false);
  wait(200, msec);
  newHeading(150);
  conveyorMotorGroup.spin(forward);
  Drivetrain.driveFor(forward, 24, inches, true);
  wait(1100, msec);
  newHeading(343);
  Drivetrain.driveFor(forward, 40, inches);
}

// Blue Rightside Auton Win Point
void blueRightMAIN() {
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
  wait(150, msec);
  // Drive toward and clamp goal
  newHeading(325);
  armMotor.stop();
  Drivetrain.setDriveVelocity(25, percent);
  Drivetrain.driveFor(reverse, 40, inches, false);
  wait(1300, msec);
  clampPneum.set(false);
  wait(200, msec);
  // Drive towards second ring and put it on goal
  Drivetrain.setDriveVelocity(30, percent);
  newHeading(156);
  conveyorMotorGroup.spin(forward);
  Drivetrain.drive(forward);
  wait(150, msec);
  // Drive to second ring in an arc
  RightDriveSmart.setVelocity(10, percent);
  LeftDriveSmart.setVelocity(35, percent);
  RightDriveSmart.spin(forward);
  LeftDriveSmart.spin(forward);
  wait(700, msec);
  RightDriveSmart.setVelocity(30, percent);
  LeftDriveSmart.setVelocity(30, percent);
  wait(400, msec);
  RightDriveSmart.setVelocity(10, percent);
  LeftDriveSmart.setVelocity(35, percent);
  wait(200, msec);
  RightDriveSmart.stop();
  LeftDriveSmart.stop();
  RightDriveSmart.setVelocity(30, percent);
  LeftDriveSmart.setVelocity(30, percent);
  Drivetrain.driveFor(forward, 8, inches, true);
  wait(700, msec);
  // Drive towards third ring and put it on goal
  newHeading(316);
  conveyorMotorGroup.spin(forward);
  Drivetrain.driveFor(forward, 17, inches, true);
  wait(200, msec);
  newHeading(14);
  intakePneum.set(true);
  Drivetrain.setDriveVelocity(40, percent);
  // Drive towards ladder
  Drivetrain.driveFor(forward, 40, inches, false);
  wait(550, msec);
  conveyorMotorGroup.stop();
  wait(900, msec);
  intakePneum.set(false);
}

// Blue Rightside Auton Goal Fill
void blueRightALT() {
  Drivetrain.setDriveVelocity(15, percent);
  Drivetrain.setTurnVelocity(12, percent);
  // Put ring on alliance stake
  // Get first ring
  newHeading(61);
  intakePneum.set(true);
  intakeMotor.spin(forward);
  Drivetrain.driveFor(forward, 10, inches, true);
  wait(100, msec);
  Drivetrain.driveFor(reverse, 8, inches, false);
  wait(100, msec);
  intakePneum.set(false);
  wait(150, msec);
  // Drive toward and clamp goal
  newHeading(328);
  armMotor.stop();
  Drivetrain.setDriveVelocity(25, percent);
  Drivetrain.driveFor(reverse, 40, inches, false);
  wait(1300, msec);
  clampPneum.set(false);
  wait(200, msec);
  // Drive towards second ring and put it on goal
  Drivetrain.setDriveVelocity(30, percent);
  newHeading(160);
  conveyorMotorGroup.spin(forward);
  Drivetrain.drive(forward);
  wait(200, msec);
  // Drive to third ring in an arc
  RightDriveSmart.setVelocity(8, percent);
  LeftDriveSmart.setVelocity(35, percent);
  RightDriveSmart.spin(forward);
  LeftDriveSmart.spin(forward);
  wait(700, msec);
  RightDriveSmart.setVelocity(30, percent);
  LeftDriveSmart.setVelocity(30, percent);
  wait(400, msec);
  RightDriveSmart.setVelocity(10, percent);
  LeftDriveSmart.setVelocity(35, percent);
  wait(200, msec);
  RightDriveSmart.stop();
  LeftDriveSmart.stop();
  RightDriveSmart.setVelocity(30, percent);
  LeftDriveSmart.setVelocity(30, percent);
  Drivetrain.driveFor(forward, 7, inches, true);
  wait(200, msec);
  // Drive towards fourth ring and put it on goal
  newHeading(316);
  Drivetrain.setDriveVelocity(30, percent);
  conveyorMotorGroup.spin(forward);
  Drivetrain.driveFor(forward, 18, inches, true);
  wait(200, msec);
  newHeading(295);
  // Drive towards corner rings
  RightDriveSmart.setVelocity(50, percent);
  LeftDriveSmart.setVelocity(50, percent);
  RightDriveSmart.spin(forward);
  LeftDriveSmart.spin(forward);
  wait(900, msec);
  RightDriveSmart.setVelocity(30, percent);
  LeftDriveSmart.setVelocity(15, percent);
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