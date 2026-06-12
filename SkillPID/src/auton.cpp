#include "vex.h"

using namespace vex;

void auton() {
  // Place ring on alliance stake
  ArmReset = true;
  armMotor.setVelocity(100, percent);
  armMotor.spin(reverse);
  wait(700, msec);
  ArmReset = false;
  armRotationReset.broadcast();
  // Drive to and clamp first goal
  arcDrive(reverse, 0, 250, 1, 1);
  clampPneum.set(false);
  // Get first, second, third, and fourth rings
  arcDrive(forward, 230, 0, 1, 1);
  conveyorMotorGroup.spin(forward);
  double arcMatrix[][4] = {{140,600,1.2,0.7}, {208,375,1.22,1}, {210,1100,1,1}, {-1,0,0,0}};
  arcDriveArray(forward, arcMatrix);
  wait(300, msec);
  // Get first wall stake ring
  arcDrive(forward, 148, 0, 1, 1);
  armPosition.broadcast();
  arcDrive(forward, 148, 300, 1, 1);
  wait(100, msec);
  intakeMotor.spin(reverse);
  arcDrive(forward, 20, 0, 1, 1);
  conveyorMotorGroup.stop();
  conveyorMotorGroup.spin(reverse);
  wait(125, msec);
  conveyorMotorGroup.stop();
  // Get second wall stake ring and align with wall stake
  intakeMotor.spin(forward);
  arcDrive(forward, 135, 1200, 1.2, 1);
  Drivetrain.setDriveVelocity(18, percent);
  Drivetrain.drive(forward);
  wait(100, msec);
  // Place first ring on wall stake
  armMotor.setVelocity(100, percent);
  armMotor.spin(reverse);
  wait(800, msec);
  Drivetrain.drive(reverse);
  wait(400, msec);
  ArmSet = false;
  wait(100, msec);
  Drivetrain.stop();
  // Place second ring on wall stake
  /*
  armPosition.broadcast();
  wait(800, msec);
  intakeMotor.stop();
  conveyorMotorGroup.spin(forward);
  wait(800, msec);
  conveyorMotorGroup.stop();
  conveyorMotorGroup.spin(reverse);
  wait(50, msec);
  conveyorMotorGroup.stop();
  Drivetrain.drive(forward);
  armMotor.setVelocity(100, percent);
  armMotor.spin(reverse);
  wait(600, msec);
  Drivetrain.drive(reverse);
  wait(400, msec);
  ArmSet = false;
  wait(100, msec);
  Drivetrain.stop();
  */
  // Put fifth and sixth rings on goal
  arcDrive(forward, 30, 0, 1, 1);
  conveyorMotorGroup.spin(forward);
  arcDrive(forward, 39, 900, 0.8, 1);
  // Store ring on conveyor
  wait(200, msec);
  conveyorStore.broadcast();
  // Store ring in intake
  arcDrive(forward, 39, 200, 1, 1);
  arcDrive(reverse, 138, 250, 1.1, 1);
  arcDrive(forward, 138, 450, 1, 1);
  // Place goal in corner
  arcDrive(forward, 258, 0, 1, 1);
  arcDrive(reverse, 258, 125, 1, 1);
  clampPneum.set(true);
  arcDrive(forward, 258, 225, 1, 1);
  // Drive to next goal
  arcDrive(forward, 132, 0, 1, 1);
  arcDrive(reverse, 132, 1700, 0.95, 1);
  clampPneum.set(false);
  // Place stored rings on goal
  conveyorMotorGroup.spin(forward);
  // Pick up third, fourth, fifth, and sixth rings
  arcDrive(forward, 190, 0, 1, 1);
  double arcMatrix2[][4] = {{300,275,1.2,0.8}, {35,0,0.8,1}, {40,900,0.8,1}, {-1,0,0,0}};
  arcDriveArray(forward, arcMatrix2);
  // Place goal in corner
  arcDrive(forward, 170, 0, 1, 1);
  arcDrive(reverse, 170, 50, 1, 1);
  clampPneum.set(false);
}