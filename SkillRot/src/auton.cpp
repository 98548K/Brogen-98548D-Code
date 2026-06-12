#include "vex.h"

using namespace vex;

void auton() {
  // Place ring on alliance stake
  ArmReset = true;
  armMotor.setVelocity(100, percent);
  armMotor.spin(reverse);
  wait(700, msec);
  ArmReset = false;
  wait(150, msec);
  armRotationReset.broadcast();
  // Drive to and clamp first goal
  arcDrive(reverse, 0, 275, 0.9, 1);
  clampPneum.set(false);
  // Drive for first ring
  armPosition.broadcast();
  arcDrive(forward, 212, 0, 1, 1);
  conveyorMotorGroup.spin(forward);
  arcDrive(forward, 212, 650, 1, 1);
  // Head straight to second rings
  //arcDrive(forward, 131, 0, 1, 1);
  //arcDrive(forward, 131, 500, 1, 1);
  // Score first wall stake
  arcDrive(forward, 173, 0, 1, 1);
  conveyorMotorGroup.spin(reverse);
  wait(150, msec);
  conveyorMotorGroup.stop();
  //arcDrive(forward, 169, 725, 1, 1);
  intakeMotor.spin(forward);
  arcDrive(forward, 137, 950, 1, 1);
  Drivetrain.setDriveVelocity(25, percent);
  Drivetrain.drive(forward);
  wait(100, msec);
  armMotor.setVelocity(100, percent);
  armMotor.spin(reverse);
  wait(400, msec);
  conveyorMotorGroup.spin(forward);
  wait(200, msec);
  Drivetrain.drive(reverse);
  wait(475, msec);
  Drivetrain.stop();
  armMotor.stop();
  ArmSet = false;
  // Drive for second, third, and fourth rings
  arcDrive(forward, 43, 0, 1, 1);
  arcDrive(forward, 43, 1175, 1, 1);
  wait(100, msec);
  arcDrive(reverse, 43, 275, 1, 1);
  // Drive for fifth ring
  arcDrive(forward, 134, 0, 1, 1);
  arcDrive(forward, 134, 275, 1, 1);
  arcDrive(reverse, 134, 175, 1, 1);
  // Place goal in corner
  arcDrive(forward, 260, 0, 1, 1);
  arcDrive(reverse, 260, 260, 1, 1);
  clampPneum.set(true);
  conveyorMotorGroup.stop();
  // Drive to second goal
  arcDrive(forward, 255, 145, 1, 1);
  arcDrive(forward, 139, 0, 1, 1);
  arcDrive(reverse, 139, 1750, 1, 1);
  clampPneum.set(false);
  // Drive for first ring
  arcDrive(forward, 225, 0, 1, 1);
  conveyorMotorGroup.spin(forward);
  arcDrive(forward, 225, 525, 1, 1);
  // Drive for second ring
  arcDrive(forward, 313, 0, 1, 1);
  arcDrive(forward, 313, 550, 1, 1);
  // Drive for third and fourth rings
  arcDrive(forward, 41, 0, 1, 1);
  arcDrive(forward, 41, 800, 1, 1);
  arcDrive(reverse, 41, 250, 1, 1);
  // Drive for fifth ring
  arcDrive(forward, 313, 0, 1, 1);
  arcDrive(forward, 313, 250, 1, 1);
  // Place goal in corner
  arcDrive(forward, 207, 0, 1, 1);
  arcDrive(reverse, 207, 175, 1, 1);
  conveyorMotorGroup.stop();
  clampPneum.set(true);
  // Drive across field and set arm
  //arcDrive(forward, 205, 0, 1, 1);
  //arcDrive(forward, 195, 1300, 1, 1);
  //armPosition.broadcast();
  //conveyorMotorGroup.spin(forward);
  //arcDrive(forward, 313, 1500, 1, 1);
  arcDrive(forward, 207, 1375, 1, 1);
  // Pick up ring
  /* Drivetrain.setDriveVelocity(25, percent);
  Drivetrain.drive(forward);
  wait(300, msec);
  Drivetrain.stop();
  wait(1500, msec);
  Drivetrain.drive(forward);
  conveyorMotorGroup.spin(reverse);
  wait(150, msec);
  conveyorMotorGroup.stop();
  // Place ring on wall stake
  armMotor.setVelocity(100, percent);
  armMotor.spin(reverse);
  wait(700, msec);
  Drivetrain.drive(reverse);
  wait(600, msec);
  Drivetrain.stop();
  ArmSet = false; */
  // Turn towards next ring
  intakeMotor.spin(forward);
  arcDrive(forward, 176, 0, 1, 1);
  // Pick up rinG
  arcDrive(forward, 176, 925, 1, 1);
  // Clamp third goal
  arcDrive(forward, 64, 0, 1, 1);
  arcDrive(reverse, 64, 550, 1, 1);
  clampPneum.set(false);
  // Place third goal in corner
  arcDrive(forward, 127, 0, 1, 1);
  clampPneum.set(true);
  arcDrive(reverse, 127, 675, 1, 1);
  arcDrive(forward, 127, 725, 1, 1);
  // Clamp fourth goal
  arcDrive(forward, 285, 0, 1, 1);
  arcDrive(reverse, 285, 400, 0.8, 1);
  clampPneum.set(false);
  // Place first ring on goal
  conveyorMotorGroup.spin(forward);
  // Drive to second ring
  arcDrive(forward, 88, 0, 1, 1);
  arcDrive(forward, 133, 1400, 1, 1.1);
  // Drive to third ring
  //arcDrive(forward, 133, 0, 1, 1);
  //arcDrive(forward, 133, 600, 1, 1);
  // Drive to fourth and fifth rings
  arcDrive(forward, 223, 0, 1, 1);
  arcDrive(forward, 223, 825, 1, 1);
  // Place goal in corner
  arcDrive(forward, 338, 0, 1, 1);
  conveyorMotorGroup.stop();
  clampPneum.set(true);
  arcDrive(reverse, 338, 250, 1, 1);
  arcDrive(forward, 338, 300, 1, 1);
  // Elevate on ladder
  ArmReset = true;
  armMotor.spin(reverse);
  arcDrive(forward, 186, 0, 1, 1);
  Drivetrain.setDriveVelocity(45, percent);
  Drivetrain.setStopping(coast);
  Drivetrain.driveFor(reverse, 45, inches, true);
  wait(100, msec);
  Drivetrain.setDriveVelocity(10, percent);
  Drivetrain.driveFor(forward, 2, inches, true);
}