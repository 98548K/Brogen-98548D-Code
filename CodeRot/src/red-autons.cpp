#include "vex.h"

using namespace vex;

// Red Stackside Auton Win Point
void redLeftMAIN() {
  // Put ring on alliance stake
  // Get first ring
}

// Red Stackside Auton Goal Fill
void redLeftALT() {
  Drivetrain.setHeading(114, deg);
  // Put ring on alliance stake
  // Get first ring
  arcDrive(forward, 45, 0);
  intakePneum.set(true);
  intakeMotor.spin(forward);
  arcDrive(forward, 45, 200);
  intakePneum.set(false);
  wait(100, msec);
  // Clamp goal
  arcDrive(reverse, 170, 900);
  Drivetrain.setDriveVelocity(40, percent);
  Drivetrain.drive(reverse);
  wait(300, msec);
  clampPneum.set(false);
  wait(50, msec);
  Drivetrain.stop();
  // Drive towards second and third rings and put them on goal
  conveyorMotorGroup.spin(forward);
  arcDrive(forward, 335, 0);
  double Deg[] = {319.2,280.1,270.1,263.2,224.9,174.6,160.7,140,-1};
  double Dist[] = {5.7,17.6,26.7,20.1,25.2,8.4,25.7,330,-1};
  arcDriveCont(forward, Deg, Dist);
  wait(200, msec);
  // Drive towards corner rings
  arcDrive(forward, 226, 840);
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
  arcDrive(reverse, 225, 400);
  double Deg1[] = {45,45,90,90,-1};
  double Dist1[] = {10,750,800,50,-1};
  arcDriveCont(forward, Deg1, Dist1);
}

// Red Goalside Auton Win Point
void redRightMAIN() {
  Drivetrain.setHeading(248, deg);
  // Put ring on alliance stake
  // Get first ring
  arcDrive(forward, 315, 0);
  intakePneum.set(true);
  intakeMotor.spin(forward);
  arcDrive(forward, 315, 200);
  intakePneum.set(false);
  wait(100, msec);
  // Clamp goal
  arcDrive(reverse, 195, 950);
  Drivetrain.setDriveVelocity(40, percent);
  Drivetrain.drive(reverse);
  wait(300, msec);
  clampPneum.set(false);
  wait(50, msec);
  Drivetrain.stop();
  // Drive towards second ring and put it on goal
  arcDrive(forward, 86, 0);
  conveyorMotorGroup.spin(forward);
  arcDrive(forward, 86, 100);
  // Drive to corner
  double Deg[] = {150,180,145,-1};
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
  arcDrive(reverse, 225, 450);
  clampPneum.set(true);
  conveyorMotorGroup.stop();
  armMotor.spin(reverse);
  arcDrive(forward, 330, 1250);
}

// Red Goalside Auton Goal Rush
void redRightALT() {
  // Put ring on alliance stake
  // Get first ring
}

// Red Solo Sig Win Point
void redSig() {
  
}