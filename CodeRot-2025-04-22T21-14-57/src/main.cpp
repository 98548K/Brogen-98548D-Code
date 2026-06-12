/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Brogen Heaton                                             */
/*    Created:      Fri Jun 07 2024                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Drivetrain           drivetrain    1, 2, 3, 4, 5   
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
using namespace vex;

competition Competition;
// Functions
void onevent_Controller1ButtonB_pressed();
event autonSelector_pressed;
event autonSelector_2_pressed;
event delaySelector;
event armSelector;

// Variables
bool distance1;
bool conveyorBool;
bool runArm = true;
int autonSelector = 0;
int autonSelector_2 = 0;
int delay = 0;
double inertialNum;

// Function for turning more acurately
void newHeading(double newHeading__Degrees) {
  Drivetrain.turnToHeading(newHeading__Degrees, degrees, false);
  Drivetrain.setStopping(hold);
  waitUntil((Drivetrain.heading() >= newHeading__Degrees - 2.0 && Drivetrain.heading() <= newHeading__Degrees + 2.0));
  Drivetrain.stop();
  wait(250, msec);
  Drivetrain.setStopping(coast);
}

void velociDrive(directionType direct, double distance_num) {
  Drivetrain.setStopping(brake);
  int new_velocity = 15;
  int wait_time = 0;
  Drivetrain.drive(direct);
  for(int i = 0; i < distance_num; i++) {
    if (new_velocity < 60){
      new_velocity += 1;
    } else {
      wait_time += 1;
    }
    if ((new_velocity - 15) / 2.5 < 13) {
      wait(20 - ((new_velocity - 15) / 2.5), msec);
    } else {
      wait(7, msec);
    }
    Drivetrain.setDriveVelocity(new_velocity, percent);
    Drivetrain.drive(direct);
  }
  wait_time -= 2;
  for(int i = 0; i < distance_num; i++) {
    if (wait_time > 0) {
      wait_time -= 1;
    } else if (new_velocity > 15){
      new_velocity -= 1;
    }
    if ((new_velocity - 15) / 2.5 < 13) {
      wait(20 - ((new_velocity - 15) / 2.5), msec);
    } else {
      wait(7, msec);
    }
    Drivetrain.setDriveVelocity(new_velocity, percent);
    Drivetrain.drive(direct);
  }
  Drivetrain.stop();
}

class PID {
private:
  double Kp;
  double Ki;
  double Kd;
  double Tp;
  double Ti;
  double integral;
  double derivative;
  double previous_error;
  double integral_error;

public:
  double speed = 0;
  int error_cycle = 0;

  void setConstants(double prop, double integ, double deriv, double integral_tolerance, double precision_tolerance) {
    Kp = prop;
    Ki = integ;
    Kd = deriv;
    Ti = integral_tolerance;
    Tp = precision_tolerance;
    integral = 0;
  }

  void calcSpeed(double error) {    // The error represents the distance left to be traveled, which gets smaller as the sensor gets closer to the correct reading.
    integral += error;    // The integral represents the accumulation of all previous errors, which helps when the value of movement is really small.
    if (error < 0) {
      integral_error = -(error);
    } else {
      integral_error = error;
    }
    if (integral_error > Ti || integral_error < Tp) {    // If the absolute value of the error is greater than the integral tolerance or less than the error tolerance, the integral will be set to zero.
      integral = 0;
    }
    derivative = error - previous_error;    // The derivative is a prediction of what the next error will be. This is used to slow down the motor if it predicts that it will overshoot.
    previous_error = error;
    speed = (Kp * error) + (Ki * integral) + (Kd * derivative);   // Multiplies all the values by their constant and adds them together, producing the speed the motor shoule be moving at.
    if (error < Tp && error > -(Tp)) {   // Increases his value everytime the error is calculated to be within the tolerance of zero. Used to set a short timer to make sure the position doesn't overshoot.
      error_cycle += 1;
    } else {
      error_cycle = 0;
    }
  }
};

void arcDrive(directionType direct, double directionDegrees, double distanceMillimeters) {
  double speed_limit = 30;
  double rightTracking = 0;
  double leftTracking = 0;
  // Reset rotations sensors
  rightDriveRot.resetPosition();
  leftDriveRot.resetPosition();
  if (direct == reverse) {
    distanceMillimeters = -(distanceMillimeters);
  }
  PID rightDriveSpecs;
  rightDriveSpecs.setConstants(0.16, 0.0002, 1.2, 200, 5.0);
  PID leftDriveSpecs;
  leftDriveSpecs.setConstants(0.16, 0.0002, 1.2, 200, 5.0);
  // Distance from tracking wheel to tracking center is 4.25 inches, or 107.95 mm.
  double rightDriveDist = 0;
  double leftDriveDist = 0;
  double rightDriveSpeed = 0;
  double leftDriveSpeed = 0;
  double driveSpeedDif = 0;
  bool reverseTurnDir = false;
  double directionFacing = Drivetrain.heading();
  double distToTurn = directionDegrees - directionFacing;
  if (directionDegrees > 270 && directionFacing < 90) {   // If robot needs to drive over degree 0, gets the absolute distance in degrees traveled
    distToTurn = (directionDegrees - 180) - (directionFacing + 180);
  } else if (directionFacing > 270 && directionDegrees < 90) {    // If robot needs to drive over degree 0, gets the absolute distance in degrees traveled
    distToTurn = (directionDegrees + 180) - (directionFacing - 180);
  } else if ((directionDegrees - directionFacing) > 180) {    // If error is greater than 180, makes it drive the opposite direction
    distToTurn = -(360 - distToTurn);
  } else if (distToTurn < -180) {   // If error is less than 180, makes it drive the opposite direction
    distToTurn += 360;
  }
  if (distToTurn < 0) {
    reverseTurnDir = true;
    distToTurn = -distToTurn;
  }
  if (distToTurn > 1) {
    // Use math to calculate how far each side has to drive in order to complete the arc turn at the right distance and angle
    distToTurn *= (3.141592 / 180);   // Convert degrees to radians
    double rad = distanceMillimeters / distToTurn;    // This calculates the radius from the arc to the center of its respective circle
    if (reverseTurnDir) {
      rightDriveDist = (rad + 107.95) * distToTurn;   // This calculates how far each tracking wheel if from the center of the robot, and gives a distance for how far that side needs to drive for the arc
      leftDriveDist = (rad - 107.95) * distToTurn;
    } else {
      rightDriveDist = (rad - 107.95) * distToTurn;
      leftDriveDist = (rad + 107.95) * distToTurn;
    }
  } else {
    rightDriveDist = distanceMillimeters;
    leftDriveDist = distanceMillimeters;
  }
  //PID turnSpecs;
  //turnSpecs.setConstants(0.42, 0.006, 0.8, 40, 2.0);
  while (true) {
    // Getting the value of the tracking wheels for each side.
    rightTracking = rightDriveRot.position(turns);
    leftTracking = leftDriveRot.position(turns);
    // Multiply the number of turns of the tracking wheels by their circumference to get distance traveled.
    rightTracking *= 159.593;
    leftTracking *= 159.593;
    // Calculate speed for right and left sides respectively
    rightDriveSpecs.calcSpeed(rightDriveDist - rightTracking);
    rightDriveSpeed = rightDriveSpecs.speed;
    leftDriveSpecs.calcSpeed(leftDriveDist - leftTracking);
    leftDriveSpeed = leftDriveSpecs.speed;
    driveSpeedDif = (rightDriveSpeed - leftDriveSpeed) / 1.4;
    // Set speed limit and account for speed values that might be above the limit
    if (rightDriveSpeed > speed_limit) {
      rightDriveSpeed = speed_limit + driveSpeedDif;
    } else if (rightDriveSpeed < -speed_limit) {
      rightDriveSpeed = -speed_limit + driveSpeedDif;
    }
    if (leftDriveSpeed > speed_limit) {
      leftDriveSpeed = speed_limit - driveSpeedDif;
    } else if (leftDriveSpeed < -speed_limit) {
      leftDriveSpeed = -speed_limit - driveSpeedDif;
    }
    // Implement the speed and tell the robot to drive
    RightDriveSmart.setVelocity(rightDriveSpeed, percent);
    LeftDriveSmart.setVelocity(leftDriveSpeed, percent);
    RightDriveSmart.spin(forward);
    LeftDriveSmart.spin(forward);
    // Gives the robot a condition to break the loop
    if (rightDriveSpecs.error_cycle > 5 && leftDriveSpecs.error_cycle > 5) {
      break;
    }
    wait(10, msec);
  }
  RightDriveSmart.stop();
  LeftDriveSmart.stop();
}


void arcDriveCont(directionType direct, double directionArray[], double distanceArray[]) {
  double speed_limit = 50;
  int count = 0;
  while (true) {
    double rightTracking = 0;
    double leftTracking = 0;
    // Reset rotations sensors
    rightDriveRot.resetPosition();
    leftDriveRot.resetPosition();
    if (direct == reverse) {
      distanceArray[count] = -(distanceArray[count]);
    }
    PID rightDriveSpecs;
    rightDriveSpecs.setConstants(0.16, 0.0002, 1.2, 200, 10.0);
    PID leftDriveSpecs;
    leftDriveSpecs.setConstants(0.16, 0.0002, 1.2, 200, 10.0);
    // Distance from tracking wheel to tracking center is 4.25 inches, or 107.95 mm.
    double rightDriveDist = 0;
    double leftDriveDist = 0;
    double rightDriveSpeed = 0;
    double leftDriveSpeed = 0;
    double driveSpeedDif = 0;
    bool reverseTurnDir = false;
    double directionFacing = Drivetrain.heading();
    double distToTurn = directionArray[count] - directionFacing;
    if (directionArray[count] > 270 && directionFacing < 90) {   // If robot needs to drive over degree 0, gets the absolute distance in degrees traveled
      distToTurn = (directionArray[count] - 180) - (directionFacing + 180);
    } else if (directionFacing > 270 && directionArray[count] < 90) {    // If robot needs to drive over degree 0, gets the absolute distance in degrees traveled
      distToTurn = (directionArray[count] + 180) - (directionFacing - 180);
    } else if ((directionArray[count] - directionFacing) > 180) {    // If error is greater than 180, makes it drive the opposite direction
      distToTurn = -(360 - distToTurn);
    } else if (distToTurn < -180) {   // If error is less than 180, makes it drive the opposite direction
      distToTurn += 360;
    }
    if (distToTurn < 0) {
      reverseTurnDir = true;
      distToTurn = -distToTurn;
    }
    if (distToTurn > 1) {
      // Use math to calculate how far each side has to drive in order to complete the arc turn at the right distance and angle
      distToTurn *= (3.141592 / 180);   // Convert degrees to radians
      double rad = (distanceArray[count] / 2) / distToTurn;    // This calculates the radius from the arc to the center of its respective circle
      if (reverseTurnDir) {
        rightDriveDist = (rad + 107.95) * distToTurn;   // This calculates how far each tracking wheel if from the center of the robot, and gives a distance for how far that side needs to drive for the arc
        leftDriveDist = (rad - 107.95) * distToTurn;
      } else {
        rightDriveDist = (rad - 107.95) * distToTurn;
        leftDriveDist = (rad + 107.95) * distToTurn;
      }
    } else {
      rightDriveDist = distanceArray[count] / 2;
      leftDriveDist = distanceArray[count] / 2;
    }
    //PID turnSpecs;
    //turnSpecs.setConstants(0.42, 0.006, 0.8, 40, 2.0);
    while (true) {
      // Getting the value of the tracking wheels for each side.
      rightTracking = rightDriveRot.position(turns);
      leftTracking = leftDriveRot.position(turns);
      // Multiply the number of turns of the tracking wheels by their circumference to get distance traveled.
      rightTracking *= 159.593;
      leftTracking *= 159.593;
      // Calculate speed for right and left sides respectively
      rightDriveSpecs.calcSpeed(rightDriveDist - rightTracking);
      rightDriveSpeed = rightDriveSpecs.speed;
      leftDriveSpecs.calcSpeed(leftDriveDist - leftTracking);
      leftDriveSpeed = leftDriveSpecs.speed;
      driveSpeedDif = (rightDriveSpeed - leftDriveSpeed);
      // Set speed limit and account for speed values that might be above the limit
      if (distanceArray[count+1] > 0) {
        if (direct == forward) {
          rightDriveSpeed = speed_limit + driveSpeedDif;
          leftDriveSpeed = speed_limit - driveSpeedDif;
        } else {
          rightDriveSpeed = -speed_limit - driveSpeedDif;
          leftDriveSpeed = -speed_limit + driveSpeedDif;
        }
        // Gives the robot a condition to break the loop
        if (rightDriveDist - rightTracking < 20 && leftDriveDist - leftTracking < 20) {
          break;
        }
      } else {
        if (rightDriveSpeed > speed_limit) {
          rightDriveSpeed = speed_limit + driveSpeedDif;
        } else if (rightDriveSpeed < -speed_limit) {
          rightDriveSpeed = -speed_limit - driveSpeedDif;
        }
        if (leftDriveSpeed > speed_limit) {
          leftDriveSpeed = speed_limit - driveSpeedDif;
        } else if (leftDriveSpeed < -speed_limit) {
          leftDriveSpeed = -speed_limit + driveSpeedDif;
        }
        // Gives the robot a condition to break the loop
        if (rightDriveSpecs.error_cycle > 5 && leftDriveSpecs.error_cycle > 5) {
          break;
        }
      }
      // Implement the speed and tell the robot to drive
      RightDriveSmart.setVelocity(rightDriveSpeed, percent);
      LeftDriveSmart.setVelocity(leftDriveSpeed, percent);
      RightDriveSmart.spin(forward);
      LeftDriveSmart.spin(forward);
      wait(10, msec);
    }
    Controller1.Screen.setCursor(3, 17);
    Controller1.Screen.print(directionArray[count]);
    count++;
    if (directionArray[count] < 0) {
      break;
    }
  }
  RightDriveSmart.stop();
  LeftDriveSmart.stop();
}


int whenStarted1() {
  // Set task priorities 1 - 15
  task_ringSorter.setPriority(10);
  task_brainScreenPrint.setPriority(5);
  // Configurate robot settings
  Brain.Screen.render(true, false);
  conveyorMotorGroup.setVelocity(100, percent);
  intakePneum.set(false);
  cornerPneum.set(false);
  armMotor.setVelocity(100, percent);
  armMotor.setStopping(hold);
  return 0;
}

int onauton_autonomous_0() {
  clampPneum.set(true);
  inertialNum = Drivetrain.heading();
  wait(50, msec);
  if (Drivetrain.heading() < 359 && Drivetrain.heading() > 1) {
    Brain.Screen.renderDisable();
    Brain.Screen.clearScreen();
    Brain.Screen.print("INERTIAL SENSOR ERROR          ");
    vexcodeInit();
  }
  wait(delay, sec);
  if (runArm) {
    armMotor.spin(reverse);
    wait(600, msec);
    armMotor.spin(forward);
  }
  if (autonSelector == 0 && autonSelector_2 == 0) {
    // Blue Leftside Auton Win Point
    blueLeftMAIN();
  } else if (autonSelector == 0 && autonSelector_2 == 1) {
    // Blue Leftside Auton Goal Rush
    blueLeftALT();
  } else if (autonSelector == 1 && autonSelector_2 == 0) {
    // Blue Rightside Auton Win Point
    blueRightMAIN();
  } else if (autonSelector == 1 && autonSelector_2 == 1) {
    // Blue Rightside Goal Fill
    blueRightALT();
  } else if (autonSelector == 2 && autonSelector_2 == 0) {
    // Red Leftside Auton Win Point
    redLeftMAIN();
  } else if (autonSelector == 2 && autonSelector_2 == 1) {
    // Red Leftside Goal Fill
    redLeftALT();
  } else if (autonSelector == 3 && autonSelector_2 == 0) {
    // Red Rightside Auton Win Point
    redRightMAIN();
  } else if (autonSelector == 3 && autonSelector_2 == 1) {
    // Red Rightside Auton Goal Rush
    redRightALT();
  } else if (autonSelector == 4) {
    // No auton
  }
  return 0;
}


int ondriver_drivercontrol_0() {
  task_ringSorter.stop();
  ArmReset = false;
  RightDriveSmart.setVelocity(100, percent);
  LeftDriveSmart.setVelocity(100, percent);
  Drivetrain.stop();
  conveyorMotorGroup.stop();
  Drivetrain.setStopping(coast);
  Drivetrain.setDriveVelocity(75, percent);
  onevent_Controller1ButtonB_pressed();
  return 0;
}


void VEXcode_driver_task() {
  // Start the driver control tasks....
  vex::task drive0(ondriver_drivercontrol_0);
  while(Competition.isDriverControl() && Competition.isEnabled()) {this_thread::sleep_for(10);}
  drive0.stop();
  return;
}

void VEXcode_auton_task() {
  // Start the auton control tasks....
  vex::task auto0(onauton_autonomous_0);
  while(Competition.isAutonomous() && Competition.isEnabled()) {this_thread::sleep_for(10);}
  auto0.stop();
  return;
}

// Function for A button pressed
void onevent_Controller1ButtonA_pressed() {
  // Clamp Toggle
  if (clampPneum.value()) {
    clampPneum.set(false);
  } else {
    clampPneum.set(true);
  }
}

// Function for B button pressed
void onevent_Controller1ButtonB_pressed() {
  // Reset rotationSes
  if (!ArmReset) {
    ArmReset = true;
    wait(50, msec);
    armMotor.setVelocity(90, percent);
    armMotor.spin(forward);
    wait(800, msec);
    rotationSens.setPosition(359, degrees);
    armMotor.stop();
    wait(50, msec);
    ArmReset = false;
    Controller1.rumble("--");
  }
}

// Function for Down button pressed
void onevent_Controller1ButtonDown_pressed() {
  // Claw Toggle
  if (intakePneum.value()) {
    intakePneum.set(false);
  }
  else {
    intakePneum.set(true);
  }
}

// Function for Left button pressed
void onevent_Controller1ButtonLeft_pressed() {
  if (cornerPneum.value()) {
    cornerPneum.set(false);
  } else {
    cornerPneum.set(true);
  }
}

// Function for Right button pressed
void onevent_Controller1ButtonRight_pressed() {
  if (clawPneum.value()) {
    clawPneum.set(false);
  } else {
    clawPneum.set(true);
  }
}

// Function for Up button pressed
void onevent_Controller1ButtonUp_pressed() {
  if (ArmSet) {
    ArmSet = false;
  } else {
    ArmSet = true;
    PID specs;
    specs.setConstants(0.7, 0.0, 0.9, 20, 2);
    double rotation_pos = 215;
    while (ArmSet) {
      specs.calcSpeed(rotation_pos - rotationSens.position(degrees));
      armMotor.setVelocity(specs.speed, percent);
      armMotor.spin(forward);
      if (specs.error_cycle > 20) {
        break;
      }
      wait(5, msec);
    }
    armMotor.stop();
  }
}

// Function for R1 button pressed
void onevent_Controller1ButtonR1_pressed() {
  conveyorMotorGroup.setVelocity(30, percent);
  conveyorMotorGroup.spin(reverse);
  wait(200, msec);
  conveyorMotorGroup.stop();
  conveyorMotorGroup.setVelocity(95, percent);
}

// Function for R2 button pressed
void onevent_Controller1ButtonR2_pressed() {
  if (ArmSet) {
    ArmSet = false;
  } else {
    ArmSet = true;
    PID specs;
    specs.setConstants(0.7, 0.0, 0.9, 40, 2);
    double rotation_pos = 331;
    if (rotationSens.position(degrees) < 329) {
      rotation_pos = 329;
    }
    while (ArmSet) {
      specs.calcSpeed(rotation_pos - rotationSens.position(degrees));
      armMotor.setVelocity(specs.speed, percent);
      armMotor.spin(forward);
      if (specs.error_cycle > 20) {
        break;
      }
      wait(5, msec);
    }
    armMotor.stop();
  }
}

// Function for cycling main auton selector
void onevent_autonSelector_pressed() {
  if (autonSelector < 4) {
    autonSelector += 1;
  } else {
    autonSelector = 0;
  }
  wait(400, msec);
}

// Function for cycling secondary auton selector
void onevent_autonSelector_2_pressed() {
  if (autonSelector_2 < 1) {
    autonSelector_2 += 1;
  } else {
    autonSelector_2 = 0;
  }
  wait(400, msec);
}

void onevent_delaySelector() {
  if (delay < 3) {
    delay += 1;
  } else {
    delay = 0;
  }
  wait(400, msec);
}

void onevent_armSelector() {
  if (runArm) {
    runArm = false;
  } else {
    runArm = true;
  }
  wait(400, msec);
}


int main() {
  vex::competition::bStopTasksBetweenModes = false;
  Competition.autonomous(VEXcode_auton_task);
  Competition.drivercontrol(VEXcode_driver_task);
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  // Function activators
  autonSelector_pressed(onevent_autonSelector_pressed);
  autonSelector_2_pressed(onevent_autonSelector_2_pressed);
  delaySelector(onevent_delaySelector);
  armSelector(onevent_armSelector);
  Controller1.ButtonA.pressed(onevent_Controller1ButtonA_pressed);
  Controller1.ButtonB.pressed(onevent_Controller1ButtonB_pressed);
  Controller1.ButtonDown.pressed(onevent_Controller1ButtonDown_pressed);
  Controller1.ButtonLeft.pressed(onevent_Controller1ButtonLeft_pressed);
  Controller1.ButtonRight.pressed(onevent_Controller1ButtonRight_pressed);
  Controller1.ButtonUp.pressed(onevent_Controller1ButtonUp_pressed);
  Controller1.ButtonX.pressed(onevent_Controller1ButtonUp_pressed);
  Controller1.ButtonR1.pressed(onevent_Controller1ButtonR1_pressed);
  Controller1.ButtonR2.pressed(onevent_Controller1ButtonR2_pressed);


  // wait for rotation sensor to fully initialize
  wait(30, msec);

  whenStarted1();
}
