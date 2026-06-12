/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Brogen Heaton                                             */
/*    Created:      Wed Sep 11 2024                                           */
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

// Function
event armPosition;
event armRotationReset;
event conveyorStore;

// Variables
double inertialNum;
bool elevationActive = false;

// Class declaration
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
  double speed = 0.0;
  int error_cycle = 0;

  void setConstants(double prop, double integ, double deriv, double integral_tolerance, double precision_tolerance) {
    Kp = prop;
    Ki = integ;
    Kd = deriv;
    Ti = integral_tolerance;
    Tp = precision_tolerance;
    integral = 0;
  }

  void calcSpeed(double error) {
    integral += error;    // The integral represents the accumulation of all previous errors, which helps when the value of movement is really small.
    if (error < 0) {
      integral_error = -(error);
    } else {
      integral_error = error;
    }
    if (error < Tp && error > -(Tp)) {   // If error is equal to zero, set integral to zero
      integral = 0;
    }
    if (integral_error > Ti){    // If the absolute value of the error is less than the integral tolerance, the integral will be set to zero.
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

void arcDrive(directionType direct, double directionDegrees, double distanceMillimeters, double drive_velocity, double turn_velocity) {
  double rightRPM = 0;
  double leftRPM = 0;
  double distanceTraveled = 0;
  if (direct == reverse) {
    distanceMillimeters = -(distanceMillimeters);
  }
  PID driveSpecs;
  driveSpecs.setConstants(0.11, 0.0002, 0.1, 300, 25.0);
  double directionFacing = 0;
  double error = 0;
  double driveSpeed = 0;
  double turnSpeed = 0;
  PID turnSpecs;
  turnSpecs.setConstants(0.43, 0.006, 1.2, 40, 5.0);
  while (true) {
    // At max RPM (480 with gear ratio calculated) and the wheels being 3.25 inches in diameter, the drivetrain will drive 81.6814 inches in one second, according to my math. Equivalates to an 82.55mm wheel, which has a circumference of 259.3385mm, and travels 2074.7078mm per second.
    // This can be converted to 0.816814 inches per 10 milliseconds, or about 20.7471mm per 10 milliseconds.
    rightRPM = RightDriveSmart.velocity(rpm);
    leftRPM = LeftDriveSmart.velocity(rpm);
    rightRPM *= 0.8;    // Convert 600 RPM to 480
    leftRPM *= 0.8;     // Convert 600 RPM to 480
    rightRPM /= 6000;   // Rotations per 10 milliseconds
    leftRPM /= 6000;    // Rotations per 10 milliseconds
    rightRPM *= 259.3384;   // Distance traveled in millimeters per 10 milliseconds.
    leftRPM *= 259.3384;    // Distance traveled in millimeters per 10 milliseconds.
    // The previous calculations can be done by dividing the RPM by 23.1358. They have not been yet so I can better remember my math.
    // The previous calculations allow me to calculate the distance each side has traveled, though the degree of accuracy is unknown to me.
    // The next step is to use the heading, as measured by the inertial sensor, and some math to figure out how far the robot moves in an arc.
    distanceTraveled += (rightRPM + leftRPM) /2;
    driveSpecs.calcSpeed(distanceMillimeters - distanceTraveled);
    driveSpeed = driveSpecs.speed;
    directionFacing = Drivetrain.heading();
    if (directionDegrees > 270 && directionFacing < 90) {   // If robot needs to drive over degree 0, gets the absolute distance in degrees traveled
      error = (directionDegrees - 180) - (directionFacing + 180);
    } else if (directionDegrees < 90 && directionFacing > 270) {    // If robot needs to drive over degree 0, gets the absolute distance in degrees traveled
      error = (directionDegrees + 180) - (directionFacing - 180);
    } else if ((directionDegrees - directionFacing) > 180) {    // If error is greater than 180, makes it drive the opposite direction
      error = -(360 - (directionDegrees - directionFacing));
    } else if ((directionDegrees - directionFacing) < -180) {   // If error is less than 180, makes it drive the opposite direction
      error = 360 + (directionDegrees - directionFacing);
    } else {
      error = directionDegrees - directionFacing;
    }
    turnSpecs.calcSpeed(error);
    turnSpeed = turnSpecs.speed;
    if (driveSpeed > 100) {
      driveSpeed = 100;
    }
    if (turnSpeed > 100) {
      turnSpeed = 100;
    }
    if (distanceMillimeters > 0 && error < 3 && error > -3) {
      turnSpeed = 0;
    }
    driveSpeed *= drive_velocity;
    turnSpeed *= turn_velocity;
    RightDriveSmart.setVelocity(driveSpeed - turnSpeed, percent);
    LeftDriveSmart.setVelocity(driveSpeed + turnSpeed, percent);
    RightDriveSmart.spin(forward);
    LeftDriveSmart.spin(forward);
    if (driveSpecs.error_cycle > 20 && turnSpecs.error_cycle > 20) {
      break;
    }
    wait(10, msec);
  }
  RightDriveSmart.stop();
  LeftDriveSmart.stop();
}

void arcDriveArray(directionType direct, double movement_array[][4]) {
  double rightRPM = 0;
  double leftRPM = 0;
  double distanceTraveled = 0;
  double distanceMillimeters = 0;
  double directionDegrees = 0;
  double directionFacing = 0;
  double error = 0;
  double driveSpeed = 0;
  double turnSpeed = 0;
  int counter = 0;
  bool stopper = true;
  while (movement_array[counter][0] >= 0) {
    directionDegrees = movement_array[counter][0];
    distanceMillimeters = movement_array[counter][1];
    distanceTraveled = 0;
    if (direct == reverse) {
      distanceMillimeters = -(distanceMillimeters);
    }
    PID driveSpecs;
    driveSpecs.setConstants(0.11, 0.0002, 0.1, 300, 25.0);
    PID turnSpecs;
    turnSpecs.setConstants(0.43, 0.006, 1.1, 40, 8.0);
    while (stopper) {
      // At max RPM (480 with gear ratio calculated) and the wheels being 3.25 inches in diameter, the drivetrain will drive 81.6814 inches in one second, according to my math. Equivalates to an 82.55mm wheel, which has a circumference of 259.3385mm, and travels 2074.7078mm per second.
      // This can be converted to 0.816814 inches per 10 milliseconds, or about 20.7471mm per 10 milliseconds.
      rightRPM = RightDriveSmart.velocity(rpm);
      leftRPM = LeftDriveSmart.velocity(rpm);
      rightRPM *= 0.8;    // Convert 600 RPM to 480
      leftRPM *= 0.8;     // Convert 600 RPM to 480
      rightRPM /= 6000;   // Rotations per 10 milliseconds
      leftRPM /= 6000;    // Rotations per 10 milliseconds
      rightRPM *= 259.3384;   // Distance traveled in millimeters per 10 milliseconds.
      leftRPM *= 259.3384;    // Distance traveled in millimeters per 10 milliseconds.
      // The previous calculations can be done by dividing the RPM by 23.1358. They have not been yet so I can better remember my math.
      // The previous calculations allow me to calculate the distance each side has traveled, though the degree of accuracy is unknown to me.
      // The next step is to use the heading, as measured by the inertial sensor, and some math to figure out how far the robot moves in an arc.
      distanceTraveled += (rightRPM + leftRPM) /2;
      driveSpecs.calcSpeed(distanceMillimeters - distanceTraveled);
      driveSpeed = driveSpecs.speed;
      directionFacing = Drivetrain.heading();
      if (directionDegrees > 270 && directionFacing < 90) {   // If robot needs to drive over degree 0, gets the absolute distance in degrees traveled
        error = (directionDegrees - 180) - (directionFacing + 180);
      } else if (directionFacing > 270 && directionDegrees < 90) {    // If robot needs to drive over degree 0, gets the absolute distance in degrees traveled
        error = (directionDegrees + 180) - (directionFacing - 180);
      } else if ((directionDegrees - directionFacing) > 180) {    // If error is greater than 180, makes it drive the opposite direction
        error = -(360 - (directionDegrees - directionFacing));
      } else if ((directionDegrees - directionFacing) < -180) {   // If error is less than 180, makes it drive the opposite direction
        error = 360 + (directionDegrees - directionFacing);
      } else {
        error = directionDegrees - directionFacing;
      }
      turnSpecs.calcSpeed(error);
      turnSpeed = turnSpecs.speed;
      if (driveSpeed > 100) {
        driveSpeed = 100;
      }
      if (turnSpeed > 100) {
        turnSpeed = 100;
      }
      if (distanceMillimeters > 0 && error < 8 && error > -8) {
        turnSpeed = 0;
      }
      driveSpeed *= movement_array[counter][2];
      turnSpeed *= movement_array[counter][3];
      if (movement_array[counter+1][0] < 0) {
        RightDriveSmart.setVelocity(driveSpeed - turnSpeed, percent);
        LeftDriveSmart.setVelocity(driveSpeed + turnSpeed, percent);
        if (driveSpecs.error_cycle > 20 && turnSpecs.error_cycle > 20) {
          break;
        }
      } else if (distanceMillimeters == 0) {
        RightDriveSmart.setVelocity(-turnSpeed, percent);
        LeftDriveSmart.setVelocity(turnSpeed, percent);
      } else if (direct == forward) {
        RightDriveSmart.setVelocity((50 * movement_array[counter][2]) - turnSpeed, percent);
        LeftDriveSmart.setVelocity((50 * movement_array[counter][2]) + turnSpeed, percent);
      } else {
        RightDriveSmart.setVelocity((-50 * movement_array[counter][2]) - turnSpeed, percent);
        LeftDriveSmart.setVelocity((-50 * movement_array[counter][2]) + turnSpeed, percent);
      }
      RightDriveSmart.spin(forward);
      LeftDriveSmart.spin(forward);
      if (distanceMillimeters - distanceTraveled < 0 && error < 5) {
        stopper = false;
      }
      wait(10, msec);
    }
    stopper = true;
    counter++;
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
  conveyorMotorGroup.setVelocity(95, percent);
  intakePneum.set(false);
  cornerPneum.set(false);
  armMotor.setVelocity(95, percent);
  armMotor.setStopping(hold);
  return 0;
}


int onauton_autonomous_0() {
  clampPneum.set(true);
  Drivetrain.setDriveVelocity(25, percent);
  Drivetrain.setTurnVelocity(10, percent);
  conveyorMotorGroup.setVelocity(95, percent);
  inertialNum = Drivetrain.heading();
  wait(50, msec);
  if (Drivetrain.heading() < (inertialNum - 0.5) || Drivetrain.heading() > (inertialNum + 0.5)) {
    Brain.Screen.renderDisable();
    Brain.Screen.clearScreen();
    Brain.Screen.print("INERTIAL SENSOR ERROR          ");
    vexcodeInit();
  }
  // Run auton function
  auton();
  return 0;
}


int ondriver_drivercontrol_0() {
  task_ringSorter.stop();
  clampPneum.set(true);
  RightDriveSmart.setVelocity(100, percent);
  LeftDriveSmart.setVelocity(100, percent);
  Drivetrain.setStopping(coast);
  Drivetrain.setDriveVelocity(75, percent);
  ArmReset = true;
  armMotor.spin(reverse);
  wait(700, msec);
  armMotor.stop();
  ArmReset = false;
  armRotationReset.broadcast();
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

// "when Controller1 ButtonA pressed" hat block
void onevent_Controller1ButtonA_pressed() {
  // Double Wing Toggle
  if (clampPneum.value()) {
    clampPneum.set(false);
  } else {
    clampPneum.set(true);
  }
}

// "when Controller1 ButtonB pressed" hat block
void onevent_Controller1ButtonB_pressed() {
  // Reset rotationSes
  if (!ArmReset) {
    ArmReset = true;
    wait(50, msec);
    armMotor.setVelocity(95, percent);
    armMotor.spin(forward);
    wait(800, msec);
    rotationSens.setPosition(359, degrees);
    armMotor.stop();
    wait(50, msec);
    ArmReset = false;
    Controller1.rumble("--");
  }
}

// "when Controller1 ButtonX pressed" hat block
void onevent_Controller1ButtonX_pressed() {
  // Prepare for elevation
  if (!elevationActive) {
    ArmReset = true;
    elevationActive = true;
    armMotor.setVelocity(75, percent);
    armMotor.spin(reverse);
    waitUntil(rotationSens.position(degrees) < 200);
    armMotor.stop();
  } else {
    armMotor.stop();
    ArmReset = false;
    elevationActive = false;
  }
}

// "when Controller1 ButtonDown pressed" hat block
void onevent_Controller1ButtonDown_pressed() {
  // Claw Toggle
  if (intakePneum.value()) {
    intakePneum.set(false);
  }
  else {
    intakePneum.set(true);
  }
}

// "when Controller1 ButtonLeft pressed" hat block
void onevent_Controller1ButtonLeft_pressed() {
  if (cornerPneum.value()) {
    cornerPneum.set(false);
  } else {
    cornerPneum.set(true);
  }
}

// "when Controller1 ButtonRight pressed" hat block
void onevent_Controller1ButtonRight_pressed() {
  if (clawPneum.value()) {
    clawPneum.set(false);
  } else {
    clawPneum.set(true);
  }
}

// "when Controller1 ButtonR1 pressed" hat block
void onevent_Controller1ButtonR1_pressed() {
  conveyorMotorGroup.setVelocity(10, percent);
  conveyorMotorGroup.spin(reverse);
  wait(1, sec);
  conveyorMotorGroup.stop();
  conveyorMotorGroup.setVelocity(95, percent);
}

// "when Controller1 ButtonR2 pressed" hat block
void onevent_Controller1ButtonR2_pressed() {
  if (ArmSet) {
    ArmSet = false;
  } else if (!elevationActive) {
    ArmSet = true;
    PID specs;
    specs.setConstants(1.2, 0.000002, 0.6, 40, 5);
    double rotation_pos = 320;
    if (rotationSens.position(degrees) < 318) {
      rotation_pos = 318;
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

// Function for storing ring on conveyor
void onevent_conveyorStore() {
  conveyorMotorGroup.spin(forward);
  waitUntil(colorSens.value() < 20);
  conveyorMotorGroup.stop();
  intakeMotor.spin(forward);
}


int main() {
  vex::competition::bStopTasksBetweenModes = false;
  Competition.autonomous(VEXcode_auton_task);
  Competition.drivercontrol(VEXcode_driver_task);
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  // Function activators
  armPosition(onevent_Controller1ButtonR2_pressed);
  armRotationReset(onevent_Controller1ButtonB_pressed);
  conveyorStore(onevent_conveyorStore);
  Controller1.ButtonA.pressed(onevent_Controller1ButtonA_pressed);
  Controller1.ButtonB.pressed(onevent_Controller1ButtonB_pressed);
  Controller1.ButtonX.pressed(onevent_Controller1ButtonX_pressed);
  Controller1.ButtonDown.pressed(onevent_Controller1ButtonDown_pressed);
  Controller1.ButtonLeft.pressed(onevent_Controller1ButtonLeft_pressed);
  Controller1.ButtonR1.pressed(onevent_Controller1ButtonR1_pressed);
  Controller1.ButtonR2.pressed(onevent_Controller1ButtonR2_pressed);


  // wait for rotation sensor to fully initialize
  wait(30, msec);

  whenStarted1();
}
