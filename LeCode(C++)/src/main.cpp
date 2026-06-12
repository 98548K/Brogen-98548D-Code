/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\S133026067                                       */
/*    Created:      Mon Jan 22 2024                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Drivetrain           drivetrain    4, 5, 7, 8, 6   
// Pneumatic1           digital_out   A               
// Pneumatic2           digital_out   B               
// IntakeMotor          motor         2               
// PuncherMotors        motor_group   12, 13          
// ---- END VEXCODE CONFIGURED DEVICES ----

#pragma region VEXcode Generated Robot Configuration

#include "vex.h"

using namespace vex;

competition Competition;

// Functions
void newHeading(double newHeading__Degrees);

int Brain_precision = 0, Console_precision = 0, Controller1_precision = 0;

float Autonomous;

bool pneumaticToggle, extenderToggle, pneumaticAuton1, pneumaticAuton2, elevationToggle;

event IntakeIn = event();
event PneumaticA = event();
event PneumaticB = event();
event IntakeOut = event();

// Function for turning more acurately
void newHeading(double newHeading__Degrees) {
  Drivetrain.turnToHeading(newHeading__Degrees, degrees, false);
  Drivetrain.setStopping(hold);
  waitUntil((Drivetrain.heading() >= newHeading__Degrees - 2.0 && Drivetrain.heading() <= newHeading__Degrees + 2.0));
  Drivetrain.stop();
  wait(0.2, seconds);
  Drivetrain.setStopping(coast);
}

// "when autonomous" hat block
int onauton_autonomous_0() {
  Drivetrain.setDriveVelocity(60.0, percent);
  Drivetrain.setTurnVelocity(10.0, percent);

  // Closeside Auton
  if (Autonomous == 1.0) {
    Drivetrain.driveFor(forward, 3.2, inches, true);
    PneumaticB.broadcast();
    Drivetrain.driveFor(reverse, 1.5, inches, true);
    newHeading(330.0);
    PneumaticB.broadcast();
    newHeading(2.0);
    Drivetrain.driveFor(forward, 7.4, inches, true);
    newHeading(44.0);
    IntakeOut.broadcast();
    wait(1.0, seconds);
    Drivetrain.driveFor(reverse, 3.4, inches, true);
    newHeading(105.0);
    Drivetrain.driveFor(forward, 10.5, inches, true);
    newHeading(135.0);
    Drivetrain.driveFor(forward, 8.8, inches, false);
    wait(0.4, seconds);
    PneumaticA.broadcast();
    wait(1.2, seconds);
    newHeading(105.0);

  // Farside Auton
  } else if (Autonomous == 2.0) {
    PneumaticA.broadcast();
    Drivetrain.driveFor(forward, 5.0, inches, true);
    newHeading(330.0);
    wait(0.2, seconds);
    newHeading(345.0);
    PneumaticA.broadcast();
    newHeading(342.0);
    Drivetrain.driveFor(forward, 10.0, inches, false);
    wait(0.3, seconds);
    IntakeOut.broadcast();
    wait(0.5, seconds);
    Drivetrain.driveFor(reverse, 5.0, inches, true);
    newHeading(255.0);
    Drivetrain.driveFor(forward, 19.0, inches, false);
    wait(0.8, seconds);
    IntakeIn.broadcast();
    wait(0.4, seconds);
    newHeading(10.0);
    Drivetrain.driveFor(forward, 6.0, inches, false);
    IntakeOut.broadcast();
    wait(0.6, seconds);
    newHeading(270.0);
    Drivetrain.driveFor(forward, 9.5, inches, false);
    wait(0.2, seconds);
    IntakeIn.broadcast();
    wait(1.2, seconds);
    newHeading(30.0);
    PneumaticA.broadcast();
    PneumaticB.broadcast();
    Drivetrain.driveFor(forward, 20.0, inches, false);
    wait(0.2, seconds);
    IntakeOut.broadcast();
    wait(1.0, seconds);
    PneumaticA.broadcast();
    PneumaticB.broadcast();
    newHeading(25.0);
    Drivetrain.driveFor(reverse, 12.0, inches, true);
    newHeading(335.0);
  }
  return 0;
}

// "when started" hat block
int whenStarted1() {
  PuncherMotors.setVelocity(60.0, percent);
  IntakeMotor.setVelocity(70.0, percent);
  pneumaticToggle = true;
  pneumaticAuton1 = true;
  pneumaticAuton2 = true;
  elevationToggle = true;
  IntakeMotor.setStopping(hold);

  
  // For selecting one of the Autons
  Autonomous = 2.0;
  Brain.Screen.setPenColor(black);
  Brain.Screen.setFillColor(red);
  Brain.Screen.drawCircle(360, 120, 100);
  Brain.Screen.setFillColor(white);
  Brain.Screen.setCursor(6, 30);
  Brain.Screen.print("Farside Auton");
  while (true) {
    while (true) {
      // Prints team number on brain
      Brain.Screen.setFont(monoXL);
      Brain.Screen.setFillColor(black);
      Brain.Screen.setPenColor(white);
      Brain.Screen.setCursor(1, 2);
      Brain.Screen.print("98548D");
      // Prints calibrate button on brain
      Brain.Screen.setFont(monoL);
      Brain.Screen.setFillColor(white);
      Brain.Screen.setPenColor(purple);
      Brain.Screen.drawRectangle(20, 50, 160, 45);
      Brain.Screen.setCursor(3, 3);
      Brain.Screen.setPenColor(black);
      Brain.Screen.print("CALIBRATE");
      Brain.Screen.setFont(monoM); 
      Brain.Screen.setFillColor(black);
      Brain.Screen.setPenColor(white);
      // Prints drivetrain heading on brain
      Brain.Screen.setCursor(6, 3);
      Brain.Screen.print("Heading:              ");
      Brain.Screen.setCursor(6, 12);
      Brain.Screen.print(Drivetrain.heading());
      // Prints battery capacity on brain
      Brain.Screen.setCursor(7, 3);
      Brain.Screen.print("Battery:              ");
      Brain.Screen.setCursor(7, 12);
      Brain.Screen.print(Brain.Battery.capacity());
      // Prints temperature of motors
      Brain.Screen.setCursor(8, 3);
      Brain.Screen.print("Drivetrain Temp:      ");
      Brain.Screen.setCursor(8, 20);
      Brain.Screen.print(Drivetrain.temperature(percent));
      // Prints temperature of puncher motor
      Brain.Screen.setCursor(9, 3);
      Brain.Screen.print("Punch Temp:            ");
      Brain.Screen.setCursor(9, 17);
      Brain.Screen.print(PuncherMotors.temperature(percent));
      if (Brain.Screen.pressing()) {
        break;
      }
    }
    if (Brain.Screen.xPosition() > 240.0) {
      if (Autonomous == 2.0) {
      
      Autonomous = 1.0;
      } else if (Autonomous == 1.0) {
        Autonomous = 2.0;
      }
      wait(300, msec);
    } else if (Brain.Screen.xPosition() > 20.0 && Brain.Screen.xPosition() < 180.0 && Brain.Screen.yPosition() > 50.0 && Brain.Screen.yPosition() < 95.0) {
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1, 1);
      vexcodeInit();
    }
    if (Autonomous == 1.0) {
      Brain.Screen.setFillColor(blue);
      Brain.Screen.drawCircle(360, 120, 100);;
      Brain.Screen.setPenColor(black);
      Brain.Screen.setFillColor(white);
      Brain.Screen.setCursor(6, 29);
      Brain.Screen.print("Closeside Auton");
    } else {
      Brain.Screen.setFillColor(red);
      Brain.Screen.drawCircle(360, 120, 100);
      Brain.Screen.setPenColor(black);
      Brain.Screen.setFillColor(white);
      Brain.Screen.setCursor(6, 30);
      Brain.Screen.print("Farside Auton");
    }
  wait(5, msec);
  }
  return 0;
}

// "when driver control" hat block
int ondriver_drivercontrol_0() {
  Pneumatic1.set(false);
  Pneumatic2.set(false);
  pneumaticToggle = true;
  extenderToggle = true;
  Drivetrain.setTurnVelocity(100.0, percent);
  Drivetrain.setDriveVelocity(95.0, percent);
  return 0;
}

// Used to find the format string for printing numbers with the
// desired number of decimal places
const char* printToController1_numberFormat() {
  // look at the current precision setting to find the format string
  switch(Controller1_precision){
    case 0:  return "%.0f"; // 0 decimal places (1)
    case 1:  return "%.1f"; // 1 decimal place  (0.1)
    case 2:  return "%.2f"; // 2 decimal places (0.01)
    case 3:  return "%.3f"; // 3 decimal places (0.001)
    default: return "%f"; // use the print system default for everthing else
  }
}

// "when started" hat block
int whenStarted2() {
  wait(2.0, seconds);
  while (true) {
    Controller1.Screen.setCursor(3, 13);
    Controller1.Screen.print(printToController1_numberFormat(), static_cast<float>(Drivetrain.heading()));
    wait(0.2, seconds);
    Controller1.Screen.clearLine(3);
    Controller1.Screen.setCursor(Controller1.Screen.row(), 1);
  wait(5, msec);
  }
  return 0;
}

// "when I receive IntakeIn" hat block
void onevent_IntakeIn_0() {
  IntakeMotor.spin(forward);
  wait(0.8, seconds);
  IntakeMotor.stop();
}

// "when Controller1 ButtonY pressed" hat block
void onevent_Controller1ButtonY_pressed_0() {
  // Puncher activator
  PuncherMotors.spin(forward);
}

// "when Controller1 ButtonY released" hat block
void onevent_Controller1ButtonY_released_0() {
  // Puncher activator
  PuncherMotors.stop();
}

// "when I receive IntakeOut" hat block
void onevent_IntakeOut_0() {
  IntakeMotor.spin(reverse);
  wait(1.6, seconds);
  IntakeMotor.stop();
}

// "when Controller1 ButtonA pressed" hat block
void onevent_Controller1ButtonA_pressed_0() {
  // Double Wing Toggle
  if (pneumaticToggle) {
    Pneumatic1.set(true);
    Pneumatic2.set(true);
    pneumaticToggle = false;
  }
  else {
    Pneumatic1.set(false);
    Pneumatic2.set(false);
    pneumaticToggle = true;
  }
}

// "when I receive PneumaticA" hat block
void onevent_PneumaticA_0() {
  // Right Wing Toggle
  if (pneumaticAuton1) {
    Pneumatic1.set(true);
    pneumaticAuton1 = false;
  }
  else {
    Pneumatic1.set(false);
    pneumaticAuton1 = true;
  }
}

// "when I receive PneumaticB" hat block
void onevent_PneumaticB_0() {
  // Left Wing Toggle
  if (pneumaticAuton2) {
    Pneumatic2.set(true);
    pneumaticAuton2 = false;
  }
  else {
    Pneumatic2.set(false);
    pneumaticAuton2 = true;
  }
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



int main() {
  vex::competition::bStopTasksBetweenModes = false;
  Competition.autonomous(VEXcode_auton_task);
  Competition.drivercontrol(VEXcode_driver_task);

  // register event handlers
  vexcodeInit();
  IntakeIn(onevent_IntakeIn_0);
  Controller1.ButtonY.pressed(onevent_Controller1ButtonY_pressed_0);
  Controller1.ButtonY.released(onevent_Controller1ButtonY_released_0);
  IntakeOut(onevent_IntakeOut_0);
  Controller1.ButtonA.pressed(onevent_Controller1ButtonA_pressed_0);
  PneumaticA(onevent_PneumaticA_0);
  PneumaticB(onevent_PneumaticB_0);

  wait(15, msec);
  // post event registration

  // set default print color to black
  printf("\033[30m");


  // wait for rotation sensor to fully initialize
  wait(30, msec);

  vex::task ws1(whenStarted2);
  whenStarted1();
}