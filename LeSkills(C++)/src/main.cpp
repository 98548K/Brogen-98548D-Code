/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\S133021581                                       */
/*    Created:      Mon Jan 22 2024                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Drivetrain           drivetrain    4, 5, 7, 8, 6   
// Controller1          controller                    
// Pneumatic1           digital_out   A               
// Pneumatic2           digital_out   B               
// IntakeMotor          motor         2               
// PuncherMotorss        motor_group   12, 13          
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

competition Competition;

// User defined function
void newHeading(double newHeading__Degree);

int Brain_precision = 0, Console_precision = 0, Controller1_precision = 0;

bool pneumaticToggle;

event Pneumatics = event();
event IntakeOut = event();

// User defined function
void newHeading(double newHeading__Degree) {
  Drivetrain.turnToHeading(newHeading__Degree, degrees, false);
  Drivetrain.setStopping(hold);
  waitUntil((Drivetrain.heading() > newHeading__Degree - 2.0 && Drivetrain.heading() < newHeading__Degree + 2.0));
  Drivetrain.stop();
  wait(0.2, seconds);
  Drivetrain.setStopping(coast);
}

// "when started" hat block
int whenStarted1() {
  pneumaticToggle = true;
  PuncherMotors.setVelocity(60.0, percent);
  IntakeMotor.setVelocity(70.0, percent);
  IntakeMotor.setStopping(hold);
  while (true) {
    while (true) {
      // Prints team number on brain
      Brain.Screen.setFont(monoXL);
      Brain.Screen.setFillColor(black);
      Brain.Screen.setPenColor(white);
      Brain.Screen.setCursor(1, 2);
      Brain.Screen.print("98548D SKILLS");
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
    if (Brain.Screen.xPosition() > 20.0 && Brain.Screen.xPosition() < 180.0 && Brain.Screen.yPosition() > 50.0 && Brain.Screen.yPosition() < 95.0) {
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1, 1);
      vexcodeInit();
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
  Drivetrain.setTurnVelocity(100.0, percent);
  Drivetrain.setDriveVelocity(95.0, percent);
  return 0;
}

// "when autonomous" hat block
int onauton_autonomous_0() {
  PuncherMotors.setVelocity(55.0, percent);
  Drivetrain.setTurnVelocity(15.0, percent);
  Drivetrain.setDriveVelocity(60.0, percent);

  // Drive into position for firing Puncher
  Drivetrain.driveFor(reverse, 13.0, inches, true);
  Drivetrain.driveFor(forward, 7.0, inches, true);
  newHeading(275.0);
  Drivetrain.setDriveVelocity(20.0, percent);
  Drivetrain.driveFor(reverse, 2.5, inches, true);
  wait(0.5, seconds);

  // Spin Puncher for launching Triballs
  PuncherMotors.spin(forward);
  IntakeMotor.spin(reverse);
  wait(24.0, seconds);
  Controller1.rumble("----");
  wait(3.0, seconds);
  PuncherMotors.stop();
  IntakeMotor.stop();

  // Drive to other side of Field
  Drivetrain.setDriveVelocity(50.0, percent);
  newHeading(165.0);
  Drivetrain.driveFor(reverse, 7.0, inches, false);
  wait(1.0, seconds);
  Drivetrain.setDriveVelocity(85.0, percent);
  newHeading(125.0);
  Drivetrain.driveFor(reverse, 34.0, inches, false);
  IntakeOut.broadcast();
  wait(2.8, seconds);
  newHeading(90.0);
  Drivetrain.driveFor(reverse, 12.0, inches, false);
  wait(1.0, seconds);
  newHeading(50.0);
  Drivetrain.driveFor(reverse, 20.0, inches, false);
  wait(1.0, seconds);
  Drivetrain.driveFor(forward, 9.0, inches, true);
  wait(0.3, seconds);
  Drivetrain.driveFor(reverse, 20.0, inches, false);
  wait(1.0, seconds);
  Drivetrain.driveFor(forward, 6.0, inches, true);
  newHeading(115.0);
  Drivetrain.driveFor(forward, 12.0, inches, true);
  newHeading(185.0);
  Drivetrain.driveFor(forward, 5.0, inches, true);
  newHeading(280.0);

  // Open wings
  Pneumatics.broadcast();
  Drivetrain.driveFor(forward, 12.0, inches, false);
  wait(1.6, seconds);
  Pneumatics.broadcast();
  Drivetrain.driveFor(reverse, 10.0, inches, true);
  newHeading(200.0);
  Drivetrain.driveFor(forward, 23.0, inches, true);
  Pneumatics.broadcast();
  newHeading(315.0);
  Drivetrain.driveFor(forward, 20.0, inches, false);
  wait(1.8, seconds);
  Pneumatics.broadcast();
  Drivetrain.driveFor(reverse, 10.0, inches, true);
  newHeading(25.0);
  Drivetrain.driveFor(forward, 10.0, inches, true);
  newHeading(300.0);
  Pneumatics.broadcast();
  Drivetrain.driveFor(forward, 20.0, inches, false);
  wait(1.6, seconds);
  Pneumatics.broadcast();
  Drivetrain.driveFor(reverse, 10.0, inches, true);
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
    wait(0.1, seconds);
    Controller1.Screen.clearLine(3);
    Controller1.Screen.setCursor(Controller1.Screen.row(), 1);
  wait(5, msec);
  }
  return 0;
}

// "when I receive Pneumatics" hat block
void onevent_Pneumatics_0() {
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

// "when I recieve IntakeOut" hat block
void onevent_IntakeOut_0() {
  IntakeMotor.spin(reverse);
  wait(2.0, seconds);
  IntakeMotor.stop();
}

void onevent_AlignPuncher_0() {
  Drivetrain.setTurnVelocity(15.0, percent);
  Drivetrain.setDriveVelocity(60.0, percent);

  // Drive into position for firing Puncher
  Drivetrain.driveFor(reverse, 13.0, inches, true);
  Drivetrain.driveFor(forward, 7.0, inches, true);
  newHeading(275.0);
  Drivetrain.setDriveVelocity(20.0, percent);
  Drivetrain.driveFor(reverse, 2.5, inches, true);
  wait(0.5, seconds);

  // Spin Puncher for launching Triballs
  PuncherMotors.spin(forward);
  IntakeMotor.spin(reverse);
  wait(24.0, seconds);
  Controller1.rumble("----");
  wait(3.0, seconds);
  PuncherMotors.stop();
  IntakeMotor.stop();

  Drivetrain.setTurnVelocity(100.0, percent);
  Drivetrain.setDriveVelocity(95.0, percent);
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
  vexcodeInit();

  // register event handlers
  Pneumatics(onevent_Pneumatics_0);
  Controller1.ButtonA.pressed(onevent_Pneumatics_0);
  Controller1.ButtonX.pressed(onevent_AlignPuncher_0);

  wait(15, msec);

  // set default print color to black
  printf("\033[30m");

  // wait for rotation sensor to fully initialize
  wait(30, msec);

  vex::task ws1(whenStarted2);
  whenStarted1();
}