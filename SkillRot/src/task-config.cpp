#include "vex.h"

using namespace vex;

bool conveyorStop = false;

int function_ringSorter() {
  colorSens.setLightPower(0);
  while (true) {
    if ((colorSens.value() > 200 && colorSens.value() < 240) && colorSens.isNearObject()) {
      wait(70, msec);
      conveyorStop = true;
      wait(200, msec);
      conveyorStop = false;
      conveyorMotorGroup.spin(forward);
    }
  }
  return 0;
}

int function_brainScreenPrint() {
  while (true) {
    while (true) {
      // Prints team number on brain
      Brain.Screen.setFont(monoXL);
      Brain.Screen.setFillColor(black);
      Brain.Screen.setPenColor(white);
      Brain.Screen.setCursor(1, 2);
      Brain.Screen.print("98548D        Defiance");
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
      // Prints drivetrain heading on controller
      Controller1.Screen.setCursor(3, 13);
      Controller1.Screen.print(int(Drivetrain.heading()));
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
      // Prints rotation sensor
      Brain.Screen.setCursor(9, 3);
      Brain.Screen.print("Rotation Sensor (deg):      ");
      Brain.Screen.setCursor(9, 25);
      Brain.Screen.print(rotationSens.position(degrees));
      // Render Brain screen
      Brain.Screen.render();
      if (Brain.Screen.pressing()) {
        break;
      }
      wait(50, msec);
      Controller1.Screen.clearLine(3);
    }
    if (Brain.Screen.xPosition() > 20.0 && Brain.Screen.xPosition() < 180.0 && Brain.Screen.yPosition() > 50.0 && Brain.Screen.yPosition() < 95.0) {
      Brain.Screen.renderDisable();
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1, 1);
      vexcodeInit();
    }
    wait(50, msec);
  }
  return 0;
}


task task_ringSorter(function_ringSorter);
task task_brainScreenPrint(function_brainScreenPrint);