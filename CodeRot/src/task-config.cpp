#include "vex.h"

using namespace vex;

bool conveyorStop = false;

int function_ringSorter() {
  colorSens.setLightPower(50);
  while (true) {
    if (autonSelector <= 1) {
      if (colorSens.value() < 22 && colorSens.isNearObject()) {
        wait(70, msec);
        conveyorStop = true;
        wait(200, msec);
        conveyorStop = false;
        conveyorMotorGroup.spin(forward);
      }
    } else if (autonSelector >= 2) {
      if ((colorSens.value() > 200 && colorSens.value() < 240) && colorSens.isNearObject()) {
        wait(70, msec);
        conveyorStop = true;
        wait(200, msec);
        conveyorStop = false;
        conveyorMotorGroup.spin(forward);
      }
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
      // Prints auton selector
      if (autonSelector == 0){
        Brain.Screen.setFillColor("#0092ff");
        Brain.Screen.drawRectangle(310, 60, 160, 90);
        Brain.Screen.setFont(monoL);
        Brain.Screen.setCursor(4, 22);
        Brain.Screen.print("BLUE  LEFT");
      } else if (autonSelector == 1) {
        Brain.Screen.setFillColor("#084dbd");
        Brain.Screen.drawRectangle(310, 60, 160, 90);
        Brain.Screen.setFont(monoL);
        Brain.Screen.setCursor(4, 22);
        Brain.Screen.print("BLUE RIGHT");
      } else if (autonSelector == 2) {
        Brain.Screen.setFillColor("#ff4100");
        Brain.Screen.drawRectangle(310, 60, 160, 90);
        Brain.Screen.setFont(monoL);
        Brain.Screen.setCursor(4, 23);
        Brain.Screen.print("RED LEFT");
      } else if (autonSelector == 3) {
        Brain.Screen.setFillColor("#bd0808");
        Brain.Screen.drawRectangle(310, 60, 160, 90);
        Brain.Screen.setFont(monoL);
        Brain.Screen.setCursor(4, 22);
        Brain.Screen.print("RED  RIGHT");
      } else if (autonSelector == 4) {
        Brain.Screen.setFillColor("#424040");
        Brain.Screen.drawRectangle(310, 60, 160, 90);
        Brain.Screen.setFont(monoL);
        Brain.Screen.setCursor(4, 23);
        Brain.Screen.print("NO AUTON");
      }
      if (autonSelector == 4) {
        Brain.Screen.setFillColor(black);
        Brain.Screen.setPenColor(black);
        Brain.Screen.drawRectangle(310, 150, 160, 90);
      } else if (autonSelector_2 == 1 && (autonSelector == 0 || autonSelector == 3)) {
        Brain.Screen.setFillColor("#08c70b");
        Brain.Screen.drawRectangle(310, 150, 160, 90);
        Brain.Screen.setFont(monoL);
        Brain.Screen.setCursor(7, 22);
        Brain.Screen.print("GOAL  RUSH");
      } else if (autonSelector_2 == 1 && (autonSelector == 1 || autonSelector == 2)) {
        Brain.Screen.setFillColor("#08c70b");
        Brain.Screen.drawRectangle(310, 150, 160, 90);
        Brain.Screen.setFont(monoL);
        Brain.Screen.setCursor(7, 22);
        Brain.Screen.print("GOAL  FILL");
      } else {
        Brain.Screen.setFillColor("#c7aa08");
        Brain.Screen.drawRectangle(310, 150, 160, 90);
        Brain.Screen.setFont(monoL);
        Brain.Screen.setCursor(7, 22);
        Brain.Screen.print("WIN  POINT");
      }
      if (!(autonSelector == 4)) {
        if (delay == 0) {
          Brain.Screen.setFillColor("#08c70b");
        } else if (delay == 1) {
          Brain.Screen.setFillColor("#c7aa08");
        } else if (delay == 2) {
          Brain.Screen.setFillColor(orange);
        } else if (delay == 3) {
          Brain.Screen.setFillColor(red);
        }
        Brain.Screen.drawCircle(235, 80, 40);
        Brain.Screen.setFont(monoM);
        Brain.Screen.setCursor(4, 22);
        Brain.Screen.print("TIMER");
        Brain.Screen.setCursor(5, 24);
        Brain.Screen.print(delay);
        if (runArm) {
          Brain.Screen.setFillColor(orange);
        } else {
          Brain.Screen.setFillColor("#424040");
        }
        Brain.Screen.drawRectangle(20, 185, 290, 55);
        Brain.Screen.setCursor(11, 6);
        Brain.Screen.print("Alliance Stake:");
        Brain.Screen.setCursor(11, 23);
        if (runArm) {
          Brain.Screen.print("true");
        } else {
          Brain.Screen.print("false");
        }
      } else {
        Brain.Screen.setFillColor(black);
        Brain.Screen.setPenColor(black);
        Brain.Screen.drawCircle(235, 80, 40);
        Brain.Screen.drawRectangle(20, 185, 290, 55);
        runArm = false;
      }
      // Render Brain screen
      Brain.Screen.setFont(monoM);
      Brain.Screen.setFillColor(black);
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
    } else if (Brain.Screen.xPosition() > 320.0 && Brain.Screen.xPosition() < 480.0 && Brain.Screen.yPosition() > 60.0 && Brain.Screen.yPosition() < 150.0) {
      autonSelector_pressed.broadcast();
    } else if (Brain.Screen.xPosition() > 320.0 && Brain.Screen.xPosition() < 480.0 && Brain.Screen.yPosition() > 150.0 && Brain.Screen.yPosition() < 240.0) {
      autonSelector_2_pressed.broadcast();
    } else if (Brain.Screen.xPosition() > 200.0 && Brain.Screen.xPosition() < 280.0 && Brain.Screen.yPosition() > 40.0 && Brain.Screen.yPosition() < 120.0) {
      delaySelector.broadcast();
    } else if (Brain.Screen.xPosition() > 20.0 && Brain.Screen.xPosition() < 320.0 && Brain.Screen.yPosition() > 185.0 && Brain.Screen.yPosition() < 240.0) {
      armSelector.broadcast();
    }
    wait(50, msec);
  }
  return 0;
}


task task_ringSorter(function_ringSorter);
task task_brainScreenPrint(function_brainScreenPrint);