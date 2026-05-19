#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;

// START V5 MACROS
#define waitUntil(condition) \
    do {                     \
        wait(5, msec);       \
    } while (!(condition))

#define repeat(iterations) \
    for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS

// Robot configuration code.
motor ClawMotor = motor(PORT10, ratio18_1, false);

motor ArmMotor = motor(PORT1, ratio18_1, false);

motor LeftMotor = motor(PORT9, ratio18_1, false);

motor RightMotor = motor(PORT8, ratio18_1, true);

controller Controller1 = controller(primary);
motor55 MC55_11 = motor55(PORT11, false);

sonar RangeFinderA = sonar(Brain.ThreeWirePort.C);

pot Potentiometer = pot(Brain.ThreeWirePort.H);
line LineTracker = line(Brain.ThreeWirePort.B);

// generating and setting random seed
void initializeRandomSeed() {
    int systemTime = Brain.Timer.systemHighResolution();
    double batteryCurrent = Brain.Battery.current();
    double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

    // Combine these values into a single integer
    int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

    // Set the seed
    srand(seed);
}

void vexcodeInit() {
    // Initializing random seed.
    initializeRandomSeed();
}

// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char* soundName) {
    printf("VEXPlaySound:%s\n", soundName);
    wait(5, msec);
}

// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

#pragma endregion VEXcode Generated Robot Configuration

// ----------------------------------------------------------------------------
//    Project:        Clawbot for Intro to Robotics (Period 5; Group 1)
// ----------------------------------------------------------------------------

#include "vex.h"

using namespace vex;

bool autoMode = false;
bool arcade = false;

// Begin project code
void controller_L1_Pressed() {
    ArmMotor.spin(forward);
    while (Controller1.ButtonL1.pressing()) {
        wait(5, msec);
    }
    ArmMotor.stop();
}

void controller_L2_Pressed() {
    ArmMotor.spin(reverse);
    while (Controller1.ButtonL2.pressing()) {
        wait(5, msec);
    }
    ArmMotor.stop();
}

void controller_R1_Pressed() {
    ClawMotor.spin(reverse);
    while (Controller1.ButtonR1.pressing()) {
        wait(5, msec);
    }
    ClawMotor.stop();
}

void controller_R2_Pressed() {
    ClawMotor.spin(forward);
    while (Controller1.ButtonR2.pressing()) {
        wait(5, msec);
    }
    ClawMotor.stop();
}

void controller_ButtonA_Pressed() { autoMode = !autoMode; }

void controller_ButtonB_Pressed() { arcade = !arcade; }

/**
 * @param target - target angle in degrees
 * @param direction - forward or reverse arm direction
 * !!! @todo after passing the target angle, automatically determine if the
 *       motor has to go in the forward or reverse direction to reach
 *       the target angle
 */
void raiseArmPID(double target, vex::directionType direction) {
    double kP = 0.5;
    double kI = 0.01;
    double kD = 0.1;

    double error;
    double prevErr = 0;
    double integral = 0;
    double derivative;

    while (true) {
        // double currAngle = Potentiometer.angle(degrees);
        double currAngle = ArmMotor.position(degrees);
        error = target - currAngle;
        derivative = error - prevErr;
        integral += error;

        double motorPower = (error * kP) + (integral * kI) + (derivative * kD);

        if (motorPower > 12) motorPower = 12;
        if (motorPower < -12) motorPower = -12;

        ArmMotor.spin(direction, motorPower, voltageUnits::volt);

        prevErr = error;
        if (fabs(error) < 1.0) {
            break;
        }

        wait(20, msec);
    }

    ArmMotor.stop();
}

void drivePID(double target, vex::directionType direction) {
    double kP = 0.5;
    double kI = 0.01;
    double kD = 0.1;

    double error;
    double prevErr = 0;
    double integral = 0;
    double derivative;

    // Reset encoders
    LeftMotor.setPosition(0, degrees);
    RightMotor.setPosition(0, degrees);

    while (true) {

        double leftPos = LeftMotor.position(degrees);
        double rightPos = RightMotor.position(degrees);

        double avgPos = (leftPos + rightPos) / 2.0;

        error = target - avgPos;

        // PID calculations
        integral += error;
        derivative = error - prevErr;

        double motorPower =
            (error * kP) +
            (integral * kI) +
            (derivative * kD);

        // Clamp motor power
        if (motorPower > 100) motorPower = 100;
        if (motorPower < -100) motorPower = -100;

        LeftMotor.spin(direction, motorPower, percent);
        RightMotor.spin(direction, motorPower, percent);

        prevErr = error;

        // Exit condition
        if (fabs(error) < 5) {
            break;
        }

        wait(20, msec);
    }

    LeftMotor.stop(brake);
    RightMotor.stop(brake);
}

void preAutonomous(void) {
  // actions to do when the program starts
  Brain.Screen.clearScreen();
  Brain.Screen.print("pre auton code");

  ClawMotor.setVelocity(40, percent);
  ClawMotor.spinFor(forward, 25, degrees);
  ClawMotor.setStopping(hold);

  wait(1, seconds);
}

void autonomous(void) {
  Brain.Screen.clearScreen();
  Brain.Screen.print("autonomous code");

  drivePID(1230, forward);

  LeftMotor.setVelocity(100, percent);
  RightMotor.setVelocity(100, percent);

  LeftMotor.spinFor(reverse, 300, degrees, false);
  RightMotor.spinFor(forward, 300, degrees, false);
  waitUntil(!LeftMotor.isSpinning() && !RightMotor.isSpinning());

  while (RangeFinderA.distance(inches) > 43) {
      LeftMotor.spin(forward);
      RightMotor.spin(forward);
  }

  LeftMotor.stop();
  RightMotor.stop();

  raiseArmPID(479.20, forward);

  LeftMotor.stop();
  RightMotor.stop();
}

void userControl(void) {
    Brain.Screen.clearScreen();
    // place driver control in this while loop
      Controller1.ButtonL1.pressed(controller_L1_Pressed);
      Controller1.ButtonL2.pressed(controller_L2_Pressed);
      Controller1.ButtonR1.pressed(controller_R1_Pressed);
      Controller1.ButtonR2.pressed(controller_R2_Pressed);
      Controller1.ButtonA.pressed(controller_ButtonA_Pressed);
      Controller1.ButtonB.pressed(controller_ButtonB_Pressed);
    while (true) {

        if (arcade) {
            LeftMotor.setVelocity(
                (Controller1.Axis2.position() + Controller1.Axis4.position()),
                  percent);
            RightMotor.setVelocity(
                (Controller1.Axis2.position() - Controller1.Axis4.position()),
                  percent);
        } else {
            LeftMotor.setVelocity(Controller1.Axis3.position(), percent);
            RightMotor.setVelocity(Controller1.Axis2.position(), percent);
        }

        LeftMotor.spin(forward);
        RightMotor.spin(forward);

        wait(20, msec);
    }
}

int main() {
    // Initializing Robot Configuration. DO NOT REMOVE!
    vexcodeInit();
    competition Competition;

    Competition.autonomous(autonomous);
    Competition.drivercontrol(userControl);
    preAutonomous();

    // Configure Arm and Claw motor hold settings and velocity
    ArmMotor.setStopping(hold);
    ClawMotor.setStopping(hold);
    ArmMotor.setVelocity(100, percent);
    ClawMotor.setVelocity(100, percent);

    while (true) {
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1,1);
        Brain.Screen.print("Pot Angle %.2f", Potentiometer.angle(degrees));
        Brain.Screen.newLine();
        Brain.Screen.print("Line Tracker: %d", LineTracker.reflectivity(percent));
        Brain.Screen.newLine();
        Brain.Screen.print("Sonar: %.2f", RangeFinderA.distance(inches));
        wait(100, msec);
    }
}