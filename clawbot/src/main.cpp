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

        double motorPower = (error * kP) + (integral * kI) + (derivative * kD);
        ArmMotor.spin(direction, motorPower, voltageUnits::volt);

        prevErr = error;
        if (fabs(error) < 1.0) {
            break;
        }

        wait(20, msec);
    }

    ArmMotor.stop();
}

void preAutonomous(void) {
    // actions to do when the program starts
    Brain.Screen.clearScreen();
    Brain.Screen.print("pre auton code");
    wait(1, seconds);
}

void autonomous(void) {
    Brain.Screen.clearScreen();
    Brain.Screen.print("autonomous code");

    LeftMotor.setVelocity(100, percent);
    RightMotor.setVelocity(100, percent);

    LeftMotor.spin(forward);
    RightMotor.spin(forward);

    while (RangeFinderA.distance(inches) > 66) {
        wait(20, msec);
    }

    LeftMotor.stop();
    RightMotor.stop();

    raiseArmPID(479.20, forward);
}

void userControl(void) {
    Brain.Screen.clearScreen();
    // place driver control in this while loop
    while (true) {
        Controller1.ButtonL1.pressed(controller_L1_Pressed);
        Controller1.ButtonL2.pressed(controller_L2_Pressed);
        Controller1.ButtonR1.pressed(controller_R1_Pressed);
        Controller1.ButtonR2.pressed(controller_R2_Pressed);
        Controller1.ButtonA.pressed(controller_ButtonA_Pressed);
        Controller1.ButtonB.pressed(controller_ButtonB_Pressed);

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
    ArmMotor.setVelocity(68, percent);
    ClawMotor.setVelocity(70, percent);

    while (true) {
        wait(100, msec);
    }
}