#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS


// Robot configuration code.
motor ClawMotor = motor(PORT10, ratio18_1, false);

motor ArmMotor = motor(PORT1, ratio18_1, false);

controller Controller1 = controller(primary);
motor LeftMotor = motor(PORT9, ratio18_1, false);

motor RightMotor = motor(PORT8, ratio18_1, true);

motor55 MC55_11 = motor55(PORT11, false);

sonar RangeFinderC = sonar(Brain.ThreeWirePort.C);
pot PotentiometerH = pot(Brain.ThreeWirePort.H);
line LineTrackerE = line(Brain.ThreeWirePort.E);


// generating and setting random seed
void initializeRandomSeed(){
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

  // Combine these values into a single integer
  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

  // Set the seed
  srand(seed);
}



void vexcodeInit() {

  //Initializing random seed.
  initializeRandomSeed(); 
}


// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}



// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

#pragma endregion VEXcode Generated Robot Configuration

// ----------------------------------------------------------------------------
//                                                                            
//    Project:        Split Arcade Control                                         
//    Description:    This example will use the left Y and right X
//                    Controller axis to control the Clawbot.
//    Configuration:  V5 Clawbot (Individual Motors)
//                    Controller
//                    Claw Motor in Port 3
//                    Arm Motor in Port 8
//                    Left Motor in Port 1
//                    Right Motor in Port 10   
//                                                                            
// ----------------------------------------------------------------------------

// Include the V5 Library
#include "vex.h"

// Allows for easier use of the VEX Library
using namespace vex;

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  // Begin project code.
  // Main Controller loop to set motors to controller axis postiions
  while (true) {
    LeftMotor.setVelocity((Controller1.Axis3.position() + Controller1.Axis1.position()), percent);
    RightMotor.setVelocity((Controller1.Axis3.position() - Controller1.Axis1.position()), percent);
    LeftMotor.spin(forward);
    RightMotor.spin(forward);

    wait(5, msec);
  }

  return 0;
}

