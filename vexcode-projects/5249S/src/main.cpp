// To complete the VEXcode V5 Text project upgrade process, please follow the
// steps below.
// 
// 1. You can use the Robot Configuration window to recreate your V5 devices
//   - including any motors, sensors, 3-wire devices, and controllers.
// 
// 2. All previous code located in main.cpp has now been commented out. You
//   will need to migrate this code to the new "int main" structure created
//   below and keep in mind any new device names you may have set from the
//   Robot Configuration window. 
// 
// If you would like to go back to your original project, a complete backup
// of your original (pre-upgraded) project was created in a backup folder
// inside of this project's folder.

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;



#include "vex.h"

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// ---- END VEXCODE CONFIGURED DEVICES ----
controller Controller = controller();
motor backLeft = motor(PORT11);
motor backRight = motor(PORT1);
motor frontLeft = motor(PORT20, true);
motor frontRight = motor(PORT10,true);
motor liftMotor = motor(PORT7);
motor rampMotor = motor(PORT4);
motor intakeLMotor = motor(PORT3,true);
motor intakeRMotor = motor(PORT8);
inertial sensor = inertial(PORT6);
// A global instance of vex::competition
competition Competition;

float sensitivity = 1;
//All robot controls
class rLib {
  public:
    //Called during auton. Opens the robot to functioning mode.
    static void deployBot(){
      // startIntake();
      raiseArms(850);
      stopArms();
      // startOuttake();
      
      // task::sleep(50);
      liftRamp(600);
      dropRamp(600);
      dropArms(350);
      startOuttake();
      task::sleep(50);
      dropArms(425);
      stopIntake();
      // stopIntake();
    }
    //Macro to lift the ramp to a perpendicular position to the ground.
    static void liftRamp(int amt){
      rampMotor.setVelocity(100 * sensitivity, percentUnits::pct);
      //fully extended is 1620
      rampMotor.spinFor(amt, rotationUnits::deg);
      task::sleep(1000);
    }
    //Macro to drop the ramp to a resting angle on the bot.
    static void dropRamp(int amt){
      rampMotor.setReversed(true);
      rampMotor.setVelocity(100 * sensitivity, percentUnits::pct);
      // full way is 1620
      rampMotor.spinFor(amt, rotationUnits::deg);
      rampMotor.setReversed(false);
    }
    //Macro to lower the arms to a resting position on the bot.
    static void dropArms(int amt){
      liftMotor.setReversed(true);
      //1080 = old val
      liftMotor.spinFor(amt, rotationUnits::deg);
      liftMotor.setReversed(false);
    }
    // static void stack(){
    //   liftRamp();
    //   stopIntake();
    //   startOuttake();
    //   drive(25,directionType::rev,3);
    //   stopIntake();
    // }
    //Deploys the arms for the first time.
    static void dropArmsDeploy(){
      liftMotor.stop(brakeType::coast);
    }
    //Macro to raise the arms to a position that is able to score low towers.
    static void raiseArms(int amt){
      liftMotor.setVelocity(100, percentUnits::pct);
      //1080 = old val
      liftMotor.spinFor(amt, rotationUnits::deg);
    }
    //To hold the ramp in its current position.
    static void stopRamp(){
      rampMotor.stop(brakeType::hold);
    }
    //To hold the arms in its current position.
    static void stopArms(){
      liftMotor.stop(brakeType::hold);
    }
    //An Autonomous method that turns a bot based on the specified degrees, velocity, and direction of the robot.
    // static void turn(double velocity, double degrees, bool dirRight){
    //   //Sets the velocity of all motors to the specified velocity above
    //   backLeft.setVelocity(velocity, velocityUnits::pct);
    //   frontLeft.setVelocity(velocity, velocityUnits::pct);
    //   backRight.setVelocity(velocity, velocityUnits::pct);
    //   frontRight.setVelocity(velocity, velocityUnits::pct);
    //   //Degrees multiplied by digits of PI in order to obtain degree calculations.
    //   degrees = degrees * 3.141592653589793238462643383279;
    //   //Determines the direction the bot will travel in.
    //   directionType backDir = dirRight ? directionType::fwd : directionType::rev;
    //   directionType frontDir = dirRight ? directionType::rev : directionType::fwd;

    //   //Turns the bot using the specified parameters and calculations done above.
    //   backLeft.rotateFor(backDir, degrees, rotationUnits::deg, false);
    //   backRight.rotateFor(backDir, degrees, rotationUnits::deg, false);
    //   frontLeft.rotateFor(frontDir, degrees, rotationUnits::deg, false);
    //   frontRight.rotateFor(frontDir, degrees, rotationUnits::deg, true);
    // }
    static void turn(double velocity, double degrees, bool dirRight){
      backLeft.setVelocity(velocity, velocityUnits::pct);
      frontLeft.setVelocity(velocity, velocityUnits::pct);
      backRight.setVelocity(velocity, velocityUnits::pct);
      frontRight.setVelocity(velocity, velocityUnits::pct);
      //
      directionType backDir = dirRight ? directionType::fwd : directionType::rev;
      directionType frontDir = dirRight ? directionType::rev : directionType::fwd;
      //
      backLeft.spin(backDir,velocity,velocityUnits::pct);
      backRight.spin(backDir,velocity,velocityUnits::pct);
      frontLeft.spin(frontDir,velocity,velocityUnits::pct);
      frontRight.spin(frontDir,velocity,velocityUnits::pct);

      waitUntil(sensor.heading() >= degrees);
      stopDrive();
      sensor.setHeading(0,rotationUnits::deg);
    }
    //An Autonomous method that drives a bot based on the specified velocity, direction, and inches wanted to travel.
    static void drive(double velocity, directionType dir, int inches){
      float degrees = inches * (360/12);
      
      backLeft.setVelocity(velocity, velocityUnits::pct);
      frontLeft.setVelocity(velocity, velocityUnits::pct);
      backRight.setVelocity(velocity, velocityUnits::pct);
      frontRight.setVelocity(velocity, velocityUnits::pct);

      backLeft.rotateFor(dir, degrees, rotationUnits::deg, false);
      backRight.rotateFor(dir, degrees, rotationUnits::deg, false);
      frontLeft.rotateFor(dir, degrees, rotationUnits::deg, false);
      frontRight.rotateFor(dir, degrees, rotationUnits::deg, true);
    }
    //Stops all of the drive motors from moving.
    static void stopDrive(){
      backLeft.stop(brakeType::coast);
      backRight.stop(brakeType::coast);
      frontLeft.stop(brakeType::coast);
      frontRight.stop(brakeType::coast);
    }
    //Stops the intake motors in its place.
    static void stopIntake() {
      intakeLMotor.stop(brakeType::hold);
      intakeRMotor.stop(brakeType::hold);
    }
    //Starts spinning the intake motors forward.
    static void startIntake() {
      intakeLMotor.spin(directionType::fwd, 90*sensitivity, percentUnits::pct);
      intakeRMotor.spin(directionType::fwd, 90*sensitivity, percentUnits::pct);
    }
    //Starts spinning the intake motors in reverse.
    static void startOuttake() {
      intakeLMotor.spin(directionType::rev, 100*sensitivity, percentUnits::pct);
      intakeRMotor.spin(directionType::rev, 100*sensitivity, percentUnits::pct);
    }
    static void startOuttakeFor(float seconds) {
      intakeLMotor.setReversed(true);
      intakeRMotor.setReversed(true);
      intakeLMotor.setVelocity(50, percentUnits::pct);
      intakeRMotor.setVelocity(50, percentUnits::pct);
      intakeLMotor.spinFor(seconds, timeUnits::sec);
      intakeRMotor.spinFor(seconds, timeUnits::sec);
    }
    //Rotates the Lift Up.
    static void startLiftUp() {
      liftMotor.spin(directionType::fwd, 100*sensitivity, percentUnits::pct);
    }
    //Rotates the Lift Down.
    static void startLiftDown() {
      if(liftMotor.rotation(rotationUnits::deg) > 0) { //bottom limit
        liftMotor.spin(directionType::rev, 100*sensitivity, percentUnits::pct);
      }
    }
    //Pushes the ramp to a 90 degree angle to the ground.
    static void startRampUp() {
      rampMotor.spin(directionType::fwd, 100*sensitivity, percentUnits::pct);
    }
    //Pulls the ramp back down to a resting angle.
    static void startRampDown() {
      rampMotor.spin(directionType::rev, 100*sensitivity, percentUnits::pct);
    }
    //Toggles the sensitivity between high and low sensitivity.
    static void toggleSensitive() {
      if(sensitivity == 1)
        sensitivity = .5;
      else
        sensitivity = 1;
    }

    //I'm trying again
    
    // static void customDrive(double inches, directionType d) {
    //   double Kp = 0;
    //   double Ki = 0;
    //   double Kd = 0;
    //   double error = inches;
    //   double previousError = 0;
    //   double P;
    //   double I;
    //   double D;
    //   double totalError = 0;
    //   double power;
    //   while(true) {
    //     totalError += error;
    //     P = Kp * error;
    //     D = (error - previousError) * kDeviceTypeMotorSensor;
    //     I = totalError * Ki;
    //     power = P + I + D;

    //   }
    // }
};
 
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
void pre_auton(void) {
  backLeft.resetRotation();
  backRight.resetRotation();
  frontLeft.resetRotation();
  frontRight.resetRotation();
  liftMotor.resetRotation();
  rampMotor.resetRotation();
  intakeLMotor.resetRotation();
  intakeRMotor.resetRotation();
  sensor.calibrate();
  waitUntil(!sensor.isCalibrating());
  sensor.setHeading(0,degrees);
}
 
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
int auton = 4;
bool redSide = false;
void autonomous(void) {
  if(auton == 0){
    //One point reverse with preload
    rLib::drive(50, directionType::rev, 12);
    rLib::drive(50, directionType::fwd, 12);
    rLib::deployBot();
  }else if(auton == 1){
    //4 point auton. blue = true, red = false
    // rLib::deployBot(); //deploys the ramp and arms.
    // rLib::startIntake(); //starts rotating the intake.

    //drive forward and intake cubes.
    rLib::drive(35, directionType::fwd, 40);
    //turn around and drive to the score zone.
    rLib::stopIntake();
    rLib::turn(25, 140, true);
    rLib::drive(40, directionType::fwd, 28);
    //make the sensitivity more precise.
    rLib::toggleSensitive();
    //stack algorithm, may replace with the stack() method.
    // rLib::stack();
    // rLib::stopIntake();
    // rLib::liftRamp();
    // rLib::startOuttakeFor(1);
    //rLib::drive(10, directionType::fwd, 2);
    //rLib::dropRamp();
    // rLib::startOuttakeFor();
    // rLib::drive(20, directionType::rev, 30);
  }else if(auton == 2){
    rLib::deployBot();
    task::sleep(1000);
    // rLib::stack();
  }else if(auton == 3){ //Akshat's attempt at a 6 point auton
    rLib::deployBot(); //deploys the ramp and arms.
    rLib::startIntake();
    rLib::drive(35, directionType::fwd, 40);
    rLib::turn(25,20,!redSide);
    rLib::drive(35,directionType::fwd,10);
    //turn around and drive to the score zone.
    rLib::stopIntake();
    rLib::turn(25, 160, redSide);
    rLib::drive(40, directionType::fwd, 28);
    //make the sensitivity more precise.
    rLib::toggleSensitive();
  }else if(auton == 4){
    rLib::deployBot();
  }
  
}
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
void usercontrol(void) {
  while (1) {
    float leftVelocity = (Controller.Axis3.value() - Controller.Axis4.value()) * sensitivity * .8;
    float rightVelocity = (Controller.Axis3.value() + Controller.Axis4.value()) * sensitivity * .8;
    frontRight.spin(directionType::fwd, leftVelocity, velocityUnits::pct);
    frontLeft.spin(directionType::fwd, leftVelocity, velocityUnits::pct);
    backRight.spin(directionType::fwd, rightVelocity, velocityUnits::pct);
    backLeft.spin(directionType::fwd, rightVelocity, velocityUnits::pct);
  }
}
 
//
// Main will set up the competition functions and callbacks.
//
int main() {
  //initializing something

  //Set up callbacks for autonomous and driver control periods.
  Competition.autonomous( autonomous );
  Competition.drivercontrol( usercontrol );

  //Set up controller callbacks.
  Controller.ButtonL1.pressed(rLib::startIntake);
  Controller.ButtonL1.released(rLib::stopIntake);
  Controller.ButtonL2.pressed(rLib::startOuttake);
  Controller.ButtonL2.released(rLib::stopIntake);

  Controller.ButtonX.released(rLib::toggleSensitive);
  
  Controller.ButtonR1.pressed(rLib::startLiftUp);
  Controller.ButtonR1.released(rLib::stopArms);
  Controller.ButtonR2.pressed(rLib::startLiftDown);
  Controller.ButtonR2.released(rLib::stopArms);

  Controller.ButtonUp.pressed(rLib::startRampUp);
  Controller.ButtonUp.released(rLib::stopRamp);
  Controller.ButtonDown.pressed(rLib::startRampDown);
  Controller.ButtonDown.released(rLib::stopRamp);

  //Run the pre-autonomous function.
  pre_auton();
  
  //Prevent main from exiting with an infinite loop.
  while(1) {
    task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
  }
}
