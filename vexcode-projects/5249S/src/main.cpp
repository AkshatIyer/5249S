#include "vex.h"

using namespace vex;


// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]

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
// ---- END VEXCODE CONFIGURED DEVICES ----
// A global instance of vex::competition
competition Competition;

float intakeSens = 1;
float sensitivity = 1;
const double defaultDeg = 285.6;
bool skills = false;
//All robot controls
class rLib {
  public:
  
    //Called during auton. Opens the robot to functioning mode.
    static void deployBot(){
      // startIntake();
      
        raiseArms(1200);
        startOuttake();
        stopArms();
        task::sleep(100);
        dropArms(1200);
        // startOuttake();
        waitUntil(!liftMotor.isSpinning());
        stopIntake();
      
      // task::sleep(50);
      // liftRamp(600);
      // dropRamp(600);
      // task::sleep(50);
      // dropArms(350);
      // startOuttake();
      // dropArms(500);
      // stopIntake();
    }
    static void deployBotSkills(){
      if(skills){
        raiseArms(1100);
        startOuttake();
        stopArms();
        task::sleep(100);
        dropArms(1100);
        // startOuttake();
        waitUntil(!liftMotor.isSpinning());
        stopIntake();
        skills = false;
        waitUntil(!Controller.ButtonRight.pressing());
      }
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
    static void stack(){
      toggleSensitive();
      startOuttake();
      task::sleep(200);
      stopIntake();
      liftRamp(1450);
      liftMotor.stop(brakeType::hold);
      // dropRamp(1450);
      waitUntil(!liftMotor.isSpinning());
      liftMotor.stop(brakeType::coast);
      liftMotor.setReversed(false);
      toggleSensitive();
      // startOuttake();
      // drive(25,directionType::rev,10);
      // stopIntake();
    }
    static void macroStack(){
      stack();
      startOuttake();
      drive(30,directionType::rev,30);
      waitUntil(!backLeft.isSpinning());
      stopIntake();
    }
    static void setRampDefault(){
      double degrees = defaultDeg;
      double proportion = (degrees - rampMotor.rotation(rotationUnits::deg))/100;
      while(rampMotor.rotation(rotationUnits::deg) < degrees){
        rampMotor.setVelocity(100-rampMotor.rotation(rotationUnits::deg)/proportion,velocityUnits::pct);
        rampMotor.spin(directionType::fwd);
        Controller.Screen.clearLine();
        
      }
      Controller.Screen.print(rampMotor.rotation(rotationUnits::deg));
      rampMotor.setRotation(degrees, rotationUnits::deg);
      
    }

    static void proportionalRampUp(){
      double degrees = 1450;
      double proportion = (degrees - rampMotor.rotation(rotationUnits::deg))/100;
      while(rampMotor.rotation(rotationUnits::deg) < degrees){
        rampMotor.setVelocity(100-rampMotor.rotation(rotationUnits::deg)/proportion,velocityUnits::pct);
        rampMotor.spin(directionType::fwd);
        Controller.Screen.clearLine();
        
      }
      Controller.Screen.print(rampMotor.rotation(rotationUnits::deg));
      rampMotor.setRotation(degrees, rotationUnits::deg);
      
    }
    static void defaultRamp(){
      rampMotor.rotateTo(defaultDeg,rotationUnits::deg);
    }
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
      Controller.Screen.clearLine();
      Controller.Screen.print(rampMotor.rotation(rotationUnits::deg));
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
      sensor.setHeading(0,rotationUnits::deg);
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
      if(dirRight){
        waitUntil(sensor.rotation() >= degrees);
      }
      else{
        waitUntil(sensor.rotation() <= degrees * -1);
      }
      Controller.Screen.print(sensor.rotation());
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
    static void startRampAxis() {
      rampMotor.spin(directionType::fwd, Controller.Axis2.value(), percentUnits::pct);
      if(Controller.Axis2.value() == 0){
        stopRamp();
      }
    }
    //Starts spinning the intake motors in reverse.
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
      if(rampMotor.rotation(rotationUnits::deg) < 0){
        rampMotor.setRotation(0,rotationUnits::deg);
      }
    }
    //Toggles the sensitivity between high and low sensitivity.
    static void toggleSensitive() {
      if(sensitivity == 1)
        sensitivity = .5;
      else
        sensitivity = 1;
    }
    static void toggleIntakeSens() {
      if(intakeSens == 1)
        intakeSens = .5;
      else
        intakeSens = 1;
      
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
int auton = 1;
bool redSide = false;
//static void drive(double velocity, directionType dir, int inches)
void autonomous(void) {
  if(auton == 0){
    //One point reverse with preload
    rLib::drive(25, directionType::rev, 12);
    rLib::drive(30, directionType::fwd, 20);
    rLib::deployBot();
  }else if(auton == 1){
    //4 point auton. blue = false, red = true
    // rLib::deployBot(); //deploys the ramp and arms.
    // rLib::startIntake(); //starts rotating the intake.
    rLib::deployBot();
    rLib::stopIntake();
    rLib::defaultRamp();
    //drive forward and intake cubes.
    rLib::startIntake();
    rLib::drive(30, directionType::fwd, 40);
    //turn around and drive to the score zone.
    rLib::stopIntake();
    sensor.resetHeading();
    sensor.resetRotation();
    waitUntil(!backLeft.isSpinning());
    
    rLib::turn(25, 140, true);
    waitUntil(!backLeft.isSpinning());
    rLib::startIntake();
    rLib::drive(40, directionType::fwd, 28);
    
    rLib::stack();
    rLib::startOuttake();
    rLib::drive(30,directionType::rev,20);
    waitUntil(!backLeft.isSpinning());
    rLib::stopIntake();
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
  }else if(auton == 5){

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
  pre_auton();
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


  Controller.Axis2.changed(rLib::startRampAxis);
  
  Controller.ButtonRight.pressed(rLib::defaultRamp);
  Controller.ButtonUp.pressed(rLib::proportionalRampUp);
  Controller.ButtonUp.released(rLib::stopRamp);
  Controller.ButtonDown.pressed(rLib::startRampDown);
  Controller.ButtonDown.released(rLib::stopRamp);
  //Controller.ButtonRight.pressed(rLib::deployBot);

  Controller.ButtonLeft.released(rLib::macroStack);
  //Run the pre-autonomous function.
  
  //Prevent main from exiting with an infinite loop.
  while(1) {
    task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
  }
}
