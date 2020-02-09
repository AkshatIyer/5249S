#include "vex.h"

using namespace vex;

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
controller Controller = controller();
motor backLeft = motor(PORT11);
motor backRight = motor(PORT1);
motor frontLeft = motor(PORT20, true);
motor frontRight = motor(PORT10, true);
motor liftMotor = motor(PORT7);
motor rampMotor = motor(PORT4);
motor intakeLMotor = motor(PORT3, true);
motor intakeRMotor = motor(PORT5);
inertial sensor = inertial(PORT6);
// ---- END VEXCODE CONFIGURED DEVICES ----

// A global instance of vex::competition
competition Competition;

double intakeSens = 1;
double sensitivity = 1;
const double defaultDeg = 285.6;
//All robot controls
class rLib { //This is a library of methods that the robot can perform. The methods listed here are used throughout the robot's autonomous and driver functions.
  public:
  
    //Called during auton. Opens the robot to functioning mode.
    static void deployBot(){ 
        raiseArms(700);
        stopArms();
        dropArms(450);
        //waitUntil(!liftMotor.isSpinning());
        startOuttake();
        task::sleep(1000);
        dropArms(250);
        stopIntake();
      
      // task::sleep(50);
      // liftRamp(600);
      // dropRamp(600);
      // task::sleep(50);
      // dropArms(350);
      //n00b
      // startOuttake();
      // dropArms(500);
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
    //Set of instructions to stack cubes for autonomous.
    static void stack(){
      startOuttake();
      task::sleep(400);
      stopIntake();
      toggleSensitive();
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
    //Macro for stacking cubes and backing up afterwards. 
    }
    static void macroStack(){
      stack();
      startOuttake();
      drive(30,directionType::rev,30);
      waitUntil(!backLeft.isSpinning());
      stopIntake();
    }
    //Pushes the ramp up quickly but slows down near the end. 
    //THIS MACRO IS UNUSED AS OF NOW
    static void proportionalRampUp(){
      double degrees = 1450;
      double proportion = (degrees - rampMotor.rotation(rotationUnits::deg))/100;
      while(rampMotor.rotation(rotationUnits::deg) < degrees - 50){
        rampMotor.setVelocity(100-rampMotor.rotation(rotationUnits::deg)/proportion,velocityUnits::pct);
        rampMotor.spin(directionType::fwd);
        Controller.Screen.clearLine();
        
      }
      Controller.Screen.print(rampMotor.rotation(rotationUnits::deg));
      rampMotor.setRotation(degrees, rotationUnits::deg);
      
    }
    //Sets the ramp at the default position.
    static void defaultRamp(){
      rampMotor.rotateTo(defaultDeg, rotationUnits::deg);
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
    static void turn(double velocity, double degrees, bool dirRight){
      sensor.setHeading(0,rotationUnits::deg); //Resets the gyroscope initial angle.
      backLeft.setVelocity(velocity, velocityUnits::pct);
      frontLeft.setVelocity(velocity, velocityUnits::pct);
      backRight.setVelocity(velocity, velocityUnits::pct);
      frontRight.setVelocity(velocity, velocityUnits::pct);
      //determines direction needed for each motor depending on a boolean parameter.
      directionType backDir = dirRight ? directionType::fwd : directionType::rev;
      directionType frontDir = dirRight ? directionType::rev : directionType::fwd;
      //Spins the motors until the gyroscope reading is greater than or equal to the desired turn angle.
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
      Controller.Screen.print(sensor.rotation()); //Prints the gyroscope reading for development purposes.
      stopDrive();
      
      sensor.setHeading(0,rotationUnits::deg); //Resets the gyroscope reading for the next turn. 
    }
    //An Autonomous method that drives a bot based on the specified velocity, direction, and inches wanted to travel.
    static void drive(double velocity, directionType dir, double inches){
      double degrees = inches * (360/12);
      
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
      intakeLMotor.spin(directionType::fwd, 85*sensitivity, percentUnits::pct);
      intakeRMotor.spin(directionType::fwd, 85*sensitivity, percentUnits::pct);
    }
    //Starts spinning the intake motors in reverse.
    static void startOuttake() {
      intakeLMotor.spin(directionType::rev, 100*sensitivity, percentUnits::pct);
      intakeRMotor.spin(directionType::rev, 100*sensitivity, percentUnits::pct);
    }
    //Short outtake and intake to fix any cubes intaken diagonally.
    static void fixCubes(int ms) {
      startOuttake();
      task::sleep(ms);
      startIntake();
    }
    //Sets up the ramp to rotate based on the axis value of the right axis.
    static void startRampAxis() {
      rampMotor.spin(directionType::fwd, Controller.Axis2.value(), percentUnits::pct);
      if(Controller.Axis2.value() == 0){
        stopRamp();
      }
    }
    //Starts spinning the intake motors in reverse.
    static void startOuttakeFor(double seconds) {
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
      rampMotor.spin(directionType::fwd, 80*sensitivity, percentUnits::pct);
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
    //Toggles the sensitivity of JUST THE INTAKE MOTORS.
    static void toggleIntakeSens() {
      if(intakeSens == 1)
        intakeSens = .5;
      else
        intakeSens = 1;
      
    }
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
void pre_auton(void) { //Used to calibrate the gyroscope and reset motor encoders.
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
enum Auton { //A list of enumerated values for easy auton selection.
  deploy,
  onePoint,
  fivePointRed,
  fivePointBlue,
  skills
};
//Declaring which auton is going to be run and on which side it is for. 
Auton auton = skills;
bool redSide = false;
//Constants that make it easy to identify whether the turn is a left or right turn. 
const bool leftTurn = false;
const bool rightTurn = true;

void autonomous(void) {
  //solely deploys the bot for testing purposes.
  if(auton == deploy){
    rLib::deployBot();
  }else if(auton == onePoint){
    //One point reverse with preload
    rLib::drive(25, directionType::rev, 12);
    rLib::drive(30, directionType::fwd, 20);
    rLib::deployBot();
  }else if(auton == fivePointBlue){
    //5 point auton. blue = false, red = true
    rLib::deployBot();
    task::sleep(150);
    //drive forward and intake cubes.
    rLib::startIntake();
    rLib::turn(25, 2, rightTurn);
    rLib::drive(15, directionType::fwd, 2.5);
    rLib::drive(30, directionType::fwd, 35);
    //turn around and drive to the score zone.
    rLib::stopIntake();
    sensor.resetHeading();
    sensor.resetRotation();
    waitUntil(!backLeft.isSpinning());
    
    rLib::turn(25, 140, leftTurn);
    waitUntil(!backLeft.isSpinning());
    rLib::startIntake();
    rLib::drive(40, directionType::fwd, 20);
    rLib::drive(20, directionType::fwd, 8.75);
    //stack cubes and back up.
    rLib::stack();
    rLib::startOuttake();
    rLib::drive(30,directionType::rev,20);
    waitUntil(!backLeft.isSpinning());
    rLib::stopIntake();
    rLib::defaultRamp();
  }else if(auton == fivePointRed){
    rLib::deployBot();
    task::sleep(150);	
    //drive forward and intake cubes.	
    rLib::startIntake();	
    //rLib::turn(5, 0.5, rightTurn);	
    rLib::drive(15, directionType::fwd, 2.5);	
    rLib::drive(30, directionType::fwd, 35);
    //turn around and drive to the score zone.
    rLib::stopIntake();
    sensor.resetHeading();
    sensor.resetRotation();
    waitUntil(!backLeft.isSpinning());
    
    rLib::turn(25, 140, rightTurn);
    waitUntil(!backLeft.isSpinning());
    rLib::startIntake();
    rLib::drive(35, directionType::fwd, 20);	
    rLib::drive(20, directionType::fwd, 8.75);
    rLib::startOuttake();
    task::sleep(30);
    rLib::startIntake();
    //stack cubes and back up.
    rLib::stack();
    rLib::drive(5,directionType::fwd,0.5);
    rLib::startOuttake();
    rLib::drive(30,directionType::rev,20);
    waitUntil(!backLeft.isSpinning());
    rLib::stopIntake();
    rLib::defaultRamp();
  }else if(auton == skills){
    rLib::deployBot();
    //Drive forward with intake.
    rLib::startIntake();
    rLib::drive(10,directionType::fwd,5);
    rLib::drive(25,directionType::fwd,40);
    rLib::fixCubes(50); //If there are any diagonal cubes, this will fix it.
    rLib::drive(30,directionType::fwd,24);
    rLib::turn(30,3,rightTurn);
    rLib::drive(30,directionType::fwd,24);
    sensor.resetRotation();
    rLib::turn(50,6,leftTurn); //launch the cube off the top of the ramp into the tower.
    //UNFINISHED
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
    //Uses the values of the Controller Axes to calculate the velocity of the motors.
    double leftVelocity = (Controller.Axis3.value() - Controller.Axis4.value()) * sensitivity * .8;
    double rightVelocity = (Controller.Axis3.value() + Controller.Axis4.value()) * sensitivity * .8;
    //Spins the motors based on the calculated velocity.
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
  Controller.ButtonUp.pressed(rLib::startRampUp);
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
