#include "vex.h"
#include "math.h"
#include <vector>
using namespace vex;

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
class RMotor : public motor{
  public:
    RMotor(int32_t PORT) :motor(PORT) {}
    RMotor(int32_t PORT,bool reversed) :motor(PORT,reversed){}
    double stop(brakeType b){
      this->motor::stop(b);
      return this->motor::rotation(rotationUnits::deg)  * 4 * M_PI; //returns distance travelled by the robot in inches.
    }
    void spin(directionType d, double velocity, velocityUnits v, double maxSpeed){
      if(velocity > maxSpeed){
        velocity = maxSpeed;
        this->motor::spin(d,velocity,v);
      }
      else{
        velocity = maxSpeed;
        this->motor::spin(d,velocity,v);
      }
    }
    void spin(directionType d, double velocity, velocityUnits v){
      this->motor::spin(d,velocity,v);
    }
    void spin(directionType d){
      this->motor::spin(d);
    }

};
brain Brain = brain();
controller Controller = controller();
RMotor backLeft = RMotor(PORT11);
RMotor backRight = RMotor(PORT1);
RMotor frontLeft = RMotor(PORT20, true);
RMotor frontRight = RMotor(PORT10, true);
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
bool isSkills = true;
const double defaultDeg = 285.6;
enum Auton {
  deploy,
  onePoint,
  fivePointRed,
  fivePointBlue,
  skills,
  reliableSkills,
  pid
};
enum towerSize {
  allianceTower,
  middleTower
};
Auton auton = pid;
towerSize tower;
class rLib { //This is a library of methods that the robot can perform. The methods listed here are used throughout the robot's autonomous and driver functions.
  public:
  
    //Called during auton. Opens the robot to functioning mode.
    static void deployBot(){ 
        raiseArms(900);
        stopArms();
        dropArms(450);
        //waitUntil(!liftMotor.isSpinning());
        startOuttake();
        task::sleep(100);
        dropArms(450);
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
    static void deployGoForward(){
      if(auton == skills && isSkills){ 
        deployBot();
        // task::sleep(150);	
        // //drive forward and intake cubes.	
        // rLib::startIntake();
        // if(sensor.rotation() < -0.5){
        //   turn(5, 0.4, true);	
        // }
        // Controller.Screen.print("hewwo");
        // drive(15, directionType::fwd, 2.5);
        // Controller.Screen.print("why would");
        // drive(30, directionType::fwd, 35);
        // Controller.Screen.print("hewwo2");

        // //turn around and drive to the score zone.
        // stopIntake();
        isSkills = false;
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
      startOuttake();
      task::sleep(400);
      stopIntake();
      toggleSensitive();
      liftRamp(1600);
      rampMotor.stop(brakeType::hold);
      // dropRamp(1450);
      waitUntil(!liftMotor.isSpinning());
      rampMotor.stop(brakeType::coast);
      toggleSensitive();
      // startOuttake();
      // drive(25,directionType::rev,10);
      // stopIntake();
    }
    static void alignArms(towerSize t){
      if(t == middleTower)
      {
        raiseArms(1000);
      }else if(t == allianceTower)
      {
        raiseArms(800);
      }
    }
    static void macroStack(){
      // toggleSensitive();
      liftRamp(1600);
      rampMotor.stop(brakeType::hold);
      // dropRamp(1450);
      waitUntil(!liftMotor.isSpinning());
      drive(10,directionType::rev,10);
      rampMotor.stop(brakeType::coast);
      // toggleSensitive();
      
      // rLib::drive(5,directionType::fwd,1);
      // rLib::drive(30,directionType::rev,20);
      
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
      while(rampMotor.rotation(rotationUnits::deg) < degrees - 50){
        rampMotor.setVelocity(100-rampMotor.rotation(rotationUnits::deg)/proportion,velocityUnits::pct);
        rampMotor.spin(directionType::fwd);
        Controller.Screen.clearLine();
        
      }
      Controller.Screen.print(rampMotor.rotation(rotationUnits::deg));
      rampMotor.setRotation(degrees, rotationUnits::deg);
      
    }
    static void defaultRamp(){
      if(!isSkills){
      rampMotor.rotateTo(defaultDeg, rotationUnits::deg);
      }
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
    static void spinChassis(directionType d){
      backLeft.spin(d);
      backRight.spin(d);
      frontLeft.spin(d);
      frontRight.spin(d);
    }
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
    static void fixCubes(int ms) {
      startOuttake();
      task::sleep(ms);
      startIntake();
    }
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
    static double getDistanceTravelled(){
      return ((backLeft.rotation(rotationUnits::rev) + backRight.rotation(rotationUnits::rev) + frontLeft.rotation(rotationUnits::rev) + frontRight.rotation(rotationUnits::rev))/4) * M_PI * 4;
    }
    static void pidDrive(double distance, double maxSpeed){
      double KP = 75.85;
      // double KI = 0;
      double KD = 0;
      double P = 0;
      // double I = 0;
      double D = 0;
      double KPDrift = 75.85;
      // double KI = 0;
      double KDDrift = 0;
      double PDrift = 0;
      // double I = 0;
      double DDrift = 0;
      // double previousIntegral = 0;
      int iterationTime = 1;
      double previousError = 0;
      double error = distance;
      double previousGyroError = 0;
      double gyroError = 0;
      double distanceTravelled = 0;
      double output = 0;
      double gyroOutput = 0;
      while(error != 0){
        distanceTravelled = getDistanceTravelled();
        error = distance - distanceTravelled;
        gyroError = 0 - sensor.rotation();
        P = error;
        PDrift = gyroError;
        DDrift = (gyroError - previousGyroError)/iterationTime;
        // I = previousIntegral + error * iterationTime;
        D = (error - previousError)/iterationTime;
        output = P * KP + D * KD;
        gyroOutput = PDrift * KPDrift + DDrift * KDDrift;
        previousError = error;
        
        // previousIntegral = I;
        if(gyroError < 0){
          backLeft.setVelocity(gyroOutput,vex::velocityUnits::pct);
          backRight.setVelocity(gyroOutput,vex::velocityUnits::pct);
          frontLeft.setVelocity(100 - gyroOutput,vex::velocityUnits::pct);
          frontRight.setVelocity(100 - gyroOutput,vex::velocityUnits::pct);
        }else if(gyroError > 0){
          frontLeft.setVelocity(gyroOutput,vex::velocityUnits::pct);
          frontRight.setVelocity(gyroOutput,vex::velocityUnits::pct);
          backLeft.setVelocity(maxSpeed - gyroOutput,vex::velocityUnits::pct);
          backRight.setVelocity(maxSpeed - gyroOutput,vex::velocityUnits::pct);
        }
        backLeft.setVelocity(output,vex::velocityUnits::pct);
        backRight.setVelocity(output,vex::velocityUnits::pct);
        frontLeft.setVelocity(output,vex::velocityUnits::pct);
        frontRight.setVelocity(output,vex::velocityUnits::pct);
        spinChassis(directionType::fwd);
        task::sleep(iterationTime);
      }
    }
};
//All robot controls
class Robot{
  private:
    double positionX = 0, positionY = 0, angleFacing = 0;
    RMotor backLeftR = backLeft;
    RMotor backRightR = backRight;
    RMotor frontLeftR = frontLeft;
    RMotor frontRightR = frontRight;
    bool intakeSpinning = false;
    bool intakeDir = false; //false is intake, true is outtake.
    

  public:
    void stateIntake(bool intakeSpinning, bool intakeDir){
      this->intakeSpinning = intakeSpinning;
      this->intakeDir = intakeDir;
      if(!intakeSpinning){
        rLib::stopIntake();
      }
      else{
        if(intakeDir)
        {
          rLib::startIntake();
        }
        else{
          rLib::startOuttake();
        }
      }
    }
    void moveTo(double x, double y, double velocity){
      double xDistance = x - positionX;
      double yDistance = y - positionY;
      double turnAngle = atan(xDistance/yDistance);
      double distanceNeeded = sqrt(xDistance * xDistance + yDistance * yDistance);
      bool turnDir = xDistance > 0 ? true : false;
      rLib::turn(velocity,turnAngle,turnDir);
      rLib::pidDrive(distanceNeeded, velocity);
      positionX = x;
      positionY = y;
      angleFacing = sensor.rotation();
    }
    void displayRobot(int x, int y){
      Brain.Screen.drawCircle(x,y,5,ClrGray);
    }
    void setX(double x){
      positionX = x;
    }
    void setY(double y){
      positionY = y;
    }
};
Robot s = Robot();
class Button{
  private:
    double x1,y1,x2,y2;
    using f_int_t = void(*)(Button b);
  public:
    Button(double x1,double x2,double y1,double y2){
      this->x1 = x1;
      this->x2 = x2;
      this->y1 = y1;
      this->y2 = y2;
    }
    void buttonPressed(void(*callback)(Button b)){
      
    }
};
static int x = 26, y = 25;
static int robotX = 0, robotY = 0;
static int markerX = 0, markerY = 0;
std::vector<Robot> point;
double multiplier = 0.01;
double proportionFactor = 272/144;
bool allowed = true;
class Field{
  public:
    static void constructField(){
      // Brain.Screen.drawImageFromFile("field.bmp", 0, 0);
      Brain.Screen.drawLine(0,0,272,0);
      Brain.Screen.drawLine(272,0,272,272);
      Brain.Screen.drawLine(272,272,0,272);
      Brain.Screen.drawLine(0,272,0,0);
      Brain.Screen.drawRectangle(0,0,272,272,ClrGreen);
      displayRobotMarker(markerX,markerY);
      s.displayRobot(robotX,robotY);
      Brain.Screen.printAt(400,50,"A = Print Point Location");
      Brain.Screen.printAt(400,60,"B = Place Robot Marker");
      Brain.Screen.printAt(400,70,"X = Place Location Marker");
      Brain.Screen.printAt(400,80,"Y = Confirm Movement");
    }
    // static void changeX(){
    //   x+=Controller.Axis4.value() * multiplier;
    //   if(x < 272 && x > 0){
    //     constructField();
    //     Brain.Screen.drawCircle(x, y, 5, ClrRed);
    //     Brain.Screen.drawCircle(x,y,5,ClrGray);
        
    //     if(markerX != 0 && markerY != 0){
    //       displayRobotMarker(markerX,markerY);
    //     }
    //     s.displayRobot(robotX,robotY);
    //   }
    //   else{
    //     x-=Controller.Axis3.value() * multiplier;
    //   }
    // }
    static void changeX(){
      x+=Controller.Axis4.value() * multiplier;
      if(x < 272 && x > 0){
        constructField();
        Brain.Screen.drawCircle(x,y,5,ClrWhite);
      }
      else{
        x-=Controller.Axis3.value() * multiplier;
      }
    }
    static void bigBrain(){
      while(true)
      {
        if(Controller.ButtonB.pressing()){
            placeRobot(x,y);
          }
          else if(Controller.ButtonX.pressing()){
            createRobotMarker();
            displayRobotMarker(markerX,markerY);
          }
          else if(Controller.ButtonY.pressing()){
            confirmRobotMove();
            break;
          }
          else if(Controller.ButtonA.pressing()){
            printPixel();
          }
      }
    }
    // static void changeY(){
    //   y-=Controller.Axis3.value() * multiplier;
    //   if(y < 272 && y > 0){
    //     constructField();
    //     Brain.Screen.drawCircle(x, y, 5, ClrRed);
        
    //     if(markerX != 0 && markerY != 0){
    //       displayRobotMarker(markerX,markerY);
    //     }
    //     s.displayRobot(robotX,robotY);
    //   }
    //   else{
    //     y+=Controller.Axis3.value() * multiplier;
    //   }
    // }
    static void changeY(){
      y-=Controller.Axis3.value() * multiplier;
      if(y < 272 && y > 0){
        constructField();
        Brain.Screen.drawCircle(x,y,5,ClrWhite);
      }
      else{
        y+=Controller.Axis3.value() * multiplier;
      }
    }
    static void printPixel(){
      Brain.Screen.printAt(400,40,"%d, %d",x,y);
    }
    static void placeRobot(int x, int y){
      robotX = x;
      robotY = y;
      s.setX(x/proportionFactor);
      s.setY(y/proportionFactor);
      s.displayRobot(robotX,robotY);
    }
    static void createRobotMarker(){
      markerX = x;
      markerY = y;
    }
    static void displayRobotMarker(int x, int y){
      Brain.Screen.drawCircle(x,y,5,ClrBlue);
      
    }
    static void confirmRobotMove(){
      double x = (markerX - robotX)/proportionFactor;
      double y = (markerY - robotY)/proportionFactor;
      moveRobotTo(x,y,70);
    }
    static void moveRobotTo(double x, double y, double velocity){
      s.moveTo(x,y,velocity);
      robotX = markerX;
      robotY = markerY;
      s.displayRobot(robotX,robotY);
    }
    //25,24 TOPLEFT
    //223,24 TOPRIGHT
    //223,221 BOTTOMRIGHT
    //25,221 BOTTOMLEFT
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

bool redSide = false;
const bool leftTurn = false;
const bool rightTurn = true;

void autonomous(void) {
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
    if(sensor.rotation() < -0.75){
      rLib::turn(5, 0.4, rightTurn);	
    }
    //drive forward and intake cubes.
    rLib::startIntake();
    // rLib::turn(25, 2, rightTurn);
    rLib::drive(15, directionType::fwd, 2.5);
    rLib::drive(30, directionType::fwd, 35);
    //turn around and drive to the score zone.
    rLib::stopIntake();
    sensor.resetHeading();
    sensor.resetRotation();
    waitUntil(!backLeft.isSpinning());
    
    rLib::turn(25, 139, leftTurn);
    waitUntil(!backLeft.isSpinning());
    rLib::startIntake();
    rLib::drive(40, directionType::fwd, 20);
    rLib::drive(20, directionType::fwd, 8.25);

    rLib::stack();
    
    rLib::startOuttake();
    rLib::drive(30,directionType::rev,20);
    waitUntil(!backLeft.isSpinning());
    rLib::stopIntake();
    rLib::defaultRamp();
  }else if(auton == fivePointRed){
    rLib::deployBot();
    task::sleep(100);	
    //drive forward and intake cubes.	
    rLib::startIntake();
    if(sensor.rotation() < -0.5){
      rLib::turn(5, 0.4, rightTurn);	
    }
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
    rLib::stack();
    rLib::drive(5,directionType::fwd,1);
    rLib::startOuttake();
    rLib::drive(30,directionType::rev,20);
    waitUntil(!backLeft.isSpinning());
    rLib::stopIntake();
    rLib::defaultRamp();
  }else if(auton == skills){
    rLib::deployBot();
    rLib::startIntake();
    rLib::drive(10,directionType::fwd,5);
    rLib::drive(25,directionType::fwd,40);
    rLib::fixCubes(50);
    rLib::drive(30,directionType::fwd,24);
    rLib::turn(30,3,rightTurn);
    rLib::drive(30,directionType::fwd,24);
    sensor.resetRotation();
    rLib::turn(50,6,leftTurn);
    // sensor.resetRotation();
    // rLib::turn(20,8,leftTurn);
    // rLib::drive(100,directionType::rev,5);
    // rLib::drive(100,directionType::fwd,5);
    //around the tower
    // rLib::turn(10,10,!leftTurn);
    // rLib::stopIntake();
    // rLib::turn(20,10,rightTurn);
    // rLib::startIntake();
    // rLib::drive(30,directionType::fwd,10);
    // rLib::turn(20,10,leftTurn);
    // rLib::turn(10,10,!rightTurn);
    // rLib::drive(20,directionType::fwd,20);
    // sensor.resetRotation();
    // rLib::turn(10,20,!leftTurn);
    // rLib::drive(20,directionType::fwd,20);
    // sensor.resetRotation();
    // rLib::turn(10,10,!rightTurn);
    // rLib::startIntake();
    // rLib::drive(30,directionType::fwd,15);
    // sensor.resetRotation();
    // rLib::turn(10,10,!rightTurn);
    // rLib::drive(20,directionType::rev,5);
    // rLib::startIntake();
    // task::sleep(300);
    // rLib::stopIntake();
    // rLib::drive(20,directionType::fwd,30);
    // sensor.resetRotation();
    // rLib::turn(20,45,!rightTurn);
    // rLib::stack();
    // rLib::startOuttake();
    // rLib::drive(10,directionType::rev,10);
    // rLib::defaultRamp();
  }else if(auton == reliableSkills){
    rLib::deployBot();
    task::sleep(150);	
    //drive forward and intake cubes.	
    rLib::startIntake();
    if(sensor.rotation() < -0.5){
      rLib::turn(5, 0.4, rightTurn);	
    }
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
    rLib::stack();
    rLib::drive(5,directionType::fwd,1);
    rLib::startOuttake();
    rLib::drive(30,directionType::rev,26.75);
    waitUntil(!backLeft.isSpinning());
    rLib::stopIntake();
    rampMotor.rotateTo(0,rotationUnits::deg,true);
    sensor.resetRotation();
    rLib::turn(25, 149.5, leftTurn);
    rLib::startIntake();
    rLib::drive(30,directionType::fwd,7);
    rLib::stopIntake();
    rLib::drive(30,directionType::fwd,5);
    rLib::drive(30,directionType::rev,14);
    // rLib::startOuttake();
    task::sleep(100);
    // rLib::stopIntake();
    rLib::raiseArms(1200);
    rLib::drive(10,directionType::fwd,8);
    // rLib::toggleSensitive();
    rLib::startOuttake();
    task::sleep(1000);
    rLib::drive(10,directionType::rev,3);
    rLib::stopIntake();
    // rLib::toggleSensitive();

  }
  else if(auton == pid){ //also for testing position tracking.
    // rLib::deployBot();
    // s.stateIntake(true,true); //intake is running
    // rLib::pidDrive(10,70);
    // s.moveTo(62,0,70); //x, y, maxVelocity.
    Field::constructField();
    
    Controller.Axis4.changed(Field::changeX);
    Controller.Axis3.changed(Field::changeY);
    Field::bigBrain();
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
    double leftVelocity = (Controller.Axis3.value() - Controller.Axis4.value()) * sensitivity * .8;
    double rightVelocity = (Controller.Axis3.value() + Controller.Axis4.value()) * sensitivity * .8;
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
  // pre_auton();
  autonomous();
  // //Set up callbacks for autonomous and driver control periods.
  // Competition.autonomous( autonomous );
  // Competition.drivercontrol( usercontrol );

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
  Controller.ButtonA.released(rLib::deployGoForward);

  Controller.ButtonLeft.released(rLib::macroStack);
  //Run the pre-autonomous function.
  
  //Prevent main from exiting with an infinite loop.
  while(1) {
    task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
  }
}
