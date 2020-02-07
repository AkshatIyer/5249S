//trying something
else if(auton == 12){
  rLib::deployBot();
  rLib::startIntake();
  rLib::drive(40,directionType::fwd,40);
  stopIntake();
  rLib::turn(10,rightTurn);
  rLib::drive(20,directionType::rev,5);
  startIntake();
  task::sleep(300);
  stopIntake();
  rLib::drive(20,directionType::fwd,30);
  rLib::turn(45,rightTurn);
  rLib::stack();
  rLib::startOuttake();
  rLib::drive(10,directionType::rev,10);
  rLib::defaultRamp();
}
