#include "vex.h"

//Assume a distance from center of 1"

vex::brain Brain;
vex::controller con;

vex::motor l1(vex::PORT1, vex::gearSetting::ratio18_1, false);
vex::motor l2(vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::motor r1(vex::PORT3, vex::gearSetting::ratio18_1, true);
vex::motor r2(vex::PORT4, vex::gearSetting::ratio18_1, true);
vex::encoder r(Brain.ThreeWirePort.A);
vex::encoder l(Brain.ThreeWirePort.C);
vex::encoder b(Brain.ThreeWirePort.E);

double const sL = 1, sR = 1, sB = 1;
double lVal, rVal, bVal, lLast, dL, dR, dB;
double absL, absR;
double angle, dAngle;
double dX, dY;
double x, y;

double distanceToPoint(double targetX, double targetY) {
  double localX = x, localY = y;
  return fabs(sqrt((targetX - localX) + (targetY - localY)));
}
double angleToPoint(double targetX, double targetY) {
  double localX = x, localY = y;
  return atan2((targetX - localX), (targetY - localY));
}

double distanceP(double error, double max) {
  double kP = 0.5;
  double speed = error * kP;
  if(speed > max) speed = max;
  if(speed < -max) speed = -max;
  return speed;
}
double angleP(double error, double max) {
  double kP = 0.5;
  double speed = error * kP;
  if(speed > max) speed = max;
  if(speed < -max) speed = -max;
  return speed;
}
void driveControl(double distanceVel, double angleVel) {
  double leftVel = distanceVel + angleVel;
  double rightVel = distanceVel - angleVel;
  l1.spin(vex::directionType::fwd, leftVel, vex::velocityUnits::pct);
  l2.spin(vex::directionType::fwd, leftVel, vex::velocityUnits::pct);
  r1.spin(vex::directionType::fwd, rightVel, vex::velocityUnits::pct);
  r2.spin(vex::directionType::fwd, rightVel, vex::velocityUnits::pct);
}
int positionTracker() {
  while(true) {
    /*
    Calculate change in tracking wheel distance
      change in left distance = (current left - last left) / 360 * (2.75 * pi)
    */
    dL = (l.rotation(vex::rotationUnits::deg) - lVal) / 360 * (M_PI * 2.75);
    dR = (r.rotation(vex::rotationUnits::deg) - rVal) / 360 * (M_PI * 2.75);
    dB = (b.rotation(vex::rotationUnits::deg) - bVal) / 360 * (M_PI * 2.75);

    /*
    Update total in tracking wheel distance
      total left distance = total left distance + change in left distance
    */
    absL += dL;
    absR += dR;
    
    /*
    Calculate change in angle
      change in angle = absolute angle - last absolute angle
    */
    dAngle = (absL - absR) / (sL + sR) - angle;

    /*
    Calculate total absolute angle relative to starting position
      absolute angle = (total left distance - total right distance) / (left distance from center + right distance from center)
    */
    angle = (absL - absR) / (sL + sR);

    /*
    Update current trackw wheel rotation
    */
    lVal = l.rotation(vex::rotationUnits::deg);
    rVal = r.rotation(vex::rotationUnits::deg);
    bVal = b.rotation(vex::rotationUnits::deg);

    /*
    Calculate change in cartesian position
      if change in right distance = change in left distance:
        change in x = absolute x + change in back distance
        change in y = absolute y + change in right distance
      else:
        change in x = chord length of arc around center radius concentric with back tracking wheel circle circle = 2 * (dB / dAngle + back distance from center) * sin(angle / 2)
        change in y = chord length of arc around center radius concentric with right tracking wheel circle = 2 * (dR / dAngle + back distance from center) * sin(angle / 2)
    */
    if(angle == 0) {
      dX = dB;
      dY = dR;
    } 
    else {
      dX = 2 * sin(angle / 2) * (dB / dAngle + sB);
      dY = 2 * sin(angle / 2) * (dR / dAngle + sR);
    }

    //Update frequency
    wait(5, vex::timeUnits::msec);
  }
  return 1;
}

void move(double targetX, double targetY, double kTurn) {
  double settleRadius = 5;
  while(true) {
    double angleError = angleToPoint(targetX, targetY);
    double distanceError = distanceToPoint(targetX, targetY);

    if(fabs(angleError) > 90.0)
      distanceError = -distanceError;
    
    if(fabs(distanceError) < settleRadius)
      angleError = 0;
    else if(distanceError < 0)
      angleError = -angleError;
    
    double angleVel = angleP(angleError, 100);
    double distanceVel = distanceP(distanceError, 100);

    driveControl(distanceVel, angleVel * kTurn);

    vex::wait(5, vex::timeUnits::msec);
  }
}

void turn(double degrees, double max, double error) {
  
}

int drive() {
  while(true) {
    double leftY = con.Axis3.position();
    double rightY = con.Axis2.position();

    l1.spin(vex::directionType::fwd, leftY, vex::velocityUnits::pct);
    l2.spin(vex::directionType::fwd, leftY, vex::velocityUnits::pct);
    r1.spin(vex::directionType::fwd, rightY, vex::velocityUnits::pct);
    r2.spin(vex::directionType::fwd, rightY, vex::velocityUnits::pct);

    vex::wait(5, vex::timeUnits::msec);
  }
}

void initScreen() {
  Brain.Screen.clearScreen();
  Brain.Screen.setOrigin(0, 0);
  Brain.Screen.setCursor(0, 0);
}

int display() {
  while(true) {
    printf();
    vex::wait(5, vex::timeUnits::msec);
  }
}

int main() {
  initScreen();
  vex::task positionTrackerTask(positionTracker);
  vex::task driveTask(drive);



  while(true) {vex::wait(5, vex::msec);}
}
