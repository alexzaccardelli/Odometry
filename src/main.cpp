#include "vex.h"

vex::brain Brain;
vex::controller con;
vex::encoder r(Brain.ThreeWirePort.A);
vex::encoder l(Brain.ThreeWirePort.C);

vex::motor l1(vex::PORT1, vex::ratio18_1, false);
vex::motor l2(vex::PORT2, vex::ratio18_1, false);
vex::motor r1(vex::PORT3, vex::ratio18_1, true);
vex::motor r2(vex::PORT4, vex::ratio18_1, true);

double totalX=0, totalY=0;
double totalTheta=0;

int tracker() {

  /*
  sL/sR: Distance from left/right wheel to tracking center
  dL/dR: Change left/right wheel distance in inches since last update
  lastL/lastR: Last updated left/right wheel distance in inches
  dTheta: Change in robot angle in _____ since last update
  totalTheta: Absolute robot angle in _____
  dDist: Change in distance in inches since last update or 'r' in polar coordinates
  avgTheta: Angle used for polar coordinates
  totalX/totalY: Absolute rectangular coordinates for robot
  */
  double sL = 1, sR = 1; //Temporary
  double lastL=0, lastR=0;
  double dL=0, dR=0;
  double dTheta=0;
  double dDist=0;
  double avgTheta=0;
  double dX=0, dY=0;

  while(1) {

    //Convert encoder ticks since last update into distance in inches
    dL = (l.rotation(vex::degrees) - lastL) / 360.0 * (M_PI * 2.75);
    dR = (r.rotation(vex::degrees) - lastR) / 360.0 * (M_PI * 2.75);

    //Set left rotation to last updated ticks
    lastL = l.rotation(vex::degrees);
    lastR = r.rotation(vex::degrees);
    
    //Calculate change in robot angle since last update
    dTheta = (lastL - lastR) / (sL + sR) - totalTheta;

    totalTheta = (lastL - lastR) / (sL + sR);

    //Calcualte distance robot has traveled (length of arc and 'r' for polar coordinates)
    if(totalTheta == 0) {
      dDist = dR;
    }
    else {
      dDist = 2 * sin(dTheta / 2.0) * (dR / dTheta + sR);
    }

    //Calculate angle robot for translational shift (theta for polar coordinates)
    avgTheta = totalTheta + (dTheta / 2.0);

    //Convert from polar coordinates to rectangular coordinates for changes in x and y
    dX = dDist * cos(avgTheta);
    dY = dDist * sin(avgTheta);

    //Add change in x and y to absolute rectangular coordinates
    totalX += dX;
    totalY += dY;

    vex::wait(5, vex::msec);
  }
  return 1;
}

int drive() {
  while(1) {
    double leftY = con.Axis3.position();
    double rightY = con.Axis2.position();

    l1.spin(vex::forward, leftY, vex::pct);
    l2.spin(vex::forward, leftY, vex::pct);
    r1.spin(vex::forward, rightY, vex::pct);
    r2.spin(vex::forward, rightY, vex::pct);

    vex::wait(5, vex::msec);
  }
  return 1;
}

//Calculate distance from current point to target point
double distanceToPoint(double targetX, double targetY) {
  double x = totalX, y = totalY;
  return fabs(sqrt((targetX - x) + (targetY - y)));
}

//Calculate angle from current point to target point
double angleToPoint(double targetX, double targetY) {
  double x = totalX, y = totalY;
  return atan2(targetX - x, targetY - y);
}

//P-controller (maybe add ID) for distance error
double getDistVel(double error, double max, double accel) {
  double kP = 0.5; //Temporary
  double vel = error * kP;
  if(vel > max) vel = max;
  if(vel < -max) vel = -max;
  return vel;
}

//P-controller (maybe add ID) for angle error
double getAngleVel(double error, double max, double accel) {
  double kP = 0.5; //Temporary
  double vel = error * kP;
  if(vel > max) vel = max;
  if(vel < -max) vel = -max;
  return vel;
}

//Use Calculated velocities move chassis
void moveChassis(double distVel, double angleVel) {
  double leftVel = distVel + angleVel;
  double rightVel = distVel - angleVel;

  //I don't understand this part
  double maxVel = std::max(fabs(leftVel), fabs(rightVel));

  if(maxVel > 100) {
    leftVel /= maxVel;
    rightVel /= maxVel;
  }

  l1.spin(vex::forward, leftVel, vex::pct);
  l2.spin(vex::forward, leftVel, vex::pct);
  r1.spin(vex::forward, rightVel, vex::pct);
  r2.spin(vex::forward, rightVel, vex::pct);
}

//Move from currnet point to target point
int move(double targetX, double targetY, double kDist, double kAngle) {

  double angleError=0, distError=0;
  double settleRadius=5;

  while(1) {
    
    //Get distance and angle error
    distError = distanceToPoint(targetX, targetY);
    angleError = angleToPoint(targetX, targetY);

    //If the angle is behind the robot, add 90 degrees to drive backwards
    if(fabs(angleError) > 90.0)
      distError = -distError;

    //I don't understand this part
    if(distError < settleRadius)
      angleError = 0;
    else
      angleError += 90.0;

    //Calculate distance and angle velocities
    double angleVel = getAngleVel(angleError, 100, 0);
    double distVel = getDistVel(distError, 100, 0);

    //Scale and move motors with given velocities
    moveChassis(distVel * kDist, angleVel * kAngle);
  }
}

int display() {
  Brain.Screen.clearScreen();
  while(1) {
    
    Brain.Screen.printAt(0, 0, "%*s: %f", 10, "X", totalX);
    Brain.Screen.printAt(0, 1, "%*s: %f", 10, "Y", totalY);
    Brain.Screen.printAt(0, 2, "%*s: %f", 10, "Angle", totalTheta);

    vex::wait(5, vex::msec);
  }
}

int main() {
  vex::task trackerTask(tracker);

  while(true) {vex::wait(5, vex::msec);}
}
