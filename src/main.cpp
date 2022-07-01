#include <iostream>
#include <math.h>
#include <cmath>

#include <robot.h>
#include <envir.h>
#include <sensor.h>
#include <sensor_range.h>
#include <sensor_bearing.h>

using namespace std;
using namespace arpro;

int main(int argc, char **argv)
{

  // default environment with moving target
  Environment envir;
  // sensors gets measurements from this environment
  Sensor::setEnvironment(envir);

  // init robot at (0,0,0)
  Robot robot1("Chitti Robo", 0,0,0);
  Robot robot2("Kutti Robo", 0,0,0);

  RangeSensor rangeSensor(robot1, 0.1, 0, 0);
  SensorBearing bearingSensor(robot2, 0.1, 0, 0);

  auto r1 = 0.07; // wheel radius
  auto b = 0.3; // base distance
  auto omegaLimit = 10; // wheel velocity limit
  robot1.initWheel(r1, b, omegaLimit);

  auto r2 = 0.05; // wheel radius
  robot2.initWheel(r2, b, omegaLimit);

  envir.addRobot(robot1);
  envir.addRobot(robot2);

  // simulate 100 sec
  while(envir.time() < 100)
  {
    cout << "---------------------" << endl;

    // update target position
    envir.updateTarget();

    // try to follow target
    robot1.goTo(envir.target());

    robot2.moveWithSensor(Twist(0.4,0,0));

  }

  // plot trajectory
  envir.plot();

}
