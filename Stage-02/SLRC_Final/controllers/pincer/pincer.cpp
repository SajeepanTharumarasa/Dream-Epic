#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/emitter.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <iostream>
#include <string>
using namespace std;


// All the webots classes are defined in the "webots" namespace
using namespace webots;


int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  Motor *motor = robot->getMotor("motor");
  
  motor->setPosition(INFINITY);
  motor->setVelocity(0);
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

   
  while (robot->step(timeStep) != -1) {
    motor->setVelocity(0.5);
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
