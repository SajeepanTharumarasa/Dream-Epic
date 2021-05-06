#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

#define TIME_STEP 32


#define MAX_SPEED 7.5

// All the webots classes are defined in the "webots" namespace
using namespace webots;

int main(int argc, char **argv) {
 Robot *robot = new Robot();

 // get a handler to the motors and set target position to infinity (speed control)
 Motor *leftMotor = robot->getMotor("left_motor");
 Motor *rightMotor = robot->getMotor("right_motor");
 leftMotor->setPosition(INFINITY);
 rightMotor->setPosition(INFINITY);
 leftMotor->setVelocity(0 * MAX_SPEED);
 rightMotor->setVelocity(0 * MAX_SPEED);
 
 //rotating distance sensor motor
 Motor *bottom_sensor = robot->getMotor("piller_detect_bottom");
 //Motor *top_sensor = robot->getMotor("right_motor");
 bottom_sensor->setPosition(INFINITY);
 //top_sensor->setPosition(INFINITY);

 // set up the motor speeds at 10% of the MAX_SPEED.
 bottom_sensor->setVelocity(-0.5 * MAX_SPEED);
 
 //leftMotor->setVelocity(-0.9 * MAX_SPEED);
 //rightMotor->setVelocity(0.9 * MAX_SPEED);

 while (robot->step(TIME_STEP) != -1);
 delete robot;
 return 0;
}