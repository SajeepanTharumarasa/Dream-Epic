#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>
using namespace webots;

#define TIME_STEP 32
#define MAX_SPEED 7.5    // anguler velocity
#define robot_length 12  // in cm
#define robot_width 10   // in cm
#define wheel_radius 2   // in cm
#define petch_lenght 15  // in cm 
   
double petch_center_time = 0.8*petch_lenght/(MAX_SPEED*wheel_radius);
double turn_90time = 3.14*(robot_width/2)/(MAX_SPEED*wheel_radius);
 
int step=0;
int junction_turn[2] ={1,0}; 
//inticial device setting
Robot *robot = new Robot();
Motor *leftMotor = robot->getMotor("left_motor");
Motor *rightMotor = robot->getMotor("right_motor");

DistanceSensor *sensor_right = robot->getDistanceSensor("right_ir");
DistanceSensor *sensor_left = robot->getDistanceSensor("left_ir");
DistanceSensor *sensor_center = robot->getDistanceSensor("center_ir");
DistanceSensor *sensor_left2 = robot->getDistanceSensor("left2_ir");
DistanceSensor *sensor_right2 = robot->getDistanceSensor("right2_ir");

int find_value(double top, double bottom){
         int size;double value;
         if (top<990){
               size=2;
               value = top;}
         else {
               size=1;
               value = bottom;} 
         if (value>410 && value<510 ) {
             size = size*4;} 
         else if (value>575 && value<675 ) {
             size = size*8;} 
         else if (value>740 && value<840 ) {
             size = size*12;} 
     return size;
     }
void turn_90(int direction){
     double left_speed =0;
     double right_speed =0;
     if (direction==1){
          left_speed=-1*MAX_SPEED;
          right_speed=MAX_SPEED;}
     else if (direction==-1){
          left_speed=MAX_SPEED;
          right_speed=-1*MAX_SPEED;}
     else {return;}
     
     double time = robot->getTime()+turn_90time;
     double current_time=0;
     
     while (robot->step(TIME_STEP) != -1) {
          current_time = robot->getTime();
          if (time>current_time){
              leftMotor->setVelocity( left_speed);
              rightMotor->setVelocity( right_speed);}
          else{break;}
     }
     return;    
     }

int main() {
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
     
     leftMotor->setVelocity(0);
           rightMotor->setVelocity(0);
     // get a handler to the motors and set target position to infinity (speed control)
     sensor_right->enable(TIME_STEP);
     sensor_left->enable(TIME_STEP);
     sensor_center->enable(TIME_STEP);
     sensor_right2->enable(TIME_STEP);
     sensor_left2->enable(TIME_STEP);
     
    DistanceSensor *sensor_bottom = robot->getDistanceSensor("bottom_sensor");
    sensor_bottom->enable(TIME_STEP);
    Motor *sensorMotor = robot->getMotor("piller_detect_bottom");
    Motor *right2Motor = robot->getMotor("right2");
    Motor *left2Motor = robot->getMotor("left2");
    
    sensorMotor->setPosition(INFINITY);
    sensorMotor->setVelocity(0);
    right2Motor->setPosition(INFINITY);
    right2Motor->setVelocity(0);
    left2Motor->setPosition(INFINITY);
    left2Motor->setVelocity(0);
    
     double time_rotate=(3.3/MAX_SPEED);
     const double start_time = robot->getTime();
     double current_time = 0;
     while (robot->step(TIME_STEP) != -1) {
            current_time =robot->getTime();
            if(start_time+time_rotate>current_time){
                sensorMotor->setVelocity(0.5*MAX_SPEED);
                left2Motor->setVelocity(MAX_SPEED);
                right2Motor->setVelocity(-1*MAX_SPEED);}
            else{
                sensorMotor->setVelocity(0);
                left2Motor->setVelocity(0);
                right2Motor->setVelocity(0);
                  break;}} 
    while (robot->step(TIME_STEP) != -1) {
        const double r_1 = sensor_right->getValue();
        const double l_1 = sensor_left->getValue();
        const double c_0 = sensor_center->getValue();
        const double r_2 = sensor_right2->getValue();
        const double l_2 = sensor_left2->getValue();
        const double c_top = sensor_bottom->getValue();
        if(c_0<420 && r_1>900 && l_1>900){
            leftMotor->setVelocity(0);
           rightMotor->setVelocity(0);
            break;}
        else if(c_top>550){
           leftMotor->setVelocity(MAX_SPEED);
           rightMotor->setVelocity(MAX_SPEED);}
        else{
            if (l_2==1000){
                turn_90(-1);
                continue;}
            else if(r_2==1000){
                turn_90(1);
                continue;}
           break;
        }
        
    }
    
  delete robot;
  return 0;
}


/*void test(){
  DistanceSensor *sensor_right = robot->getDistanceSensor("right_ir");
  DistanceSensor *sensor_left = robot->getDistanceSensor("left_ir");
  DistanceSensor *sensor_center = robot->getDistanceSensor("top_sensor");
  LightSensor *light = robot->getLightSensor("light_sensor");
  
  sensor_right->enable(TIME_STEP);
  sensor_left->enable(TIME_STEP);
  sensor_center->enable(TIME_STEP);
  light->enable(TIME_STEP);
    
  while (robot->step(TIME_STEP) != -1) {
    const double value_right = sensor_right->getValue();
    const double value_left = sensor_left->getValue();
    const double value_center = sensor_center->getValue();
    
    const double light_value = light->getValue();
    //std::cout << "Sensor left is: " << value_left << std::endl;
    //std::cout << "Sensor right is: " << value_right << std::endl;
    std::cout << "Sensor center is: " << light_value << std::endl;
  }
  Camera *camera_s = robot->getCamera("camera");
  camera_s->enable(TIME_STEP);
  
  const int width = camera_s->getWidth();
  const int height = camera_s->getHeight();
  std::cout << "colours"<< std::endl;
  while (robot->step(TIME_STEP) != -1) {
      const unsigned char *image =camera_s->getImage();
      int red=0, blue=0, green=0;
      for (int i = width / 3; i < 2 * width / 3; i++) {
            for (int j = height / 2; j < 3 * height / 4; j++) {
              red += camera_s->imageGetRed(image, width, i, j);
              blue += camera_s->imageGetBlue(image, width, i, j);
              green += camera_s->imageGetGreen(image, width, i, j);
            }
          }
       if (red>(blue+green)){
       std::cout <<"red" << red << std::endl;}
       else if(blue>(red+green)){std::cout <<"blue" << blue << std::endl;
       }
       else if(green>(blue+red)){std::cout << "green"<< green<< std::endl;
       }
       else{ 
       std::cout <<"red" << red << std::endl;
        std::cout <<"green" << green << std::endl;
        std::cout <<"blue" << blue << std::endl;
       break;}

   }
  
  }*/