#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>

#define TIME_STEP 32
#define MAX_SPEED 7.5

using namespace webots;
int step=0;
//inticial device setting
Robot *robot = new Robot();
Motor *leftMotor = robot->getMotor("left_motor");
Motor *rightMotor = robot->getMotor("right_motor");

DistanceSensor *sensor_right = robot->getDistanceSensor("right_ir");
DistanceSensor *sensor_left = robot->getDistanceSensor("left_ir");
DistanceSensor *sensor_center = robot->getDistanceSensor("center_ir");
     
int junction_turn[2] ={-1,0}; 


void colour_detecting(){
      Camera *camera_s = robot->getCamera("camera");
      camera_s->enable(TIME_STEP);
      const int width = camera_s->getWidth();
      const int height = camera_s->getHeight();
     
      robot->step(TIME_STEP);
          const unsigned char *image =camera_s->getImage();
          
          int red=0, blue=0, green=0;
          for (int i = width / 3; i < 2 * width / 3; i++) {
                for (int j = height / 2; j < 3 * height / 4; j++) {
                  red += camera_s->imageGetRed(image, width, i, j);
                  blue += camera_s->imageGetBlue(image, width, i, j);
                  green += camera_s->imageGetGreen(image, width, i, j);
                }}
           camera_s->disable();
           if (red>(blue+green)){
               std::cout << "petch colour is: Red " << std::endl;
               return;}
           else if(blue>(red+green)){
               std::cout << "petch colour is: blue " << std::endl;
               return;}
           else if(green>(blue+red)){
               std::cout << "petch colour is: green " << std::endl;
               return; }
           else{ 
               return;}
         
   }
   
   
int find_value(double top, double bottom){
         int size;double value;
         std::cout << top << std::endl;
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
     std::cout << size << std::endl;
     return size;
     
     }

int piller_detecting() {
    DistanceSensor *sensor_top = robot->getDistanceSensor("top_sensor");
    DistanceSensor *sensor_bottom = robot->getDistanceSensor("bottom_sensor");
    sensor_top->enable(TIME_STEP);
    sensor_bottom->enable(TIME_STEP);
    Motor *sensorMotor = robot->getMotor("piller_detect_bottom");
    sensorMotor->setPosition(INFINITY);
    sensorMotor->setVelocity(0);
    
    double time_rotate=3.28/MAX_SPEED;
    
    robot->step(TIME_STEP) ;
    double value_top = sensor_top->getValue();
    double value_bottom = sensor_bottom->getValue();
    int value_left = find_value(value_top,value_bottom);
     
     
     const double start_time = robot->getTime();
     double current_time = 0;
     while (robot->step(TIME_STEP) != -1) {
            current_time =robot->getTime();
            if(start_time+time_rotate>current_time){
                sensorMotor->setVelocity(MAX_SPEED);}
            else{
                sensorMotor->setVelocity(0);
            break;}
            
     }
     robot->step(TIME_STEP) ;
     value_top = sensor_top->getValue();
     value_bottom = sensor_bottom->getValue();
     int value_right = find_value(value_top,value_bottom);
     int x =(abs(value_right-value_left));
     return x;
     
}


void forword_petch(int step){
      if (step==2){
          colour_detecting();}
      while (robot->step(TIME_STEP) != -1) {
      
          const double value_right = sensor_right->getValue();
          const double value_left = sensor_left->getValue();
          const double value_center = sensor_center->getValue();
          if (abs(value_center-value_left)<20 && abs(value_right-value_center)<20){
              leftMotor->setVelocity( MAX_SPEED);
              rightMotor->setVelocity( MAX_SPEED);}
          else if (step==3){
              leftMotor->setVelocity(0* MAX_SPEED);
              rightMotor->setVelocity(0* MAX_SPEED);
              junction_turn[1]= piller_detecting();
              //std::cout << junction_turn[1] << std::endl;
              return;}
          else{
          return;}
      } 
}   
   
int main(int argc, char **argv) {
     leftMotor->setPosition(INFINITY);
     rightMotor->setPosition(INFINITY);
     leftMotor->setVelocity(0 * MAX_SPEED);
     rightMotor->setVelocity(0 * MAX_SPEED);
     
     // get a handler to the motors and set target position to infinity (speed control)
     
     sensor_right->enable(TIME_STEP);
     sensor_left->enable(TIME_STEP);
     sensor_center->enable(TIME_STEP);
  
  while (robot->step(TIME_STEP) != -1) {
    const double value_right = sensor_right->getValue();
    const double value_left = sensor_left->getValue();
    const double value_center = sensor_center->getValue();
    
    if (abs(value_center-value_left)<20 && abs(value_right-value_center)<20){
          step+=1;
          if(step<4){forword_petch (step);}}
    else if(abs(value_center-value_left)>320 && abs(value_right-value_center)>320){
          leftMotor->setVelocity( MAX_SPEED);
          rightMotor->setVelocity( MAX_SPEED);}
    else{break;}
    
  };
  // Enter here exit cleanup code.

  delete robot;
  return 0;
}

