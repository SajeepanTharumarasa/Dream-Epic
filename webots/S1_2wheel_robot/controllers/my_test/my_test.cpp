#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/camera.hpp>
#include <webots/Camera.hpp>
#include <iostream>

#define TIME_STEP 32
#define MAX_SPEED 7.5

using namespace webots;
Robot *robot = new Robot();

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

int main() {
    DistanceSensor *sensor_top = robot->getDistanceSensor("top_sensor");
    DistanceSensor *sensor_bottom = robot->getDistanceSensor("bottom_sensor");
    sensor_top->enable(TIME_STEP);
    sensor_bottom->enable(TIME_STEP);
    Motor *sensorMotor = robot->getMotor("piller_detect_bottom");
    sensorMotor->setPosition(INFINITY);
    sensorMotor->setVelocity(0);
    double time_rotate=3.275/MAX_SPEED;
    
    robot->step(TIME_STEP) ;
    double value_top = sensor_top->getValue();
    double value_bottom = sensor_bottom->getValue();
    std::cout << "Sensor left is: " << value_bottom << std::endl;
    int value_left = find_value(value_top,value_bottom);
     
     
     double start_time = robot->getTime();
     double current_time = start_time;
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
     int x =abs(value_right-value_left);
     std::cout << "Sensor right is: " << x << std::endl;
    
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