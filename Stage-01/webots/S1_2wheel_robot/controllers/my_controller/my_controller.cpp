#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>

#define TIME_STEP 32
#define MAX_SPEED 5.0

using namespace webots;

//inticial device setting
Robot *robot = new Robot();
   
int colour_detecting(){
      Camera *camera_s = robot->getCamera("camera");
      camera_s->enable(TIME_STEP);
      const int width = camera_s->getWidth();
      const int height = camera_s->getHeight();
     
      while (robot->step(TIME_STEP) != -1) {
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
               return 1;}
           else if(blue>(red+green)){
               return 2;}
           else if(green>(blue+red)){
               return 3;}
           else{ 
               return  0;}
         }
   }
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

int piller_detecting() {
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
    int t= robot->step(TIME_STEP) ;
     value_top = sensor_top->getValue();
     value_bottom = sensor_bottom->getValue();
     int value_right = find_value(value_top,value_bottom);
     int x =abs(value_right-value_left);
     std::cout << "Sensor right is: " << x << std::endl;
}

int main(int argc, char **argv) {
     Motor *leftMotor = robot->getMotor("left_motor");
     Motor *rightMotor = robot->getMotor("right_motor");
     leftMotor->setPosition(INFINITY);
     rightMotor->setPosition(INFINITY);
     leftMotor->setVelocity(0 * MAX_SPEED);
     rightMotor->setVelocity(0 * MAX_SPEED);
     
     // get a handler to the motors and set target position to infinity (speed control)
     DistanceSensor *sensor_right = robot->getDistanceSensor("right_ir");
     DistanceSensor *sensor_left = robot->getDistanceSensor("left_ir");
     DistanceSensor *sensor_center = robot->getDistanceSensor("center_ir");
     sensor_right->enable(TIME_STEP);
     sensor_left->enable(TIME_STEP);
     sensor_center->enable(TIME_STEP);
     
      while (robot->step(TIME_STEP) != -1){
              const double value_right = sensor_right->getValue();
              const double value_left = sensor_left->getValue();
              const double value_center = sensor_center->getValue();
             
              if((value_right-value_center<100) and (value_left-value_center<100)){
                 leftMotor->setVelocity(0.9 * MAX_SPEED);
                 rightMotor->setVelocity(0.9 * MAX_SPEED);
                 }
               else{
                 break;}}
      while (robot->step(TIME_STEP) != -1){
              const double value_right = sensor_right->getValue();
              const double value_left = sensor_left->getValue();
              const double value_center = sensor_center->getValue();
             
              if((value_right>(value_center+300)) and (value_left>(value_center+300))){
                 leftMotor->setVelocity(0.9 * MAX_SPEED);
                 rightMotor->setVelocity(0.9 * MAX_SPEED);
                 }
               else{
               int colour=colour_detecting();
                  std::cout << "colour"<< colour << std::endl;
                 break;}       
                 
       };
       
        double start_time,current_time=0;
        start_time= robot->getTime();
        double period = 1.1;
        
        while (robot->step(TIME_STEP) != -1){
                  current_time = robot->getTime();
                  if  (start_time+period>current_time){
                        
                        leftMotor->setVelocity(0.9 * MAX_SPEED);
                        rightMotor->setVelocity(0.9 * MAX_SPEED);}
                  else{
                  
                  break;}
                 }
        while (robot->step(TIME_STEP) != -1){
              const double value_right = sensor_right->getValue();
              const double value_left = sensor_left->getValue();
              const double value_center = sensor_center->getValue();
             
              if((value_right>(value_center+300)) and (value_left>(value_center+300))){
                 leftMotor->setVelocity(0.9 * MAX_SPEED);
                 rightMotor->setVelocity(0.9 * MAX_SPEED);
                 }
               else{
                 break;}       
                 
       };
       
        current_time=0;
        start_time= robot->getTime();
        
         period = 1.1;
        
        while (robot->step(TIME_STEP) != -1){
                  current_time = robot->getTime();
                  if  (start_time+period>current_time){
                        //std::cout << "haha"<< current_time << std::endl;
                        leftMotor->setVelocity(0.9 * MAX_SPEED);
                        rightMotor->setVelocity(0.9 * MAX_SPEED);}
                  else{break;}
                 }
         while (robot->step(TIME_STEP) != -1){
              const double value_right = sensor_right->getValue();
              const double value_left = sensor_left->getValue();
              const double value_center = sensor_center->getValue();
             
              if((value_right>(value_center+300)) and (value_left>(value_center+300))){
                 leftMotor->setVelocity(0.9 * MAX_SPEED);
                 rightMotor->setVelocity(0.9 * MAX_SPEED);
                 }
               else{
                //leftMotor->setVelocity(0.0 * MAX_SPEED);
                 //rightMotor->setVelocity(0.0 * MAX_SPEED);
                 break;}       
                 
             };
         current_time=0;
         start_time= robot->getTime();
        
         period = 5/(15*0.9);
        
        while (robot->step(TIME_STEP) != -1){
                  current_time = robot->getTime();
                  if  (start_time+period>current_time){
                        //std::cout << "haha"<< current_time << std::endl;
                        leftMotor->setVelocity(0.9 * MAX_SPEED);
                        rightMotor->setVelocity(0.9 * MAX_SPEED);}
                  else{break;}
                 }
                 
                 
        current_time=0;
        start_time= robot->getTime();
        
         period = 2*3.14/(0.9*6);
        //std::cout << "haha"<< start_time << std::endl;
        while (robot->step(TIME_STEP) != -1){
                  current_time = robot->getTime();
                  if  (start_time+period>current_time){
                        //std::cout << "haha"<< current_time << std::endl;
                        leftMotor->setVelocity(0.9 * MAX_SPEED);
                        rightMotor->setVelocity(-0.9 * MAX_SPEED);}
                  else{break;}
                 }  
        while (robot->step(TIME_STEP) != -1){
              const double value_right = sensor_right->getValue();
              const double value_left = sensor_left->getValue();
              const double value_center = sensor_center->getValue();
             
              if((value_right>(value_center+300)) and (value_left>(value_center+300))){
                 leftMotor->setVelocity(0.9 * MAX_SPEED);
                 rightMotor->setVelocity(0.9 * MAX_SPEED);
                 }
               else{
                //leftMotor->setVelocity(0.0 * MAX_SPEED);
                 //rightMotor->setVelocity(0.0 * MAX_SPEED);
                 break;}       
                 
             };
        current_time=0;
        start_time= robot->getTime();
        
         period = 5/(15*0.9);
        
        while (robot->step(TIME_STEP) != -1){
                  current_time = robot->getTime();
                  if  (start_time+period>current_time){
                        //std::cout << "haha"<< current_time << std::endl;
                        leftMotor->setVelocity(0.9 * MAX_SPEED);
                        rightMotor->setVelocity(0.9 * MAX_SPEED);}
                  else{break;}
                 }
                 
                 
        current_time=0;
        start_time= robot->getTime();
         period = 2*3.14/(0.9*6);
        //std::cout << "haha"<< start_time << std::endl;
        while (robot->step(TIME_STEP) != -1){
                  current_time = robot->getTime();
                  if  (start_time+period>current_time){
                        //std::cout << "haha"<< current_time << std::endl;
                        leftMotor->setVelocity(-0.9 * MAX_SPEED);
                        rightMotor->setVelocity(0.9 * MAX_SPEED);}
                  else{break;}
                 }
          while (robot->step(TIME_STEP) != -1){
              const double value_right = sensor_right->getValue();
              const double value_left = sensor_left->getValue();
              const double value_center = sensor_center->getValue();
             
              if((value_right>(value_center+300)) and (value_left>(value_center+300))){
                 leftMotor->setVelocity(0.9 * MAX_SPEED);
                 rightMotor->setVelocity(0.9 * MAX_SPEED);
                 }
               else if (value_right>900 and value_left<400){
                leftMotor->setVelocity(0.9 * MAX_SPEED);
                rightMotor->setVelocity(-0.5 * MAX_SPEED);} 
                else if (value_right<400 and value_left>900){
                leftMotor->setVelocity(-0.5 * MAX_SPEED);
                rightMotor->setVelocity(0.9 * MAX_SPEED);}
                else{break;}}
                
        leftMotor->setVelocity(0.0 * MAX_SPEED);
        rightMotor->setVelocity(0.0 * MAX_SPEED);  
        
       delete robot;
       return 0;
}
void left_turn(){       
}






