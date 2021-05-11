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
int junction_turn[2] ={-1,0}; 
//inticial device setting
Robot *robot = new Robot();
Motor *leftMotor = robot->getMotor("left_motor");
Motor *rightMotor = robot->getMotor("right_motor");

DistanceSensor *sensor_right = robot->getDistanceSensor("right_ir");
DistanceSensor *sensor_left = robot->getDistanceSensor("left_ir");
DistanceSensor *sensor_center = robot->getDistanceSensor("center_ir");
     
void colour_detecting(){
      Camera *camera_s = robot->getCamera("camera");
      camera_s->enable(TIME_STEP);
      const int width = camera_s->getWidth();
      const int height = camera_s->getHeight();
     std::cout << "petch colour is:  " << std::endl;
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
               std::cout << "petch colour is: Red " << std::endl;}
           else if(blue>(red+green)){
               std::cout << "petch colour is: blue " << std::endl;}
           else if(green>(blue+red)){
               std::cout << "petch colour is: green " << std::endl; }
           return;
   }   
//find the right and left piller values   
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
//detect the distance and piller height
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
     int x =(abs(value_right-value_left))/4-2;
     return x;
}
// turn the robot 90' right or left to given direction
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
// move the robot in forword direction during the time
void move_center(double time){ 
     time += robot->getTime();;
     double current_time=0;
     
     while (robot->step(TIME_STEP) != -1) {
          current_time= robot->getTime();
          if (time>current_time){
              leftMotor->setVelocity( MAX_SPEED);
              rightMotor->setVelocity( MAX_SPEED);}
          else{break;}
     }
     return;    
}

void stop(){
    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
}

void forword_petch(int step){
          if (step==2){
              move_center(petch_center_time);
              colour_detecting();}
          else if (step==3) {
              move_center(petch_center_time);
              stop();
             junction_turn[1] = piller_detecting();}
          else if(step==4 || step==5){
              move_center(0.27);
              turn_90(junction_turn[step-4]);}
          else if(step==6){
              move_center(petch_center_time);
              stop();
              return;}               
      while (robot->step(TIME_STEP) != -1) {
          const double value_right = sensor_right->getValue();
          const double value_left = sensor_left->getValue();
          const double value_center = sensor_center->getValue();
          if (abs(value_center-value_left)<20 && abs(value_right-value_center)<20){
              leftMotor->setVelocity( MAX_SPEED);
              rightMotor->setVelocity( MAX_SPEED);}
          else{
              break;}
      } 
    return;
} 
void factor_N(int value){
    for(int i=1;i<=value;i++){
        if (value%i==0){
        std::cout << i << std::endl;}
    }

}  
   
int main(int argc, char **argv) {
     
     DistanceSensor *sensor_left2 = robot->getDistanceSensor("left2_ir");
     DistanceSensor *sensor_right2 = robot->getDistanceSensor("right2_ir");
     leftMotor->setPosition(INFINITY);
     rightMotor->setPosition(INFINITY);
     stop();
     // get a handler to the motors and set target position to infinity (speed control)
     sensor_right->enable(TIME_STEP);
     sensor_left->enable(TIME_STEP);
     sensor_center->enable(TIME_STEP);
     sensor_right2->enable(TIME_STEP);
     sensor_left2->enable(TIME_STEP);
  
  while (robot->step(TIME_STEP) != -1) {
    const double r_1 = sensor_right->getValue();
    const double l_1 = sensor_left->getValue();
    const double c_0 = sensor_center->getValue();
    const double r_2 = sensor_right2->getValue();
    const double l_2 = sensor_left2->getValue();
    
    
    if (abs(c_0-r_1)<20 && abs(c_0-l_1)<20){
          step+=1;
          forword_petch (step);
          if(step==6){break;}}
    else if(c_0<420 && r_1>900 && l_1>900){
          leftMotor->setVelocity( MAX_SPEED);
          rightMotor->setVelocity( MAX_SPEED);}
    else if (r_1>900 && r_2>900 && l_1<420 && l_2>900){
         leftMotor->setVelocity( MAX_SPEED);
         rightMotor->setVelocity(-0.5*MAX_SPEED);}
    else if (r_1<420 && r_2>900 && l_1>900 && l_2>900){
         leftMotor->setVelocity( -0.5*MAX_SPEED);
         rightMotor->setVelocity(MAX_SPEED);}
    else if(r_1>900 && r_2>900 && c_0<420 && l_1<420 && l_2<420){
         move_center(0.27);
         turn_90(-1);}
    else if(r_1<420 && r_2<420 && c_0<420 && l_1>900 && l_2>900){
         move_center(0.27);
         turn_90(1);}      
    else{break;}
  };
  sensor_right->disable();
  sensor_left->disable();
  sensor_center->disable();
  sensor_right2->disable();
  sensor_left2->disable();
  std::cout <<junction_turn[1] << std::endl;
  //factor_N((junction_turn[1]+2)*4);
  delete robot;
  double time = robot->getTime();
  std::cout <<time << std::endl;
  
  return 0;
}

