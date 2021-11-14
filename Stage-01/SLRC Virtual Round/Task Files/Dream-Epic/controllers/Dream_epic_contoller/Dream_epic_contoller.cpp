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
#define left -1          //turning directions
#define right 1                

char petch_colour[3][7] ={"red","Green","blue"};   
double petch_center_time = 0.8*petch_lenght/(MAX_SPEED*wheel_radius);
double turn_90time = 3.14*(robot_width/2)/(MAX_SPEED*wheel_radius); 
int junction_turn[2] ={left,0};
int colour=0,step=0, N; 
//inticial device setting
Robot *robot = new Robot();
Motor *leftMotor = robot->getMotor("left_motor");
Motor *rightMotor = robot->getMotor("right_motor");

DistanceSensor *sensor_right = robot->getDistanceSensor("right_ir");
DistanceSensor *sensor_left = robot->getDistanceSensor("left_ir");
DistanceSensor *sensor_center = robot->getDistanceSensor("center_ir");
DistanceSensor *sensor_left2 = robot->getDistanceSensor("left2_ir");
DistanceSensor *sensor_right2 = robot->getDistanceSensor("right2_ir");

void stop(){
    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
}

int colour_detecting(){
      int colour_num=0; 
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
               colour_num = 1;}
           else if(blue>(red+green)){
               colour_num = 3;}
           else if(green>(blue+red)){
               colour_num = 2; }
               
           camera_s->disable();
           return colour_num;
   }   
//find the right and left piller values   
int find_value(double top, double bottom){
         int size;double value;
         if (top<990){
               size=2;
               value = top;}
         else {size=1;
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
     N =(abs(value_right-value_left));
     std::cout << "Value of N: " << N << std::endl;
     return (N/4-2);
}
// turn the robot 90' right or left to given direction
void turn_90(int direction){
     double left_speed =0;           //assign new variable for wheel speed for chnae the direction appopriately
     double right_speed =0;
     if (direction == right){
          left_speed=-1*MAX_SPEED;
          right_speed=MAX_SPEED;}
     else if (direction  == left){
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
          else{stop();
               break;}
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

void wall_flow(){
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
    
     // rotate the edge, top cylinder sensors to the directions left, right, foroword
     double time_rotate=(3.5/MAX_SPEED);
     double start_time = robot->getTime();
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
    // follow the wall  
    while (robot->step(TIME_STEP) != -1) {
        const double r_1 = sensor_right->getValue();
        const double l_1 = sensor_left->getValue();
        const double c_0 = sensor_center->getValue();
        const double r_2 = sensor_right2->getValue();
        const double l_2 = sensor_left2->getValue();
        const double c_top = sensor_bottom->getValue();
        if(c_0<420 && r_1>900 && l_1>900){                     // check the end point of wall, therefore begining of the line
            break;} 
        else if(c_top>570){                                   // check front obstacle for finding turns
           leftMotor->setVelocity(MAX_SPEED);
           rightMotor->setVelocity(MAX_SPEED);}
        else{
            if (l_2==1000){         // turn left side
                turn_90(left); 
                continue;}
            else if(r_2==1000){     // turn right side
                turn_90(right);
                continue;}
             }
        }
     // rotate the edge distance sensor to bottom direction
     start_time = robot->getTime();
     current_time = 0;
     while (robot->step(TIME_STEP) != -1) {
            current_time =robot->getTime();
            if(start_time+time_rotate>current_time){
                left2Motor->setVelocity(-1*MAX_SPEED);
                right2Motor->setVelocity(MAX_SPEED);}
            else{
                left2Motor->setVelocity(0);
                right2Motor->setVelocity(0);
                  break;}}           
}

// find the final distination from check petch one by one
void find_distination(){
    int steps=0;
    int petch=0;
    while (robot->step(TIME_STEP) != -1) {
        const double r_1 = sensor_right->getValue();
        const double l_1 = sensor_left->getValue();
        const double c_0 = sensor_center->getValue();
        if (abs(c_0-r_1)<20 && abs(c_0-l_1)<20){
            if (steps==0){
                  move_center(0.27);
                  steps =1;
                  if (petch==0){  move_center(0.1);  N=8;}        // move to center petch
                  else if(petch==1){ turn_90(left); N=4;}       //  move to left petch
                  else if(petch==2){ move_center(0.1);  N=4;}    // move to right petch
                  }
            else {
                  move_center(petch_center_time);
                  int x = colour_detecting();
                  if (x == colour){                     // sucessfully find the final destination
                      stop();
                      break;}
                  else{ stop();
                        turn_90(right);                         // rotate the bot half circle for reverce direction moving
                        turn_90(right);      
                        steps=0;  petch+=1;
                        move_center(0.2);}    }}
        else if(c_0< 420 && r_1>900 && l_1>900){         // moving stright && fllow the line 
                  leftMotor->setVelocity( MAX_SPEED);
                  rightMotor->setVelocity( MAX_SPEED);}
        else if(r_1>900 && c_0<420 && l_1<420){          //left turn
                   move_center(0.27);
                   turn_90(left);}
        else if(r_1<420 && c_0<420 && l_1>900){         //right turn
                   move_center(0.27);
                   turn_90(right);} 
            }
 }     

// find the junction and petch appopriately and do the proper action
void forword_petch(int step){
          if (step==2){                           // enter the color finding petch and find the destination petch coiour
              move_center(petch_center_time);
              colour = colour_detecting();
              std::cout << "petch colour is: " << petch_colour[colour-1] << std::endl; }
          else if (junction_turn[0]==left){                 // curve line following mode
              if (step==3) {
                  move_center(petch_center_time);           // enter the piller petch and find the value of N
                  stop();
                  junction_turn[1] = piller_detecting();}
              else if(step==4 || step==5){                  // fine the first and end point junctions and take correct turn
                  move_center(0.27);
                  turn_90(junction_turn[step-4]);}
              else if(step==6){                            // enter the end destination petch and stand
                  move_center(petch_center_time);
                  stop();
                  return;}}
           else if(junction_turn[0]==right){               // wall following mode
               if(step==4){                               // junction one turn right and follow the wall
                    move_center(0.27);
                    turn_90(right);}
               else if (step==5){                         // enter the wall following path
                     wall_flow();}
               else if(step==6){
                     find_distination();                 
                     stop();
                     return;}  
              }
      //move forword all bottom sensor values are same             
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
// find the factors of destination petch value
void factor_N(){
    std::cout << "Factors of "<< N <<" :";
    for(int i=1;i<=N;i++){
        if (N%i==0){
            std::cout << i << " ";}
    }
}  
   
int main(int argc, char **argv) {
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
    else if(c_0<420 && r_1>900 && l_1>900){                 // move stright  and fllow the stright line
          leftMotor->setVelocity( MAX_SPEED);
          rightMotor->setVelocity( MAX_SPEED);}
    else if (r_1>900 && r_2>640 && l_1<420 && l_2>640){    // turn the bot in right bit to follow the curve line
         leftMotor->setVelocity( 0.5*MAX_SPEED);
         rightMotor->setVelocity(-0.25*MAX_SPEED);}
    else if (r_1<420 && r_2>640 && l_1>900 && l_2>640){
         leftMotor->setVelocity( -0.25*MAX_SPEED);
         rightMotor->setVelocity(0.5*MAX_SPEED);}
    else if(r_1>900 && r_2>640 && c_0<420 && l_1<420 && l_2<230){          //left turn
         move_center(0.27);
         turn_90(left);}
    else if(r_1<420 && r_2<230 && c_0<420 && l_1>900 && l_2>640){         //right turn
         move_center(0.27);
         turn_90(right);}      
    else{break;}
  };
  sensor_right->disable();                                              // disable the resourses
  sensor_left->disable();
  sensor_center->disable();
  sensor_right2->disable();
  sensor_left2->disable();
  factor_N();
  delete robot;
  double time = robot->getTime();
  std::cout <<" " << std::endl;
  std::cout << " Total Running Time : "<<time << std::endl;
  
  return 0;
}



