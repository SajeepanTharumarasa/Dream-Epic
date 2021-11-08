#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/emitter.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <iostream>
#include <string>
using namespace std;

using namespace webots;
Robot *robot = new Robot();
int TIME_STEP = 32;

// define wheel motor and speed parameters
Motor *leftMotor = robot->getMotor("left_motor");
Motor *rightMotor = robot->getMotor("right_motor");

Motor *arm_vr_motor = robot->getMotor("vr_arm_motor");
Motor *arm_rotate_motor = robot->getMotor("arm_rotate_motor");
Motor *g_r_motor = robot->getMotor("right_arm_gr_motor");
Motor *g_l_motor = robot->getMotor("left_arm_gr_motor");

double l_speed = 0;         //  motor speed 
double r_speed = 0;         

// line following IR sensor parameters
DistanceSensor *ds[8];
char dsNames[8][10] = {"IR_1","IR_2","IR_3","IR_4","IR_5","IR_6","IR_L","IR_R"};
int reading[8] ={0,0,0,0,0,0,0,0};
// object detection distance sensor
DistanceSensor *o_ds;
// line detection color sensor parameter
Camera* camera;
string colors[4] = { "gray","red","green","blue" };

//initializing the position sensors
PositionSensor* leftPs = robot->getPositionSensor ("left_ps");
PositionSensor* rightPs = robot->getPositionSensor ("right_ps");

PositionSensor* vr_arm_ps = robot->getPositionSensor ("vr_arm_ps");
PositionSensor* arm_rotate_ps = robot->getPositionSensor ("arm_rotate_ps");
PositionSensor* left_arm_gr_ps = robot->getPositionSensor ("left_arm_gr_ps");
PositionSensor* right_arm_gr_ps = robot->getPositionSensor ("right_arm_gr_ps");

// PID controller parameters
double previous_error=0.0;
double kp=10;  //3
double kd=0.5; //0.5
double ki=0.4;
double Integral=0.3; //0.3

// robot physical parameters
float robot_width=0.065;
float wheel_radius= 0.02;
float n=(robot_width/wheel_radius)/2*3.1416;          // 90 degree turn wheel rotation in radian


double move_dis=0;
///////////## Define the Functions ##/////////////////

int getColorAt (int x, int y);
void getReading();
double PID();
void setup();
void turn_90(int dir);
void move_forward(double dis);

///////////////////////////////////////////////////////
void grip_squre(int grib,double dis){
      int l_gr = left_arm_gr_ps->getValue ();
      int r_gr = right_arm_gr_ps->getValue ();
      
      int d=0;
      
      if (grib==0){
         d=-1;
         dis=move_dis;}
      else{
         d=1;}
        
      double time= dis;
      double s_time = robot->getTime();
      
      while(robot->step(TIME_STEP) != -1){
        double c_time = robot->getTime();
        double time_diff =c_time-s_time-time;
        
        double leftPsVal = left_arm_gr_ps->getValue ();
        double rightPsVal = right_arm_gr_ps->getValue ();
        
        
        move_dis=(leftPsVal-l_gr)*100;
        
        if(time_diff<0){
          g_r_motor->setVelocity(-d*0.01);
          g_l_motor->setVelocity(d*0.01);}
         else{
           g_r_motor->setVelocity(0);
           g_l_motor->setVelocity(0);
           break;}       
       }
       
       
}


void lift_arm(double dist){
      double vr_ps = vr_arm_ps->getValue ();
      
      while(robot->step(TIME_STEP) != -1){
        double vr_ps_val = vr_arm_ps->getValue ();
        double mov_dis=(vr_ps_val-vr_ps)*100;
        
        if(abs(mov_dis)<abs(dist)){
           arm_vr_motor->setVelocity(dist*0.01);}
        else{
           arm_vr_motor->setVelocity(0);
           break;
         }       
       }     
}

int main(int argc, char **argv) {
      setup();

      leftMotor->setVelocity(0);
      rightMotor->setVelocity(0.0); 
      
      while (robot->step(TIME_STEP) != -1) {
        // rightMotor->setVelocity(-5);
        // leftMotor->setVelocity(5);
        lift_arm(-7); 
        grip_squre(1,2);
        lift_arm(7);
        //grip_squre(0,2);
        break;
        getReading();
        if((reading[6] || reading[7])){
             turn_90(-1);
            leftMotor->setVelocity(0.0);
            rightMotor->setVelocity(0.0); 
          }
         else{
            PID();
           }
        
      };

  delete robot;
  return 0;
}

void move_forward(double dis){
      double mov_time= dis/14;    // distance/speed
      double s_time =robot->getTime();
      
      while(robot->step(TIME_STEP) != -1){
        double c_time = robot->getTime();
        double diff = c_time-s_time;
        
        if(diff< mov_time){
          leftMotor->setVelocity(7);
          rightMotor->setVelocity(7);}
        else{
           leftMotor->setVelocity(0);
           rightMotor->setVelocity(0);
           break;}
      }      
}

void turn_90(int dir){
    move_forward(6);
    int lpsn = leftPs->getValue ();
    int rpsn = rightPs->getValue ();
    
    int r_s,l_s;
    
    if(dir==1){ r_s=-5; l_s=5;}
    else{ r_s=5; l_s=-5;}
    
    while(robot->step(TIME_STEP) != -1){
        double leftPsVal = leftPs->getValue ();
        double rightPsVal = rightPs->getValue ();
        
        if((leftPsVal-lpsn<n) && (rightPsVal-rpsn<n)){
          leftMotor->setVelocity(l_s);
          rightMotor->setVelocity(r_s);}
         else{
           leftMotor->setVelocity(0);
           rightMotor->setVelocity(0);
           break;
         }       
       }
}

int getColorAt (int x, int y) {
    // x,y specify the point of color extraction
    
    const unsigned char* image = camera->getImage ();
    
    int image_width = camera->getWidth ();
    
    int r = camera->imageGetRed (image, image_width, x, y);
    int g = camera->imageGetGreen (image, image_width, x, y);
    int b = camera->imageGetBlue (image, image_width, x, y);
    
    int color=0;
    if ( r > g && r > b ) color = 1;
    if ( g > r && g > b ) color = 2;
    if ( b > g && b > r ) color = 3;

    //cout << "Detected color = " << colors[color] << '\n';

    return color;
}
/// get reading from IR sensor panel
void getReading(){
    for (int i = 0; i < 8; i++) {
      if (ds[i]->getValue()>512){
        reading[i]=0;}
      else{
        reading[i]=1;}
     }
      return;    
}  

// PID controlling for perfect ling following
double PID(){
    double error = 0.0;
    int coefficient[6]= {-3000,-2000,-1000,1000,2000,3000};
 
    for (int i = 0; i < 6; i++) {
      error += coefficient[i]*reading[i];
      } 
   
    double P = kp*error;
    double I = Integral+(ki*error);
    double D = kd*(error-previous_error);
    double correction = (P+I+D)/1000;
    
    l_speed = 5+correction;
    r_speed = 5-correction;
    
    if (l_speed<0.0)  {l_speed=0;}
    else if (l_speed>10.0) {l_speed=10.0;}
    if (r_speed<0.0)  {r_speed=0;}
    else if (r_speed>10.0) {r_speed=10.0;}
    
    leftMotor->setVelocity(l_speed);
    rightMotor->setVelocity(r_speed);
    
    Integral=I;
    previous_error=error;
    return 0;
  }
  
/// initial setup of the motor and sensors.
void setup(){
      for (int i = 0; i < 8; i++) {
        ds[i] = robot->getDistanceSensor(dsNames[i]);
        ds[i]->enable(TIME_STEP);
      }
      
      leftMotor->setPosition(INFINITY);
      rightMotor->setPosition(INFINITY);
      
      arm_vr_motor->setPosition(INFINITY);
      arm_rotate_motor->setPosition(INFINITY);
      g_r_motor->setPosition(INFINITY);
      g_l_motor->setPosition(INFINITY);
      
      arm_vr_motor->setVelocity(0);
      arm_rotate_motor->setVelocity(0.0);
      g_r_motor->setVelocity(0);
      g_l_motor->setVelocity(0.0);
      
      camera = robot->getCamera ("color_sensor");
      camera->enable (TIME_STEP);
      
      leftPs->enable (TIME_STEP);
      rightPs->enable (TIME_STEP);
      
      vr_arm_ps->enable (TIME_STEP);
      arm_rotate_ps->enable (TIME_STEP);
      left_arm_gr_ps->enable (TIME_STEP);
      right_arm_gr_ps->enable (TIME_STEP);
      
}

