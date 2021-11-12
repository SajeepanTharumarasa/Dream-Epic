#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <iostream>
#include <string>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#define COMMUNICATION_CHANNEL 1


#include <vector>
using namespace std;
using namespace webots;

Robot *robot = new Robot();
int TIME_STEP = 32;

// define the motors in wheel and arm
Motor *leftMotor = robot->getMotor("left_motor");
Motor *rightMotor = robot->getMotor("right_motor");

Motor *arm_vr_motor = robot->getMotor("vr_arm_motor");
Motor *arm_rotate_motor = robot->getMotor("arm_rotate_motor");
Motor *g_r_motor = robot->getMotor("right_arm_gr_motor");
Motor *g_l_motor = robot->getMotor("left_arm_gr_motor"); 

// line following IR sensor parameters
DistanceSensor *ds[8];
char dsNames[8][10] = {"IR_1","IR_2","IR_3","IR_4","IR_5","IR_6","IR_L","IR_R"};
int reading[8] ={0,0,0,0,0,0,0,0};
// object detection ultra sonic sensor.
DistanceSensor *ob_ds[9];
char ob_dsNames[9][10] = {"D_L4","D_L3","D_L2","D_L1","D_C","D_R1","D_R2","D_R3","D_R4"};
int ob_reading[9] ={0,0,0,0,0,0,0,0,0};
int ob_reading_ds[9] ={0,0,0,0,0,0,0,0,0};
// line detection color sensor parameter
Camera* camera;
Camera* camera_arm;
string colors[5] = { "white","red","green","blue","black"};

Receiver *receiver = robot->getReceiver("receiver");
Emitter *emitter = robot->getEmitter("emitter");


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
float robot_width=0.09;
float wheel_radius= 0.032;
float n=(robot_width/wheel_radius)/2*3.3;          // 90 degree turn wheel rotation in radian

double move_dis=0;

double l_speed=0;
double r_speed=0;
/////////////////////
int image_width=9;
/////////////////////
int maze_solved=0;
int task_completed=0;
int junction;

///////////     ## Define the Functions ##////////////////
int getColorAt ();
void getReading();
double PID();
void setup();
void turn_90(int dir);
void move_forward(double dis);
void grip_squre(int grib,double dis);
void lift_arm(double dist);
int junction_detect();
int obstacle_detection();
void emmiter();
void reciever();
void ds_getReading();
double ob_PID();

///////////////////////////////////////////////////////
void emmiter(){
}

void reciever(){
    if (receiver->getQueueLength() > 0) {
          /* read current packet's data */
          const int *buffer = (const int*)receiver->getData();
          const double *position= receiver->getEmitterDirection();
          /* print null-terminated message */
          std::cout << "Communicating: received " <<buffer[0]<<std::endl;
          std::cout << "Communicating: received " <<buffer[1]<<std::endl;
          std::cout << "Communicating: received " <<buffer[2]<<std::endl;
          //printf("Communicating: received \"%s\"\n", x);
          // test pour voir comment son organiser les données de position
          printf("Position received:x=\"%lf\"y=\"%lf\"z=\"%lf\"\n", position[0],position[1],position[2]);
          /* fetch next packet */
          receiver->nextPacket();
       } 
       else {
            printf("Communication broken!\n");
          }
  }
int obstacle_detection(){
  return 0;}


int main(int argc, char **argv) {
      setup();
      
      while (task_completed!=1) {
        robot->step(TIME_STEP);                           // step up the time
        ds_getReading();
        // for (int i = 0; i < 9; i++) {
            // std::cout << ob_dsNames[i] << " " <<ob_reading_ds[i]<<std::endl;
        // }
        // break;
        ob_PID();
        continue;
        if(maze_solved==0){
            if(obstacle_detection() !=0){}                // obstacle detection to avoding collision 
            else{
              getReading();
              junction=junction_detect();
                    
              if(junction>0){ 
                  turn_90(1);}
              else{  PID();}
              } 
          }
          else{}                                           // solve the maze with full data quickly
      };

  delete robot;
  return 0;
}

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
        
        move_dis=(abs(leftPsVal-l_gr)+abs(rightPsVal-r_gr))*100*0.5;
        
        if(time_diff<0){
          g_r_motor->setVelocity(-d*0.01);
          g_l_motor->setVelocity(d*0.01);}
         else{
           g_r_motor->setVelocity(0);
           g_l_motor->setVelocity(0);
           break;}       
       }       
}

int junction_detect(){
      if(reading[6]==0 && reading[7]==1 && (reading[2]==1 || reading[3]==1)){
          return 1; } // left turn junction detected
      else if(reading[6]==1 && reading[7]==1 && (reading[2]==1 || reading[3]==1)){
          return 2;}  // white patch or centered junction detected
      else if(reading[6]==1 && reading[7]==0 && (reading[2]==1 || reading[3]==1)){
          return 3;}  // right turn junction detected
      else if(reading[6]==1 && reading[7]==1 && (reading[2]==0 || reading[3]==0)){
          return 4;}  // inverted patch junction detected
      else{ return 0;} 
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
           break;}       
       }     
}


void move_forward(double dis){
      double mov_angle= dis/(wheel_radius*100);    // distance/speed
      
      int lpsn = leftPs->getValue ();
      int rpsn = rightPs->getValue ();
      
      while(robot->step(TIME_STEP) != -1){
        double leftPsVal = leftPs->getValue ();
        double rightPsVal = rightPs->getValue ();
        double diff = (abs(leftPsVal-lpsn)+abs(rightPsVal-rpsn))/2;
        
        if(diff< mov_angle){
          leftMotor->setVelocity(7);
          rightMotor->setVelocity(7);}
        else{
           leftMotor->setVelocity(0);
           rightMotor->setVelocity(0);
           break;}
      }      
}

void turn_90(int dir){
    move_forward(14);
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

int getColorAt () {
    const unsigned char* image = camera->getImage ();
    
    int r = camera->imageGetRed (image, image_width, 1, 1);
    int g = camera->imageGetGreen (image, image_width, 1, 1);
    int b = camera->imageGetBlue (image, image_width, 1, 1);
    
    int color;
    if ( r>200 && b>200 && g>200 ) color = 0;                  // white color detected
    if ( r>200 && b<100 && g<100 ) color = 1;                  // red color detected
    if ( r<100 && b<100 && g>200 ) color = 2;                  // green color detected
    if ( r<100 && b>200 && g<100 ) color = 3;                  // blue color detected
    if ( r<100 && b<100 && g<100 ) color = 4;                  // black color detected

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

void ds_getReading(){
    for (int i = 0; i < 9; i++) {
        ob_reading_ds[i]=ob_ds[i]->getValue();
      if (ob_ds[i]->getValue()>990){
        ob_reading[i]=0;}
      else{
        ob_reading[i]=1;}
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
  
 // PID controlling for perfect ling following
double ob_PID(){
    double error = 0.0;
    int coefficient[9]= {-4,-3,-2,-1,0,1,2,3,4};
 
    for (int i = 0; i < 9; i++) {
      error += -coefficient[i]*ob_reading_ds[i];
      } 
   std::cout << " t" << error<<std::endl;
    double P = kp*error;
    double I = Integral+(ki*error);
    double D = kd*(error-previous_error);
    double correction = (P+I+D)/100;
    std::cout << " correction" << correction<<std::endl;
    l_speed = (5+correction)/5;
    r_speed = (5-correction)/5;
    
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
      
      for (int i = 0; i < 9; i++) {
        ob_ds[i] = robot->getDistanceSensor(ob_dsNames[i]);
        ob_ds[i]->enable(TIME_STEP);
      }
      
      leftMotor->setPosition(INFINITY);
      rightMotor->setPosition(INFINITY);
      
      arm_vr_motor->setPosition(INFINITY);
      arm_rotate_motor->setPosition(INFINITY);
      g_r_motor->setPosition(INFINITY);
      g_l_motor->setPosition(INFINITY);
      
      leftMotor->setVelocity(0);
      rightMotor->setVelocity(0.0); 
      
      arm_vr_motor->setVelocity(0);
      arm_rotate_motor->setVelocity(0.0);
      g_r_motor->setVelocity(0);
      g_l_motor->setVelocity(0.0);
      
      camera = robot->getCamera ("color_sensor");
      camera->enable (TIME_STEP);
      
      camera_arm = robot->getCamera ("color_sensor_arm");
      camera_arm->enable (TIME_STEP);
      
      
      leftPs->enable (TIME_STEP);
      rightPs->enable (TIME_STEP);
      
      vr_arm_ps->enable (TIME_STEP);
      arm_rotate_ps->enable (TIME_STEP);
      left_arm_gr_ps->enable (TIME_STEP);
      right_arm_gr_ps->enable (TIME_STEP);
      
      const int channel = receiver->getChannel();
      if (channel != COMMUNICATION_CHANNEL) {
        receiver->setChannel(COMMUNICATION_CHANNEL);}
      receiver->enable(TIME_STEP);
      
}

