#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>

#include <iostream>
#include <string>
#include <vector>

using namespace std;
using namespace webots;

// Bobby to bunny communication (Bobby->sender, Bunny-> reciever)
#define COMMUNICATION_CHANNEL_1 1 
// Bobby to bunny communication (Bunny->sender, Bobby-> reciever)
#define COMMUNICATION_CHANNEL_2 2              


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
DistanceSensor *ob_ds[8];
char ob_dsNames[8][10] = {"D_L4","D_L3","D_L2","D_L1","D_R1","D_R2","D_R3","D_R4"};
int ob_reading[8] ={0,0,0,0,0,0,0,0,};
int ob_sum=0;
int ob_reading_ds[8] ={0,0,0,0,0,0,0,0,};
//color sensor parameter
Camera* camera[2];
string colors[5] = { "white","red","green","blue","black"};
int image_width=9;
// initalizing reciever and emitter
Receiver *receiver = robot->getReceiver("receiver");
Emitter *emitter = robot->getEmitter("emitter");
//initializing the position sensors
PositionSensor* leftPs = robot->getPositionSensor ("left_ps");
PositionSensor* rightPs = robot->getPositionSensor ("right_ps");
PositionSensor* vr_arm_ps = robot->getPositionSensor ("vr_arm_ps");
PositionSensor* arm_rotate_ps = robot->getPositionSensor ("arm_rotate_ps");
PositionSensor* left_arm_gr_ps = robot->getPositionSensor ("left_arm_gr_ps");
PositionSensor* right_arm_gr_ps = robot->getPositionSensor ("right_arm_gr_ps");

//////////////////////////////////////////////////////////////////////////////////////////////
////                      ## Declare the essential varibles ##                            ////
//////////////////////////////////////////////////////////////////////////////////////////////
// line following PID controller parameters
double previous_error=0.0;
double kp=0.07;  //3
double kd=0.0008; //0.5
double ki=0;
double Integral=0; //0.3
//robot alining with object PID controller
double ob_previous_error=0.0;
double ob_kp=9;  //3
double ob_kd=0.6; //0.5
double ob_ki=0.4;
double ob_Integral=0.3; //0.3
// robot physical parameters
float robot_width=0.09;
float wheel_radius= 0.032;
double max_speed= 15/(wheel_radius*100);
float n=(robot_width/wheel_radius)/2*3.3;          // 90 degree turn wheel rotation in radian
double move_dis=0;                                 // arm moved distance while closing
int maze_solved=0;
int task_completed=0;
int junction;

//////////////////////////////////////////////////////////////////////////////////////////////
////                           ## Define the Functions ##                                ////
//////////////////////////////////////////////////////////////////////////////////////////////
int getColorAt(int x);                          // detect the color object/line
void getReading();                              // mesure the reading of line follow IR sensor
void ds_getReading();                           // measur the reading of distance sensor
double PID();                                   // follow the center white line
void setup();                                   // enable and set initial valuse of sensors and actuaters
void turn_90(int dir);                          // turn the robot given direction left or right
void move_forward(double dis);                  // move the robot in a forword direction given distance
void grip_squre(int grib,double dis);           // close or open the arm finger
void lift_arm(double dist);                     // lift the arm in a virtical direction in given distance
int junction_detect();                          //  detect junctions
int obstacle_detection();                       // detect the obstacle infront of thre robot
void emmiter();                                 // emmit the message signal to bunny
void reciever();                                // recieve the signal from bunny                        
double ob_PID();                                // align the robot with object
int bottom_square_size();                       // fing the detected bottom squre size
void navi_s_m(int from , int to);
///////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {
      setup();
      
      while (task_completed!=1) {
        robot->step(TIME_STEP);                           // step up the time
        ds_getReading();
        navi_s_m(1,3);
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


void emmiter(int address){
      vector<int> message;
      message.push_back(address);
      if(address==1){
          message={1,2,3,4};}
      else if(address==2){
          message.push_back(6);
          message.push_back(7);}
      else{ 
          message.push_back(90);
          message.push_back(10);
          message.push_back(20);}
      while(robot->step(TIME_STEP) != -1){
          //const char *message = "Hello!";
          //emitter->send(values, strlen(message) + 1);
          emitter->send(&message, sizeof(message) + 1);
      }
}

int bottom_square_size(){
      if(ob_sum==3){return 1;}
      else if(ob_sum==5){return 2;}
      else if(ob_sum==7){return 3;}
      else{return 0;}
}

void reciever(){
    if (receiver->getQueueLength() > 0) {
          /* read current packet's data */
          const int *buffer = (const int*)receiver->getData();
          //const double *position= receiver->getEmitterDirection();
          /* print null-terminated message */
          std::cout << "Communicating: received " <<buffer[0]<<std::endl;
          std::cout << "Communicating: received " <<buffer[1]<<std::endl;
          std::cout << "Communicating: received " <<buffer[2]<<std::endl;
          //printf("Communicating: received \"%s\"\n", x);
          // test pour voir comment son organiser les données de position
          //printf("Position received:x=\"%lf\"y=\"%lf\"z=\"%lf\"\n", position[0],position[1],position[2]);
          /* fetch next packet */
          receiver->nextPacket();
       } 
  }
int obstacle_detection(){
      if(ob_sum>0){                               // obstacle detected in 30cm range
          leftMotor->setVelocity(0);
          rightMotor->setVelocity(0);
          ds_getReading(); 
          
          int pre_readings[8];
          for (int i = 0; i < 8; i++){
              pre_readings[i]=ob_reading_ds[i];
          }
         
          int steps=0;
          
          // check the obstacle is other robot or not          
          while(steps<10){
              robot->step(TIME_STEP);  
              ds_getReading();
              for (int i = 0; i < 8; i++){
                  if(pre_readings[i]!=ob_reading_ds[i]){
                      return 1;}
               }  
           }
           return 2;

           // move towords object
           // grib the oblect
           // switch back to line
           // move to previous junction          
      }
      return 0;
      }
//return values 1--> robot// 2--> object // 3--> normal junction // 4--> white patch // 5--> inverted patch
int move_until_next_decision_point(){
         // release from next current junction point
         while(robot->step(TIME_STEP) != -1){
             getReading();
             if(junction_detect()>0){
                 leftMotor->setVelocity(max_speed);
                 rightMotor->setVelocity(max_speed);
             }
             else{break;}
          }
          
        // move the robot 
        while(robot->step(TIME_STEP) != -1){
              int object=obstacle_detection();
              getReading();
              if(object>0){
                  return object;}
              else{
                  int jun = junction_detect();
                  if(jun>0){
                        move_forward(4);
                        if(reading[6]==1 && reading[7]==1 && (reading[2]==1 || reading[3]==1)){
                               move_forward(-4);
                               return 4;}
                        move_forward(-4);
                        return jun;
                      }
                  else{
                        PID();
                      }              
                  }
        
        }
        return 0;     
}
void navi_s_m(int from , int to){
           // 1--> start point // 2--> intermediate patch // 3--> matrix origin point
        move_until_next_decision_point();
        move_forward(14);
        if((from-to)==-1){
            turn_90(-1);}
        else if((from-to)==1){
            turn_90(1);}
        move_until_next_decision_point();  

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
          return 3; } // left turn junction detected
      else if(reading[6]==1 && reading[7]==1 && (reading[2]==1 || reading[3]==1)){
          
          return 3;}  // white centered junction detected
      else if(reading[6]==1 && reading[7]==0 && (reading[2]==1 || reading[3]==1)){
          return 3;}  // right turn junction detected
      else if(reading[6]==1 && reading[7]==1 && (reading[2]==0 || reading[3]==0)){
      
          return 5;}  // inverted patch junction detected
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
          if(dis>0){
              leftMotor->setVelocity(max_speed);
              rightMotor->setVelocity(max_speed);}
          else{
              leftMotor->setVelocity(-1*max_speed);
              rightMotor->setVelocity(-1*max_speed);}}
        else{
           leftMotor->setVelocity(0);
           rightMotor->setVelocity(0);
           break;}
      }      
}

void turn_90(int dir){                             // direction 1 for right and -1 for left
    move_forward(14);
    int lpsn = leftPs->getValue ();
    int rpsn = rightPs->getValue ();
    
    while(robot->step(TIME_STEP) != -1){
        double leftPsVal = leftPs->getValue ();
        double rightPsVal = rightPs->getValue ();
        
        if((leftPsVal-lpsn<n) && (rightPsVal-rpsn<n)){
           leftMotor->setVelocity(dir*max_speed);
           rightMotor->setVelocity(dir*max_speed);}
         else{
           leftMotor->setVelocity(0);
           rightMotor->setVelocity(0);
           break;
         }       
       }
}

int getColorAt (int x) {
    const unsigned char* image = camera[x]->getImage ();
    
    int r = camera[x]->imageGetRed (image, image_width, 1, 1);
    int g = camera[x]->imageGetGreen (image, image_width, 1, 1);
    int b = camera[x]->imageGetBlue (image, image_width, 1, 1);
    
    int color;
    if ( r>200 && b>200 && g>200 ) color = 0;                  // white color detected
    if ( r>200 && b<100 && g<100 ) color = 1;                  // red color detected
    if ( r<100 && b<100 && g>200 ) color = 2;                  // green color detected
    if ( r<100 && b>200 && g<100 ) color = 3;                  // blue color detected
    if ( r<100 && b<100 && g<100 ) color = 4;                  // black color detected

    return color;
}

// get reading from IR sensor panel
void getReading(){
    for (int i = 0; i < 8; i++) {
      if (ds[i]->getValue()>512){
        reading[i]=0;}
      else{
        reading[i]=1;}
     }
      return;    
}  
void white_box_place(int grib){
     move_forward(4);
     turn_90(-1);
     move_forward(5);
     lift_arm(-7);
     grip_squre(grib,3);
     lift_arm(7);
     move_forward(4);
     turn_90(1);
}


void ds_getReading(){
    ob_sum=0;
    for (int i = 0; i < 8; i++) {
        ob_reading_ds[i]=ob_ds[i]->getValue();
      if (ob_ds[i]->getValue()>990){
        ob_reading[i]=0;}
      else{
        ob_reading[i]=1;
        ob_sum+=1;}
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
    double l_speed=0;
    double r_speed=0;
    double P = kp*error;
    double I = Integral+(ki*error);
    double D = kd*(error-previous_error);
    double correction = (P+I+D)/1000;
    
    l_speed = max_speed/2+correction;
    r_speed = max_speed/2-correction;
    
    if (l_speed<0.0)  {l_speed=0;}
    else if (l_speed>max_speed) {l_speed=max_speed;}
    if (r_speed<0.0)  {r_speed=0;}
    else if (r_speed>max_speed) {r_speed=max_speed;}
    
    leftMotor->setVelocity(l_speed);
    rightMotor->setVelocity(r_speed);
    
    Integral=I;
    previous_error=error;
    return 0;
  }
  
 // PID controlling for perfect ling following
double ob_PID(){
    double error = 0.0;
    int coefficient[8]= {-4,-3,-2,-1,1,2,3,4};
 
    for (int i = 0; i < 8; i++) {
      error += -coefficient[i]*ob_reading_ds[i];
      } 
    
    //std::cout << " t" << error<<std::endl;
    double l_speed=0;
    double r_speed=0;
    double P = ob_kp*error;
    double I = ob_Integral+(ob_ki*error);
    double D = ob_kd*(error-previous_error);
    double correction = (P+I+D)/1000;
    //std::cout << " correction" << correction<<std::endl;
    l_speed = (max_speed/2+correction)/5;
    r_speed = (max_speed/2-correction)/5;
    
    if (l_speed<0.0)  {l_speed=0;}
    else if (l_speed>max_speed) {l_speed=max_speed;}
    if (r_speed<0.0)  {r_speed=0;}
    else if (r_speed>max_speed) {r_speed=max_speed;}
    
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
        ds[i]->enable(TIME_STEP);}
      
      for (int i = 0; i < 8; i++) {
        ob_ds[i] = robot->getDistanceSensor(ob_dsNames[i]);
        ob_ds[i]->enable(TIME_STEP);}
      
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
      
      camera[0] = robot->getCamera ("color_sensor");
      camera[0]->enable (TIME_STEP);
      camera[1] = robot->getCamera ("color_sensor_arm");
      camera[1]->enable (TIME_STEP);
      
      leftPs->enable (TIME_STEP);
      rightPs->enable (TIME_STEP);
      vr_arm_ps->enable (TIME_STEP);
      arm_rotate_ps->enable (TIME_STEP);
      left_arm_gr_ps->enable (TIME_STEP);
      right_arm_gr_ps->enable (TIME_STEP);
      
      const int channel = receiver->getChannel();
      if (channel != COMMUNICATION_CHANNEL_2) {
        receiver->setChannel(COMMUNICATION_CHANNEL_2);}
      receiver->enable(TIME_STEP);
      
      const int channel_e = emitter->getChannel();
      if (channel_e != COMMUNICATION_CHANNEL_1) {
            emitter->setChannel(COMMUNICATION_CHANNEL_1);}
      emitter->setRange(5);
    
      
}

