#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>
using namespace webots;

#define TIME_STEP 2


Robot *robot = new Robot();
//////////////////////////////////////////////////////////////////
     DistanceSensor *left__1 = robot->getDistanceSensor("left_1");
     DistanceSensor *left__2 = robot->getDistanceSensor("left_2");
     DistanceSensor *left__3 = robot->getDistanceSensor("left_3");
     DistanceSensor *left__4 = robot->getDistanceSensor("left_4");
     DistanceSensor *right__1 = robot->getDistanceSensor("right_1");
     DistanceSensor *right__2 = robot->getDistanceSensor("right_2");
     DistanceSensor *right__3 = robot->getDistanceSensor("right_3");
     DistanceSensor *right__4 = robot->getDistanceSensor("right_4");
     
     
     Motor *leftMotor = robot->getMotor("left_motor");
     Motor *rightMotor = robot->getMotor("right_motor");
     
     
/////////////////////////////////////////////////////////////////////////

void forward(){
        leftMotor->setVelocity( 10);
        rightMotor->setVelocity( 10);
}

void stop_(){
        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
}
void left(){
    leftMotor->setVelocity( -2.5);
    rightMotor->setVelocity( 5);  
}
void right(){
        leftMotor->setVelocity( 5);
        rightMotor->setVelocity( -2.5);
}

void slowleft(){ 
        leftMotor->setVelocity( 5);
        rightMotor->setVelocity( 0);
}

void slowright(){
        leftMotor->setVelocity( 0);
        rightMotor->setVelocity(5);
}


int main(int argc, char **argv) {
  
     int timeStep = (int)robot->getBasicTimeStep();
     left__1->enable(TIME_STEP);
     left__2->enable(TIME_STEP);
     left__3->enable(TIME_STEP);
     left__4->enable(TIME_STEP);
     right__1->enable(TIME_STEP);
      right__2->enable(TIME_STEP);
      right__3->enable(TIME_STEP);
      right__4->enable(TIME_STEP);
     
     leftMotor->setPosition(INFINITY);
     rightMotor->setPosition(INFINITY);
     
     stop_();
     
  while (robot->step(timeStep) != -1) {
  
         double Left4=left__4->getValue();
         double Left3=left__3->getValue();
         double Left2=left__2->getValue();
         double Left1=left__1->getValue();
         double Right4=right__4->getValue();
         double Right3=right__3->getValue();
         double Right2=right__2->getValue();
         double Right1=right__1->getValue();
    
          if(Left4>900 && Left3>900 && Left1<420 && Right1<420 && Right3>900 && Right4>900 && Right2>900 && Left2>900){
             forward();}
          else if((Left4<420 && Left3<420 && Left2<420 && Left1<420 && Right1<420 && Right2<420 && Right3<420 && Right4<420)) {
             stop_();}
             
          else if((Left4>901 ||  Left3>901  || Left2>901 ) && ( Left1<420 && Right1<420 && Right2<420 && Right3<420 && Right4<420)){
               forward();
               while (robot->step(timeStep) != -1) {
                       Right4=right__4->getValue();
                       
                       Left4=left__4->getValue();
                     if((Left4>901 && Right4>901) ){
                        break;}
                }
                left();
                while (robot->step(timeStep) != -1) {
                       Right1=right__1->getValue();
                       Left1=left__1->getValue();
                       Right1=right__1->getValue();
                       Left1=left__1->getValue();
                     if((Left4>901 && Left1<420 && Right1<420 && Right4>901 )==true ){
                        break;}
                    }
                stop_();
            }
            
            else if((Left4<420 &&  Left3<420  && Left2<420 && Left1<420 && Right1<420) && ( Right2>901 || Right3>901 || Right4>901)){
                   forward();
                   while (robot->step(timeStep) != -1) {
                       Right4=right__4->getValue();
                       // Right1=right__1->getValue();
                       // Left1=left__1->getValue();
                       Left4=left__4->getValue();
                     if(Left4>901 &&  Right4>901){
                        break;}
                    }
                    
                   right();
                   while (robot->step(timeStep) != -1) {
                       Right1=right__1->getValue();
                       Left1=left__1->getValue();
                        Right1=right__1->getValue();
                        Left1=left__1->getValue();
                     if((Left4>901 && Left1<420 && Right1<420 && Right4>901)==true){
                        break;}
                    } 
                    stop_();
            }
            
            else if(Left4>900 && (Left3<420 || Left2<420) && Right2>900){
                right();}
                
            else if(Right4>900 && (Right3<420 || Right2<420) && Left2>900){
                left();}
          }
          
  stop_();
  delete robot;
  return 0;
}

 
