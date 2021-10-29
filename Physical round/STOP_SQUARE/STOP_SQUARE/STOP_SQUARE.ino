#include <Encoder.h>

// IR sensor inputs
#define LEFT_EDGE A8
#define RIGHT_EDGE A9
#define L4 A0
#define L3 A1
#define L2 A2
#define L1 A3
#define R1 A4
#define R2 A5
#define R3 A6
#define R4 A7

// Set the pins for left motor
#define LEFT_FORWARD 25
#define LEFT_BACKWARD 23
#define LEFT_MOTOR_ENABLE 27
#define LEFT_MOTOR_PWM 8

// Set the pins for right motor
#define RIGHT_FORWARD 29
#define RIGHT_BACKWARD 31
#define RIGHT_MOTOR_ENABLE 33
#define RIGHT_MOTOR_PWM 9

Encoder leftEncoder(19,18);
Encoder rightEncoder(2,3);

void setup(){
  pinMode(L4,INPUT);
  pinMode(L3,INPUT);
  pinMode(L2,INPUT);
  pinMode(L1,INPUT);
  pinMode(R1,INPUT);
  pinMode(R2,INPUT);
  pinMode(R3,INPUT);
  pinMode(R4,INPUT);

  
  pinMode(LEFT_FORWARD,OUTPUT);
  pinMode(LEFT_BACKWARD,OUTPUT);
  pinMode(LEFT_MOTOR_ENABLE,OUTPUT);
  pinMode(LEFT_MOTOR_PWM,OUTPUT);

  pinMode(RIGHT_FORWARD,OUTPUT);
  pinMode(RIGHT_BACKWARD,OUTPUT);
  pinMode(RIGHT_MOTOR_ENABLE,OUTPUT);
  pinMode(RIGHT_MOTOR_PWM,OUTPUT);
  
//  pinMode(19,INPUT);
//  pinMode(18,INPUT);
//  pinMode(2,INPUT);
//  pinMode(3,INPUT);
}

int Left4,Left3,Left2,Left1,Right1,Right2,Right3,Right4;

bool is_even(){
  if(Left4==Right4 && Left3==Right3 && Left2==Right2 && Left1==Right1){
    return true;
  }
  else{
    return false;
  }
}




void motor_left(bool N,int l){  //forward 1 backward 0 speedmax 400
  if(N==1){
  digitalWrite(LEFT_FORWARD,HIGH);
  digitalWrite(LEFT_BACKWARD,LOW);
  }
  else if(N==0){
  digitalWrite(LEFT_FORWARD,LOW);
  digitalWrite(LEFT_BACKWARD,HIGH);
  }
  digitalWrite(LEFT_MOTOR_ENABLE,HIGH);
  analogWrite(LEFT_MOTOR_PWM,l*51/80);
}





void motor_right(bool N,int l){  //forward 1 backward 0 speedmax 400
  if(N==1){
  digitalWrite(RIGHT_FORWARD,HIGH);
  digitalWrite(RIGHT_BACKWARD,LOW);
  }
  else if(N==0){
  digitalWrite(RIGHT_FORWARD,LOW);
  digitalWrite(RIGHT_BACKWARD,HIGH);
  }
  digitalWrite(RIGHT_MOTOR_ENABLE,HIGH);
  analogWrite(RIGHT_MOTOR_PWM,l*51/80);
}


void forward(){
  motor_left(1,400);
  motor_right(1,400);
}

void hit_break(){
  digitalWrite(RIGHT_FORWARD,LOW);
  digitalWrite(RIGHT_BACKWARD,LOW);
  digitalWrite(RIGHT_MOTOR_ENABLE,HIGH);
  analogWrite(RIGHT_MOTOR_PWM,255);

  digitalWrite(LEFT_FORWARD,LOW);
  digitalWrite(LEFT_BACKWARD,LOW);
  digitalWrite(LEFT_MOTOR_ENABLE,HIGH);
  analogWrite(LEFT_MOTOR_PWM,255);
}



void turn_left(){
  motor_left(0,200);
  motor_right(1,200);
}


void turn_right(){
  motor_left(1,200);
  motor_right(0,200);
}






long oldposl=0;
long oldposr=0;

long newposl;
long newposr;


bool square_det=false;
unsigned long T_start;
unsigned long T_current;


void square(){

  if(square_det==false){
  leftEncoder.write(0);
  rightEncoder.write(0);
  oldposl=0;
  oldposr=0;
  T_start=micros();
  square_det=true;
  }

  
  
  newposl=leftEncoder.read();
  newposr=rightEncoder.read();

  T_current=micros();
  
  if(newposl!=oldposl){
  oldposl=newposl;
}




if(newposr!=oldposr){
  oldposr=newposr;
}
  
  if(oldposr/2248.86*2*3.14*3.35 < 18 ||  oldposl/2248.86*2*3.14*3.35 < 18){
  forward();
  }
  else{
  hit_break();
  delay(1000);
  exit(0);
  }

if((T_current-T_start)/100000>7){
  hit_break();
  delay(1000);
  exit(0);
}
  //length of the robot is 175.04mm, diameter 67mm
}





void loop(){
Left4=digitalRead(L4);
Left3=digitalRead(L3);
Left2=digitalRead(L2);
Left1=digitalRead(L1);
Right1=digitalRead(R1);
Right2=digitalRead(R2);
Right3=digitalRead(R3);
Right4=digitalRead(R4);

//newposl=leftEncoder.read();
//newposr=rightEncoder.read();
//
//if(newposl!=oldposl){
//  oldposl=newposl;
//}
//
//
//if(newposr!=oldposr){
//  oldposr=newposr;
//}



if(is_even){
  if(Left4==1 && Left3==1 && Left1==0){
    forward();
  }
  if(Left4==1 && Left3==1 && Left2==1 && Left1==1){
    hit_break();
  }
  if(Left4==0 && Left3==0 && Left2==0 && Left1==0){
    square();
  }
  
}



else{
  if((Left4==0 ||  Left3==0  ) &&  (Right1==1 || Right2==1 || Right3==1 || Right4==1)){
    turn_left();
  }
   if((Left4==1 ||  Left3==1  || Left2==1 || Left1==1) &&  ( Right3==0 || Right4==0)){
    turn_right();
  }
}

}
