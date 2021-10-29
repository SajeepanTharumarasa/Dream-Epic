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

// rgb light pin setting
#define RED 46
#define GREEN 44
int ir_sensors[8] = {L4,L3,L2,L1,R1,R2,R3,R4};
int right;
int left;
void setup(){
  pinMode(RED,OUTPUT);
  pinMode(GREEN,OUTPUT);

  for(int i=0;i<8;i++){
    pinMode(ir_sensors[i],INPUT);}
  }

void loop(){
  for (int i=0;i<4;i++){
    left = digitalRead(ir_sensors[i]);
    right = digitalRead(ir_sensors[7-i]);

   if (left==right ==1){
      analogWrite(RED,85*0);
      analogWrite(GREEN,85*0);
      }
   else if(right==1){
      analogWrite(GREEN,85*0);
      analogWrite(RED,85*(3-i));
      delay(50);
      break;}
    else if(left==1){
      analogWrite(GREEN,85*(3-i));
      analogWrite(RED,85*0);
      delay(50);
      break;}
    
    }
  }
