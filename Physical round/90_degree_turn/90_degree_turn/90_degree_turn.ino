#include <Encoder.h>
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

int Left4,Left3,Left2,Left1,Right1,Right2,Right3,Right4;
Encoder leftEncoder(19,18);
Encoder rightEncoder(2,3);

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
    pinMode (LEFT_FORWARD, OUTPUT);//leftmotor
    pinMode (LEFT_BACKWARD, OUTPUT);//leftmotor
    pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
    pinMode(LEFT_MOTOR_PWM,OUTPUT);
    pinMode (RIGHT_FORWARD, OUTPUT);//rightmotor
    pinMode (RIGHT_BACKWARD, OUTPUT);//rightmotor
    pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);
    pinMode(RIGHT_MOTOR_PWM,OUTPUT);
      
     
    pinMode (A0, INPUT);//leftmost
    pinMode (A1, INPUT);
    pinMode (A2, INPUT);
    pinMode (A3, INPUT);
    pinMode (A4, INPUT);
    pinMode (A5, INPUT);
    pinMode (A6, INPUT);
    pinMode (A7, INPUT);//rightmost

    digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
    digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
}

void loop() {

    int Left4=digitalRead(A0);
    int Left3=digitalRead(A1);
    int Left2=digitalRead(A2);
    int Left1=digitalRead(A3);
    int Right1=digitalRead(A4);
    int Right2=digitalRead(A5);
    int Right3=digitalRead(A6);
    int Right4=digitalRead(A7);  



      if(Left4==1 && Left3==1 && Left1==0 && Right1==0 && Right3==1 && Right4==1){
        forward();
      }
      else if((Left4==0 && Left3==0 && Left2==0 && Left1==0 && Right1==0 && Right2==0 && Right3==0 && Right4==0)|| (Left3==0 && Left2==0 && Left1==0 && Right1==0 && Right2==0 && Right3==0)) {
         stop_();
         exit(0);
      }
      else if((Left4==0 &&  Left3==0  && Left2==0 && Left1==0 && Right1==0) && ( Right2==1 || Right3==1 || Right4==1)){
        while((Left4==1 && Left3==1 && Left2==1 && Left1==1 && Right1==1 && Right2==1 && Right3==1 && Right4==1) == false){
          forward();
        }
        stop_();
        while ((Left4==1 && Left3==1 && Left1==0 && Right1==0 && Right3==1 && Right4==1 && Left2==Right2)==false){
        left();
        }
        stop_();
      }
      else if((Left4==1 ||  Left3==1  || Left2==1 ) && ( Left1==0 && Right1==0 && Right2==0 && Right3==0 && Right4==0)){
        while((Left4==1 && Left3==1 && Left2==1 && Left1==1 && Right1==1 && Right2==1 && Right3==1 && Right4==1) == false){
          forward();
        }
        stop_();
        while ((Left4==1 && Left3==1 && Left1==0 && Right1==0 && Right3==1 && Right4==1 && Left2==Right2)==false){
        right();
        }
        stop_();
      }
      else if((Left4==0 ||  Left3==0  || Left2==0 ) &&  (Right1==1 || Right2==1 || Right3==1 || Right4==1)){
        slowright();
      }
      else if((Left4==1 ||  Left3==1  || Left2==1 || Left1==1) &&  ( Right2==0 || Right3==0 || Right4==0)){
        slowleft();
      }        
}


void forward()
{
analogWrite(LEFT_MOTOR_PWM, 255);
analogWrite(RIGHT_MOTOR_PWM, 255);  
digitalWrite(LEFT_FORWARD, HIGH);
digitalWrite(LEFT_BACKWARD, LOW);
digitalWrite(RIGHT_FORWARD, HIGH);
digitalWrite(RIGHT_BACKWARD, LOW);
}

void stop_()
{
analogWrite(LEFT_MOTOR_PWM, 255);
analogWrite(RIGHT_MOTOR_PWM, 255); 
digitalWrite(LEFT_FORWARD, LOW);
digitalWrite(LEFT_BACKWARD, LOW);
digitalWrite(RIGHT_FORWARD, LOW);
digitalWrite(RIGHT_BACKWARD, LOW);
}
void left()
{
analogWrite(LEFT_MOTOR_PWM, 100);
analogWrite(RIGHT_MOTOR_PWM, 200);  
digitalWrite(LEFT_FORWARD, LOW);
digitalWrite(LEFT_BACKWARD, HIGH);
digitalWrite(RIGHT_FORWARD, HIGH);
digitalWrite(RIGHT_BACKWARD, LOW);  
}
void right()
{
analogWrite(LEFT_MOTOR_PWM, 200);
analogWrite(RIGHT_MOTOR_PWM, 100);  
digitalWrite(LEFT_FORWARD, HIGH);
digitalWrite(LEFT_BACKWARD, LOW);
digitalWrite(RIGHT_FORWARD, LOW);
digitalWrite(RIGHT_BACKWARD, HIGH);
}

void slowleft(){
analogWrite(LEFT_MOTOR_PWM, 200);
analogWrite(RIGHT_MOTOR_PWM, 200);
digitalWrite(LEFT_FORWARD, HIGH);
digitalWrite(LEFT_BACKWARD, LOW);
digitalWrite(RIGHT_FORWARD, LOW);
digitalWrite(RIGHT_BACKWARD, LOW); 
}

void slowright(){
analogWrite(LEFT_MOTOR_PWM, 200);
analogWrite(RIGHT_MOTOR_PWM, 200);  
digitalWrite(LEFT_FORWARD, LOW);
digitalWrite(LEFT_BACKWARD, LOW);
digitalWrite(RIGHT_FORWARD, HIGH);
digitalWrite(RIGHT_BACKWARD, LOW);
}
