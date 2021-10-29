void setup() {
  // put your setup code here, to run once:
pinMode (A0, INPUT);
pinMode (A1, INPUT);
pinMode (A2, INPUT);
pinMode (A3, INPUT);
pinMode (A4, INPUT);
pinMode (A5, INPUT);
pinMode (A6, INPUT);
pinMode (A7, INPUT);
}
int s1,s2,s3,s4,s5,s6,s7,s8;
int stage=1;

void forward(){

}


void turn_left_90(){
  
}

void turn_right_90(){
  
}

void square(){
  
}

void loop() {
  // put your main code here, to run repeatedly:
s1 = digitalRead(A0);
s2 = digitalRead(A1);
s3 = digitalRead(A2);
s4 = digitalRead(A3);
s5 = digitalRead(A4);
s6 = digitalRead(A5);
s7= digitalRead(A6);
s8 = digitalRead(A7);


if(s1==s8 && s2==s7 && s3==s6  &&  s4==s5){
  if(s4==0 && s1==1){
    forward();
  }
  if(s1==0 && s2==0  &&  s3==0){
    stage=2;
    square();
  }
}
if(s1==0 && s2==0 && s3==0 && s3==0 && s4==0  && s5==0  && s6==1  && s7==1 && s8==1){
  turn_left_90(); 
}

if(s1==1 && s2==1 && s3==1 && s3==1 && s4==0  && s5==0  && s6==0  && s7==0 && s8==0){
  turn_right_90(); 
}
}
