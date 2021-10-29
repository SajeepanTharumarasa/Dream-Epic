
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_TCS34725.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

int buttonPin =  6;
int buttonState = 1;
String color[3] = {"RED","GREEN","BLUE"};
int calibrater[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
//int calibrater[3][3] = {{150,70,40},{140,200,60},{90,130,210}};
int i = 0;


Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

#define OLED_RESET  -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(9600);
  pinMode(buttonPin, INPUT_PULLUP); 
  tcs.begin(0x29);

//  if (tcs.begin(0x29)) {
//    Serial.println("Found sensor");
//  } else {
//    Serial.println("No TCS34725 found ... check your connections");
//    while (1); // halt!
//  }
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);//0x7B
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,10);
  display.println("DREAM EPIC");
  display.println("  sensor ");
  display.display(); 
  
}

void loop() {
   uint16_t cleared, red, green, blue;
   while(i<3){
         buttonState = digitalRead(buttonPin);
         if (buttonState == LOW){
              tcs.setInterrupt(false); 
              delay(60);                                                        
              tcs.getRawData(&red, &green, &blue, &cleared);                    
              tcs.setInterrupt(true);
              uint32_t sum = cleared; float R,G,B;
              R = red; R/=sum; R*=256; int r = R;
              G = green; G/=sum; G*=256; int g = G;
              B = blue; B/=sum; B*=256; int b = B;
              calibrater[i][0]= r;
              calibrater[i][1]= g;
              calibrater[i][2]= b;
              display.clearDisplay();
              display.setTextSize(2);           
              display.setTextColor(WHITE);        
              display.setCursor(0, 10);           
              display.println(color[i]+" "+"done");
              
//              Serial.print(r,DEC); 
//              Serial.println(g,DEC);
//              Serial.println(b,DEC);
//              
//              display.println(red,DEC); 
//              display.println(green,DEC);
//              display.println(blue,DEC);
                    
              display.display();
              delay(1000);
             
              i=i+1;        
              }
        return;
     }
    checkcolor();
}
void checkcolor(){
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,10);
  display.println("  Insert"); 
  display.println("  Colour");
  display.print("  Patch");
  display.display(); 
  uint16_t cleared, red, green, blue; 
  buttonState = digitalRead(buttonPin);
  if (buttonState == LOW){
    tcs.setInterrupt(false);
    delay(60);                                                        
    tcs.getRawData(&red, &green, &blue, &cleared);                    
    tcs.setInterrupt(true);
    uint32_t sum = cleared; float R,G,B;
    R = red; R/=sum; R*=256; int r = R;
    G = green; G/=sum; G*=256; int g = G;
    B = blue; B/=sum; B*=256; int b = B;
    int value[3] = {r,g,b};
    //int value[3] = {82,136,204};
    for (int i = 0; i < 3; i++) {
      int a = 0;
      for (int j = 0; j < 3; j++) {
          if (calibrater[i][j]==(value[j])||(value[j]==(constrain(value[j],calibrater[i][j]-10,calibrater[i][j]+10)))){
              a+=1;
          }
      }
      if ((a==3)&&(i==0)){
      display.clearDisplay();
      display.setTextSize(2);           
      display.setTextColor(WHITE);        
      display.setCursor(0, 10);           
      display.print(color[i]+" "+"Color");
      display.display();
      delay(2000);
      break;
      }
      else if ((a==3)&&(i==1)){
      display.clearDisplay();
      display.setTextSize(2);           
      display.setTextColor(WHITE);        
      display.setCursor(0, 10);           
      display.print(color[i]+" "+"Color");       
      display.display(); 
      delay(2000);
      break;
      }
      else if ((a==3)&&(i==2)){
      display.clearDisplay();
      display.setTextSize(2);           
      display.setTextColor(WHITE);        
      display.setCursor(0, 10);           
      display.print(color[i]+" "+"Color");       
      display.display(); 
      delay(2000);
      break;
      }
      else{
        continue;
      }
        
      
    }
    
    
  }
  
}


/*
// --------------------------------------
// i2c_scanner

 
#include <Wire.h>
 
 
void setup()
{
  Wire.begin();
 
  Serial.begin(9600);
  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner");
}
 
 
void loop()
{
  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
 
  delay(5000);           // wait 5 seconds for next scan
}

*/
    
       
    
