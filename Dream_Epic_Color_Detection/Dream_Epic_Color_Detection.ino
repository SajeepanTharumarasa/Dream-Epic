#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_TCS34725.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

int buttonPin =  6;
int buttonState = 0;
String color[3] = {"RED","GREEN","BLUE"};
int calibrater[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
int i = 0;


Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

#define OLED_RESET  -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  
  pinMode(buttonPin, INPUT); 
  tcs.begin(0x29); // TCS 34725 colour sensor begin
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);// OLED 1306 begin
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,10);
  display.println("DREAM EPIC");
  display.println("  sensor ");
  display.display();
  delay(2000);  
  
}

void loop() {
   uint16_t cleared, red, green, blue;
   while(i<3){
         buttonState = digitalRead(buttonPin);
         if (buttonState == HIGH){

              tcs.setInterrupt(false); // color sensor reading values begin.
              delay(60);                                                        
              tcs.getRawData(&red, &green, &blue, &cleared);                    
              tcs.setInterrupt(true); // End color sensor reading values.
              uint32_t sum = cleared; float R,G,B;
              R = red; R/=sum; R*=256; int r = R;
              G = green; G/=sum; G*=256; int g = G;
              B = blue; B/=sum; B*=256; int b = B;
              calibrater[i][0]= r; // store the color sensor reading values in a calibration array.
              calibrater[i][1]= g;
              calibrater[i][2]= b;
              display.clearDisplay();
              display.setTextSize(2);           
              display.setTextColor(WHITE);        
              display.setCursor(0, 10);           
              display.println(color[i]+" "+"done"); // display message after the each calibration steps.
              display.display();
              delay(1000);
             
              i=i+1;        
              }
        return;
     }
    checkcolor(); // call the check color function after the calibration.
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
  if (buttonState == HIGH){
    tcs.setInterrupt(false); // color sensor reading values begin.
    delay(60);                                                        
    tcs.getRawData(&red, &green, &blue, &cleared);                    
    tcs.setInterrupt(true); // End color sensor reading values.
    uint32_t sum = cleared; float R,G,B;
    R = red; R/=sum; R*=256; int r = R;
    G = green; G/=sum; G*=256; int g = G;
    B = blue; B/=sum; B*=256; int b = B;
    int value[3] = {r,g,b}; // store the color sensor reading values in a value array.
    for (int i = 0; i < 3; i++) {
      int a = 0;
      for (int j = 0; j < 3; j++) {
          if (calibrater[i][j]==(value[j])||(value[j]==(constrain(value[j],calibrater[i][j]-10,calibrater[i][j]+10)))){
              a+=1; // compare the values from the beginng values of the calibration array.
          }
      }
      if ((a==3)&&(i==0)){
      display.clearDisplay();
      display.setTextSize(2);           
      display.setTextColor(WHITE);        
      display.setCursor(0, 10);           
      display.print(color[i]+" "+"Color"); // Display the corresponding color in OLED.
      display.display();
      delay(2000);
      break;
      }
      else if ((a==3)&&(i==1)){
      display.clearDisplay();
      display.setTextSize(2);           
      display.setTextColor(WHITE);        
      display.setCursor(0, 10);           
      display.print(color[i]+" "+"Color"); // Display the corresponding color in OLED.      
      display.display(); 
      delay(2000);
      break;
      }
      else if ((a==3)&&(i==2)){
      display.clearDisplay();
      display.setTextSize(2);           
      display.setTextColor(WHITE);        
      display.setCursor(0, 10);           
      display.print(color[i]+" "+"Color"); // Display the corresponding color in OLED.      
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
