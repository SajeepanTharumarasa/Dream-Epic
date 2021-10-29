/*
The range readings are in units of mm. */

#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET 4
#define I2C_address 0x30


Adafruit_SSD1306 display(SCREEN_WIDTH,SCREEN_HEIGHT,&Wire,OLED_RESET);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

#if (SSD1306_LCDHEIGHT != 32)
 #error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif


int pButton = 26;
int count = 0;
int d = 0;
float D4 =40 ,D8 =80,D;

int dCalculate(float D4, float D8, float D);


void setup()
{
  Serial.begin(9600);
  pinMode(pButton, INPUT);
    
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);  // initialize with the I2C addr 0x30
  display.display();
  delay(1000);
    
  
  Wire.begin();

    // text display big!
  display.setTextSize(4);
  display.setTextColor(WHITE);

  if (!lox.begin(I2C_address)) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
 
}

void loop()
{
  
  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    if (digitalRead(pButton)==HIGH){
       if (count = 0){                                                              //caliberation state one

            //Display that this is caliberation stage...
            display.clearDisplay();
            display.setCursor(0,10);
            display.print("This is the caliberation State for 4cm");
            display.display();

            //Getting value for the caliberation...
            D4 = measure.RangeMilliMeter;

            //Avoiding detecting button press as multiple ones...
           while(digitalRead(pButton)==HIGH){
              delay(50);
            } 

            //Incase fuction is too fast slow a bit
            delay(1000);
            count++;
      }

      
      else if (count == 1){                                                     //caliberation state two
            
            //Display that this is caliberation state
            display.clearDisplay();
            display.setCursor(0,10);
            display.print("This is the caliberation State for 8cm");
            display.display();

            //Getting the Reading 
            D8= measure.RangeMilliMeter;

            //Avoid button press as multiple ones
            while(digitalRead(pButton)==HIGH){
              delay(50);
            }
            
            
            //Slow a bit
            delay(1000);
            count++;
      }
    }



    // Displaying Distances after Caliberation
    if (count>1){
      D = measure.RangeMilliMeter;
      d = dCalculate(D4,D8,D);
      display.print("Current Reading : ");
      display.print(d);
      delay(1000);
    }
  }


 // If the measurements out of range...
  else {
    display.display();
    display.print("Beyond the Limit");
    return;
  }
}



// function for calculating Distances
int dCalculate(float D4, float D8, float D){
  int d;
  d = round(4 + 4*(D-D4)/(D8-D4));
  return d;
  
}
