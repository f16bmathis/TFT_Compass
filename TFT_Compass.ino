/*   Compass that displays airport runway number (heading divided by 10) Compass needs calibration. Jumps alot around 270
* Only works on a MEGA so far... Works with LSM303DLHC Sensor, from Amazon Dec 2021. VCC, GRD, SCL to SCL (or Pin 21), SDA to SDA (or Pin 19).
* This source code is protected under the terms of the MIT License and is copyright (c) 2013 by David Bird and permission is hereby granted, free of charge, to
* any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software
* without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, but not to sub-license and/or
* to sell copies of the Software or to permit persons to whom the Software is furnished to do so, subject to the following conditions:
* The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
* I've edited parts and pieces from several sketches to put this together. Brian Mathis
*/  

//(c) D BIRD 2013
//An Arduino code example for interfacing with the HMC5883
//Uses:
//Analog input 4 I2C SDA or equivlanet for MEGA
//Analog input 5 I2C SCL

#include <Arduino.h>
#define USE_ADAFRUIT_SHIELD_PINOUT 1
#include <SPI.h>
#include <Wire.h> //I2C Arduino Library
#include <Adafruit_LSM303DLH_Mag.h>            // Using this sensor

const int centerX  = 120;               // centers circle Left to Right
const int centerY  = 120;               // sets circle to top of screen
const int diameter = 115;               // Sets the diameter of the circle

#include <Adafruit_GFX.h>     // Core graphics library
#include <MCUFRIEND_kbv.h>    // For display
#include <Adafruit_TFTLCD.h>  // Hardware-specific library  ?? for Display ??

MCUFRIEND_kbv tft;            // For display

//   Assign human-readable names to some common 16-bit color values:

#define  BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#define address 0x1E //0011110b, I2C 7bit address of HMC5883

const int x_offset = 30;        // Original   30;
const int y_offset = 128;       // Original   128;
const int z_offset = 0;

int last_dx, last_dy, dx, dy;


void setup(){

  Serial.begin(9600);                                                    // Added for troubleshooting
  tft.reset();
  uint16_t identifier = tft.readID(); // Found ILI9325 LCD driver
  tft.begin(identifier);
  tft.setRotation(0);                 // Flips Rotation 180 from 0 to 2, Landscape / portrait
  tft.fillScreen(BLACK);

  //Initialize I2C communications

  Wire.begin();

  delay(300); // Slight delay for screen to start

  last_dx = centerX;
  last_dy = centerY;

}

void loop(){

  double angle;

  int x,y,z; //triple axis data

  //Tell the HMC5883 where to begin reading data

  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();

 //Read data from each axis, 2 registers per axis

  Wire.requestFrom(address, 6);

  if(6<=Wire.available()){

    x = Wire.read() << 8 | Wire.read();
    z = Wire.read() << 8 | Wire.read();
    y = Wire.read() << 8 | Wire.read();

  }

  Serial.write (x);                                      //added for troubleshooting

  Draw_Compass_Rose();

  angle= atan2((double)y + y_offset,(double)x + x_offset)* (180 / 3.141592654) + 180;

  dx = (diameter * cos((angle-90)*3.14/180)) + centerX;    // calculate X position

  dy = (diameter * sin((angle-90)*3.14/180)) + centerY;    // calculate Y position

  Serial.print(angle);
  tft.setTextColor(WHITE,BLACK); tft.setTextSize(4);       // Angle readout
  tft.setCursor(5, 260); tft.print ("         ");          // Erases old value, so a digit is not left behind
  tft.setCursor(100, 260); tft.println(angle, 0);           // Position of angle readout
                                                           // digits behind the decimal point I.E. 355.00 degrees, 
                                                           // "0" means none, 1=one, 2=two digits behind decimal point

  arrow(last_dx,last_dy, centerX, centerY, 15, 15,BLACK);  // Erase last arrow 
  arrow(dx,dy, centerX, centerY, 15, 15, YELLOW);          // Draw arrow in new position
     
  //      ?, ?,  center of guage, triangle size, color                      
  
  last_dx = dx;
  last_dy = dy;

  delay(25);

   }


  void display_item(int x, int y, String token, int txt_color, int txt_size) {

  tft.setCursor(x, y);
  tft.setTextColor(txt_color);
  tft.setTextSize(txt_size);
  tft.print(token);
  tft.setTextSize(2); // Back to default text size

   }
 

  void arrow(int x2, int y2, int x1, int y1, int alength, int awidth, int color) {

  float distance;

  int dx, dy, x2o,y2o,x3,y3,x4,y4,k;

  distance = sqrt(pow((x1 - x2),2) + pow((y1 - y2), 2));

  dx = x2 + (x1 - x2) * alength / distance;
  dy = y2 + (y1 - y2) * alength / distance;
  k = awidth / alength;
  x2o = x2 - dx;
  y2o = dy - y2;
  x3 = y2o * k + dx;
  y3 = x2o * k + dy;
  x4 = dx - y2o * k;
  y4 = dy - x2o * k;

  tft.drawLine(x1, y1, x2, y2, color);
  tft.drawLine(x1, y1, dx, dy, color);
  tft.drawLine(x3, y3, x4, y4, color);
  tft.drawLine(x3, y3, x2, y2, color);
  tft.drawLine(x2, y2, x4, y4, color);

   }
 

void Draw_Compass_Rose() {

  int dxo, dyo, dxi, dyi;

  tft.drawCircle(centerX,centerY,diameter,GREEN);  // Draw compass circle

  for (float i = 0; i <360; i = i + 22.5) {

    dxo = diameter * cos((i-90)*3.14/180);
    dyo = diameter * sin((i-90)*3.14/180);
    dxi = dxo * 0.9;
    dyi = dyo * 0.9;

    tft.drawLine(dxi+centerX,dyi+centerY,dxo+centerX,dyo+centerY,GREEN);  

   }

  display_item((centerX-5),(centerY-90),"N",WHITE,2);
  display_item((centerX-5),(centerY+75),"S",WHITE,2);
  display_item((centerX+85),(centerY-5),"E",WHITE,2);
  display_item((centerX-90),(centerY-5),"W",WHITE,2);

   }
