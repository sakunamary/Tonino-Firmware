// tonino_lcd.h
//----------------
// library for Tonino display control
//
// Basic parts of the LCD code has been adapted from
// https://github.com/adafruit/Adafruit-LED-Backpack-Library
//
// *** BSD License ***
// ------------------------------------------------------------------------------------------
// Copyright (c) 2016, Paul Holleis, Marko Luther
// All rights reserved.
//
// Authors:  Paul Holleis, Marko Luther
//
// Redistribution and use in source and binary forms, with or without modification, are 
// permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice, this list of 
//   conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice, this list 
//   of conditions and the following disclaimer in the documentation and/or other materials 
//   provided with the distribution.
//
//   Neither the name of the copyright holder(s) nor the names of its contributors may be 
//   used to endorse or promote products derived from this software without specific prior 
//   written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL 
// THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// ------------------------------------------------------------------------------------------


#ifndef _TONINO_ST7789_H
#define _TONINO_ST7789_H

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> 
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>


#define TFT_CS         7
#define TFT_RST        9 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC         8
#define TFT_MOSI 11  // Data out
#define TFT_SCLK 13  // Clock out

//



class LCD : public Adafruit_ST7789{
  public:
    LCD(int8_t cs, int8_t dc, int8_t mosi, int8_t sclk,
                  int8_t rst = -1);
    ~LCD(void);
  void setRotation(uint8_t m);
  void init(uint16_t width, uint16_t height, uint8_t spiMode = SPI_MODE0);



    // initializes display using I2C start sequence
    // sets max brightness, no blinking
   // void init(uint8_t addr);
    void init();
    // write bitmask on digit d (0, 1, 3, 4)
    //void writeDigitRaw(uint8_t d, uint8_t bitmask); //作废 
    
    // write number num (0-F) on digit d (0, 1, 3, 4)//作废 
    //void writeDigitNum(uint8_t d, uint8_t num);
    
    // display a number n from -999 to 9999
    void printNumber(int16_t n);

    // sets the display brightness (0-15, 15=max brightness)//作废 
    //void setBrightness(uint8_t b);
    
    // get the display brightness (0-15, 15=max brightness)//作废 
    //uint8_t getBrightness();
    
    // sets the display to blink (0=no, 1=fast ... 3=slow)//作废 
    //void setBlinkRate(uint8_t rate);

    
    // draw a horizontal line ////作废 
    void line();
    
    // display 4 eights, i.e. light all segments //作废 
   // void eights();
    
    // display 4 eights one after the other, takes 1 second //作废
   // void eightSequence();
    
    // display the letters "dose"
    void dose();
    
    // display the letters "hi" //替换成开机画面
    void hi();
    
    // display the letters "up" // 显示up 动画
    void up();
    
    // display the letters "PC" //显示PC静态画面
    void connected();
    
    // display the letters "EEEE" //显示错误画面
    void error();
    
    // clears the display (all segments to dark)
    void clear();
    
    // indicates averaging (light first dot)
    void averaged(boolean dot);
    
    // shows a rotating circle; total time =repeat*timePerCircle //作废
    void circle(uint8_t repeat = 1, uint16_t timePerCircle = 700);
    
    // displays a number; effect: each digit sequentially counts from 0
    void dropNumber(uint16_t num);
    
    // displays a number; effect: quickly count from 0 to num
    void countToNumber(uint16_t num);
    
    // displays a number; effect: quickly count down from 888
    void countDownToNumber(uint16_t num);
    
    // displays a number; effect: start with 8888, remove all segments until num appears
    void numberFromEights(uint16_t num);
    
    // displays a number; effect: start empty, add all segments until num appears
    void numberFromEmpty(uint16_t num);
    
    // displays a number; effect: simulate oscillating value until final value num
    void approx(uint16_t num);
    
    // displays a number; effect: like a snake leaving behind the num
    void snake(uint16_t num);
    
    // shows 'CAL' and, if circle=true a small one time rotating circle in the rightmost digit
    // the circle takes roughly 600ms
    void calibration(bool circle = true);
    
    // shows 'CAL1' //显示cal1 字样
    void calibration1();
    
    // shows 'CAL2'   //显示cal2字样
    void calibration2();
    
    // shows 'done'   //显示完成字样
    void done();
    
    // if digit<0 shows a '-' moving from left to right with given delays
    // if digit in {0,1,3,4}, displays a '-' at that digit
    //void lineAnim(int8_t digit, uint16_t dtime = 200); //作废

protected:
  uint8_t _colstart2 = 0, ///< Offset from the right
      _rowstart2 = 0;     ///< Offset from the bottom


  private:
    // stores the display's I2C address

    uint16_t windowWidth;
    uint16_t windowHeight;
    //static uint8_t i2cAddr;
    // virtual display buffer; needs to be sent by writeDisplay()
    //static uint16_t displaybuffer[8]; 
    // specifies LCD brightness
    //uint8_t  _brightness;

    // writes software display buffer to physical display
    void writeDisplay();
    // helper to remove leading zeros
    void calcDigits(uint8_t* dig, int16_t num);

};

#endif