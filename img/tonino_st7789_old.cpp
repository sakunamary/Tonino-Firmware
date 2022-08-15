// tonino_lcd.cpp
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


//#include <tonino_lcd.h>



LCD::LCD() {
  // empty
}

LCD::~LCD() {
  // empty
}

// initializes display using I2C start sequence
// sets max brightness, no blinking
void LCD::init() {
  tft.init(240, 280);           // Init ST7789 240x240
  tft.setRotation(2); 

}


// write bitmask on digit d (0, 1, 3, 4)
inline void LCD::writeDigitRaw(uint8_t d, uint8_t bitmask) {
  if (d > 4) {
    error();
    return;
  }
  displaybuffer[d] = bitmask;
}
    
// write number num (0-F) on digit d (0, 1, 3, 4)
inline void LCD::writeDigitNum(uint8_t d, uint8_t num) {
  if (d > 4 || num > 15) {
    error();
    return;
  }
  writeDigitRaw(d, numbertable[num]);
}

// display a number n from -999 to 9999
void LCD::printNumber(int16_t n) {
 
}

// sets the display brightness (0-15, 15=max brightness)
void LCD::setBrightness(uint8_t b) {

}

// get the display brightness (0-15, 15=max brightness)
uint8_t LCD::getBrightness() {
  return _brightness;
}

// draw a horizontal line
void LCD::line() {


}  




// display the letters "dose"
void LCD::dose() {

}

// display the letters "hi"
void LCD::hi() {

}

// display the letters "up"
void LCD::up() {
//显示 举高的GIF图片
}

// display the letters "PC"
void LCD::connected() {
//显示pc图片

}

// display the letters "EEEE"
void LCD::error() {
//显示错误图片

}

// clears the display (all segments to dark)
void LCD::clear() {


}

// light up the dot of the first digit
void LCD::averaged(boolean dot) {
  writeDigitRaw(0, (displaybuffer[0] & 0b01111111) | (dot << 7));
  writeDisplay();
}

// shows a rotating circle; total time =repeat*timePerCircle
void LCD::circle(uint8_t repeat, uint16_t timePerCircle) {
  int16_t dtime = timePerCircle / 12;
//转菊花图片
}

// displays a number; effect: each digit sequentially counts from 0
void LCD::dropNumber(uint16_t num) {
 
}

// displays a number; effect: quickly count from 0 to num
void LCD::countToNumber(uint16_t num) {
  if (num > 9999) {
    error();
    return;
  }
  for (int16_t i = 0; i <= num; ++i) {
    printNumber(i);
    delay(5);
  }
}

// displays a number; effect: quickly count down from 888
void LCD::countDownToNumber(uint16_t num) {
 
}

// displays a number; effect: start with 8888, remove all segments until num appears
void LCD::numberFromEights(uint16_t num) {
  if (num > 9999) {
    error();
    return;
  }

  int16_t dtime = 100;
  uint8_t dig[5];
  uint8_t eight[5];

  for (int led = 0; led < 6; led++) {
    eight[led] = 0b01111111;
  }
  
  dig[0] = numbertable[num / 1000];
  num %= 1000;
  dig[1] = numbertable[num / 100];
  num %= 100;
  dig[3] = numbertable[num / 10];
  dig[4] = numbertable[num % 10];
  if (dig[0] == numbertable[0]) {
    dig[0] = 0;
    if (dig[1] == numbertable[0]) {
      dig[1] = 0;
      if (dig[3] == numbertable[0]) {
        dig[3] = 0;
      }
  }
  }
  
  eights();
  
  for (int i = 0; i < 7; i++) {
    delay(dtime);
    for (int led = 0; led < 5; led++) {
      if (led == 2) {
        continue;
      }
      if ((dig[led] >> i) % 2 == 0) {
        eight[led] -= (1 << i);
        writeDigitRaw(led, eight[led]);
        writeDisplay();
      }
    }
  }
}

// displays a number; effect: start empty, add all segments until num appears
void LCD::numberFromEmpty(uint16_t num) {
  if (num > 9999) {
    error();
    return;
  }
  int16_t dtime = 100;
  uint8_t dig[5];
  uint8_t eight[5];

  for (int led = 0; led < 6; led++) {
    eight[led] = 0b0;
  }
  
  dig[0] = numbertable[num / 1000];
  num %= 1000;
  dig[1] = numbertable[num / 100];
  num %= 100;
  dig[3] = numbertable[num / 10];
  dig[4] = numbertable[num % 10];
  if (dig[0] == numbertable[0]) {
    dig[0] = 0;
    if (dig[1] == numbertable[0]) {
      dig[1] = 0;
      if (dig[3] == numbertable[0]) {
        dig[3] = 0;
      }
    }
  }

  clear();
  
  for (int i = 0; i < 7; i++) {
    delay(dtime);
    for (int led = 0; led < 5; led++) {
      if (led == 2) {
        continue;
      }
      if ((dig[led] >> i) % 2 == 1) {
        eight[led] += (1 << i);
        writeDigitRaw(led, eight[led]);
        writeDisplay();
      }
    }
  }
}

// displays a number; effect: simulate oscillating value until final value num
void LCD::approx(uint16_t num) {
  if (num > 9999) {
    error();
    return;
  }
  uint8_t dtime = 20;
  for (int8_t i = 10; i > -8; --i) {
    printNumber(num + i);
    delay(dtime);
  }
  for (int8_t i = -4; i < 4; ++i) {
    printNumber(num + i);
    delay(2*dtime);
  }
  for (int8_t i = 2; i > -1; --i) {
    printNumber(num + i);
    delay(3*dtime);
  }
}

// displays a number; effect: like a snake leaving behind the num
void LCD::snake(uint16_t num) {
  
}

// shows 'CAL' and, if circle=true a small one time rotating circle in the rightmost digit
// the circle takes roughly 600ms
void LCD::calibration(bool circle) {
  
}

// shows 'CAL1'
void LCD::calibration1() {
//cal1 过程显示

}
//'cal1 finish'
void LCD::calibration1_done() {
//完成 cal1 显示

}


// shows 'CAL2'
void LCD::calibration2() {
//cal2 过程显示
}

//'cal2 finish'
void LCD::calibration2_done() {
//完成 cal1 显示

}


// shows 'done'
void LCD::done() {

}


// writes software display buffer to physical display
void LCD::writeDisplay(void) {
  Wire.beginTransmission(i2cAddr);
  Wire.write((uint8_t)0x00);

  for (uint8_t i = 0; i < 8; ++i) {
    Wire.write(displaybuffer[i] & 0xFF);    
    Wire.write(displaybuffer[i] >> 8);    
  }
  Wire.endTransmission();
}

