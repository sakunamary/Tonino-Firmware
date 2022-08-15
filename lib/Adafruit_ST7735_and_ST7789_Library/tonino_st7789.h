#ifndef _TONINO_ST7789H_
#define _TONINO_ST7789H_

#include "Adafruit_ST77xx.h"

/// Subclass of ST77XX type display for ST7789 TFT Driver
class LCD : public Adafruit_ST77xx {
public:
  LCD(int8_t cs, int8_t dc, int8_t mosi, int8_t sclk,
                  int8_t rst = -1);
  LCD(int8_t cs, int8_t dc, int8_t rst);

  void setRotation(uint8_t m);
  void init(uint16_t width, uint16_t height, uint8_t spiMode = SPI_MODE0);

  //append tonino display function 
    // display a number n from -999 to 9999
    void printNumber(int16_t n);
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
       




protected:
  uint8_t _colstart2 = 0, ///< Offset from the right
      _rowstart2 = 0;     ///< Offset from the bottom

private:
  uint16_t windowWidth;
  uint16_t windowHeight;
      // writes software display buffer to physical display
    void writeDisplay();
      // helper to remove leading zeros
    void calcDigits(uint8_t* dig, int16_t num);

};

#endif // _TONINO_ST7789H_
