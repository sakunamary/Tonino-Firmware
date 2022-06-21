


#define VERSION "3"



#include <Arduino.h>


#include <tonino.h>//define setting 的宏
#include <SerialCommand.h> //将电脑发过来的串口数据编程命令。
#include <tonino_serial.h> //串口处理函数汇集
#include <tonino_config.h> //EEPROM里面的数据设置
// i2c lib for LCD, built-in, see http://arduino.cc/en/Reference/Wire
#include <Wire.h>
// frequence counting lib for color sensor, http://www.pjrc.com/teensy/td_libs_FreqCount.html, Version 1.0
#include <FreqCount.h>
// lib to access EEPROM, built-in, see http://arduino.cc/en/Reference/EEPROM
#include <EEPROM.h>
// low power library, built-in, see http://playground.arduino.cc/Learning/arduinoSleepCode
#include <avr/power.h>  
// low power library, https://github.com/rocketscream/Low-Power, Version 1.30
#include <LowPower.h> 




// LCD object
LCD display = LCD(); //待换

// color sensor object
#define CS_POWER 2
#define CS_S2    7
#define CS_S3    6
#define CS_LED   3
TCS3200 colorSense = TCS3200(CS_S2, CS_S3, CS_LED, CS_POWER, &display);

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}