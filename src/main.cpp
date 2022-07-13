


/*D1 mini  PINOUT table 
 5+           USB IN
 GND          USB GND
 TX IO1  TX0  Serial(PAD)
 D4 IO2       serial debug 
 RX IO3  RX0  Serial(PAD)
 D2 IO4  SDA  BMP180 I2C
 D1 IO5  SCL  BMP180 I2C
 D7 IO13      DHT 11 DATA
 D6 IO12      RX1   serial_to_drumer
 D5 IO14      TX1 serial_to_drumer        
 D8 IO15      
 D0 IO16      
*/









#define VERSION "3"

#include <Arduino.h>


#include <tonino.h>//define setting 的宏
#include <SerialCommand.h> //将电脑发过来的串口数据编程命令。
#include <tonino_serial.h> //串口处理函数汇集
#include <tonino_config.h> //EEPROM里面的数据设置

// frequence counting lib for color sensor, http://www.pjrc.com/teensy/td_libs_FreqCount.html, Version 1.0
#include <FreqCount.h>
// lib to access EEPROM, built-in, see http://arduino.cc/en/Reference/EEPROM
#include <EEPROM.h>


// color sensor object
#define CS_POWER 2
#define CS_S2    7
#define CS_S3    6
#define CS_LED   3
TCS3200 colorSense = TCS3200(CS_S2, CS_S3, CS_LED, CS_POWER, &display);



// object for all parameters
ToninoConfig tConfig = ToninoConfig(&colorSense, &display);
// object for serial communication
ToninoSerial tSerial = ToninoSerial(&colorSense, &display, &tConfig, VERSION);



boolean checkCommands() {   //检测是否有串口命令
  return tSerial.checkCommands();
}




// called when detected first calibration plate with quick scan
boolean calibrate() {
  sensorData sd;
  
  display.calibration1();
  WRITEDEBUGLN("Cal.");
  delay(500);
  
  // scan plate 1
  colorSense.scan(NULL, false, &sd);
  if (checkCommands()) return false;
  display.calibration1();
  delay(500);
  if (checkCommands()) return false;

  float redavg = sd.value[RED_IDX];
  float blueavg = sd.value[BLUE_IDX];
  
  if ((abs(redavg - LOW_RED) < RED_RANGE_LOW) && (abs(blueavg - LOW_BLUE) < BLUE_RANGE_LOW)) {

    float rb_low = redavg / blueavg;

    WRITEDEBUG(redavg);
    WRITEDEBUG("/");
    WRITEDEBUG(blueavg);
    WRITEDEBUG("=");
    WRITEDEBUGF(rb_low, 5);
    WRITEDEBUG("; ");

    // wait for other plate
    uint16_t delayTillUpTest = tConfig.getDelayTillUpTest() * 100;
    while (true) {
      display.calibration2();

      // wait till can is lifted and put down again
      while (colorSense.isDark()) {
        delay(delayTillUpTest);
        if (checkCommands()) return false;
      }
      display.up();
      while (colorSense.isLight()) {
        delay(delayTillUpTest);
        if (checkCommands()) return false;
      }
      display.calibration2();
      delay(500);

      // scan plate 2
      colorSense.scan(NULL, false, &sd);
      if (checkCommands()) return false;
      display.calibration2();
      delay(500);
      if (checkCommands()) return false;

      redavg = sd.value[RED_IDX];
      blueavg = sd.value[BLUE_IDX];
      float rb_high = redavg / blueavg;

      float cal[2];
      cal[0] = (HIGH_TARGET - LOW_TARGET) / (rb_high - rb_low);
      cal[1] = LOW_TARGET - cal[0]*rb_low;

      WRITEDEBUG(redavg);
      WRITEDEBUG("/");
      WRITEDEBUG(blueavg);
      WRITEDEBUG("=");
      WRITEDEBUGF(rb_high, 5);
      
      WRITEDEBUG("=>");
      WRITEDEBUGF(cal[0], 5);
      WRITEDEBUG(",");
      WRITEDEBUGLNF(cal[1], 5);

      if (checkCommands()) return false;

      tConfig.setCalibration(cal);
      delay(10);
      return true;
    }

  } else {
    // could not detect first plate even though quick check thought so - error and continue
    display.error();
    delay(3000);
    return false;
  }
}


// make a full scan and display on LCD
inline void scanAndDisplay(float* lastRaw) {
  boolean averaged = false; // indicates if readings got averaged with the lastRaw (the previous one)
  
  // make a measurement with stored configuration, parameters:
  // 1: lastRaw: passes lastRaw readings for averaging
  // 2: false: no display animation during scan
  // 3: NULL: not interested in raw values
  // 4: true: switch on LEDs
  // 5: false: no explicit external light removal
  // 6: averaged: return flag that indicates that result got averaged
  int32_t tval = colorSense.scan(lastRaw, false, NULL, true, false, &averaged);
 

  displayNum(tval);
  display.averaged(averaged); // display the averaged indicator
}








void setup() {
  // put your setup code here, to run once:

  // initialize serial communication
  tSerial.init(115200);

  // directly check if there is an incoming serial command (connected to computer)
  checkCommands();

  // color sensor init
  colorSense.init();

  // read parameters from EEPROM or write defaults if not available
  // be sure to call this _after_ display and colorSense have been initialized (using .init())
  tConfig.init();

  // check whether we are calibrating (detect first calibration plate)
  if (colorSense.isDark()) {
    if (tConfig.getCheckCalInit() && colorSense.isCalibrating() == LOW_PLATE) {
      display.clear();
      if (calibrate()) {
        display.done();
      }
    } else {
      // no calibration plate detected, directly make first scan
      scanAndDisplay(NULL);
    }
  } else {
    display.clear();
  }




}

void loop() {
  // put your main code here, to run repeatedly:
}