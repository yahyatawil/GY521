#pragma once
//
//    FILE: GY521.h
//  AUTHOR: Rob Tillaart
// VERSION: 0.5.2
// PURPOSE: Arduino library for I2C GY521 accelerometer-gyroscope sensor
//     URL: https://github.com/RobTillaart/GY521


#include "Arduino.h"
#include "Wire.h"


#define GY521_LIB_VERSION           (F("0.5.2"))


//  THROTTLE TIMING
#ifndef GY521_THROTTLE_TIME
#define GY521_THROTTLE_TIME         10   //  milliseconds
#endif


//  ERROR CODES
#define GY521_OK                     0
#define GY521_THROTTLED              1
#define GY521_ERROR_READ            -1
#define GY521_ERROR_WRITE           -2
#define GY521_ERROR_NOT_CONNECTED   -3


//  CONVERSION CONSTANTS
#define GY521_RAD2DEGREES          (180.0 / PI)
#define GY521_RAW2DPS              (1.0 / 131.0)
#define GY521_RAW2G                (1.0 / 16384.0)

typedef enum {
ACC_1000_HZ_ODR = 0x00, //8k gyro
ACC_500_HZ_ODR = 0x01, // 4k
ACC_250_HZ_ODR = 0x02, // 2k
ACC_125_HZ_ODR = 0x03, // 1k
ACC_62_HZ_ODR = 0x04, // 500hz
ACC_31_HZ_ODR = 0x05, // 250hz
ACC_15_HZ_ODR = 0x06, // 125hz
ACC_7_HZ_ODR = 0x07, // 62hz
ACC_3_5_HZ_ODR = 0x08, // 31hz
ACC_1_5_HZ_ODR = 0x09, // 15hz
ACC_0_7_HZ_ODR = 0x0A, // 7hz
ACC_0_3_HZ_ODR = 0x0B, // 3.5hz
ACC_0_1_HZ_ODR = 0x0C // 1.5 hz
} t_acc_odr;

class GY521
{
public:
  //  address == 0x68 or 0x69
  GY521(uint8_t address = 0x69, TwoWire *wire = &Wire);

  bool     begin();
  bool     isConnected();
  void     reset();
  void     intEnable();
  uint8_t intStatus();
  void     accUpdateRate(t_acc_odr rate);
  //  EXPERIMENTAL
  //  calibrate needs to be called to compensate for errors.
  //  must be called after setAccelSensitivity(as); and setGyroSensitivity(gs);
  void     calibrate(uint16_t times);

  bool     wakeup();
  //  throttle to force delay between reads.
  void     setThrottle(bool throttle = true) { _throttle = throttle; };
  bool     getThrottle()                     { return _throttle; };
  //  0..65535 max milliseconds == roughly 1 minute.
  void     setThrottleTime(uint16_t ti )     { _throttleTime = ti; };
  uint16_t getThrottleTime()                 { return _throttleTime; };


  //  SET BEFORE READ
  //  as = 0,1,2,3 ==> 2g 4g 8g 16g
  bool     setAccelSensitivity(uint8_t as);
  uint8_t  getAccelSensitivity();          //  returns 0,1,2,3
  //  gs = 0,1,2,3  ==>  250, 500, 1000, 2000 degrees/second
  bool     setGyroSensitivity(uint8_t gs);
  uint8_t  getGyroSensitivity();           //  returns 0,1,2,3
  //  normalizes Pitch Roll and Yaw.
  void     setNormalize(bool normalize = true) { _normalize = normalize; };
  bool     getNormalize() { return _normalize; };


  //  READ THE SENSOR
  //  returns GY521_OK or one of the error codes above.
  int16_t  read();
  //  optimized partial reading
  //  read accelerometer only
  int16_t  readAccel();
  //  read gyroscope only can be done too
  //  however for pitch roll yaw you need all.
  int16_t  readGyro();
  //  read temperature only, does not affect throttle.
  int16_t  readTemperature();


  //  CALL AFTER READ
  float    getAccelX()   { return _ax; };
  float    getAccelY()   { return _ay; };
  float    getAccelZ()   { return _az; };
  
  int    getAccelX_raw()   { return _ax_raw; };
  int    getAccelY_raw()   { return _ay_raw; };
  int    getAccelZ_raw()   { return _az_raw; };
  
  int    getGyroX_raw()   { return _gx_raw; };
  int    getGyroY_raw()   { return _gy_raw; };
  int    getGyroZ_raw()   { return _gz_raw; };
  
  
  float    getAngleX()   { return _aax; };
  float    getAngleY()   { return _aay; };
  float    getAngleZ()   { return _aaz; };
  float    getTemperature() { return _temperature; };
  float    getGyroX()    { return _gx; };
  float    getGyroY()    { return _gy; };
  float    getGyroZ()    { return _gz; };

  //  EXPERIMENTAL
  //  pitch, roll and yaw is work in progress.
  float    getPitch()    { return _pitch; };
  float    getRoll()     { return _roll; };
  float    getYaw()      { return _yaw; };


  //  last time sensor is actually read.
  uint32_t lastTime()    { return _lastTime; };


  //  generic worker to get access to all functionality
  uint8_t  setRegister(uint8_t reg, uint8_t value);
  uint8_t  getRegister(uint8_t reg);


  //  get last error and reset error to OK.
  int16_t  getError()    { return _error; _error = GY521_OK; };


  //  calibration errors
  float    axe = 0, aye = 0, aze = 0;  //  accelerometer errors
  float    gxe = 0, gye = 0, gze = 0;  //  gyro errors


private:
  uint8_t  _address;                //  I2C address
  bool     _throttle = true;        //  to prevent reading too fast
  uint16_t _throttleTime = GY521_THROTTLE_TIME;
  uint32_t _lastTime = 0;           //  to measure duration for math & throttle
  uint32_t _lastMicros = 0;         //  to measure duration for math & throttle
  int16_t  _error = GY521_OK;       //  initially everything is OK

  uint8_t  _afs = 0;                //  sensitivity factor
  float    _raw2g = GY521_RAW2G;    //  raw data to gravity g's
  float    _ax, _ay, _az;           //  accelerometer raw
  int      _ax_raw,_ay_raw,_az_raw; 
  int      _gx_raw,_gy_raw,_gz_raw; 
  float    _aax, _aay, _aaz;        //  accelerometer processed

  uint8_t  _gfs = 0;
  float    _raw2dps = GY521_RAW2DPS;
  float    _gx, _gy, _gz;           //  gyro raw
  float    _gax, _gay, _gaz;        //  gyro processed
  float    _pitch, _roll, _yaw;     //  used by user

  float    _temperature = 0;

  bool     _normalize = true;       //  default true.

  //  to read register of 2 bytes.
  int16_t  _WireRead2();

  TwoWire*  _wire;
};


//  -- END OF FILE --
