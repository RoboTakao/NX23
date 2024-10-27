#include <Wire.h>
#include "PCA9685.h"
#if defined(ARDUINO_SAM_DUE)
 #define WIRE Wire1
#else
 #define WIRE Wire
#endif


PCA9685::PCA9685(uint8_t addr) {
  i2c_addr = addr;
}

void PCA9685::begin(void) {
 WIRE.begin();
 reset();
}

void PCA9685::reset(void) {
 write8(PCA9685_MODE1, 0x0);
}

void PCA9685::setPWMFreq(float freq) {  
  float prescaleval = 25000000;
  prescaleval /= 4096;
  prescaleval /= freq;
  prescaleval -= 1;
  uint8_t prescale = floor(prescaleval + 0.5);
  
  uint8_t oldreg = read8(PCA9685_MODE1);
  uint8_t newreg = (oldreg&0x7F) | 0x10; // sleep
  write8(PCA9685_MODE1, newreg); // go to sleep
  write8(PCA9685_PRESCALE, prescale); // set the prescaler
  write8(PCA9685_MODE1, oldreg);
  delay(5);
  write8(PCA9685_MODE1, oldreg | 0xa1);  //  This sets the MODE1 register to turn on auto increment.
}

void PCA9685::setPWM(uint8_t num, uint16_t on, uint16_t off) {
  //Serial.print("Setting PWM "); Serial.print(num); Serial.print(": "); Serial.print(on); Serial.print("->"); Serial.println(off);

  WIRE.beginTransmission(i2c_addr);
  WIRE.write(LED0_ON_L+4*num);
  WIRE.write(on);
  WIRE.write(on>>8);
  WIRE.write(off);
  WIRE.write(off>>8);
  WIRE.endTransmission();
}

void PCA9685::write8(uint8_t addr, uint8_t d) {
  WIRE.beginTransmission(i2c_addr);
  WIRE.write(addr);
  WIRE.write(d);
  WIRE.endTransmission();
}

uint8_t PCA9685::read8(uint8_t addr) {
  WIRE.beginTransmission(i2c_addr);
  WIRE.write(addr);
  WIRE.endTransmission();

  WIRE.requestFrom((uint8_t)i2c_addr, (uint8_t)1);
  return WIRE.read();
}