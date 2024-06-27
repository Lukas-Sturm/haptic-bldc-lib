# Haptic BLDC Library

###
Thanks to __VIPQualityPost__, I just changed some stuff because it didn't want to build on my machine.  
Also changed it so that 0 would always be the minimum pos and start pos would be the current position.  

### Standalone Arduino library for smartknob-like devices and general digitally controlled knob user interfaces.

A simple, standalone library for implementing basic haptic-controlled knobs as user-interfaces.
Based on "Drive by wire" interfaces and @scottbez1 SmartKnob project.

## Features 
* Easy to setup (minimal use just requires information about motor, driver IC, and encoder)
* Fully configurable (all PID controllers, haptic parameters and configuration are user-tunable during operation)
* High efficiency due to use of FOC for motor control (convenient to use with devices plugged into USB or battery)
* Wide variety of hardware support due to underlying SimpleFOC abstraction.

## Example code
```cpp
#include <Arduino.h>
#include <SimpleFOC.h>
#include "encoders/smoothing/SmoothingSensor.h"
#include <haptic.h>

BLDCMotor motor = BLDCMotor(blcd_pp, bldc_pr, bldc_KV, bldc_Li);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_U, PIN_V, PIN_W, PIN_EN_U, PIN_EN_V, PIN_EN_W);
MagneticSensorMT6701SSI encoder(PIN_MT_CSN);
SmoothingSensor smooth = SmoothingSensor(encoder, motor);
HapticInterface haptic = HapticInterface(&motor);

void setup(){
    Serial.begin(115200);
    SPIClass* spi = new SPIClass(FSPI);
    spi->begin(PIN_MT_CLOCK, PIN_MT_DATA, -1, PIN_MT_CSN);
    encoder.init(spi);

    driver.voltage_power_supply = driver_supply;
    driver.voltage_limit = driver_voltage_limit;
    driver.init();
    
    motor.linkSensor(&smooth);
    motor.linkDriver(&driver);
    motor.current_limit = 1.22;
    motor.init();
    motor.initFOC();
    
    haptic.init();

    delay(1500);
}

void loop(){
    haptic.haptic_loop();
}
```