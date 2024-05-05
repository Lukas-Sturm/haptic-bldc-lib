#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/mt6701/MagneticSensorMT6701SSI.h"
#include <motor.h>
#include "haptic_api.h"
 
class HapticState 
{
public:
    HapticState(void);
    HapticState(DetentProfile profile);
    ~HapticState();

    DetentProfile detent_profile;
    DetentProfile DefaultProgressiveForceProfile;
    DetentProfile DefaultVernierProfile;
    DetentProfile TempProfile;

    uint16_t current_pos = 0;
    uint16_t last_pos = 0; 

    float attract_angle = 0; 
    float last_attract_angle = 0;
    float attract_hysteresis = 0.05;

    float detent_strength_unit = 4; // PID (estimated) Current Limit
    float endstop_strength_unit = 1; // PID (estimated) Current Limit

    bool atLimit = false;
    bool wasAtLimit = false;

    void load_profile(DetentProfile);
};

class HapticInterface
{
public:
    HapticState haptic_state;   // Haptic state

    BLDCMotor* motor;
    PIDController* haptic_pid;

    // All the various constructors.
    HapticInterface(BLDCMotor* _motor);
    HapticInterface(BLDCMotor* _motor, PIDController* _pid);

    void init(void);
    void haptic_loop(void);
    void HapticEventCallback(HapticEvt);
    void UserHapticEventCallback(HapticEvt, float, uint16_t) __attribute__((weak));

private:
    void offset_detent(void);
    void find_detent(void);
    void detent_handler(void);
    void update_position(void);
    float haptic_target(void);
    void correct_pid(void);
};
