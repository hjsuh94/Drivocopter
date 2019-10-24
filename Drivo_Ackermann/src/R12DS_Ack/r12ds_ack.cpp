#include "r12ds_ack.h"

namespace Drivocopter {

    R12DS_Ack::R12DS_Ack(void) {

        // Set all the pins to input mode (Iterate later for efficiency)   
    }

    void R12DS_Ack::initialize(void) {
        pinMode(pin_map_.THROTTLE_RIGHT, INPUT);
        pinMode(pin_map_.STEER_RIGHT, INPUT);
        pinMode(pin_map_.THROTTLE_LEFT, INPUT);
        pinMode(pin_map_.STEER_LEFT, INPUT);
        pinMode(pin_map_.MODE, INPUT);
    }

    float R12DS_Ack::parse_signal(uint32_t value) {
        return ((float) value - 1500.0F) / 520.0F;
    }

    uint8_t R12DS_Ack::parse_mode(uint32_t value, bool type) {
        if (type) {
            if (value > 1500) {
                return 0;
            }
            else return 1;
        }
        else {
            if (value > 1700) {
                return 0;
            }
            else if (value < 1300) {
                return 1;
            }
            else return 2;
        }
    }

    void R12DS_Ack::update(void) {
        cmd_.throttle_right = -parse_signal(pulseIn(
            pin_map_.THROTTLE_RIGHT, HIGH));
        cmd_.throttle_left = parse_signal(pulseIn(
            pin_map_.THROTTLE_LEFT, HIGH));
        cmd_.steer_right = parse_signal(pulseIn(
            pin_map_.STEER_RIGHT, HIGH));
        cmd_.steer_left = parse_signal(pulseIn(
            pin_map_.STEER_LEFT, HIGH));
        cmd_.mode = parse_mode(pulseIn(
            pin_map_.MODE, HIGH), false);
    }

    void R12DS_Ack::print_cmd(void) {
        Serial.print("Throttle Right: ");
        Serial.print(cmd_.throttle_right, 3);
        Serial.print(" | Steer Right: ");
        Serial.print(cmd_.steer_right, 3);
        Serial.print(" | Throttle Left: ");
        Serial.print(cmd_.throttle_left, 3);
        Serial.print(" | Steer Left: ");
        Serial.print(cmd_.steer_left, 3);
        Serial.print(" | Mode: ");
        Serial.println(cmd_.mode);
    }

    bool R12DS_Ack::is_connected(void) {
        // To be done later 
    }
}