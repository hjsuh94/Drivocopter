#ifndef __R12DS_H_INCLUDED__
#define __R12DS_H_INCLUDED__

#include "Arduino.h"

namespace Drivocopter {

    class R12DS {
        public:

        // Attributes 
        struct CMD { // Memory storage for receiver values 
            float throttle_right = 0.0F;
            float throttle_left = 0.0F;
            float steer_right = 0.0F;
            float steer_left = 0.0F;
            uint8_t mode = 0; 
        };

        CMD cmd_;

        // Methods 
        R12DS(void);
        void initialize(void);
        bool is_connected(void);
        void update(void);
        void print_cmd(void);
        float parse_signal(uint32_t value);
        uint8_t parse_mode(uint32_t value, bool type);

        private:

        // Attributes

        struct PIN_MAP {
            uint8_t THROTTLE_RIGHT = 5;
            uint8_t STEER_RIGHT    = 3;
            uint8_t THROTTLE_LEFT  = 6;
            uint8_t STEER_LEFT     = 9; 
            uint8_t MODE           = 0;
        };

        PIN_MAP pin_map_;

    };
}

#endif // __R12DS_H_INCLUDED