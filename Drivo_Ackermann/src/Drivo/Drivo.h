#ifndef __DRIVO_H_INCLUDED__
#define __DRIVO_H_INCLUDED__

#include "Arduino.h"
#include "../R12DS_Ack/r12ds_ack.h"
#include <DynamixelWorkbench.h>
#include "../RoboClaw/RoboClaw.h"

namespace Drivocopter {

    class Drivo {

        public:         

        struct CONFIG {
            uint32_t BAUDRATE = 1000000;
            uint8_t MOTOR_COUNT = 2;
            uint8_t DRIVING_MOTOR_COUNT = 0;
            uint8_t AXIS_MOTOR_COUNT = 2; 
            uint32_t ROBOCLAW_BAUDRATE = 38400;
            uint8_t ROBOCLAW_ADDRESS = 0x80;
            float ROBOCLAW_MAXSPEED = 128.0f;
        };

        struct MODE {
            uint8_t SKID_STEER = 0;
            uint8_t ACKERMANN = 1;
            uint8_t VECTOR = 2;
        };

        // Attributes
        DynamixelWorkbench dx_wb_; 
        R12DS_Ack r12ds_;
        MODE mode_;
        CONFIG config_;
        RoboClaw roboclaw_{&Serial2, 10000};

        
        const char *log_;
        uint16_t model_number_ = 0;

        uint32_t timenow_ = micros();
        uint32_t timebefore_ = micros();

        uint8_t dx_id_pos_[2];
        int32_t dx_goal_position_[2];

        // Methods
        Drivo(void);
        void initialize(void);
        void print_log(String msg);
        void update_cmd(void);
        void execute_sync_write(void);
        float get_current(void);
        float get_time(void);
        int32_t speed_encode(int32_t speed);



        private:

        struct Dynamixel_id {
            uint8_t AX12A_R = 1;
            uint8_t AX12A_L = 2;
        };

        struct Dynamixel_default {
            int32_t AX12A_R= 512;
            int32_t AX12A_L = 512;
        };

        Dynamixel_id dx_id_;
        Dynamixel_default dx_default_;




    };
}

#endif