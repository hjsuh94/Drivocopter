#ifndef __DRIVO_H_INCLUDED__
#define __DRIVO_H_INCLUDED__

#include "Arduino.h"
#include "../R12DS/r12ds.h"
#include <DynamixelWorkbench.h>
#include <string>
#include <iostream>
#include <SPI.h>
#include <SD.h>

namespace Drivocopter {

    class Drivo {

        public:         

        struct CONFIG {
            uint32_t BAUDRATE = 1000000;
            uint8_t MOTOR_COUNT = 8;
            uint8_t DRIVING_MOTOR_COUNT = 4;
            uint8_t AXIS_MOTOR_COUNT = 4; 
        };

        struct MODE {
            uint8_t SKID_STEER = 0;
            uint8_t ACKERMANN = 1;
            uint8_t VECTOR = 2;
        };

        // Attributes
        DynamixelWorkbench dx_wb_; 
        R12DS r12ds_;
        MODE mode_;
        CONFIG config_;
        const char *log_;
        uint16_t model_number_ = 0;

        uint32_t timenow_ = micros();
        uint32_t timebefore_ = micros();

        uint8_t dx_id_vel_[4];
        uint8_t dx_id_pos_[4];
        int32_t dx_goal_position_[4];
        int32_t dx_goal_velocity_[4];

        float current_now_;
        String record_string_;

        // SD Card Attributes
        const int SD_id_ = 4;
        File data_log_;

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
            uint8_t MX12W_RT = 1;
            uint8_t MX12W_RB = 2;
            uint8_t MX12W_LB = 3;
            uint8_t MX12W_LT = 4;
            uint8_t AX12A_RT = 5;
            uint8_t AX12A_RB = 6;
            uint8_t AX12A_LB = 7;
            uint8_t AX12A_LT = 8;
        };

        struct Dynamixel_default {
            int32_t MX12W_RT = 0;
            int32_t MX12W_RB = 0;
            int32_t MX12W_LB = 0;
            int32_t MX12W_LT = 0;
            int32_t AX12A_RT = 512;
            int32_t AX12A_RB = 512;
            int32_t AX12A_LB = 512;
            int32_t AX12A_LT = 512;
        };

        Dynamixel_id dx_id_;
        Dynamixel_default dx_default_;




    };
}

#endif