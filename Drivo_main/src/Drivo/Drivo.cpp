#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif   

#include "Drivo.h"

namespace Drivocopter {

    Drivo::Drivo(void) {
    }

    void Drivo::initialize(void) {
        // First initialize the receiver 
        r12ds_.initialize();

        timenow_ = micros();
        timebefore_ = micros();

        bool result = false;
        bool pass = true;

        // Now keep trying to initialize the Dynamixels
        while(true) {

            bool pass = true;
            // Now initialize the dynamixel
            result = dx_wb_.init(DEVICE_NAME, config_.BAUDRATE, &log_);
            if (result == false) {
                // Serial.println(log_);
                print_log("ERROR - Failed to init");
                pass = false;
            }
            else {
                print_log("Succeeded to initalize at: " + 
                String(config_.BAUDRATE));
            }

            // Check if each Dynamixel can be pinged.
            for (uint8_t i = 1; i < (config_.MOTOR_COUNT + 1); i++) {
                result = dx_wb_.ping(i, &model_number_, &log_);
                if (result == false) {
                    // Serial.println(log_);
                    print_log("ERROR - Failed to pint at ID: " + String(i));
                    pass = false;
                }
                else {
                    print_log("Confirmed Ping at ID :" +
                        String(i) + " | Model Number: " + 
                        String(model_number_));
                }
            }

            // Configure Bottom Dynamixels in Wheel Mode
            for (uint8_t i = 1; i < (config_.DRIVING_MOTOR_COUNT + 1); i++) {
                result = dx_wb_.wheelMode(i, 0, &log_);
                if (result == false) {
                    // Serial.println(log_);
                    print_log("ERROR - Failed to change wheel_mode at ID: " +
                    String(i));
                    pass = false;
                }
                else {
                    print_log("Succeeded to change wheel mode at ID: " +
                    String(i));
                }
            }

            // Configure Top Dynamixels in Servo Mode
            for (uint8_t i = (config_.DRIVING_MOTOR_COUNT + 1); 
            i < (config_.DRIVING_MOTOR_COUNT + config_.AXIS_MOTOR_COUNT + 1);
            i++) {
                result = dx_wb_.jointMode(i, 0, 0, &log_);
                if (result == false) {
                    // Serial.println(log_);
                    print_log("ERROR - Failed to change joint_mode at ID: " +
                    String(i));
                    pass = false;
                }
                else {
                    print_log("Succeeded to change joint mode at ID: " + 
                    String(i));
                }
            } 
            
            // Initialize Sync Write
            result = dx_wb_.addSyncWriteHandler(dx_id_.AX12A_RT, "Goal_Position", &log_);
            Serial.println(log_);
            if (result == false) pass = false;

            result = dx_wb_.addSyncWriteHandler(dx_id_.MX12W_RT, "Moving_Speed", &log_);
            Serial.println(log_);
            if (result == false) pass = false;

            // Initialize Vectors
            dx_id_vel_[0] = dx_id_.MX12W_RT; 
            dx_id_vel_[1] = dx_id_.MX12W_RB;
            dx_id_vel_[2] = dx_id_.MX12W_LB;
            dx_id_vel_[3] = dx_id_.MX12W_LT;

            dx_id_pos_[0] = dx_id_.AX12A_RT;
            dx_id_pos_[1] = dx_id_.AX12A_RB;
            dx_id_pos_[2] = dx_id_.AX12A_LB;
            dx_id_pos_[3] = dx_id_.AX12A_LT;

            dx_goal_velocity_[0] = dx_default_.MX12W_RT;
            dx_goal_velocity_[1] = dx_default_.MX12W_RB;
            dx_goal_velocity_[2] = dx_default_.MX12W_LB;
            dx_goal_velocity_[3] = dx_default_.MX12W_LT;

            dx_goal_position_[0] = dx_default_.AX12A_RT;
            dx_goal_position_[1] = dx_default_.AX12A_RB;
            dx_goal_position_[2] = dx_default_.AX12A_LB;
            dx_goal_position_[3] = dx_default_.AX12A_LT;



            // Initialize SD card 
            if (!SD.begin(SD_id_)) {
                print_log("SD CARD Initialization Failed");
            }
            else {
                print_log("SD CARD Initialization Successful!");

            }
            
            // If initialization fails, try again after a second.
            if(pass == false) {
                print_log("ERROR - DRIVO Initialization Failed. Retrying..");
                delay(1000);
            }
            else {
                print_log("DRIVO Initialization Successful!");
                // Set Dynamixels to default positions 
                execute_sync_write();
                break;
            }
        }
    }

    void Drivo::print_log(String msg) {
            Serial.println("[DRIVO LOG: " + msg + "]");
    }

    float Drivo::get_current(void) {
        int sensorValue = analogRead(17);
        float voltage = 3300.0F * (float) sensorValue / 1024.0;
        float current = (2500.0F - voltage) / (66.0F);
        // Serial.println(current);
        return current;
    }

    float Drivo::get_time(void) {
        return micros();
    }

    void Drivo::update_cmd(void) {

        r12ds_.update(); // update the receiver first

        // Serial.println(timenow_ - timebefore_);

        float forward_vel = r12ds_.cmd_.throttle_right;
        float steer_vel = r12ds_.cmd_.steer_right;
        int8_t mode = r12ds_.cmd_.mode;
        int32_t dx_cmd_vel = (int32_t) 0;
        int32_t dx_cmd_str = (int32_t) 512;
        int32_t dx_cmd_str_pos = (int32_t) 512;
        int32_t dx_cmd_str_neg = (int32_t) 512;

        bool result = false;
        switch (mode) {
            case 0: // Ackermann 
                dx_cmd_vel = (int32_t) (forward_vel * 512.0F);
                dx_cmd_str = (int32_t) (512.0F + (steer_vel * 256.0F));

                dx_goal_velocity_[0] = speed_encode(-dx_cmd_vel);
                dx_goal_velocity_[1] = speed_encode(-dx_cmd_vel);
                dx_goal_velocity_[2] = speed_encode(dx_cmd_vel);
                dx_goal_velocity_[3] = speed_encode(dx_cmd_vel);

                dx_goal_position_[0] = dx_cmd_str;
                dx_goal_position_[1] = (int32_t) 512;
                dx_goal_position_[2] = (int32_t) 512;
                dx_goal_position_[3] = dx_cmd_str;
              
            break;
            case 1: // Skid-Steering 
                dx_cmd_vel = (int32_t) (forward_vel * 512.0F);
                dx_cmd_str = (int32_t) (steer_vel * 512.0F);

                dx_goal_velocity_[0] = speed_encode((-dx_cmd_vel + dx_cmd_str) / 2);
                dx_goal_velocity_[1] = speed_encode((-dx_cmd_vel + dx_cmd_str) / 2);
                dx_goal_velocity_[2] = speed_encode((dx_cmd_vel + dx_cmd_str) / 2);
                dx_goal_velocity_[3] = speed_encode((dx_cmd_vel + dx_cmd_str) / 2);

                dx_goal_position_[0] = (int32_t) 512;
                dx_goal_position_[0] = (int32_t) 512;
                dx_goal_position_[0] = (int32_t) 512;
                dx_goal_position_[0] = (int32_t) 512;

            break;
            case 2: // Double Ackermann
                dx_cmd_vel = (int32_t) (forward_vel * 512.0F);
                dx_cmd_str_pos  = (int32_t) (512.0F + (steer_vel * 256.0F));
                dx_cmd_str_neg  = (int32_t) (512.0F - (steer_vel * 256.0F));

                dx_goal_velocity_[0] = speed_encode(-dx_cmd_vel);
                dx_goal_velocity_[1] = speed_encode(-dx_cmd_vel);
                dx_goal_velocity_[2] = speed_encode(dx_cmd_vel);
                dx_goal_velocity_[3] = speed_encode(dx_cmd_vel);

                dx_goal_position_[0] = dx_cmd_str_pos;
                dx_goal_position_[1] = dx_cmd_str_neg;
                dx_goal_position_[2] = dx_cmd_str_neg;
                dx_goal_position_[3] = dx_cmd_str_pos; 
            break;
        }

        execute_sync_write();

        // Data Logging 

        timenow_ = micros(); 

        if (timenow_ - timebefore_ > 1e5) {

            File data_log_ = SD.open("datalog.csv", FILE_WRITE);
            String record_string_ = "";

            record_string_ = record_string_ + String(micros());
            record_string_ = record_string_ + ",";
            record_string_ = record_string_ + String((uint32_t) (1e3F * get_current()));
            record_string_ = record_string_ + "\n";

            if (data_log_) {
                data_log_.print(record_string_);
                data_log_.close();
                Serial.println(String((uint32_t) (1e3F * get_current())));

                timebefore_ = timenow_;
            }
            else {
                print_log("Error opening SD card for memory storage");
            }
        }
    }

    void Drivo::execute_sync_write(void) {
        dx_wb_.syncWrite(0, dx_id_pos_, 4, dx_goal_position_, 1, &log_);
        dx_wb_.syncWrite(1, dx_id_vel_, 4, dx_goal_velocity_, 1, &log_);
    }

    int32_t Drivo::speed_encode(int32_t speed) {
        if (speed < 0) {
            int32_t encoded_speed;
            encoded_speed = (-1) * speed;
            return encoded_speed |= 1024;
        }
        else {
            return speed;
        }
    }
}