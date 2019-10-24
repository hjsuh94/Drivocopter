#if defined(__OPENCM904__)
  #define DEVICE_NAME "1" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
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

            // Configure Top Dynamixels in Servo Mode
            for (uint8_t i = 1; 
            i < (config_.AXIS_MOTOR_COUNT + 1);
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
            result = dx_wb_.addSyncWriteHandler(dx_id_.AX12A_R, "Goal_Position", &log_);
            Serial.println(log_);
            if (result == false) pass = false;

            // Initialize Vectors


            dx_id_pos_[0] = dx_id_.AX12A_R;
            dx_id_pos_[1] = dx_id_.AX12A_L;

            dx_goal_position_[0] = dx_default_.AX12A_R;
            dx_goal_position_[1] = dx_default_.AX12A_L;
            
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
        
        roboclaw_.begin(config_.ROBOCLAW_BAUDRATE);
    }

    void Drivo::print_log(String msg) {
            Serial.println("[DRIVO LOG: " + msg + "]");
    }

    float Drivo::get_time(void) {
        return micros();
    }

    void Drivo::update_cmd(void) {

        r12ds_.update(); // update the receiver first
        r12ds_.print_cmd();

        float forward_vel = r12ds_.cmd_.throttle_right;
        float steer_vel = r12ds_.cmd_.steer_right;
        int8_t mode = r12ds_.cmd_.mode;
        int32_t dx_cmd_vel = (int32_t) 0;
        int32_t dx_cmd_str = (int32_t) 512;
        int32_t dx_cmd_str_pos = (int32_t) 512;
        int32_t dx_cmd_str_neg = (int32_t) 512;

        bool result = false;
        dx_cmd_str = (int32_t) (512.0F + (steer_vel * 256.0F));

        dx_goal_position_[0] = dx_cmd_str;
        dx_goal_position_[1] = dx_cmd_str;
        
        execute_sync_write();
        
        // Roboclaw 

        if (forward_vel > 0) {
            roboclaw_.ForwardM1(config_.ROBOCLAW_ADDRESS,
            (uint8_t) (config_.ROBOCLAW_MAXSPEED * forward_vel));
            roboclaw_.ForwardM2(config_.ROBOCLAW_ADDRESS,
            (uint8_t) (config_.ROBOCLAW_MAXSPEED * forward_vel));
        }
        else {
            roboclaw_.BackwardM1(config_.ROBOCLAW_ADDRESS,
            (uint8_t) (config_.ROBOCLAW_MAXSPEED * -forward_vel));
            roboclaw_.BackwardM2(config_.ROBOCLAW_ADDRESS,
            (uint8_t) (config_.ROBOCLAW_MAXSPEED * -forward_vel));

        }
        

        // Data Logging 

        timenow_ = micros(); 
    }

    void Drivo::execute_sync_write(void) {
        dx_wb_.syncWrite(0, dx_id_pos_, 2, dx_goal_position_, 1, &log_);
    }

}