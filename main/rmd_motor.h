#ifndef RMD_MOTOR_H
#define RMD_MOTOR_H

#include <mcp_can.h>
#include "utils.h"
#define CAN0_INT                            2
#define CAN0_PIN                            7
#define MOTOR_CAN_ADDR                      0x141
#define CAN0_EXT                            0
#define CAN0_LEN                            8

#define IDX_CMD                             0

#define CMD_READ_PID                        0x30
#define CMD_WRITE_RAM_PID                   0x31
#define CMD_WRITE_ROM_PID                   0x32
#define CMD_READ_ACC                        0x33
#define CMD_WRITE_RAM_ACC                   0x34
#define CMD_READ_ENCODER                    0x90
#define CMD_WRITE_ENCODER_OFFSET            0x91
#define CMD_WRITE_ROM_CURRENT_POS_AS_ZERO   0x19
#define CMD_READ_MULTI_TURNS_ANGLE          0x92
#define CMD_READ_SINGLE_CIRCLE_ANGLE        0x94
#define CMD_READ_MOTOR_STATUS1_ERRORFLAG    0x9A
#define CMD_CLEAR_MOTOR_ERRORFLAG           0x9B
#define CMD_READ_MOTOR_STATUS2              0x9C
#define CMD_READ_MOTOR_STATUS3              0x9D
#define CMD_MOTOR_OFF                       0x80
#define CMD_MOTOR_STOP                      0x81
#define CMD_MOTOR_RUNNING                   0x88
#define CMD_TORQUE_CL                       0xA1
#define CMD_SPEED_CL                        0xA2
#define CMD_POSITION_CL1                    0xA3
#define CMD_POSITION_CL2                    0xA4
#define CMD_POSITION_CL3                    0xA5
#define CMD_POSITION_CL4                    0xA6

#define CMD_MOTOR_DIR_CCW                   0x01
#define CMD_MOTOR_DIR_CW                    0x00

#define CW_ENCODER_SIGN                     (1 )
#define CCW_ENCODER_SIGN                    (-1)
#define CW_ANGLE_SIGN                       (1 )
#define CCW_ANGLE_SIGN                      (-1)

#define ENCODER_MAX                         16383

typedef struct motorState2{
    uint8_t cmd;
    int8_t temperature;
    int16_t iq;
    int16_t speed;
    uint16_t encoder_pos;
} motor_state2;

class RMDmotor{
private:
    bool state_update_flag;
    MCP_CAN mc;
    uint32_t rxId;
    byte frame_read_status2[8]      = {CMD_READ_MOTOR_STATUS2,           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_read_status1[8]      = {CMD_READ_MOTOR_STATUS1_ERRORFLAG, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_running[8]           = {CMD_MOTOR_RUNNING,                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; 
    byte frame_stop[8]              = {CMD_MOTOR_STOP,                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; 
    byte frame_off[8]               = {CMD_MOTOR_OFF,                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  
    byte frame_read_mult_angle[8]   = {CMD_READ_MULTI_TURNS_ANGLE,       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  
    byte frame_read_single_angle[8] = {CMD_READ_SINGLE_CIRCLE_ANGLE,     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  
    byte frame_clear_error_flag[8]  = {CMD_CLEAR_MOTOR_ERRORFLAG,        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    byte frame_pos_control1[8]      = {CMD_POSITION_CL1,                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_pos_control2[8]      = {CMD_POSITION_CL2,                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_pos_control3[8]      = {CMD_POSITION_CL3,                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_pos_control4[8]      = {CMD_POSITION_CL4,                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    byte frame_speed_control[8]     = {CMD_SPEED_CL,                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_torque_control[8]    = {CMD_TORQUE_CL,                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    byte frame_res_mult_angle[8];
    byte frame_res_single_angle[8];
    
    uint8_t rxLen;
    byte current_state1[8];
    motor_state2 current_state2;
    uint16_t single_angle;

    int64_t mult_angle;
    int64_t ccw_lock_mult_angle;
    int64_t cw_lock_mult_angle;
    int64_t zero_mult_angle;
    uint8_t sendCMD(byte req[], byte res[]);

public:    
    RMDmotor();
    // RMDmotor(uint8_t pin_num): mc(pin_num), mult_angle(0) {}
    void begin();

/* motor initialize */
    bool runningMotor();
    bool offMotor();
    bool stopMotor();

/* set lock to lock angle()*/
    inline void setCCWLock(uint64_t angle)  {ccw_lock_mult_angle = angle;}
    inline void setCWLock(uint64_t angle)   {cw_lock_mult_angle  = angle;}
    inline void setZeroMultAngle()          {zero_mult_angle     = (ccw_lock_mult_angle + cw_lock_mult_angle) / 2;}

/* read motor states */
    bool updateCurrentState2();
    bool updateMultAngle();
    bool updateSingleAngle();

/* control motor */
    bool posControl1    (const uint32_t& pos);
    bool posControl2    (const uint16_t& speed_limit, const uint32_t& pos);
    bool posControl3    (const uint8_t& direction, const uint16_t& pos);
    bool posControl4    (const uint8_t& direction, const uint16_t & speed_limit, const uint16_t& pos);
    bool speedControl1  (const int32_t& speed);
    bool speedControl2  (const uint8_t& direction, const int32_t& speed);
    bool torqueControl1 (const int16_t& torque);
    bool torqueControl2 (const uint8_t& direction , const int16_t& torque);

    bool multAngleControl (const int64_t& target_mult_angle, const int32_t& speed, const int64_t& offset);

/* return motor states(inline) */
    inline const int8_t &getTemeperatureRAW()   {return current_state2.temperature;}
    inline const int16_t &getIqRAW()            {return current_state2.iq;}
    inline const int16_t &getSpeedRAW()         {return current_state2.speed;}
    inline const uint16_t &getEncoderRAW()      {return current_state2.encoder_pos;}
    inline int64_t getMultAngleRAW()            {return mult_angle;}
    inline uint16_t getSingleAngleRAW()         {return single_angle;}
    inline const motor_state2 &getCurrentState2() 
    {
        state_update_flag = false;
        return current_state2;
    }
    inline const int64_t &getCWLockAngle()      {return cw_lock_mult_angle;}
    inline const int64_t &getCCWLockAngle()     {return ccw_lock_mult_angle;}
    inline const int64_t &getZeroMultAngle()    {return zero_mult_angle;}

    inline const bool& getUpdateFlag()          {return state_update_flag;}
/* diagnostic behavior */
    bool updateCurrentState1();
    bool clearErrorFlag();

/* verbose */
    void showCurrentState1(bool update);
    void showCurrentState2(bool update);
    friend void print_frame(char frame[], char len);
/* util */
    // static uint16_t encoderDiff(const uint16_t & prev, const uint16_t & curr);
};
#endif
