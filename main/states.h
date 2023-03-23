#ifndef STATES_H
#define STATES_H
#include "utils.h"
#include "rmd_motor.h"
// #include "fsm.h"

#define GEAR_RATIO                          3.65277777

class motorStateMachine; /*forward declaration to set friend*/
class State {
    friend class motorStateMachine;
public:
    State(){ }
    State(RMDmotor& rm_, STATES_IDX idx){
        rm = &rm_;
        start_moment = 0;
        terminate = false;
        stateidx = idx;
    }
    inline STATES_IDX getStateIdx() {return stateidx;}
    virtual void enter() {}
    virtual void update(byte* cmd) {}
    virtual void exit() {}
    virtual void run() {}
    virtual STATES_IDX getNextState(byte* cmd) = 0;
protected:
    ms_t start_moment;
    bool terminate;
    RMDmotor *rm;
    STATES_IDX stateidx;

};

/*****************************/ 
/*****declare init state******/
/*****************************/ 
class initState:public State{
public:
    initState(RMDmotor& rm, STATES_IDX idx): State(rm, idx){}
    virtual void update(byte* cmd); 
    virtual STATES_IDX getNextState(byte* cmd);
};

/*****************************/ 
/*****declare ready state*****/
/*****************************/ 
class readyState:public State{
    bool ready_flag;
public:
    readyState(RMDmotor& rm, STATES_IDX idx): State(rm, idx){}
    virtual void update(byte* cmd);
    virtual STATES_IDX getNextState(byte* cmd);
    virtual void run();
};

/*****************************/ 
/******declare l2l state******/
/*****************************/ 
class l2lState:public State{
    uint16_t cw_target_torque;
    uint16_t ccw_target_torque;


    uint16_t cw_stuck_count;
    uint16_t ccw_stuck_count;
    int64_t prev_mult_angle;

    uint8_t direction;
    // uint16_t step_torque;
    bool ccw_done;
    bool cw_done;
    void ccwCalibration();
    void cwCalibration();

    int64_t threshold_stuck_angle;
    uint16_t threshold_stuck_count; /*5 sec*/


public:
    l2lState(RMDmotor& rm, STATES_IDX idx): 
        State(rm, idx),
        cw_target_torque(0), 
        ccw_target_torque(0), 
        // step_torque(10),
        cw_done(false),
        ccw_done(false),
        direction(CMD_MOTOR_DIR_CCW)
    {}

    virtual void update(byte* cmd); 
    virtual void run();
    STATES_IDX getNextState(byte* cmd);
};

/*****************************/ 
/*****declare const state*****/
/*****************************/ 
class constState:public State{
    uint8_t direction;
    bool pause;
    ms_t pause_time;
    ms_t pause_moment;
    ms_t pause_end_time;
    uint8_t cycle;
    uint8_t cmd_rps;
    uint8_t cmd_cycle;
    uint16_t cmd_amp;

    uint8_t target_cycle;
    int32_t target_speed;
    int64_t target_amp;
public:
    constState(RMDmotor& rm, STATES_IDX idx):
        State(rm, idx),
        target_speed(0),
        pause_time(0),
        cmd_rps(0),
        cmd_cycle(0),
        cmd_amp(0),
        direction(CMD_MOTOR_DIR_CW)
    {}
    virtual void update(byte* cmd);
    virtual void run();
    STATES_IDX getNextState(byte* cmd);
};

/*****************************/ 
/*****declare sine state******/
/*****************************/ 
class sineState:public State{
    int32_t target_speed;
    uint8_t direction;
    bool pause;
    ms_t pause_time;
    ms_t pause_moment;
    ms_t pause_end_time;

    uint8_t cycle;
    uint8_t cmd_fq;
    uint8_t cmd_cycle;
    uint8_t cmd_offset;
    uint8_t cmd_amp;

    float target_fq;
    uint8_t target_cycle;
    int64_t target_offset;
    int64_t target_amp;
    int64_t init_mult_angle;

    bool offset_flag;
    
public:
    sineState(RMDmotor& rm, STATES_IDX idx):
        State(rm, idx),
        target_speed(0),
        pause_time(0),
        direction(CMD_MOTOR_DIR_CW),
        offset_flag(false)
    {}
    virtual void update(byte* cmd);
    virtual void run();
    STATES_IDX getNextState(byte* cmd);
};
#endif
