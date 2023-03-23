#include "states.h"
#include "Arduino.h"

/*****************************/ 
/******define init state******/
/*****************************/ 
void initState::update(byte* cmd) {
    // Serial.println("init mode");
    rm->stopMotor();
}
STATES_IDX initState::getNextState(byte* cmd){ // //Serial.println("stopState::getNextState");
    // if(cmd.equals("L2L\n")) {
    if(cmd[0] == STATES_IDX::IDX_L2L){
        return STATES_IDX::IDX_L2L;
    }
    return STATES_IDX::IDX_INIT;
}

/*****************************/ 
/******define l2l state*******/
/*****************************/ 
void l2lState::update(byte* cmd){
    start_moment = millis();
    // Serial.println("L2L mode");
    rm->updateCurrentState2();
    direction = CMD_MOTOR_DIR_CCW;
    ccw_target_torque = 200;
    cw_target_torque = 200;
    cw_done = false;
    ccw_done = false;
    prev_mult_angle = 0;
    
    terminate = false;

    cw_stuck_count = 0;
    ccw_stuck_count = 0;
    threshold_stuck_angle = 50;
    threshold_stuck_count = 500;
}
STATES_IDX l2lState::getNextState(byte* cmd){
    if(terminate) return STATES_IDX::IDX_READY;
    else if(cmd[0] == STATES_IDX::IDX_INIT) return STATES_IDX::IDX_INIT;
    return STATES_IDX::IDX_L2L;
}
void l2lState::run(){
    rm->updateMultAngle();
    if(!ccw_done){
        ccwCalibration();
    }
    else if(!cw_done){
        cwCalibration();
    }
    else {
        rm->setZeroMultAngle();
        terminate = true;
    }
    prev_mult_angle = rm->getMultAngleRAW();
}
void l2lState::ccwCalibration()
{
    if(abs(rm->getMultAngleRAW() - prev_mult_angle) < threshold_stuck_angle) ccw_stuck_count++;
    else ccw_stuck_count = 0;
    if(ccw_stuck_count > threshold_stuck_count) {
        ccw_done = true;
        rm->setCCWLock(rm->getMultAngleRAW());
    }
    rm->torqueControl2(CMD_MOTOR_DIR_CCW, ccw_target_torque);
}
void l2lState::cwCalibration()
{
    if(abs(rm->getMultAngleRAW() - prev_mult_angle) < threshold_stuck_angle) cw_stuck_count++;
    else cw_stuck_count = 0;
    if(cw_stuck_count > threshold_stuck_count) {
        cw_done = true;
        rm->setCWLock(rm->getMultAngleRAW());
    }
    rm->torqueControl2(CMD_MOTOR_DIR_CW, cw_target_torque);
}

/*****************************/ 
/*****define reday state*****/
/*****************************/ 
void readyState::update(byte* cmd) {
    ready_flag = false;
    stateidx = STATES_IDX::IDX_R2Z;
}

void readyState::run(){
    rm->updateMultAngle();
    if((rm->getZeroMultAngle() - rm->getMultAngleRAW() > -100)&&
    (rm->getZeroMultAngle() - rm->getMultAngleRAW() < 100)) {
        rm->stopMotor();
        ready_flag = true;
        stateidx = STATES_IDX::IDX_READY;
    }
    else if(rm->getZeroMultAngle() >  rm->getMultAngleRAW()) rm->speedControl1(36000);
    else if(rm->getZeroMultAngle() <= rm->getMultAngleRAW()) rm->speedControl1(-36000);
}
STATES_IDX readyState::getNextState(byte* cmd){
    if(cmd[0] == STATES_IDX::IDX_INIT) {return STATES_IDX::IDX_INIT;}
    if(ready_flag){
        if(cmd[0] == STATES_IDX::IDX_CONST) {return STATES_IDX::IDX_CONST;}
        else if(cmd[0] == STATES_IDX::IDX_SINE) {return STATES_IDX::IDX_SINE;}
    }
    return STATES_IDX::IDX_READY;
}
/*****************************/ 
/*****define const state******/
/*****************************/ 
void constState::update(byte* cmd){
    // Serial.println("const mode");

    direction = CMD_MOTOR_DIR_CW;
    terminate = false;
    pause = false;
    start_moment = millis();
    pause_time = 0;
    cycle = 0;

    cmd_rps = cmd[1];
    cmd_cycle = cmd[2];
    cmd_amp = *((uint16_t*)(cmd + 3));

    target_speed = (cmd_rps*360 * 13/10 + 18000)*GEAR_RATIO;
    target_cycle = cmd_cycle;
    target_amp = ((rm->getCWLockAngle() - rm->getCCWLockAngle()) / 2) * ((double)cmd_amp / (double)5000);
}
void constState::run(){
    if(pause){
        rm->stopMotor();
    }
    else{
        rm->updateMultAngle();
        if(cycle != target_cycle){
            rm->speedControl2(direction, target_speed);
            if(direction == CMD_MOTOR_DIR_CW){
                if(rm->getMultAngleRAW() > rm->getZeroMultAngle() + target_amp){
                    direction = CMD_MOTOR_DIR_CCW;
                }
            }
            else if(direction == CMD_MOTOR_DIR_CCW){
                if(rm->getMultAngleRAW() < rm->getZeroMultAngle() - target_amp){
                    direction = CMD_MOTOR_DIR_CW;
                    cycle++;
                }
            }
        }
        else{
            if(rm->multAngleControl(rm->getZeroMultAngle(), target_speed, 100)) terminate = true;
        }
    }
}
STATES_IDX constState::getNextState(byte* cmd){
    if(cmd[0] == STATES_IDX::IDX_INIT) {
        return STATES_IDX::IDX_INIT;
    }

    if(terminate) return STATES_IDX::IDX_READY;
    else if(cmd[0] == STATES_IDX::IDX_PAUSE){
        pause = true; 
        pause_moment = millis();// 이번 pause가 시작된 순간.
        stateidx = STATES_IDX::IDX_PAUSE;
    }

    if(pause){
        if(cmd[0] == STATES_IDX::RESUME){
            stateidx = STATES_IDX::IDX_CONST;
            ms_t pause_end_moment = millis(); // 이번 pause가 끝난 순간.
            pause_time += (pause_end_moment - pause_moment); // 이번 pause의 지속시간을 총 pause의 지속시간에 더해줌.
            pause = false;
        }
        else if(cmd[0] == STATES_IDX::IDX_R2Z){
            return STATES_IDX::IDX_READY;
        }
    }
    return STATES_IDX::IDX_CONST;
}

/*****************************/ 
/******define sine state******/
/*****************************/ 
void sineState::update(byte * cmd){
    // Serial.println("sine mode");
    stateidx = STATES_IDX::IDX_SINE;
    direction = CMD_MOTOR_DIR_CW;
    terminate = false;
    pause = false;
    start_moment = millis();
    offset_flag = false;
    pause_time = 0;
    cycle = 0;

    cmd_fq      = cmd[1];
    cmd_cycle   = cmd[2];
    cmd_offset  = cmd[3];
    cmd_amp     = cmd[4];

    target_fq = (float)cmd_fq * 0.01;
    target_cycle = cmd_cycle;
    /*0.5%/LSB*/
    target_offset = ((rm->getCWLockAngle() - rm->getCCWLockAngle()))*((float)cmd_offset / (float)200);
    /*0.5%/LSB*/
    target_amp = ((rm->getCWLockAngle() - rm->getCCWLockAngle()))*((float)cmd_amp / (float)200);

    init_mult_angle = rm->getCCWLockAngle() + target_offset;
    target_speed = 36000 * ((float)cmd_fq / (float)100);
}
void sineState::run(){
    if(pause){
        rm->stopMotor();
    }
    else{
        rm->updateMultAngle();
        if(cycle != target_cycle){
            rm->speedControl2(direction, target_speed);
            if(direction == CMD_MOTOR_DIR_CW){
                if(rm->getMultAngleRAW() > init_mult_angle + target_amp){
                    direction = CMD_MOTOR_DIR_CCW;
                }
            }
            else if(direction == CMD_MOTOR_DIR_CCW){
                if(rm->getMultAngleRAW() < init_mult_angle - target_amp){
                    direction = CMD_MOTOR_DIR_CW;
                    cycle++;
                }
            }
        }
        else{
            if(rm->multAngleControl(init_mult_angle, target_speed, 100)) terminate = true;
        }
    }
}

STATES_IDX sineState::getNextState(byte* cmd){
    if(cmd[0] == STATES_IDX::IDX_INIT) {return STATES_IDX::IDX_INIT;}
    if(terminate){
        // rm->stopMotor();
        // stateidx = STATES_IDX::IDX_SINETERM;
        if(cmd[0] == STATES_IDX::IDX_SINE) update(cmd);
        else if(cmd[0] == STATES_IDX::IDX_R2Z) return STATES_IDX::IDX_READY;
    }
    else if(cmd[0] == STATES_IDX::IDX_PAUSE){
        pause = true; 
        stateidx = STATES_IDX::IDX_PAUSE;
        pause_moment = millis();// 이번 pause가 시작된 global_moment.
    }
    if(pause){
        if(cmd[0] == STATES_IDX::RESUME){
            stateidx = STATES_IDX::IDX_SINE;
            ms_t pause_end_moment = millis(); // 이번 pause가 끝난 global_moment.
            pause_time += (pause_end_moment - pause_moment); // 이번 pause의 지속시간을 총 pause의 지속시간에 더해줌.
            pause = false;
        }
        else if(cmd[0] == STATES_IDX::IDX_R2Z){
            return STATES_IDX::IDX_READY;
        }
    }
    return STATES_IDX::IDX_SINE;
}
