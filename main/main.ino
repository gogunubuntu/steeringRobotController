#include "fsm.h"
#include "states.h"
#include "rmd_motor.h"
#include "mbed.h"

#define FLAG_FSM (1UL << 0)
#define FLAG_SUB (1UL << 1)
#define CMD_FROM_PC_LEN 5
#define CMD_TO_PC_LEN 9
using namespace mbed;
using namespace rtos;

// String idx2str[8] = {String("IDX_INIT"), String("IDX_READY"), String("IDX_L2L"), String("IDX_CONST"), String("IDX_SINE"), String("NUM_STATE"), String("IDX_PAUSE"), String("IDX_SINETERM")};
EventFlags ef;

RMDmotor rm;
motorStateMachine fsm = motorStateMachine(rm);
bool cmd_complete = false;
int tickcnt = 0;

void taskFSM();
void setFlag();
void serialEvent();

bool r_led_state; 
bool g_led_state;
bool b_led_state;

Thread threadFsm(osPriorityRealtime); /* operating fsm and publish information*/
Thread threadSub(osPriorityRealtime1); /* subscribe user msg*/
Ticker tick10ms;   /* 10 ms*/
byte cmd_from_pc[CMD_FROM_PC_LEN];
byte do_nothing_cmd[CMD_FROM_PC_LEN];
void setup() {
  rm.begin();
  do_nothing_cmd[0] = STATES_IDX::DEPRECATED;
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  String init_cmd = "";
  Serial.begin(9600);
   tick10ms.attach(&setFlag, 0.01);
   threadFsm.start(taskFSM);
   threadSub.start(taskSub);

  while(!Serial);
  fsm.getCurrentState().update(do_nothing_cmd);
}
void loop(){
  /*do nothing*/
}
void taskFSM(){
  while(true){
    ef.wait_any(FLAG_FSM);

    if(cmd_complete) {
      fsm.updateCurrentState(cmd_from_pc);
      cmd_from_pc[0] = STATES_IDX::DEPRECATED;
      cmd_complete = false;
    }
    else{
      fsm.updateCurrentState(cmd_from_pc);
    }
    fsm.getCurrentState().run();
    Serial.write(fsm.getMsgToPC(), 9);
  }
}
void taskSub(){
  while(true){
    ef.wait_any(FLAG_SUB);
    serialEvent();
  }
}

void serialEvent(){
  if (Serial.available() >= CMD_FROM_PC_LEN) {
    Serial.readBytes(cmd_from_pc, CMD_FROM_PC_LEN);
    cmd_from_pc[0] = fsm.cmd2idx[cmd_from_pc[0]]; 
    cmd_complete = true;
  }
}
void setLED(int a){
  digitalWrite(LEDR, (a >> 0) & 1);
  digitalWrite(LEDG, (a >> 1) & 1);
  digitalWrite(LEDB, (a >> 2) & 1);
}
void setFlag(){
  ef.set(FLAG_FSM);
  if(tickcnt % 3 == 0) ef.set(FLAG_SUB);
  ++tickcnt;
}