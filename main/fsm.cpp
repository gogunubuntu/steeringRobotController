#include "fsm.h"
#include "rmd_motor.h"
#include "utils.h"

motorStateMachine::~motorStateMachine(){
	for(int i = 0; i < STATES_IDX::NUM_STATE; i++){
		delete states[i];
	}
	delete [] states;
}
motorStateMachine::motorStateMachine(RMDmotor& rm)
{
  	String init_cmd = "";
	currentState = STATES_IDX::IDX_INIT;
	nextState = STATES_IDX::IDX_INIT;

	stateChangeTime = 0;
	states = new State*[STATES_IDX::NUM_STATE];

	/*managing derived states with base state class pointer array*/
	states[STATES_IDX::IDX_INIT]  = new initState (rm, STATES_IDX::IDX_INIT);
	states[STATES_IDX::IDX_READY] = new readyState(rm, STATES_IDX::IDX_READY);
	states[STATES_IDX::IDX_L2L]   = new l2lState  (rm, STATES_IDX::IDX_L2L);
	states[STATES_IDX::IDX_CONST] = new constState(rm, STATES_IDX::IDX_CONST);
	states[STATES_IDX::IDX_SINE]  = new sineState (rm, STATES_IDX::IDX_SINE);

	/*for publish current state to PC*/
	idx2cmd[STATES_IDX::IDX_INIT] 	  = 0; /*setup*/ 
	idx2cmd[STATES_IDX::IDX_READY] 	  = 1; /*stand by*/
	idx2cmd[STATES_IDX::IDX_L2L]      = 5; /*L2L*/
	idx2cmd[STATES_IDX::IDX_CONST] 	  = 6; /*const str*/
	idx2cmd[STATES_IDX::IDX_SINE] 	  = 7; /*sine str*/
	idx2cmd[STATES_IDX::IDX_PAUSE] 	  = 4; /*cancel(pause)*/
	idx2cmd[STATES_IDX::IDX_SINETERM] = 7; /*sine str*/
	idx2cmd[STATES_IDX::IDX_R2Z]      = 3; /*return to zero*/
	

	/*for subscribe CMD from PC */
	cmd2idx[0] = STATES_IDX::IDX_INIT;
	cmd2idx[1] = STATES_IDX::DEPRECATED;
	cmd2idx[2] = STATES_IDX::IDX_R2Z;
	cmd2idx[3] = STATES_IDX::IDX_PAUSE;
	cmd2idx[4] = STATES_IDX::RESUME;
	cmd2idx[5] = STATES_IDX::IDX_L2L;
	cmd2idx[6] = STATES_IDX::IDX_CONST;
	cmd2idx[7] = STATES_IDX::IDX_SINE;

	// states[currentState] -> update(init_cmd);
}
byte* motorStateMachine::getMsgToPC(){
	msg_to_pc.mcu_status = (getCurrentState().getStateIdx() != STATES_IDX::IDX_INIT);
	msg_to_pc.mcu_status = (msg_to_pc.mcu_status << 1) | 0;
	msg_to_pc.mcu_status = (msg_to_pc.mcu_status << 6) | idx2cmd[getCurrentState().getStateIdx()];
	msg_to_pc.mcu_status = msg_to_pc.mcu_status;
	if(!rm.getUpdateFlag()) rm.updateCurrentState2();
	memcpy((byte*)(&msg_to_pc) + 1, &rm.getCurrentState2(), 8);
	return (byte*)(&msg_to_pc);
}
void motorStateMachine::updateCurrentState(byte* cmd){

	STATES_IDX IDX = states[currentState] -> getNextState(cmd);
	transitionTo(IDX, cmd);
}
motorStateMachine& motorStateMachine::transitionTo(STATES_IDX IDX, byte* cmd){
	nextState = IDX;
	stateChangeTime = millis();
	update(cmd);
	return *this;
}
motorStateMachine& motorStateMachine::update(byte* cmd) {
	if (currentState != nextState){
		immediateTransitionTo(nextState);
		states[currentState]->update(cmd);
	}
	return *this;
}
motorStateMachine& motorStateMachine::immediateTransitionTo(STATES_IDX IDX){
	states[currentState]->exit();
	currentState = nextState = IDX;
	states[currentState]->enter();
	stateChangeTime = millis();
	return *this;
}
State& motorStateMachine::getCurrentState() {
	return *states[currentState];
}
boolean motorStateMachine::isInState(STATES_IDX &state){
	if (state == currentState) {
		return true;
	} else {
		return false;
	}
}
unsigned long motorStateMachine::timeInCurrentState() { 
	millis() - stateChangeTime; 
}
