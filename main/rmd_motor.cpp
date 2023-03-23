#include "rmd_motor.h"
#include "utils.h"
#include <mcp_can.h>
#include <cstring>
RMDmotor::RMDmotor(): mc(CAN0_PIN), mult_angle(0)
{
  delay(1000);
}

void RMDmotor::begin(){
  mc.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ);
  mc.setMode(MCP_NORMAL);
  Serial.print("clear motor previous operating status...");
  if(offMotor()) Serial.println("success!"); 
  else Serial.println("fail");

  Serial.print("set motor status to stop...");
  if(stopMotor()) Serial.println("success!");
  else Serial.println("fail");

  Serial.print("set motor status to running...");
  if(runningMotor()) Serial.println("success!");
  else Serial.println("fail");
}
bool RMDmotor::runningMotor() {
  byte res[8] = {0, };
  uint8_t can_res = sendCMD(frame_running, res);
  if(can_res == CAN_OK){
    return memcmp(res, frame_running, 8) == 0;
  }
  return false;
}
bool RMDmotor::offMotor() {
  byte res[8] = {0, };
  uint8_t can_res = sendCMD(frame_off, res);
  if(can_res == CAN_OK){
    return memcmp(res, frame_off, 8) == 0;
  }
  return false;
}
bool RMDmotor::stopMotor() {
  byte res[8] = {0, };
  uint8_t can_res = sendCMD(frame_stop, res);
  if(can_res == CAN_OK){
    return memcmp(res, frame_stop, 8) == 0;
  }
  return false;
}
uint8_t RMDmotor::sendCMD(byte req[], byte res[]){
  // return can communication result
  static byte dummy[8] = {0, };
  while(CAN_MSGAVAIL == mc.checkReceive()){ // clear CAN buffer
    mc.readMsgBuf(&rxId, &rxLen, dummy);
  }
  uint8_t send_result = mc.sendMsgBuf(MOTOR_CAN_ADDR, CAN0_EXT, CAN0_LEN, req);
  if(send_result == CAN_OK){
    while(CAN_MSGAVAIL != mc.checkReceive());
    mc.readMsgBuf(&rxId, &rxLen, res);
  }
  return send_result;
}
bool RMDmotor::posControl1(const uint32_t & pos){
  state_update_flag = true;
  // static byte req[8] = {CMD_POSITION_CL1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  // memcpy(frame_pos_control1 + 4, &pos, 32);
  *((uint32_t*)(frame_pos_control1 + 4)) = pos;
  return CAN_OK == sendCMD(frame_pos_control1, (byte*)&current_state2);
}
bool RMDmotor::posControl2(const uint16_t& speed_limit, const uint32_t& pos) { 
  state_update_flag = true;
  // memcpy(frame_pos_control2 + 2, &speed_limit, 16);
  *((uint16_t*)(frame_pos_control2 + 2)) = speed_limit;
  // memcpy(frame_pos_control2 + 4, &pos, 32);
  *((uint16_t*)(frame_pos_control2 + 4)) = pos;
  return CAN_OK == sendCMD(frame_pos_control2, (byte*)&current_state2);
}
bool RMDmotor::posControl3(const uint8_t& direction, const uint16_t& pos) { 
  state_update_flag = true;
  // static byte req[8] = {CMD_POSITION_CL3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  // memcpy(frame_pos_control3 + 1, &direction, 8);
  *((uint8_t*)(frame_pos_control3 + 1)) = direction;
  //memcpy(frame_pos_control3 + 4, &pos, 16);
  *((uint16_t*)(frame_pos_control3 + 4)) = pos;
  return CAN_OK == sendCMD(frame_pos_control3, (byte*)&current_state2);
}
bool RMDmotor::posControl4(const uint8_t& direction, const uint16_t & speed_limit, const uint16_t& pos){
  state_update_flag = true;
  // static byte req[8] = {CMD_POSITION_CL4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  // memcpy(frame_pos_control4 + 1, &direction, 8);
  *((uint8_t*)(frame_pos_control4 + 1)) = direction;
  // memcpy(frame_pos_control4 + 2, &speed_limit, 16);
  *((uint16_t*)(frame_pos_control4 + 2)) = speed_limit;
  // memcpy(frame_pos_control4 + 4, &pos, 16);
  *((uint16_t*)(frame_pos_control4 + 4)) = pos;
  return CAN_OK == sendCMD(frame_pos_control4, (byte*)&current_state2);
}
/*speed resoultion: 0.01dps*/
bool RMDmotor::speedControl1(const int32_t& speed) { //0xA2 -> Speed control command
  state_update_flag = true;
  // static byte req[8] = {CMD_SPEED_CL, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  *((int32_t*)(frame_speed_control + 4)) = speed;
  // memcpy(frame_speed_control + 4, &speed, 32);
  return CAN_OK == sendCMD(frame_speed_control, (byte*)&current_state2);
}
bool RMDmotor::speedControl2(const uint8_t& direction, const int32_t& speed){
  //TODO: make speed unsigned.. 
  state_update_flag = true;
  if(direction == CMD_MOTOR_DIR_CCW) return speedControl1(speed * (-1));
  else return speedControl1(speed);
}
bool RMDmotor::torqueControl1(const int16_t & torque){
  state_update_flag = true;
  // byte req[8] = {CMD_TORQUE_CL, 0x00, 0x00, 0x00 ,0x00, 0x00, 0x00, 0x00};
  // memcpy(frame_torque_control + 4, &torque, 16);
  *((int16_t*)(frame_torque_control + 4)) = torque;
  return CAN_OK == sendCMD(frame_torque_control, (byte*)&current_state2);
}
bool RMDmotor::torqueControl2(const uint8_t & direction, const int16_t & torque){
  state_update_flag = true;
  if(direction == CMD_MOTOR_DIR_CCW) return torqueControl1(torque * (-1));
  else return torqueControl1(torque);
}
bool RMDmotor::multAngleControl(const int64_t& target_mult_angle, const int32_t& speed, const int64_t& offset){
  updateMultAngle();
  if((target_mult_angle - mult_angle > -offset)&&(target_mult_angle - mult_angle < offset)) {
    stopMotor();
    // posControl2((uint16_t)speed, target_mult_angle % 36000);
    return true;
  }
  else if(target_mult_angle > mult_angle) speedControl1(speed);
  else if(target_mult_angle <= mult_angle) speedControl1(-speed);
  return false;
}


bool RMDmotor::updateMultAngle(){
  bool result = CAN_OK == sendCMD(frame_read_mult_angle, (byte*)frame_res_mult_angle);
  mult_angle = *((int64_t*)frame_res_mult_angle) / 0x100;
  return result;
}
bool RMDmotor::updateSingleAngle(){
  bool result = CAN_OK == sendCMD(frame_read_single_angle, (byte*)frame_res_single_angle);
  single_angle = *((uint16_t*)(frame_res_single_angle + 6));
  return result;
}
bool RMDmotor::updateCurrentState2(){
  
  return CAN_OK == sendCMD(frame_read_status2, (byte* )&current_state2);
}
void RMDmotor::showCurrentState2(bool update){
  if(update) updateCurrentState2();
  Serial.print("current encoder_pos: ");
  Serial.println(current_state2.encoder_pos);
  Serial.print("current iq: ");
  Serial.println(current_state2.iq);
  Serial.print("current speed: ");
  Serial.println(current_state2.speed);
  Serial.print("current temperature: ");
  Serial.println(current_state2.temperature);
}
bool RMDmotor::updateCurrentState1(){
  return CAN_OK == sendCMD(frame_read_status1, (byte* )&current_state1);
}
void RMDmotor::showCurrentState1(bool update){
  if(update) updateCurrentState1();
  Serial.print("cmd: ");
  Serial.println(current_state1[0], HEX);
  Serial.print("currunt error state: ");
  Serial.println(current_state1[7], HEX);
  Serial.print("current temeperature: ");
  Serial.println(*((int8_t*)(current_state1+1)));
  Serial.print("current voltage: ");
  Serial.println(*((uint16_t*)(current_state1+3)));
  Serial.print("size: ");
  Serial.println(sizeof(current_state1));
}
bool RMDmotor::clearErrorFlag(){
  bool res = (CAN_OK == sendCMD(frame_clear_error_flag, (byte* )&current_state1)); 
  return res;
}
