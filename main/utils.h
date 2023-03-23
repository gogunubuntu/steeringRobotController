#ifndef UTILS_H
#define UTILS_H
void print_frame(char frame[], char len);
enum STATES_IDX  {IDX_INIT, IDX_READY, IDX_L2L, IDX_CONST, IDX_SINE, NUM_STATE, IDX_PAUSE, IDX_SINETERM, IDX_R2Z, RESUME, DEPRECATED};
typedef unsigned long ms_t;
#endif