#ifndef _TEMPERATURE_CTRL_H_
#define _TEMPERATURE_CTRL_H_

void Simulation_PWM_Init(void);
void Simulation_PWM_Output(uint16_t width);

void Temperature_Ctrl_Init(void);
void Temperature_Ctrl(void);
void Temperature_State_Check(void);


extern uint8_t Temperature_Stable_Flag;


#endif

