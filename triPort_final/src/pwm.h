/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PWM_H
#define __PWM_H

#endif


#include "stdio.h"
#include "stm32f4xx.h"


extern void USART_puts(USART_TypeDef *USARTx, volatile char *str);

//TIMERS INITIALIZATION
void timer2Init(int period, uint32_t prescaler);//TIMER 2
void timer3Init(int period, uint32_t prescaler);//TIMER 3
void timer4Init();//TIMER 4
void timer9Init(int period, uint32_t prescaler);//TIMER 9

//TIMERS CHANNELS INITIALIZATION
void pwmInit1(int duty1,int period, int stop);//Linear actuator1 CH1 init

void pwmInit2(int duty2,int period, int stop);//Linear actuator1 CH2 init

void pwmInit3(int duty2,int period, int stop);//Linear actuator2 CH1 init

void pwmInit4(int duty2,int period, int stop);//Linear actuator2 CH2 init

void pwmInit5(int duty5, int period, int stop); //Linear actuator 3 CH1 init

void pwmInit6(int duty6, int period, int stop); //Linear actuator 4 CH2 init

void pwmInit7(int duty7, int period, int stop); //Linear actuator 3 CH1 init

void pwmInit8(int duty8, int period, int stop); //Linear actuator 4 CH2 init

void pwmInit9(int duty9,  int stop); //DC motor CH1 init

void pwmInit10(int duty10, int stop); //DC motor CH2 init

void pwmOutInit1(); // Linear Actuator 1 init

void pwmOutInit2(); // Linear Actuator 2 init

void pwmOutInit3(); // Linear Actuator 3 init

void pwmOutInit4(); // Linear Actuator 4 init

void pwmOutInitDC(); // DC motor init

//Enable pins
void init_pwm_GPIO();

//DRIVING FUNCTIONS:
void linAct1(int linDuty1, int linPeriod1, int linDir1); // Linear Actuator 1

void linAct2(int linDuty2, int linPeriod2, int linDir2); // Linear Actuator 2

void linAct3(int linDuty3, int linPeriod3, int linDir3); // Linear Actuator 3

void linAct4(int linDuty4, int linPeriod4, int linDir4); // Linear Actuator 4

void dcMove(int dcDuty, int dcDir);// DC motor

//TIMER START/STOP FUNCTIONS
void timer2Start();

void timer2Stop();

void timer3Start();

void timer3Stop();

void timer4Start();

void timer4Stop();

void timer9Start();

void timer9Stop();
