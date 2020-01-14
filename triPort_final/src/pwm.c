/*
 * pwm.c
 *
 *  Created on: 26. mar. 2019
 *      Author: AntonP
 */


#include "stdio.h"
#include "stm32f4xx.h"
#include "pwm.h"


void timer2Start(){
    TIM_Cmd(TIM2, ENABLE);
}

void timer2Stop(){
    TIM_Cmd(TIM2, DISABLE);
}

void timer3Start(){
    TIM_Cmd(TIM3, ENABLE);
}

void timer3Stop(){
    TIM_Cmd(TIM3, DISABLE);
}

void timer4Start(){
    TIM_Cmd(TIM4, ENABLE);
}

void timer4Stop(){
    TIM_Cmd(TIM4, DISABLE);
}


void timer9Start(){
    TIM_Cmd(TIM9, ENABLE);
}

void timer9Stop(){
    TIM_Cmd(TIM9, DISABLE);
}

void timer2Init( int period, uint32_t prescaler){
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseInitTypeDef timerInitStructure;
    timerInitStructure.TIM_Prescaler = prescaler;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = period;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &timerInitStructure);
}

void timer3Init( int period, uint32_t prescaler){
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseInitTypeDef timerInitStructure;
    timerInitStructure.TIM_Prescaler = prescaler;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = period;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &timerInitStructure);
}

void timer4Init(){
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    TIM_TimeBaseInitTypeDef timerInitStructure;
    timerInitStructure.TIM_Prescaler = 8399;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = 199;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &timerInitStructure);
}

void timer9Init(int period9, uint32_t prescaler){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
    TIM_TimeBaseInitTypeDef timerInitStructure;
    timerInitStructure.TIM_Prescaler = prescaler;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = period9;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM9, &timerInitStructure);
}


void pwmInit1(int duty1, int period, int stop){ // Linear Actuator 1 CH1
    int pulse1 = (duty1 * period) / 100;
    TIM_OCInitTypeDef outputChannelInit = {0,};
    outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit.TIM_Pulse = pulse1;
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM2, &outputChannelInit);

    if(stop == 1){
        TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);

    }else if(stop == 0){
        TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    }

    GPIO_PinAFConfig(GPIOA,GPIO_PinSource0 ,GPIO_AF_TIM2);
}


void pwmInit2(int duty2, int period, int stop){ // Linear Actuator 1 CH2
    int pulse2 = (duty2 * period) / 100;
    TIM_OCInitTypeDef outputChannelInit = {0,};
    outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit.TIM_Pulse = pulse2;
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC2Init(TIM2, &outputChannelInit);

    if(stop == 1){
        TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);

    }else if(stop == 0){
        TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    }

    GPIO_PinAFConfig(GPIOA,GPIO_PinSource1 ,GPIO_AF_TIM2);
}

void pwmInit3(int duty3, int period, int stop){// Linear Actuator 2 CH1
    int pulse3 = (duty3 * period) / 100;
    TIM_OCInitTypeDef outputChannelInit = {0,};
    outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit.TIM_Pulse = pulse3;
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC3Init(TIM2, &outputChannelInit);

    if(stop == 1){
        TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Disable);

    }else if(stop == 0){
        TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
    }

    GPIO_PinAFConfig(GPIOA,GPIO_PinSource2 ,GPIO_AF_TIM2);
}


void pwmInit4(int duty4, int period, int stop){// Linear Actuator 2 CH2
    int pulse4 = (duty4 * period) / 100;
    TIM_OCInitTypeDef outputChannelInit = {0,};
    outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit.TIM_Pulse = pulse4;
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC4Init(TIM2, &outputChannelInit);

    if(stop == 1){
        TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Disable);

    }else if(stop == 0){
        TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
    }

    GPIO_PinAFConfig(GPIOA,GPIO_PinSource3 ,GPIO_AF_TIM2);
}

void pwmInit5(int duty5, int period, int stop){ // Linear Actuator 3 CH1

    int pulse5 = (duty5 * period) / 100;
    TIM_OCInitTypeDef outputChannelInit = {0,};
    outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit.TIM_Pulse = pulse5;
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM3, &outputChannelInit);

    if(stop == 1){
        TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);

    }else if(stop == 0){
        TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    }


    GPIO_PinAFConfig(GPIOA,GPIO_PinSource6 ,GPIO_AF_TIM3);

}

void pwmInit6(int duty6, int period, int stop){ // Linear Actuator 3 CH2
    int pulse6 = (duty6 * period) / 100;
    TIM_OCInitTypeDef outputChannelInit = {0,};
    outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit.TIM_Pulse = pulse6;
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC2Init(TIM3, &outputChannelInit);

    if(stop == 1){
        TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable);

    }else if(stop == 0){
        TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    }

    GPIO_PinAFConfig(GPIOA,GPIO_PinSource7 ,GPIO_AF_TIM3);
}

void pwmInit7(int duty7, int period, int stop){ // Linear Actuator 4 CH1
    int pulse7 = (duty7 * period) / 100;
    TIM_OCInitTypeDef outputChannelInit = {0,};
    outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit.TIM_Pulse = pulse7;
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM9, &outputChannelInit);

    if(stop == 1){
        TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Disable);

    }else if(stop == 0){
        TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);
    }

    GPIO_PinAFConfig(GPIOE,GPIO_PinSource5 ,GPIO_AF_TIM9);
}

void pwmInit8(int duty8, int period, int stop){ // Linear Actuator 4 CH2
    int pulse8 = (duty8 * period) / 100;
    TIM_OCInitTypeDef outputChannelInit = {0,};
    outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit.TIM_Pulse = pulse8;
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC2Init(TIM9, &outputChannelInit);

    if(stop == 1){
        TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Disable);

    }else if(stop == 0){
        TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);
    }

    GPIO_PinAFConfig(GPIOE,GPIO_PinSource6 ,GPIO_AF_TIM9);
}


void pwmInit9(int duty9, int stop){ // DC motor CH1
    TIM_OCInitTypeDef outputChannelInit = {0,};
    outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit.TIM_Pulse = ((199 + 1) * duty9)/100 - 1;
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC3Init(TIM4, &outputChannelInit);

    if(stop == 1){
        TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Disable);
    }else if(stop == 0){
        TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
    }

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
}

void pwmInit10(int duty10, int stop){ // DC motor CH2
    TIM_OCInitTypeDef outputChannelInit = {0,};
    outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit.TIM_Pulse = ((199 + 1) * duty10)/100 - 1;
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC4Init(TIM4, &outputChannelInit);

    if(stop == 1){
        TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Disable);

    }else if(stop == 0){
        TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
    }

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);
}


void pwmOutInit1(){ // Linear actuator 1
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef gpioStructure;
    gpioStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; //
    gpioStructure.GPIO_Mode = GPIO_Mode_AF;
    gpioStructure.GPIO_Speed = GPIO_Speed_100MHz;
    gpioStructure.GPIO_OType = GPIO_OType_PP;
    gpioStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpioStructure);
}

void pwmOutInit2(){ // Linear actuator 2
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef gpioStructure;
    gpioStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //
    gpioStructure.GPIO_Mode = GPIO_Mode_AF;
    gpioStructure.GPIO_Speed = GPIO_Speed_100MHz;
    gpioStructure.GPIO_OType = GPIO_OType_PP;
    gpioStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpioStructure);
}

void pwmOutInit3(){ // Linear actuator 3
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef gpioStructure;
    gpioStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //
    gpioStructure.GPIO_Mode = GPIO_Mode_AF;
    gpioStructure.GPIO_Speed = GPIO_Speed_100MHz;
    gpioStructure.GPIO_OType = GPIO_OType_PP;
    gpioStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpioStructure);
}

void pwmOutInit4(){ // Linear actuator 4
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    GPIO_InitTypeDef gpioStructure;
    gpioStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6; //
    gpioStructure.GPIO_Mode = GPIO_Mode_AF;
    gpioStructure.GPIO_Speed = GPIO_Speed_100MHz;
    gpioStructure.GPIO_OType = GPIO_OType_PP;
    gpioStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &gpioStructure);
}

void pwmOutInitDC(){ // Linear actuator 4
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef gpioStructure;
    gpioStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; //
    gpioStructure.GPIO_Mode = GPIO_Mode_AF;
    gpioStructure.GPIO_Speed = GPIO_Speed_100MHz;
    gpioStructure.GPIO_OType = GPIO_OType_PP;
    gpioStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &gpioStructure);
}

//ENABLE PINS FOR LINEAR ACTUATOR DRIVERS
void init_pwm_GPIO(){
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef gpioStructure1;
    gpioStructure1.GPIO_Pin = GPIO_Pin_2; //
    gpioStructure1.GPIO_Mode = GPIO_Mode_OUT;
    gpioStructure1.GPIO_Speed = GPIO_Speed_100MHz;
    gpioStructure1.GPIO_OType = GPIO_OType_PP;
    gpioStructure1.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &gpioStructure1);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    GPIO_InitTypeDef gpioStructure2;
    gpioStructure2.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 ; //
    gpioStructure2.GPIO_Mode = GPIO_Mode_OUT;
    gpioStructure2.GPIO_Speed = GPIO_Speed_100MHz;
    gpioStructure2.GPIO_OType = GPIO_OType_PP;
    gpioStructure2.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &gpioStructure2);

}


void linAct1(int linDuty1,int linPeriod1, int linDir1){
    if(linDir1 == 0){
        //timer2Stop();
        //delay_ms(50);
        GPIO_ResetBits(GPIOE, GPIO_Pin_7);
        pwmInit1(0, 0, 1);
        pwmInit2(0, 0, 1);
    }else if(linDir1 == 1){
        //timer2Start();
        delay_ms(2);
        GPIO_SetBits(GPIOE, GPIO_Pin_7);
        pwmInit2(0, 0, 1);
        pwmInit1(linDuty1, linPeriod1, 0);
        pwmOutInit1();

    }else if (linDir1 == 2){
        //timer2Start();
        delay_ms(2);
        GPIO_SetBits(GPIOE, GPIO_Pin_7);
        pwmInit1(0, 0, 1);
        pwmInit2(linDuty1, linPeriod1, 0);
        pwmOutInit1();
    }

}

void linAct2(int linDuty2,int linPeriod2, int linDir2){
    if(linDir2 == 0){
        GPIO_ResetBits(GPIOB, GPIO_Pin_2);
        pwmInit3(0, 0, 1);
        pwmInit4(0, 0, 1);
    }else if(linDir2 == 1){
        GPIO_SetBits(GPIOB, GPIO_Pin_2);
        pwmInit4(0, 0, 1);
        pwmInit3(linDuty2, linPeriod2, 0);
        pwmOutInit2();

    }else if (linDir2 == 2)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_2);
        pwmInit3(0, 0, 1);
        pwmInit4(linDuty2, linPeriod2, 0);
        pwmOutInit2();
    }

}

void linAct3(int linDuty3,int linPeriod3, int linDir3){
    if(linDir3 == 0){
        GPIO_ResetBits(GPIOE, GPIO_Pin_9);
        pwmInit5(0, 0, 1);
        pwmInit6(0, 0, 1);
    }else if(linDir3 == 1){
        GPIO_SetBits(GPIOE, GPIO_Pin_9);
        pwmInit6(0, 0, 1);
        pwmInit5(linDuty3, linPeriod3, 0);
        pwmOutInit3();

    }else if (linDir3 == 2)
    {
        GPIO_SetBits(GPIOE, GPIO_Pin_9);
        pwmInit5(0, 0, 1);
        pwmInit6(linDuty3, linPeriod3, 0);
        pwmOutInit3();
    }

}

void linAct4(int linDuty4,int linPeriod4, int linDir4){
    if(linDir4 == 0){
        GPIO_ResetBits(GPIOE, GPIO_Pin_8);
        pwmInit7(0, 0, 1);
        pwmInit8(0, 0, 1);
    }else if(linDir4 == 1){
        GPIO_SetBits(GPIOE, GPIO_Pin_8);
        pwmInit8(0, 0, 1);
        pwmInit7(linDuty4, linPeriod4, 0);
        pwmOutInit4();

    }else if (linDir4 == 2)
    {
        GPIO_SetBits(GPIOE, GPIO_Pin_8);
        pwmInit7(0, 0, 1);
        pwmInit8(linDuty4, linPeriod4, 0);
        pwmOutInit4();
    }
}

void dcMove(int dcDuty, int dcDir){
    if(dcDir == 0){
        pwmInit9(1, 1);
        pwmInit10(1,1);
    }else if(dcDir == 1){
        pwmInit10(1,1);
        delay_ms(1);
        pwmInit9(dcDuty, 0);
        pwmOutInitDC();
    }else if(dcDir == 2){
        pwmInit9(1,1);
        delay_ms(1);
        pwmInit10(dcDuty, 0);
        pwmOutInitDC();
    }
}
