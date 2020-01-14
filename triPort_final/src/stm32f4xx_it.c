#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stm32f4xx.h>
#include "stm32f4xx_it.h"
#include "stm32f4xx_conf.h"
#include "tm_stm32f4_adc.h"

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
__attribute__((weak)) void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
__attribute__((weak)) void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
__attribute__((weak)) void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
__attribute__((weak)) void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
__attribute__((weak)) void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
__attribute__((weak)) void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
__attribute__((weak)) void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
__attribute__((weak)) void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
__attribute__((weak)) void SysTick_Handler(void)
{

}

//=========EXTERNAL INTERRUPTS FOR ROTARY ENCODER========
extern int encoderTick;
//Interrupt1 - PD0
void EXTI0_IRQHandler(void){
  /* Make sure that interrupt flag is set */
  if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
    /* Do your stuff when PD0 is changed */
    //USART_puts(USART2, "Interrupt1 - PD0!\n\r");

    if(!GPIO_ReadInputDataBit(GPIOD, GPIO_PIN_1)){ //Going CW
      encoderTick++;
      //USART_puts(USART2,"CW\n\r");
      if(encoderTick > 2096 ){ //DFROBOT FIT0185 - 2096 CPR
        encoderTick = 0;
      }
    }else{ //Going CCW
      //USART_puts(USART2,"CCW\n\r");
      if(encoderTick < 1){
        encoderTick = 2096; //The end is reached
      }else{
        encoderTick--;
      }
    }

    /* Clear interrupt flag */
    EXTI_ClearITPendingBit(EXTI_Line0);
  }
}

/*
//Interrupt2 - PD1
void EXTI1_IRQHandler(void){
  if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
    USART_puts(USART2, "Interrupt2 - PD1!\n\r");
    EXTI_ClearITPendingBit(EXTI_Line1);
  }
}
*/




#define MAX_WORDLEN 15
char rec_str[MAX_WORDLEN +1];

extern void USART_puts(USART_TypeDef *USARTx, volatile char *str);
extern void modemUSART_puts(USART_TypeDef *USARTx, volatile char  *modem_str);

//=====Stepper====
char pre_commands[2][3] = {"set","get"};
char commandsStepp[5][4] = {"_spd","_dir","_pos","_stp","_deg"};
extern int spd;
extern int dir;
extern int stp;
extern int move;
extern int newPos;
extern float curPos;
extern int degInc;
//===PWM:
char commandsPWM[13][8] = {"_dty1","_dty2","_dty3","_dty4",
"_act1dir","_act2dir","_act3dir","_act4dir","_act1pos","_act2pos","_act3pos","_act4pos",
"_tblpos"};
extern  int prescalerS;
extern  int dutyS[4];
extern  int periodS[4];
extern  int linDirS[4];
extern  float posFP[4];
extern  float new_posF[4];
extern  int table_stop;
extern  float new_table_pos;
extern  int flag_move[5];

extern int transOn;

void USART2_IRQHandler(void){

    if (USART_GetITStatus(USART2, USART_IT_RXNE)){
        static int cnt = 0;
        // Flag is set (every time a byte is sent)
        char ch = USART2 -> DR; //byte that was sent

        if((ch != '\n') && (cnt < MAX_WORDLEN)){
            rec_str[cnt++] = ch;
        }else{
          rec_str[cnt] = '\0';
          //USART_puts(USART2,"GOT A MESSAGE!");
           cnt = 0;
            USART_puts(USART2,"\r\n");
            //USART_puts(USART2,rec_str);

            if(transOn){
            	modemUSART_puts(USART6, rec_str);
            }

            //Moving command:
            if(strncmp(rec_str,"MS",2) == 0){ // Stepper moves for number of stepps
               USART_puts(USART2,"Command: Move steps\r\n");
               move = 1;
             }else if(strncmp(rec_str,"MTD",3) == 0){ // Stepper moves to the degree
                  USART_puts(USART2,"Command: Move to degree\r\n");
                  move = 2;
             }else if(strncmp(rec_str,"transON",7) == 0){
            	 USART_puts(USART2,"Direct communication with modem enabled\n\r");
            	 transOn = 1;
             }else if(strncmp(rec_str,"transOFF",8) == 0){
            	 USART_puts(USART2,"Direct communication with modem disabled\n\r");
            	 transOn = 0;
             }else if(strncmp(rec_str,"MFD",3) == 0){ // Stepper moves for degree
                  USART_puts(USART2, "Command: Move for degree");
                  move = 3;
             }else if(strncmp(rec_str,"get_all",7) == 0){ // get all values
                 USART_puts(USART2,"Stepper motor values: \r\n");

                 USART_puts(USART2,"\t");

                 USART_puts(USART2,"Speed = ");
                 char speedC[4];
                 sprintf(speedC,"%d",spd);
                 USART_puts(USART2,speedC);

                 USART_puts(USART2,"\n\t");

                 USART_puts(USART2,"Direction = ");
                 if(dir == 0){
                    USART_puts(USART2,"Clockwise");
                 }else if(dir == 1){
                    USART_puts(USART2,"Anti-clockwise");
                 }

                  USART_puts(USART2,"\n\t");

                  USART_puts(USART2, "New position = ");
                  char positionC[4];
                  sprintf(positionC,"%d",newPos);
                  USART_puts(USART2, positionC);

                  USART_puts(USART2,"\n\t");

                  USART_puts(USART2, "Current position = ");
                  positionC[6];
                  sprintf(positionC,"%4.2f",curPos);
                  USART_puts(USART2, positionC);

                  USART_puts(USART2,"\n\t");

                  USART_puts(USART2, "Steps = ");
                  char stpC[4];
                  sprintf(stpC,"%d", stp);
                  USART_puts(USART2, stpC);

                  USART_puts(USART2,"\n\t");

                  USART_puts(USART2,"Degrees = ");
                  char degC[4];
                  sprintf(degC, "%d", degInc);
                  USART_puts(USART2,degC);

                  USART_puts(USART2,"\r\n");

                  USART_puts(USART2,"Linear actuator values: \r\n\t");

                  char prescalerC[4];
                  sprintf(prescalerC, "%d", prescalerS);
                  USART_puts(USART2,"Prescaler = ");
                  USART_puts(USART2, prescalerC);
                  USART_puts(USART2,"\n\t");

                  for(int i = 0; i < 4 ;i++){
                      USART_puts(USART2,"Period ");

                      char iC[1];
                      sprintf(iC, "%d", i+1);
                      USART_puts(USART2, iC);

                      USART_puts(USART2," = ");


                      char periodC1[4];
                      sprintf(periodC1,"%d",periodS[i]);
                      USART_puts(USART2,periodC1);

                      USART_puts(USART2,"\n\t");

                      }
                  for(int i = 0; i < 4; i++){


                      USART_puts(USART2,"Duty");

                      char iC[1];
                      sprintf(iC, "%d", i+1);
                      USART_puts(USART2, iC);

                      USART_puts(USART2," = ");

                      char dutyC[4];
                      sprintf(dutyC,"%d",dutyS[i]);
                      USART_puts(USART2, dutyC);
                      USART_puts(USART2," %");

                      USART_puts(USART2,"\n\t");
                  }
                  for(int i = 0; i < 4; i++){
                      USART_puts(USART2,"Direction ");

                      char iC[1];
                      sprintf(iC, "%d", i+1);
                      USART_puts(USART2, iC);

                      USART_puts(USART2," = ");

                      char dirC[4];
                      sprintf(dirC,"%d",linDirS[i]);
                      USART_puts(USART2, dirC);

                      USART_puts(USART2,"\n\t");
                  }
                  for(int i = 0; i < 4; i++){
                      USART_puts(USART2, "Position");

                      char iC[1];
                      sprintf(iC, "%d", i+1);
                      USART_puts(USART2, iC);

                      USART_puts(USART2," = ");

                      char posC[10];
                      sprintf(posC, "%4.2f mm\n\t", posFP[i]);
                      USART_puts(USART2, posC);
                  }
                  USART_puts(USART2,"\r\n");
                }else if(strncmp(rec_str,"table_stop",10) == 0){
                    USART_puts(USART2, "Table stop\n\r");
                    table_stop = 1;
                }else if(strncmp(rec_str,"table_start",11) == 0){
                    USART_puts(USART2, "Table start\n\r");
                    table_stop = 0;
                }


             int pre_comm_len = strcspn(rec_str,"_"); //Position of "_" - determines the set or get command
             int comm_len = strcspn(rec_str,"="); // Position of "=" so that the program knows where is the end of command
             int arg_len = strlen(rec_str) - comm_len; //Length of argument

             char pre_command[3];
             char command[pre_comm_len];
             char argument[arg_len];

             for (int i = 0; i < strlen(rec_str); i++)
             {
                 if(i < pre_comm_len){
                     pre_command[i] = rec_str[i];
                 }else if(i >= pre_comm_len && i < comm_len  ){
                      command[i - pre_comm_len] = rec_str[i];
                 }else if(i >= (comm_len + 1)){
                      argument[i- comm_len - 1] = rec_str[i];
                  }
          }





             //===============STEPPER COMMANDS==================================
              for(int i = 0; i < 2; i++){
                //==================SET/GET===========================//
                  if(strncmp(pre_command,pre_commands[i],3) == 0){
                      switch(i){
                        //==================SET===========================//
                          case 0: //USART_puts(USART2,"it is a SET!");
                          //============Speed==========================//
                              if (strncmp(command,commandsStepp[0],4) == 0){

                                  USART_puts(USART2,"Setting the speed = ");
                                  USART_puts(USART2, argument);
                                  spd = atoi(argument);
                          //====================Direction====================//
                              }else if(strncmp(command,commandsStepp[1],4) == 0){

                                   USART_puts(USART2,"Setting direction = ");
                                       if(strncmp(argument,"clk",3) == 0){
                                          USART_puts(USART2,"Clockwise\r\n");
                                          dir = 0;
                                       }else if(strncmp(argument, "aclk",4) == 0){
                                          dir = 1;
                                          USART_puts(USART2,"Anti-clockwise\r\n");
                                       }
                          //================Position============================//
                              }else if(strncmp(command,commandsStepp[2],4) == 0){

                                   USART_puts(USART2, "Setting new position = ");
                                   USART_puts(USART2, argument);
                                   newPos = atoi(argument);


                                    USART_puts(USART2, "Current position = ");
                                    char positionC[6];
                                    sprintf(positionC,"%4.2f",curPos);
                                    USART_puts(USART2, positionC);

                                 }
                          //=============Steps==================================//
                                else if(strncmp(command,commandsStepp[3],4) == 0){
                                    USART_puts(USART2, "Setting stepps = ");
                                    USART_puts(USART2,argument);
                                    stp = atoi(argument);
                                 }
                          //=============Degrees=============================//
                                 else if(strncmp(command,commandsStepp[4],4) == 0){
                                    USART_puts(USART2, "Setting degrees = ");
                                    USART_puts(USART2,argument);
                                    degInc = atoi(argument);
                                 }


                                 //================LA COMMANDS=================================
                                 for(int i= 0; i < 13 ; i++){
                                    if(i < 4 && strncmp(command,commandsPWM[i],5) == 0){
                                      USART_puts(USART2,"Duty");
                                      char iC[1];
                                      sprintf(iC,"%d = ",i+1);
                                      USART_puts(USART2,iC);
                                      float dty = atoi(argument);
                                      if(dty < 0){
                                          dutyS[i] = 0;
                                          USART_puts(USART2,"0");
                                          USART_puts(USART2,"\n\r");
                                      }else if(dty > 100){
                                          dutyS[i] = 100;
                                          USART_puts(USART2,"100");
                                          USART_puts(USART2,"\n\r");
                                      }else{
                                          dutyS[i] = dty;
                                          USART_puts(USART2,argument);
                                          USART_puts(USART2,"\n\r");


                                          /*
                                          char iC[1];
                                          sprintf(iC,"%f",dutyS[i]);
                                          USART_puts(USART2,iC);
                                          USART_puts(USART2,"\n\r")
                                          */
                                      }
                                  }else if(i < 8 && i >= 4 && ((strncmp(command,commandsPWM[i],8) == 0))){
                                      USART_puts(USART2, "Direction");
                                      char iC[1];
                                      sprintf(iC,"%d = ",i - 3);
                                      USART_puts(USART2,iC);

                                      USART_puts(USART2,argument);
                                      USART_puts(USART2,"\n\r");
                                      linDirS[i-4] = atoi(argument);
                                  }else if(i <= 11 && i >= 8 && (strncmp(command,commandsPWM[i],8) == 0)){
                                      USART_puts(USART2, "Position");

                                      //flag_move[i-7] = 1;


                                      char iC[1];
                                      sprintf(iC,"%d = ", i - 7);
                                      USART_puts(USART2,iC);


                                      float pos = strtod(argument, NULL);
                                      if (pos < 0)
                                      {
                                        new_posF[i - 8] = 0;
                                        USART_puts(USART2,"0");
                                        USART_puts(USART2,"\n\r");
                                      }else if(pos > 402){
                                        new_posF[i - 8] = 402;
                                        USART_puts(USART2,"402");
                                        USART_puts(USART2,"\n\r");
                                      }else{
                                        new_posF[i - 8] = pos;
                                        USART_puts(USART2,argument);
                                        USART_puts(USART2,"\n\r");
                                      }
                                  }else if(i > 11 && (strncmp(command,commandsPWM[i],7) == 0)){
                                    USART_puts(USART2, "Table position: ");
                                    flag_move[0] = 3;
                                    new_table_pos = strtod(argument, NULL);

                                    if (new_table_pos < 0)
                                    {
                                      new_table_pos = 0;
                                      USART_puts(USART2, "0\n\r");
                                    }else if(new_table_pos > 402){
                                      new_table_pos = 402;
                                      USART_puts(USART2, "402\n\r");
                                    }else{
                                      USART_puts(USART2, argument);
                                      USART_puts(USART2,"\n\r");
                                    }
                                  }

                                 }
                          USART_puts(USART2, "\r\n");
                          break;
                          //=============GET==============================//
                          case 1: //USART_puts(USART2,"it is a GET!");
                          //=====================Speed======================//
                              if (strncmp(command,commandsStepp[0],4) == 0){
                                  USART_puts(USART2,"Speed = ");
                                  char speedC[4];
                                  sprintf(speedC,"%d",spd);
                                  USART_puts(USART2,speedC);
                          //=====================Direction===================//
                                }else if(strncmp(command,commandsStepp[1],4) == 0){
                                     USART_puts(USART2,"Direction = ");
                                     if(dir == 0){
                                        USART_puts(USART2,"Clockwise\r\n");
                                     }else if(dir == 1){
                                        USART_puts(USART2,"Anti-clockwise\r\n");
                                     }
                          //======================Position====================//
                                   }else if(strncmp(command,commandsStepp[2],4) == 0){
                                        USART_puts(USART2, "Position = ");
                                        char positionC[4];
                                        sprintf(positionC,"%f",curPos);
                                        USART_puts(USART2, positionC);
                                      }
                          //======================Steps========================//
                                      else if(strncmp(command,commandsStepp[3],4) == 0){
                                        USART_puts(USART2, "Steps = ");
                                        char stpC[4];
                                        sprintf(stpC,"%d", stp);
                                        USART_puts(USART2, stpC);
                                      }

                          //======================Degrees======================//
                                      else if(strncmp(command,commandsStepp[4],4) == 0){
                                        USART_puts(USART2,"Degrees = ");
                                        char degC[4];
                                        sprintf(degC, "%d", degInc);
                                        USART_puts(USART2,degC);
                                      }

                           //====================LINEAR ACTUATOR==================//
                                      for(int i= 0; i < 8;i++){
                                          if(strncmp(command,commandsPWM[i],5) == 0){
                                              USART_puts(USART2,"Duty");
                                              char iC[1];
                                              sprintf(iC,"%d",i+1);
                                              USART_puts(USART2,iC);
                                              USART_puts(USART2," = ");

                                              char dutyC[4];
                                              sprintf(dutyC,"%d", dutyS[i]);
                                              USART_puts(USART2, dutyC);
                                              USART_puts(USART2,"%");
                                         }
                                      }
                          USART_puts(USART2, "\r\n");
                          break;


                        }
                    }
                }



            }

    }

   }





//CO2 USART HANDLER:
extern int sensorFeedback[7];
extern int max_feedback_len;
extern int sensorFeedback_cnt;

extern void sensUSART_puts(USART_TypeDef *USARTx, volatile int num);

void USART1_IRQHandler(void) {
      if (USART_GetITStatus(USART1, USART_IT_RXNE)){
        // Flag is set (every time a byte is sent)
        int ch2 = USART1 -> DR; //byte that was sent
        //sensorFeedback[sensorFeedback_cnt] = ch2;
        //sensUSART_puts(USART2,sensorFeedback[sensorFeedback_cnt]) ;
        //sensorFeedback_cnt++;

        //sensUSART_puts(USART2, ch2);


        if(sensorFeedback_cnt < max_feedback_len){
          sensorFeedback[sensorFeedback_cnt] = ch2;
          //sensUSART_puts(USART2,sensorFeedback[sensorFeedback_cnt]) ;
          sensorFeedback_cnt++;
        }
        /*else{
          sensorFeedback_cnt = 0;
         }*/

    }
}



//Modem USART handler
#define modem_MAX_WORDLEN 64
char modem_rec_str[65];

//AT commands flags:
extern int at_ok;
extern int at_err;

//CREG feedback (module registred to a network)
extern int creg;
extern char cregC[2];
//CCLK feedback
extern cclk;
char modem_t[20];
//FTP PUT feedback:
extern int ftp;
extern char ftpPUT1[3];
extern char ftpPUT2[3];
extern char ftpLEN[4];

void USART6_IRQHandler(void) {
	if (USART_GetITStatus(USART6, USART_IT_RXNE)){
	        static int cnt = 0;
	        char ch3 = USART6 -> DR; //byte that was sent

	        if(ch3 != '\n'){
	            modem_rec_str[cnt++] = ch3;
	        }else{
	          modem_rec_str[cnt] = '\0';
	          cnt = 0;
	          if(transOn){
		          USART_puts(USART2,modem_rec_str);
	          }

	          //PARSING:
	          if(strncmp(modem_rec_str,"OK",2) == 0){
	        	  at_ok = 1;
	          }else if(strncmp(modem_rec_str,"ERROR",5) == 0){
	        	  at_err = 1;
	          }else if(strncmp(modem_rec_str,"+CREG",5) == 0){
	        	  creg = 1;
	        	  /*
	        	  for (int i = 0; i < strlen(modem_rec_str); i++){
	        		  cregC[i] = modem_rec_str[i];
	        	  }
	        	  */
	        	  cregC[0] = modem_rec_str[7];
	        	  cregC[1] = modem_rec_str[9];

	          }else if(strncmp(modem_rec_str,"+CCLK",5)==0){
	        	  cclk = 1;
	        	  for(int i = 2; i < 19; i++){
	        		  modem_t[i] = modem_rec_str[i+6];
	        	  }
	          }else if(strncmp(modem_rec_str,"+FTPPUT",7)==0){
	        	  ftp = 1;
	        	  //FTPPUT1
	        	  if(modem_rec_str[9] == '1'){
	        		  ftpPUT1[0] = modem_rec_str[9];
	        		  if(modem_rec_str[11] != '1' && modem_rec_str[11] != '0'){
	        			  ftpPUT1[1] = modem_rec_str[11];
	        			  ftpPUT1[2] = modem_rec_str[12];
	        		  }else if(modem_rec_str[11] == '1'){
	        			  ftpPUT1[1] = modem_rec_str[11];
	        			  ftpLEN[0] = modem_rec_str[13];
	        			  ftpLEN[1] = modem_rec_str[14];
	        			  ftpLEN[2] = modem_rec_str[15];
	        			  ftpLEN[3] = modem_rec_str[16];
	        		  }

	        	  }else if(modem_rec_str[9] == '2'){
	        		  ftpPUT2[0] = modem_rec_str[9];

	        	  }


	        	  USART_puts(USART2,modem_rec_str);

	          }



	        }
	}
}
