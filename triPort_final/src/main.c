#include "stdio.h" //Standardna knjižnica
#include "stm32f4xx.h" //Knjižnica za mikrokrmilnik STM32
#include "string.h" //Knjižnica za znakovne nize
#include "math.h" //Knjižnica s funkcijami in matematičnimi operatorji
#include "pwm.h"  //Knjižnica za rokovanje s časovniki
#include "tm_stm32f4_adc.h" //Knjižnica za rokovanje z analogno-digitalnimi pretvorniki:
//Dostopna na: https://stm32f4-discovery.net/download/tm_stm32f4_adc/

//SPREMENLJIVKE:===================================================
//DC MOTOR=========================================================
int newPos = 0; // nova pozicija gredi DC motorja

int startDC = 0; //Zastavica, ki skrbi, da se DC motor giblje takrat ko se sme

int encoderTick = 21; //Začetna vrednost prekinitev iz enkoderja - potrebno je nastavljanje glede na geometrijo mehanizma
float positionDC = 15.12; //Začetna pozicija gredi DC motorja - tudi potrebno nastaviti
int dutyDC;          //Delovni cikel pulzno-širinske modulacije na DC motorju
int minDTY= 30;      //Empirično dobljena minimalna vrednost delovnega cikla
int maxDTY = 100;     //Empirično dobljena maximalna vrednost delovnega cikla
float minDeg = 15.12; //Minimalna pozicija glede na geometrijo merilne postaje
float maxDeg = 344.88; //Maximalna pozicija glede na geometrijo merilne postaje
int minEncoder = 21; //Minimalna pozicija izražena v številu prekinitev
int maxEncoder = 479; //Maksimalna pozicija izražena v številu prekinitev

//PID
float DcKp = 0.01;  //Kp oz proporcionalni koeficient PID krmilnika
float DcKi = 0.01;  //Ki oz integralni koeficient PID krmilnika
float DcKd = 100;   //Kd oz diferencialni koeficient PID krmilnika
float intgrl = 0;   //Začetna vrednost integralnega dela
float deriv = 0;    //Začetna vrednost diferencialnega dela
float errorDC = 0;  //Trenutni odstopek
float prevErrorDC = 0; //Prejšnji odstopek
float ssError = 1;  //Odstopek v stacionarnem stanju
int dc_moveFlag = 0; //Zastavice, ki dovoljuje gibanje DC motorju

char tick[20];     //Znakovna spremenljivka za USART komunikacijo
//=================================================================
//Linearni aktuatorji==============================================
int prescalerS = 4000; // deljitelj osnovne frekvence časovnika
int periodS[4] = {500,500,500,500}; // perioda pulzno-širinske modulacije
int dutyS[4] = {50,50,50,50}; //Delovni cikel (ang. duty cycle) pulzno-širinske modulacije

int linDirS[4] = {0,0,0,0}; // Smeri linearnih aktuatorjev
// 0 - stop, 1- gor, 2- dol

float posF[4]; //Trenutna pozicija linearnega aktuatorja
float posFP[4]; //Prefiltrirana trenutna pozicija linearnega aktuatorja
float posF_avg[4]= {0,0,0,0};//Povprečje pozicije linearnega aktuatorja, za eliminacijo šumov
char pos[5]; //Spremenljivka za USART komunikacijo


float currF_init[4]; //Začetni tokovi na linearnih aktuatorjih
float currF[4]; //Trenutni tokovi na linearnih aktuatorjih
float currF_avg[4]; //Povprečje tokov
char curr[6]; //Spremenljivka za USART komunikacijo

float new_posF[4] = {0,0,0,0}; //Nova pozicija linearnega aktuatorja (vezana na upravljanje preko USART komunikacije)
int pos_diff[4]; //Razlika med željeno in trenutno pozicijo linearnega aktuatorja (pomembna pri implementaciji PID krmilnikov)

int la_n[4] = { 2863, 2948, 2828, 2859}; //Parameter surove digitalne vrednosti iz potenciometra linearnega aktuatorja (koeficient n v: Y= kx + n)
float la_k[4] = {4.0784,4.0974,4.0115, 4.0836}; //Parameter surove digitalne vrednosti iz potenciometra linearnega aktuatorja (koeficient k v: Y= kx + n)
float posBias[4] = {0.7, 0.9, 0.6 ,2.3}; //Izmerek pozicije pri dejanski ničelni poziciji linearnega aktuatorja; pozicijski bias

//PID Linearnih aktuatorjev=======================================
float dt = 0.003; // časovni inkrement: določen preko empiričnih meritev
float Kp[4] = {5,5,5,3}; //Kp oz proporcionalni koeficient PID krmilnika
float Ki[4] = {1,1,1,1}; // Ki oz integralni koeficient PID krmilnika
float Kd[4] = {1,1,1,100}; //Kd oz diferencialni koeficient PID krmilika
float integral = 0; //Začetna vrednost integralnega dela
float derivative = 0; //Začetna vrednost diferencialnega dela
float errorlast[4] = {0,0,0,0}; //Prejšna napaka
float sseLA = 2.5; //Odstopek v stacionarnem stanju
int act_moveFlag[4] = {1,1,1,1}; //Zastavice, ki dovoljujejo gibanje linearnim aktuatorjem
//=================================================================
//EMA algorithm (eksponentno plavajoče povprečje)==================
int srPos = 100; //Število meritev, ki jih analogno-digitalni pretvornik izvede zaporedno in iz njih določi vrednost potenciometrov
int sT_flag = 0; //Zastavica, ki označuje prvo meritev
float sT_prev[4]; //Prejšnja filtrirana vrednost
float alp[4] = {0.01, 0.01, 0.01, 0.01}; //Utež EMA algoritma za posamezni linearni aktuator
//=================================================================
//CO2 Merilno zazanavalo===========================================
int sensorFeedback_cnt = 0; //Števec dolžine odgovora iz merilnega zazanvala
int max_feedback_len = 7;   //Maksimalna dolžina odgovora merilnega zazanavala
int initCommand[8] = {0xFE, 0x41, 0x00, 0x60, 0x01, 0x35, 0xE8, 0x53} ; //Ukaz za začetek meritves
int readCommand[7] = {0xFE, 0x44, 0x00, 0x08, 0x02, 0x9F, 0x25}; //Ukaz za branje koncentracije CO2
int sensorFeedback[7]; //Odgovor senzorja
double co2Value = 0;   //Začetna vrednost CO2
//=================================================================
//SIM modul========================================================
int transOn = 1; // 1 - Direktna USART komunikacija z modulom, 0- ni komunikacije med uporabnikom in modulom
//Osnovni odgovori OK/ERROR
int at_ok = 0; //Zastavica za odgovor OK
int at_err = 0; //Zastavica za odgovor ERROR
//Ostali odgovori
int creg = 0;   //Zastavica za odgovor CREG?
int cclk = 0;       //Zastavica za odgovor na CCLK?
int ftp= 0;    //Odgovor na FTP ukaz

char cregC[2];  //Zapis odgovora na CREG?
char ftpPUT1[3]; //Zapis odgovora na ukaz FTP=1
char ftpPUT2[3]; //Zapis odgovora na ukaz FTP=2,<dolžina zapisa> ali FTP=2,0
char ftpLEN[4];  //Zapis odgovora na ukaz FTP=1, ki predstavlja dolžino zapisa na strežnik

//Zapis odgovora za čas
char modem_t[20];  //Celotni čas
char modem_yr[4];  //Leto
char modem_mnt[2]; //Mesec
char modem_day[2]; //Dan
char modem_h[2];   //Ura
char modem_min[2]; //Minuta
char modem_s[2];   //Sekunda
//=================================================================





//FUNKCIJE=========================================================
/*
 * Funkcija ki meri pozicijo in tok na linearnih aktuatorjih
 */
void pos_curr_measure(){
    //Pozicije linearnih aktuatorjev
    for(int i  = 0; i < 4; i++){
        posF_avg[i] = 0;
        currF_avg[i] = 0;
    }

    //Podatki iz potenciometrov(pozicija) in ACS712 čipov(tok)
    for(int i = 0; i < srPos; i++){
        //Pozicija
        posF[0] = TM_ADC_Read(ADC1, ADC_Channel_14);
        posF_avg[0] += posF[0];
        posF[1] = TM_ADC_Read(ADC1, ADC_Channel_15);
        posF_avg[1] += posF[1];
        posF[2] = TM_ADC_Read(ADC1, ADC_Channel_4);
        posF_avg[2] += posF[2];
        posF[3] = TM_ADC_Read(ADC1, ADC_Channel_5);
        posF_avg[3] +=posF[3];

        //Tok
        currF[0] = TM_ADC_Read(ADC1,ADC_Channel_12);
        currF_avg[0] += currF[0];
        currF[1] = TM_ADC_Read(ADC1,ADC_Channel_13);
        currF_avg[1] += currF[1];
        currF[2] = TM_ADC_Read(ADC1,ADC_Channel_10);
        currF_avg[2] += currF[2];
        currF[3] = TM_ADC_Read(ADC1,ADC_Channel_11);
        currF_avg[3] += currF[3];
    }

    for (int i = 0; i < 4; i++)
    {
        posF[i] = posF_avg[i]/srPos;
        currF[i] = currF_avg[i]/srPos;
    }

    //Povprečna pozicija
    for (int i = 0; i < 4; i++)
    {
        posF[i] = (float)((la_n[i] - posF[i])/(la_k[i])) - posBias[i];
        currF[i] = (float)(((currF_init[i] -  currF[i])/4096) * 3);
        currF[i] = (float)(currF[i]/0.1); //ACS712ELCTR-20A-T -> Občutljivost 100mV/A
        currF[i] = (currF[i] < 0) ? (-currF[i]) : (currF[i]);
    }

    for(int i = 0; i < 4; i++){
        if(sT_flag){
            posFP[i] = alp[i] * posF[i] + (1 - alp[i]) * sT_prev[i];
            sT_prev[i] = posFP[i];
        }else if(sT_flag == 0){
            posFP[i] = posF[i];
        }

    }
}

/*Funkcija, ki skrbi za premikanje in regulacijo linearnih aktuatorjev
 * act        - kateri aktuator želimo krmiliti
 * act_pos    - željena pozicija aktuatorja
 * table_stop - zastavica, ki dovoli oziroma zavira gibanje
 */
void act_move(int act, float act_pos, int table_stop){
    //pos_curr_measure();

    if(table_stop == 1){
        linAct1(dutyS[0],periodS[0],0);
        linAct2(dutyS[1],periodS[1],0);
        linAct3(dutyS[2],periodS[2],0);
        linAct4(dutyS[3],periodS[3],0);

    }else if(table_stop == 0){
        pos_diff[act] =  act_pos - posFP[act];
        if (pos_diff[act] < 0)
        {
            pos_diff[act] = - pos_diff[act];
        }

        //PID
        integral = (float) (integral + (pos_diff[act] * dt));
        derivative = (float)(pos_diff[act] - errorlast[act])/dt;
        dutyS[act] = (int)(pos_diff[act] * Kp[act] + Ki[act] * integral + Kd[act] * derivative);
        errorlast[act] = pos_diff[act];

        if (dutyS[act] < 25)
        {
            dutyS[act] = 25;
        }else if (dutyS[act] > 100)
        {
            dutyS[act] = 100;
        }

        if(act_pos > (posFP[act] + sseLA) ){
            switch(act){
                case 0 : linAct1(dutyS[act],periodS[act],2);
                break;

                case 1 : linAct2(dutyS[act],periodS[act],2);
                break;

                case 2 : linAct3(dutyS[act],periodS[act],2);
                break;

                case 3 : linAct4(dutyS[act],periodS[act],2);
                break;
            }
        }else if(act_pos < (posFP[act] - sseLA)){
            switch(act){
                case 0 : linAct1(dutyS[act],periodS[act],1);
                break;

                case 1 : linAct2(dutyS[act],periodS[act],1);
                break;

                case 2 : linAct3(dutyS[act],periodS[act],1);
                break;

                case 3 : linAct4(dutyS[act],periodS[act],1);
                break;
            }
        }else if((posFP[act] < (act_pos + sseLA)) && (posFP[act] > (act_pos - sseLA))){
            switch(act){
                case 0 : linAct1(dutyS[act],periodS[act],0);
                break;

                case 1 : linAct2(dutyS[act],periodS[act],0);
                break;

                case 2 : linAct3(dutyS[act],periodS[act],0);
                break;

                case 3 : linAct4(dutyS[act],periodS[act],0);
                break;
            }

        }
    }
}

/*
 * Funkcija, ki skrbi za pozicijsko kalibracijo linearnih aktuatorjev
 */
void act_cal(){
    //Začetna pozicija - krčenje
    USART_puts(USART2, "Kalibriranje mize\n\r");
    while(1){
        linAct1(100,periodS[0],1);
        linAct2(100,periodS[1],1);
        linAct3(100,periodS[2],1);
        linAct4(100,periodS[3],1);

        delay_ms(30000);

        USART_puts(USART2, "Zacetna pozicija dosezena\n\r");

        linAct1(100,periodS[0],0);
        linAct2(100,periodS[1],0);
        linAct3(100,periodS[2],0);
        linAct4(100,periodS[3],0);

        for(int i = 0; i < 4; i++){
            posF_avg[i] = 0;
        }

        for(int i = 0; i < 1000; i++){
            posF[0] = TM_ADC_Read(ADC1, ADC_Channel_14);

            posF[1] = TM_ADC_Read(ADC1, ADC_Channel_15);

            posF[2] = TM_ADC_Read(ADC1, ADC_Channel_4);

            posF[3] = TM_ADC_Read(ADC1, ADC_Channel_5);

            for (int i = 0; i < 4; i++)
            {
                posF_avg[i] += posF[i];
            }
        }

        for (int i = 0; i < 4; i++)
        {
            la_n[i] = posF_avg[i]/1000;
            sprintf(pos, "N = %d\n\r", la_n[i]);
            USART_puts(USART2, pos);
        }
        break;
    }
    //Končna pozicija - iztegovanje
    while(1){
        linAct1(100,periodS[0],2);
        linAct2(100,periodS[1],2);
        linAct3(100,periodS[2],2);
        linAct4(100,periodS[3],2);
        delay_ms(60000);

        USART_puts(USART2, "Koncna pozicija dosezena\n\r");
        linAct1(100,periodS[0],0);
        linAct2(100,periodS[1],0);
        linAct3(100,periodS[2],0);
        linAct4(100,periodS[3],0);

        for(int i = 0; i < 4; i++){
            posF_avg[i] = 0;
        }

        for(int i = 0; i < 1000; i++){
            posF[0] = TM_ADC_Read(ADC1, ADC_Channel_14);

            posF[1] = TM_ADC_Read(ADC1, ADC_Channel_15);

            posF[2] = TM_ADC_Read(ADC1, ADC_Channel_4);

            posF[3] = TM_ADC_Read(ADC1, ADC_Channel_5);

            for (int i = 0; i < 4; i++)
            {
                posF_avg[i] += posF[i];
            }
        }

        for (int i = 0; i < 4; i++)
        {
            posF[i] = posF_avg[i]/1000;
            sprintf(pos, "avg = %4.4f\n\r", posF_avg[i]);
            USART_puts(USART2, pos);

            la_k[i] = (float)((la_n[i] - posF[i])/(400));

            sprintf(pos, "K = %4.4f\n\r", la_k[i]);
            USART_puts(USART2, pos);
        }
        break;
    }

    USART_puts(USART2, "Aktuatorji kalibrirani.\n\r");
}

/*
 * Nastavitev periferije za USART1 - Merilno zazanvalo K30
 */
void setup_Periph1(){
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    //Ura za USART 1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    //Ura za GPIOB
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    //Nastavitev Rx in Tx pinov
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    USART_Cmd(USART1, ENABLE);
}

/*
 * Nastavitev periferije za USART2 - PC
 */
void setup_Periph2(){
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    //Ura za USART 2
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    //Ura za GPIOD
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    //Nastavitev Rx in Tx pinov
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; //PD6 Rx PD5 Tx
    USART_Init(USART2, &USART_InitStructure);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    USART_Cmd(USART2, ENABLE);
}

/*
 * Nastavitev periferije za USART3 - Modem SIM7000E
 */
void setup_Periph3(){
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//Ura za USART 6
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	//Ura za GPIOC
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	//Nastavitev Rx in Tx pinov
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; //PC6 Rx PC7 Tx
	USART_Init(USART6, &USART_InitStructure);
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	USART_Cmd(USART6, ENABLE);
}

/*
 * Fukcija ki skrbi za USART komunikacijo s PCjem
 */
void USART_puts(USART_TypeDef *USARTx, volatile char  *str){
    while(*str){
        while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
        USART_SendData(USARTx, *str);
        *str++;
    }
}

/*
 * Funkcija, ki skrbi za USART komunikacijo z merilnim zaznavalom K30
 */
void sensUSART_puts(USART_TypeDef *USARTx, volatile int num){
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); //until TC flag is one !(USARTx -> SR & 0x040)
            USART_SendData(USARTx, num);
}

/*
 * Funkcija, ki skrbi za USART komunikacijo z modemom SIM7000E
 */
void modemUSART_puts(USART_TypeDef *USARTx, volatile char  *modem_str){
    while(*modem_str){
        while(USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET);
        USART_SendData(USARTx, *modem_str);
        *modem_str++;
    }
}

/*
 * Iniciacija pinov radialnih končnih stikal
 * PB4 - Začetna lega
 * PB5 - Končna lega
 */
void endSwitch_GPIO(){
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*
 * Iniciacija pinov, ki vklaplajo merilno zaznavalo in ventilator
 * PC8 - Ventilator
 * PC9 - Merilno zazanavalo
 */
void fan_sensor_GPIO(){
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

}

/*
 * Iniciacija pinov za prižig modema SIM7000E
 * PD3 - Modem PWRKEY (2 sekundni pulz)
 * PD4 - Modem ON/OFF
 */
void modem_GPIO(){
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/*
 * Funkcija, ki izmeri eno vrednost toka (ACS712),
 * uporablja se na začetku, preden se kateri koli od
 * linearnih aktuatorjev začne gibati
 */
void init_curr_ADC(){
    //Tok
    TM_ADC_Init(ADC1, ADC_Channel_13);
    TM_ADC_Init(ADC1, ADC_Channel_12);
    TM_ADC_Init(ADC1, ADC_Channel_11);
    TM_ADC_Init(ADC1, ADC_Channel_10);
    for (int i = 0; i < 1000; i++)
    {
        currF_init[0] += TM_ADC_Read(ADC1,ADC_Channel_12);
        currF_init[1] += TM_ADC_Read(ADC1,ADC_Channel_13);
        currF_init[2] += TM_ADC_Read(ADC1,ADC_Channel_10);
        currF_init[3] += TM_ADC_Read(ADC1,ADC_Channel_10);
    }

    for(int i = 0; i < 4; i++){
        currF_init[i] = (float)(currF_init[i]/1000);
    }
}

/*
 * Iniciacija ure
 */
void systickInit(uint16_t frequency){
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);
    (void) SysTick_Config(RCC_Clocks.HCLK_Frequency / frequency);
}

/*
 * Takt ure
 */
static volatile uint32_t ticks;
void SysTick_Handler(void){
    ticks++;
}

/*
 * Funkcija ki vrne milisekunde od začetka programa
 */
uint32_t millis(void){
    return ticks;
}

/*
 * Zakasnitev, ki ne ugasne mikrokrmilnika.
 * Vseeno je boljše, če se ji da izogniti.
 */
void delay_ms(uint32_t t){
    uint32_t elapsed;
    uint32_t start = millis();
    do{
        elapsed = millis() - start;
    } while(elapsed < t);
}

/*
 * Nastavitev pinov za znanje prekinitve enkoderja
 */
void interrupt1(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);
    EXTI_InitStruct.EXTI_Line = EXTI_Line0;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_Init(&EXTI_InitStruct);
    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

/*
 * Iniciacija pina, ki skrbi za detekcijo smeri gredi
 * DC motorja (enkoder)
 */
void encoderDirection(void){
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/*
 * Ugašanje možnosti zunanjih prekinitev
 */
void disableInterrupt1(void){
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);
    EXTI_InitStruct.EXTI_Line = EXTI_Line0;
    EXTI_InitStruct.EXTI_LineCmd = DISABLE;
    NVIC_InitStruct.NVIC_IRQChannelCmd = DISABLE;
}

/*
 * Funkcija, ki skrbi za kalibracijo DC motorja
 */
void dcCalib(){
    int calibrationFlag = 0;
    positionDC = 0;
    encoderTick = 0;

    uint32_t start_t_calib = millis();
    uint32_t elapsed_t_calib;

    USART_puts(USART2, "Calibrating DC motor\r\n");
    //Smer urinega kazalca
    while(positionDC < 360){
        if(!GPIO_ReadInputDataBit(GPIOB, GPIO_PIN_4))
        {
            dcMove(minDTY, 0);
            positionDC = 0;
            encoderTick = 0;
            delay_ms(50);
            USART_puts(USART2, "DC motor calibrated!\r\n");
            calibrationFlag = 1;
            break;
        }else if(!GPIO_ReadInputDataBit(GPIOB, GPIO_PIN_5)){
            dcMove(minDTY, 0);
            positionDC = 360;
            encoderTick = 2096;
            delay_ms(50);
            USART_puts(USART2, "DC motor calibrated!\r\n");
            calibrationFlag = 1;
            break;
        }
        positionDC = (float)((360/2096) * encoderTick);
        dcMove(minDTY, 2);

        elapsed_t_calib = millis() - start_t_calib;
        if(elapsed_t_calib > 250){
            sprintf(tick, "%4.2f,", positionDC);
            USART_puts(USART2,tick);

            start_t_calib = millis();
        }
    }
    //Smer proti urinem kazalcu
    if(!calibrationFlag){
        positionDC = 360;
        encoderTick = 2096;

        while(positionDC > 0) {
            if(!GPIO_ReadInputDataBit(GPIOB, GPIO_PIN_4))
            {
                dcMove(minDTY, 0);
                positionDC = 0;
                encoderTick = 0;
                delay_ms(50);
                USART_puts(USART2, "DC motor calibrated!\r\n");
                calibrationFlag = 1;
                break;
            }else if(!GPIO_ReadInputDataBit(GPIOB, GPIO_PIN_5)){
                dcMove(minDTY, 0);
                positionDC = 360;
                encoderTick = 2096;
                delay_ms(50);
                USART_puts(USART2, "DC motor calibrated!\r\n");
                calibrationFlag = 1;
                break;
            }

            positionDC = (float)((360/2096) * encoderTick);
            dcMove(minDTY, 1);

            elapsed_t_calib = millis() - start_t_calib;
            if(elapsed_t_calib > 250){
                sprintf(tick, "%4.2f,", positionDC);
                USART_puts(USART2,tick);

                start_t_calib = millis();
            }
        }
    }

}

/*
 * Funkcija, ki skrbi za pozicioniranje gredi DC motorja
 */
void dcPosition(float positionSet){

    /*
     * Končna stikala:
     */
        if(!GPIO_ReadInputDataBit(GPIOB, GPIO_PIN_4))
            {
        		positionDC = minDeg;
        	    encoderTick = minEncoder;
                delay_ms(5);
            }else if(!GPIO_ReadInputDataBit(GPIOB, GPIO_PIN_5)){
            	positionDC = maxDeg;
            	encoderTick = maxEncoder;
                delay_ms(5);
        }

        positionDC = (float)((360.0/500.0) * encoderTick);
        errorDC = positionDC - positionSet;
        if((( errorDC < 0) && (errorDC > -ssError)) || ((errorDC > 0) && (errorDC < ssError))){ // Steady state error
            dcMove(dutyDC,0);
            delay_ms(50);
        }else{
            if((errorDC < 0) && (errorDC <= (- ssError))){

                intgrl = intgrl + (-errorDC * 0.0002);
                deriv = (-errorDC + prevErrorDC)/0.0002;
                dutyDC = (int)(DcKp * (-errorDC) + DcKi * intgrl + DcKd * deriv);

                if(dutyDC < minDTY){
                    dutyDC = minDTY;
                }else if(dutyDC > 100){
                    dutyDC = 100;
                }
                dcMove(dutyDC, 1);

            }else if((errorDC > 0) && (errorDC >= ssError)){

                intgrl = intgrl + (errorDC * 0.0002);
                deriv = (errorDC - prevErrorDC)/0.0002;
                dutyDC = (int)(DcKp * (errorDC) + DcKi * intgrl+ DcKd * deriv);

                if(dutyDC < minDTY){
                    dutyDC = minDTY;
                }else if(dutyDC > 100){
                    dutyDC = 100;
                }
                dcMove(dutyDC, 2);
            }
        }
    prevErrorDC = errorDC;
}

/*
 * Začetek meritve
 * fan = 0 -> Ventilator je ugasnjen
 * fan = 1 -> Ventilator je prižgan
 */
void initOneMeasure(int fan){
	//Merilno zaznavalo se prižge
	TM_GPIO_SetPinHigh(GPIOC, GPIO_Pin_9);
    USART_puts(USART2,"Zacenjam inicializacijo meritev\n\r");
    //Pošiljanje ukaza za začetek meritve:
    for(int i = 0; i < 8; i++){
        sensUSART_puts(USART1, initCommand[i]);
    }
    uint16_t init_start = millis();
    uint16_t init_t;
    int sensConnected = 0;

    while(init_t < 100){
        if((sensorFeedback[0] == initCommand[0]) && (sensorFeedback[1] == initCommand[1])){
            sensConnected = 1;
        }

        init_t = millis() - init_start;
    }
    if(sensConnected){
        USART_puts(USART2, "Povezana\n\r");
        sensorFeedback_cnt = 0;
        max_feedback_len = 7;

        for(int i = 0; i < 7; i++){
            sensorFeedback[i] = 0;
        }
    }

    delay_ms(20000);
    USART_puts(USART2, "Meritve se lahko zacnejo\n\r");
	if(fan){
		TM_GPIO_SetPinHigh(GPIOC, GPIO_Pin_8);
	}else if(!fan){
		TM_GPIO_SetPinLow(GPIOC, GPIO_Pin_8);
	}

}

/*
 * Branje vrednosti koncentracije iz zaznavala
 */
void readSensVal(){
    sensorFeedback_cnt = 0;
    for(int i = 0; i < 7; i++){
        sensUSART_puts(USART1,readCommand[i]);
    }
    uint32_t measure_start = millis();
	uint32_t measure_t;
	int measureOK = 0;
    delay_ms(100);
	if((sensorFeedback[0] == readCommand[0]) && (sensorFeedback[1] == readCommand[1]) ){
		measureOK = 1;
	}
	if(measureOK){
        co2Value = sensorFeedback[3]*256 + sensorFeedback[4];
	}else{
		char feed[4];
		for(int i = 0; i < 7; i++){
			   sprintf(feed,"%d,",sensorFeedback[i]);
		}

	   for(int i = 0; i < 7; i++){
            sprintf(feed,"%d,",readCommand[i]);
		}
	   if((sensorFeedback[0] == readCommand[0]) && (sensorFeedback[1] == readCommand[1]) ){
	   			co2Value = sensorFeedback[3]*256 + sensorFeedback[4];
	   	}
	}
   for(int i = 0; i < 7; i++){
	   sensorFeedback[i] = 0;
   }

}

/*
 * Funkcija za prižiganje in ugašanje modema SIM7000E
 */
void modemOn(int on){
	if(on){
		//Modem power on:
		TM_GPIO_SetPinHigh(GPIOD, GPIO_Pin_3);
		TM_GPIO_SetPinHigh(GPIOD, GPIO_Pin_4);
		delay_ms(2000);
	}else if(!on){
		//Modem power on:
	    TM_GPIO_SetPinLow(GPIOD, GPIO_Pin_3);
	    TM_GPIO_SetPinLow(GPIOD, GPIO_Pin_4);
	    delay_ms(200);
	}
	delay_ms(10000);
    modemUSART_puts(USART6,"ATE0\n\r");
}

/*
 * Funkcija za osnovno preverjanje odgovorov modula.
 * Funkcija preveri samo odgovore OK/ERROR.
 * Uporabnik kot prvi parameter poda AT ukaz,
 * drugi parameter predstavlja izpis ob prejetju OK,
 * tretji pa ob prejetju ERROR odgovora
 */
void modemFeedbackCheck(volatile char  *cmdAT, volatile char *okAT, volatile char *errAT){
	at_ok = 0;
	at_err = 0;
	//Pošiljanje ukaza
	while(*cmdAT){
		while(USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET); //until TC flag is one !(USARTx -> SR & 0x040)
		USART_SendData(USART6, *cmdAT);
		*cmdAT++;
	}
	//Čakanje na odgovor OK ali ERROR
	while(!at_ok || !at_err){
		if(at_ok){
			while(*okAT){
	            while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET); //until TC flag is one !(USARTx -> SR & 0x040)
	            USART_SendData(USART2, *okAT);
	            *okAT++;
	        }
			break;
		}else if(at_err){
			while(*errAT){
				while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET); //until TC flag is one !(USARTx -> SR & 0x040)
				USART_SendData(USART2, *errAT);
				*errAT++;
			}
			break;
		 }
	}
}

/*
 * Funkcija za nastavljanje APN profila izbranega ponudnika interneta
 */
void modemAPN(){
    //Checking if module is registred to a network:
	//USART_puts(USART2,"Is modem registred to a network:\n\r");
	creg = 0;
	modemUSART_puts(USART6,"AT+CREG?\r\n");


	while(!creg){
		//wait
	}

	//Checking the first digit
	switch (cregC[0]) {
		case '0':
			USART_puts(USART2,"0 - Disable network registration unsolicited result code\r\n");
			break;
		case '1':
			USART_puts(USART2,"1 - Enable network registration unsolicited result code\r\n");
			break;
		case '2':
			USART_puts(USART2,"2 - Enable network registration unsolicited result code with location information\r\n");
			break;
		default:
			USART_puts(USART2, "Error\r\n");
			break;
	}

	//Checking the second digit
	switch (cregC[1]) {
		case '0':
			USART_puts(USART2,"0 - Not registered, MT is not currently searching a new operator to register to\r\n");
			break;
		case '1':
			USART_puts(USART2,"1 - Registered, home network\r\n");
			break;
		case '2':
			USART_puts(USART2,"2 - Not registered, but MT is currently searching a new operator to register to\r\n");
			break;
		case '3':
			USART_puts(USART2,"3 - Registration denied\r\n");
			break;
		case '4':
			USART_puts(USART2,"4 - Unknown\r\n");
			break;
		case '5':
			USART_puts(USART2,"5 - Registered, roaming\r\n");
			break;
		default:
			USART_puts(USART2, "Error\r\n");
			break;
	}

    //Configuring bearer profile 1
    modemFeedbackCheck("AT+SAPBR=3,1,CONTYPE,GPRS\r\n","Configuring bearer profile 1 OK\r\n" ,"Error with configuring bearer profile 1\r\n" );
    //Configuring access point:
    modemFeedbackCheck("AT+SAPBR=3,1,APN,em\r\n","Access point OK\r\n","Error with access point\r\n");
	//Configuring user and password:
    modemFeedbackCheck("AT+SAPBR=3,1,USER,''\r\n","User settings OK\r\n","Error with user settings\r\n");
    modemFeedbackCheck("AT+SAPBR=3,1,PWD,''\r\n","Password settings OK\r\n","Error with password settings\r\n");

}

/*
 * Odpiranje povezave do mobilnega interneta
 * open = 1 -> odpritanje, 0 -> zapiranje
 */
void openAPN(int open){
	if(open){
		modemFeedbackCheck("AT+SAPBR=1,1\r\n","Bearer opened\r\n","Error opening the bearer\r\n");
	}else if(!open){
		modemFeedbackCheck("AT+SAPBR=0,1\r\n","Bearer closed\r\n","Error closing the bearer\r\n");
	}
}

/*
 * Funkcija za pridobivanje ure iz NTP strežnika
 */
void modemCLK(){
	cclk = 0;
	modemUSART_puts(USART6,"AT+CCLK?\r\n");

	while(!cclk){
		//Wait
	}
	//Year:
	modem_yr[0] = '2';
	modem_yr[1] = '0';
	modem_yr[2] = modem_t[2];
	modem_yr[3] = modem_t[3];
	//Month:
	modem_mnt[0] = modem_t[5];
	modem_mnt[1] = modem_t[6];
	//Day:
	modem_day[0] = modem_t[8];
	modem_day[1] = modem_t[9];
	//Hour:
	modem_h[0] = modem_t[11];
	modem_h[1] = modem_t[12];
	//Minute;
	modem_min[0] = modem_t[14];
	modem_min[1] = modem_t[15];
	//Second:
	modem_s[0] = modem_t[17];
	modem_s[1] = modem_t[18];
	//USART_puts(USART2,modem_t);
	//USART_puts(USART2,"\r\n");

}

/*
 * Funkcija za dostop do FTP strežnika in
 * zapisovanje podatkov.
 * Sprejme spremenljivko flux, ki prestavlja preračunani tok CO2
 */
void FTP(double flux){
	//Opening bearer
	modemAPN();
	openAPN(1);
	//Use bearer profile 1 for FTP connection
	modemFeedbackCheck("AT+FTPCID=1\r\n","Bearer profile 1 chosen\r\n","Bearer profile 1 error\r\n");
	//Server adress for the FTP transmission:
	modemFeedbackCheck("AT+FTPSERV=193.2.23.31\r\n","Server address set\r\n","Server address error\r\n");
	//Username:
	modemFeedbackCheck("AT+FTPUN=meteo_len\r\n","User name set\r\n","User name error\r\n");
    //Password
	modemFeedbackCheck("AT+FTPPW=bH1qqUI\r\n","Password set\r\n","Password error\r\n");
    //Path ANTON/
	modemFeedbackCheck("AT+FTPPUTPATH=/ANTON/\r\n","Path set\r\n","Path error\r\n");
    //Upload file filename:
	modemCLK();
	char flnmAT[40];
	strcpy(flnmAT,"AT+FTPPUTNAME=");
	strcat(flnmAT,"000_T_");
	strcat(flnmAT,"RS_");
	strcat(flnmAT,modem_yr);
	strcat(flnmAT,modem_mnt);
	strcat(flnmAT,modem_day);
	strcat(flnmAT,modem_h);
	strcat(flnmAT,modem_min);
	strcat(flnmAT,modem_s);
	strcat(flnmAT,".txt\r\n");
	//USART_puts(USART2,flnmAT);
	modemFeedbackCheck(flnmAT,"File name ok\r\n","File name error\r\n");

    //FTP mode 0- active, 1 -passive
	modemFeedbackCheck("AT+FTPMODE=1\r\n","Mode ok\r\n","Mode error\r\n");


    //FTP upload
    USART_puts(USART2,"Begining seassion:\n\r");
    ftp = 1; //0
    //modemUSART_puts(USART6,"AT+FTPPUT=1\n\r"); //Begining the seassion
    while(!ftp){
    	//Wait
    }
    if(ftpPUT1[0] == '1'){
    	if(ftpPUT1[1] == '1'){
        	USART_puts(USART2,"Seassion started OK\r\n");
    	}else if(ftpPUT1[1] != '1'){
        	USART_puts(USART2,"Seassion did not start\r\n");
        	switch (ftpPUT1[1]) {
				case '6':
					switch (ftpPUT1[2]) {
						case '1':
							USART_puts(USART2,"Net error\r\n");
							break;
						case '2':
							USART_puts(USART2,"DNS error\r\n");
							break;
						case '3':
							USART_puts(USART2,"Connection error\r\n");
							break;
						case '4':
							USART_puts(USART2,"Timeout\r\n");
							break;
						case '5':
							USART_puts(USART2,"Server error\r\n");
							break;
						case '6':
							USART_puts(USART2, "Operation not allowed\r\n");
							break;
					}

					break;
				case '7':
					switch (ftpPUT1[2]) {
						case '0':
							USART_puts(USART2,"Reply error\r\n");
							break;
						case '1':
							USART_puts(USART2,"User error\r\n");
							break;
						case '2':
							USART_puts(USART2,"Password error\r\n");
							break;
						case '3':
							USART_puts(USART2,"Type error\r\n");
							break;
						case '4':
							USART_puts(USART2, "Rest error\r\n");
							break;
						case '5':
							USART_puts(USART2,"Passive error\r\n");
							break;
						case '6':
							USART_puts(USART2,"Active error\r\n");
							break;
						case '7':
							USART_puts(USART2,"Operate error\r\n");
							break;
						case '8':
							USART_puts(USART2,"Upload error\r\n");
							break;
						case '9':
							USART_puts(USART2,"Download error\r\n");
							break;

					}
					break;

			}
    	}
    }

	//The main message
	//float t[20] = {21.80,21.80,21.80,21.80,21.45,21.77,21.77,21.88,21.88,21.88,21.45,21.45,21.45,21.45,21.77,21.88,21.88,21.77,21.77,21.77};
    //float f[20] = {1.46,2.78,0.18,1.80,2.18,2.29,0.57,2.36,0.97,1.91,0.54,1.88,0.37,0.17,0.54,-0.84,1.40,0.04,0.60,1.06};
	float t = 21.8;

	//modemCLK();
	char upload[265];
	for(int i = 0; i < 256;i++){
		upload[i] = '\0';
	}
	strcat(upload,"v0.1\r\n*"); //header
	strcat(upload,modem_yr);
	strcat(upload,modem_mnt);
	strcat(upload,modem_day);
	strcat(upload,modem_h);
	strcat(upload,modem_min);
	strcat(upload,modem_s);
	strcat(upload,"\t000\tRS\tRS_000\t\t\t\r\n");


	for(int i = 0; i < 5;i++){
		char msg[40];
		sprintf(msg,"T\t%d\t%3.1f\r\nP\t\%d\t1013\r\nF_CO2_g\t%d\t%3.2f\r\n",i,t,i,i,flux);
		strcat(upload,msg);
	}


    USART_puts(USART2,"Uploading:\n\r");
	/*
    char ftpput2[15];
	strcat(ftpput2,"AT+FTPPUT=2,");

    int uploadLEN = strlen(upload) - 12; //AT+FTPPUT=2, - allways ends up in the message
	char lenC[5];
	sprintf(lenC,"%d\r\n",uploadLEN);
	strcat(ftpput2,lenC);
	modemUSART_puts(USART6,ftpput2);

	ftp = 0;
	while(!ftp){
	//Wait
	}

	at_ok = 0;
	*/
	//modemUSART_puts(USART6,upload);
	USART_puts(USART2,upload);

	//FTP end transmission:
	/*
	while(!at_ok){
		//
	}
	*/

	USART_puts(USART2,"\r\nEnd of transmission:\r\n");
	delay_ms(1000);
	//modemUSART_puts(USART6,"AT+FTPPUT=2,0\r\n"); //Ending the seassion

	delay_ms(100);


	//Closing bearer
	openAPN(0);

}

/*
 * Glavna funkcija programa
 */
main(){
    //Začetne inicializacije in nastavljanje
    endSwitch_GPIO();
    modem_GPIO();

    setup_Periph1();
    setup_Periph2();
    setup_Periph3();
    systickInit(1000); // 1000 - millisekund
    fan_sensor_GPIO();

    //Ventilator in merilno zaznavalo sta ugasnjena
    TM_GPIO_SetPinLow(GPIOC, GPIO_Pin_8);
    TM_GPIO_SetPinLow(GPIOC, GPIO_Pin_9);

    //==PWM=====
    init_pwm_GPIO();

    pwmOutInit1(); //Linear Aktuator 1
    pwmOutInit2(); //Linear Aktuator 2
    pwmOutInit3(); //Linear Aktuator 3
    pwmOutInit4(); //Linear Aktuator 4

    timer2Init(periodS[0],prescalerS);
    timer3Init(periodS[0],prescalerS);
    timer9Init(periodS[0],prescalerS);

    //DC motor:
    pwmOutInitDC();
    timer4Init();

    //Prižig časovnikov za PWM
    timer2Start();
    timer3Start();
    timer4Start();
    timer9Start();

    //ADC za prvo meritev toka:
    init_curr_ADC();

    //Pozicija linearnih aktuatorjev
    TM_ADC_Init(ADC1, ADC_Channel_14);
    TM_ADC_Init(ADC1, ADC_Channel_15);
    TM_ADC_Init(ADC1, ADC_Channel_4);
    TM_ADC_Init(ADC1, ADC_Channel_5);

    //Prekinitve:
    interrupt1();
    encoderDirection();

    //USART_puts(USART2, "Human I await your command:\r\n");
    USART_puts(USART2,"Osnovne nastavitve končane\r\n");

    //Prva meritev za EMA algoritem
    pos_curr_measure();
    for(int i = 0; i < 3; i++){
        sT_prev[i] = posFP[i];
    }
    sT_flag = 1;


    //Spremenljivke za merilno zaznavalo
    //initOneMeasure(0);
    int sec = 0; //Sekunde
    int measure = 0; //Števec meritev
    int measureLength = 180; //Dolžina meritev
    double co2Values[measureLength + 1]; //Vektorski zapis meritev
    double time[measureLength + 1];//Vektorski zapis časa
    char valueC[12]; //Znakovna spremenljivka, ki služi izpisu vrednosti meritev

    //Spremenljivke pozicije
    float ac_legs[3][2] = {{15,45},{135,165},{260,290}}; //Pozicije nog
    float dc_test_positions[9] = {50, 85, 120, 170, 205, 240, 290, 315,340}; //Prednastavljena merilna mesta
    float act_test_positionsL[9] ={0,0,0,0,0,0,0,0,0}; //Prednastavljene spodnje pozicije komore
    float act_test_positionsH[10] ={210,30,30,210,30,30,210,30,30,30}; //Prednastavljene zgornje pozicije komore
    float act_test_radial_positions[2][3] = {{0,100,200},{200,100,0}}; //Prednastavljene radialne pozicije merilnih mest
    int test_position = 0; //Števec merilnih mest
    int test_dir = 0;  //Zastavica smeri poteka meritev
	int radialDirection = 0; //Smer poteka meritev

	//Opcija kalibracije linearnih aktuatorjev
	//act_cal();

	//Končna stikala na merilni komori
	TM_GPIO_Init(GPIOC, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_DOWN, TM_GPIO_Speed_Medium);
	//Dvigovanje komore na varno pozicijo:
	USART_puts(USART2, "Moving table to the safe position!\n\r");
	uint32_t start_t = millis();
	uint32_t elapsed_t;

	//Nastavitev zastavic, ki dovolijo gibanje linearnim aktuatorjem
	for(int i = 0; i < 4; i++){
		act_moveFlag[i] = 1;
	}

    //Premik vseh linearnih aktautorjev
    while(act_moveFlag[0] || act_moveFlag[1] || act_moveFlag[2]|| act_moveFlag[3]){
	    pos_curr_measure();
	    act_move(3,0,0);
		for(int i = 0; i < 3; i++){
		    act_move(i,210,0);
		}
	}

    //Možnost kalibracije DC motorja
    //dcCalib();

	//DC motor je kalibriran, aktuatorji se lahko spet gibljejo
	for(int i = 0; i < 3; i++){
		act_moveFlag[i] = 1;
	}

    /*
     * Ob uporabi funkcije pošiljanja na
     * FTP strežnik, je treba ta del odkomentirati.
     */
    //modemOn(1);

     /*
      *
      *
     * Glavna zanka programa =====================================================================
     */
    while(1){

    	//Dvig vertikalnih aktuatorjev
		while(act_moveFlag[0] || act_moveFlag[1] || act_moveFlag[2]) {
			pos_curr_measure();
			for(int i = 0; i < 3; i++){
				act_move(i,210,0);
			}
		}

		/*
		 * Ko vsi trije aktautorji dosežejo željeno pozicijo, se postavi
		 * zastavica za premik DC motorja
		*/
        dc_moveFlag = 1;
        while(dc_moveFlag){
		    dcPosition(dc_test_positions[test_position]);
	    }

        /*
         * DC motor je dosegel željeno pozicijo,
         * aktuatorji se lahko spustijo do tal
         */
        dc_moveFlag = 0;
        int radialTests = 0; // Prvi izmed 3 redialnih testov

        while(radialTests < 3){
        	/*
        	 * Določanje pozicije prve radialne
        	 * meritve
        	 */
			if(posF[3] < 10){
				radialDirection = 0;
			}else if(posF[3] > 190){
				radialDirection = 1;
			}

			act_moveFlag[3] = 1;

			start_t = millis();
			/*
			 * Premik horizontalnega linearnega aktuatorja
			 */
			while(act_moveFlag[3]) {
				pos_curr_measure();
				act_move(3, act_test_radial_positions[radialDirection][radialTests], 0);
			}

			/*
			 * Ko horizontalni linearni aktuator doseže željeno pozicijo,
			 * se lahko gibljejo vertikalni trije
			 */
			for (int i = 0; i < 3; i++)
			{
				act_moveFlag[i] = 1;
			}

			while(act_moveFlag[0] || act_moveFlag[1] || act_moveFlag[2]) {
				pos_curr_measure();
				for(int i = 0; i < 3; i++){
					act_move(i, act_test_positionsL[test_position], 0);
            }

				/*
				 * Preverjanje sklenjenosti končnih stikal na komori
				 */
				if(!GPIO_ReadInputDataBit(GPIOB, GPIO_PIN_3) || !GPIO_ReadInputDataBit(GPIOB,GPIO_PIN_6) || !GPIO_ReadInputDataBit(GPIOB,GPIO_PIN_7))
				{
					for(int i = 0; i < 3; i++){
						act_moveFlag[i] = 0; //linearni aktuatorji se ne morejo več gibati
						act_move(i, act_test_positionsL[test_position], 1);
					}
					break;
				}
			}
			/*
			 * Vmesni dvig vertikalnih aktuatorjev
			 */
			for(int i = 0; i < 3; i++){
				act_move(i,50,1);
			}
			delay_ms(150);

			/*
			 * Meritve koncentracije CO2
			 */

			uint32_t startCO_t = millis();
			uint32_t elapsedCO_t;
			int sec;

			if(elapsedCO_t > 1000 && sec < measureLength){
				readSensVal();
				co2Values[measure] = co2Value;
				time[measure] = measure;
				sprintf(valueC, "%lf,", co2Value);
				USART_puts(USART2, valueC);
				measure++;
				sec++;
				startCO_t = millis();

			}else if(sec == measureLength){
				TM_GPIO_SetPinLow(GPIOC, GPIO_Pin_8);
				TM_GPIO_SetPinLow(GPIOC, GPIO_Pin_9);
				measure = 0;

				/*
				 * Izpis koncentracije
				 */
				char valueOut[4];

				for(int i = 0; i < measureLength; i++){
					sprintf(valueOut,"%lf,",co2Values[i]);
					USART_puts(USART2, valueOut);
				}

				sec++;
			}

			/*
			 * Konec meritev koncentracije
			 */
			//Dvig aktutorjev
			 for(int i = 0; i < 3; i++){
				 act_moveFlag[i] = 1;
			 }

			while(act_moveFlag[0] || act_moveFlag[1] || act_moveFlag[2]) {
				pos_curr_measure();
				for(int i = 0; i < 3; i++){
					act_move(i, 30, 0);
				}
		   }

			/*
			 * Inkrement radialnih meritev:
			 */
			radialTests++;
			if(radialTests == 3){
				break;
			}
		}

		/*
		 * Zastavica za dvg vertikalnih aktuatorjev se postavi
		 */
		for (int i = 0; i < 3; i++)
		{
			act_moveFlag[i] = 1;
		}

		/*
		 * Določevanje naslednjega merilnega mesta
		 */
		if(test_dir == 0){
			prev_test_point = dc_test_positions[test_position];
			test_position++;
			next_test_point = dc_test_positions[test_position];
			if(test_position > 8){
				test_position = 7;
				test_dir = 1;
				//dc_cal(1);
			}
		}else if(test_dir == 1){
			prev_test_point = dc_test_positions[test_position];
			test_position--;
			next_test_point = dc_test_positions[test_position];
			if(test_position < 1){
			test_position = 0;
			test_dir = 0;
			//dc_cal(0);
			}
		}
    }
    return 0;
}
