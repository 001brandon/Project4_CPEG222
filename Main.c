/*===================================CPEG222=================================
 * Program:        CPEG222_project_4
 * Authors:     Brandon Brooks
 * Date:         10/26/2019

 =============================================================================*/
/*------------------------ System setting part please DO NOT CHANGE ---------*/
#ifndef _SUPPRESS_PLIB_WARNING          //suppress the plib warning during compiling
    #define _SUPPRESS_PLIB_WARNING
#endif

#include <xc.h>   //Microchip XC processor header which links to the PIC32MX370512L header
#include "config.h" // Basys MX3 configuration header
#include "ssd.h"
#include "lcd.h"
#include "led.h"
#include "utils.h"
#include "btn.h"
#include <plib.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#pragma config OSCIOFNC =	ON
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config FPLLODIV = DIV_1         // System PLL Output Clock Divider (PLL Divide by 1)
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config POSCMOD = XT             // Primary Oscillator Configuration (XT osc mode)
#pragma config FPBDIV = DIV_8           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/8)

/*-------------------------------------Definitions Needed-----------------------*/
#define PB_CLK       10000000   //Speed of PB_CLK


#define  LED0  LATAbits.LATA0      //LEDS
#define  LED1  LATAbits.LATA1
#define  LED2  LATAbits.LATA2
#define  LED3  LATAbits.LATA3
#define  LED4  LATAbits.LATA4
#define  LED5  LATAbits.LATA5
#define  LED6  LATAbits.LATA6
#define  LED7  LATAbits.LATA7
#define SW0    PORTFbits.RF3    //SWITCHES
#define SW1    PORTFbits.RF5
#define SW2    PORTFbits.RF4
#define SW3    PORTDbits.RD15
#define SW4    PORTDbits.RD14
#define SW5    PORTBbits.RB11
#define SW6    PORTBbits.RB10
#define SW7    PORTBbits.RB9
#define BTNC PORTFbits.RF0
#define BTNR PORTBbits.RB8




/*-------------------------------------Functions Needed-----------------------*/
void delay_ms(int ms);
void __ISR(_TIMER_2_VECTOR,ipl7SOFT) T2_Handler(void);
void configLEDS();
void configOC2();
void Timer2Setup();
void configSwitches();
void configBTNS();
unsigned int MICval();
void initMIC();
unsigned int AnalogRead(unsigned char analogPIN);
void initADC();







/*-------------------------------------Variables Needed-----------------------*/
int mode =0;                   //State machine mode
int alternator=0;              //Used to toggle LEDS to test ISR
int T2count=0;
int counter=0;
int PWM_PERIOD;
float dutyCycle;            //Duty Cycle
int sampleCount=0;
int buttonLock=0;
int playBack=0;
char buffer[96];
int sampleFlag=0;
float displayNum=0;
float tempDisplayNum=0;
int maxCounter=0;


/*---------------------------------------Array's------------------------------*/
int SW0waveForm[26]={0x200,0x27f,0x2f6,0x35e,0x3af,0x3e6,0x3fe,0x3f6,0x3ce,0x38a,0x32c,0x2bc,0x240,0x1bf,0x143,0xd3,0x75,0x31,0x9,0x1,0x19,0x50,0xa1,0x109,0x180};
int SW1waveForm[23]={0x200,0x290,0x314,0x382,0x3d1,0x3fa,0x3fa,0x3d1,0x382,0x314,0x290,0x200,0x16f,0xeb,0x7d,0x2e,0x5,0x5,0x2e,0x7d,0xeb,0x16f,0x1ff};
int SW2waveForm[22]={0x200,0x296,0x320,0x38f,0x3dc,0x3fe,0x3f2,0x3ba,0x35b,0x2dd,0x24c,0x1b3,0x122,0xa4,0x45,0xd,0x1,0x23,0x70,0xdf,0x169,0x200};
int SW3waveForm[20]={0x200,0x2a6,0x33a,0x3ac,0x3ef,0x3fd,0x3d4,0x378,0x2f3,0x254,0x1ab,0x10c,0x87,0x2b,0x2,0x10,0x53,0xc5,0x159,0x200};
int SW4waveForm[18]={0x200,0x2b8,0x358,0x3c9,0x3fd,0x3eb,0x398,0x30d,0x25d,0x1a2,0xf2,0x67,0x14,0x2,0x36,0xa7,0x147,0x200};
int SW5waveForm[17]={0x200,0x2c3,0x369,0x3d8,0x3ff,0x3d8,0x369,0x2c3,0x200,0x13c,0x96,0x27,0x0,0x27,0x96,0x13c,0x200};
int SW6waveForm[15]={0x200,0x2dd,0x38f,0x3f2,0x3f2,0x38f,0x2dd,0x200,0x122,0x70,0xd,0xd,0x70,0x122,0x200};
int SW7waveForm[14]={0x200,0x2ed,0x3a4,0x3fb,0x3de,0x353,0x27a,0x185,0xac,0x21,0x4,0x5b,0x112,0x200};
unsigned short int Samples[55001]={};


int main(void){
    configLEDS();
    Timer2Setup();
    configOC2();
    configSwitches();
    LCD_Init();
    configBTNS();
    initMIC();
    initADC();
    
    
    UART_Init(9600); 
    UART_PutString("UART Demo \n\r");
    LCD_WriteStringAtPos("1: Play Tone",0,2);
    
    while(1){
        //LED2=1;
        delay_ms(500);
        //LED2=0;
        delay_ms(500);
    }
    //take sample divide 511 then multiple duty cycle by PRX
    //
}

void __ISR(_TIMER_2_VECTOR,ipl7SOFT) T2_Handler(void){        
    IEC0bits.T2IE =0;     //Disables interrupt
    switch(mode){
        case 0:
        if(SW0){                                                    //Evaluates Switches
            float dutyCycle=SW0waveForm[((++counter))%25];
            OC2RS=(int)((PR2 +1)*((float)dutyCycle/1023));
            LED0=1,LED1=LED2=LED3=LED4=LED5=LED6=LED7=0;
        }
        else if(SW1){
            float dutyCycle=SW2waveForm[((++counter))%22];
            OC2RS=(int)((PR2 +1)*((float)dutyCycle/1023));
            LED1=1,LED0=LED2=LED3=LED4=LED5=LED6=LED7=0;
        }
        else if(SW2){
            float dutyCycle=SW2waveForm[((++counter))%21];
            OC2RS=(int)((PR2 +1)*((float)dutyCycle/1023));
            LED2=1,LED1=LED0=LED3=LED4=LED5=LED6=LED7=0;
        }
        else if(SW3){
            float dutyCycle=SW2waveForm[((++counter))%19];
            OC2RS=(int)((PR2 +1)*((float)dutyCycle/1023));
            LED3=1,LED1=LED2=LED0=LED4=LED5=LED6=LED7=0;
        }
        else if(SW4){
            float dutyCycle=SW4waveForm[((++counter))%17];
            OC2RS=(int)((PR2 +1)*((float)dutyCycle/1023));
            LED4=1,LED1=LED2=LED3=LED0=LED5=LED6=LED7=0;
        }
        else if(SW5){
            float dutyCycle=SW5waveForm[((++counter))%16];
            OC2RS=(int)((PR2 +1)*((float)dutyCycle/1023));
            LED5=1,LED1=LED2=LED3=LED4=LED0=LED6=LED7=0;
        }
        else if(SW6){
            float dutyCycle=SW6waveForm[((++counter))%14];
            OC2RS=(int)((PR2 +1)*((float)dutyCycle/1023));
            LED6=1,LED1=LED2=LED3=LED4=LED5=LED0=LED7=0;
        }
        else if(SW7){
            float dutyCycle=SW7waveForm[((++counter))%13];
            OC2RS=(int)((PR2 +1)*((float)dutyCycle/1023));
            LED7=1,LED1=LED2=LED3=LED4=LED5=LED6=LED0=0;
        }
        else{
            OC2RS=0;
            LED0=LED1=LED2=LED3=LED4=LED5=LED6=LED7=0;
        }
        if(BTNC&&!buttonLock){                  //Mode change
            mode=1;
            counter=0;
            LCD_WriteStringAtPos("                   ",0,0);
            LCD_WriteStringAtPos("                   ",1,0);
            LCD_WriteStringAtPos("2: Voice Record",0,1);
            buttonLock=1;
            OC2RS=0;
            LED0=LED1=LED2=LED3=LED4=LED5=LED6=LED7=0;
        }
        if(!BTNR&&buttonLock&&!BTNC){           //Button Lock
                buttonLock=0;
                int i=0;
                for(i;i<10000;i++);
        }
        break;
        case 1:   //RECORD
            if(BTNR&&!buttonLock){      //While holding R increments the counter and adds to array
                if(sampleFlag){            //Resets sample array is pressed R before
                    displayNum=0;
                    sampleCount=0;
                    int i=0;
                    for(i;i<55001;i++){
                        Samples[i]=0;
                    }
                    sampleFlag=0;
                    LCD_WriteStringAtPos("                   ",1,0);
 //                   sprintf(buffer, "Recorded for:%f",displayNum);
//                    LCD_WriteStringAtPos(buffer,1,0);
                    
                }
                ++sampleCount;
                LED2=1;
                if(sampleCount==55001){         //Checks to see if it is maxxed
                    buttonLock=1;
                }
                Samples[sampleCount-1]=AnalogRead(4);
            }
            else{
                LED2=0;
            }
            if(!BTNR&&sampleCount!=0&&!BTNC){                   //Button Lock
                sampleFlag=1;
            }
            if(!BTNR&&!BTNC){
                buttonLock=0;
                int i=0;
                for(i;i<10000;i++);
            }
            if(sampleCount%1100==0&&sampleCount>0&&!buttonLock&&!sampleFlag){
                displayNum+=0.1;
                sprintf(buffer, "Recorded for:%0.1f",displayNum);
                LCD_WriteStringAtPos(buffer,1,0);
                
            }
            if(BTNC&&!buttonLock){                      //Mode change
                mode=2;
                LCD_WriteStringAtPos("                   ",0,0);
                LCD_WriteStringAtPos("                   ",1,0);
                LCD_WriteStringAtPos("3: Play Back",0,2);
                sprintf(buffer,"%0.1fsec remaining",displayNum);
                LCD_WriteStringAtPos(buffer,1,0);
                //LCD_WriteStringAtPos("  ",1,14);
                tempDisplayNum=displayNum;
                maxCounter=displayNum*11100;
                buttonLock=1;
            }
            break;
        case 2:
            if(BTNR&&!buttonLock){                      //Enables Play back
                playBack=1;
                counter=0;
                buttonLock=1;
                
            }
            if(maxCounter==0){
                playBack=0;
            }
            if(playBack){
                float dutyCycle=Samples[counter++];
                if(counter==maxCounter){
                    playBack=0;
                }
                if(counter%1100==0&&counter!=0){
                    LCD_WriteStringAtPos("                   ",0,0);
                    LCD_WriteStringAtPos("                   ",1,0);
                    LCD_WriteStringAtPos("3: Play Back",0,2);
                    tempDisplayNum-=.1;
                    if(tempDisplayNum<=0){
                        tempDisplayNum=0;
                    }
                    sprintf(buffer,"%0.1fsec remaining",tempDisplayNum);
                    LCD_WriteStringAtPos(buffer,1,0);
                    //LCD_WriteStringAtPos("  ",1,14);
                }
                OC2RS=(int)((PR2 +1)*((float)dutyCycle/1023));   
            }
            else{
                OC2RS=0;
            }
            if(!BTNR&&buttonLock&&!BTNC&&!playBack){
                buttonLock=0;
                playBack=0;
                tempDisplayNum=displayNum;
                int i=0;
                for(i;i<10000;i++);
            }
            if(BTNC&&!buttonLock){
                mode=0;
                LCD_WriteStringAtPos("                   ",0,0);
                LCD_WriteStringAtPos("                   ",1,0);
            
                LCD_WriteStringAtPos("1: Play Tune",0,2);
                buttonLock=1;
            }
            break;
    }
    
    IFS0bits.T2IF =0;     //Clears interrupt flag            
    IEC0bits.T2IE =1;     //Enables interrupt   
}





void delay_ms(int ms){
	int		i,counterDelay;
	for (counterDelay=0; counterDelay<ms; counterDelay++)
    {
        for(i=0;i<1426;i++)
        {
        }   //software delay 1 millisec
    }
}
void configLEDS(){
    DDPCONbits.JTAGEN = 0; //Statement is required to use pin RA0 as I/O
    TRISAbits.TRISA0 = 0;   //Configure ports of LEDS to outputs for use
    TRISAbits.TRISA1 = 0;
    TRISAbits.TRISA2 = 0;
    TRISAbits.TRISA3 = 0;
    TRISAbits.TRISA4 = 0;
    TRISAbits.TRISA5 = 0;
    TRISAbits.TRISA6 = 0;
    TRISAbits.TRISA7 = 0;
    
    LED0=0;
    LED1=0;
    LED2=0;
    LED3=0;
    LED4=0;
    LED5=0;
    LED6=0;
    LED7=0;
}

void configOC2(){
    OC2CONbits.ON=0;
    OC2CONbits.OCM=6;
    OC2CONbits.OCTSEL=0;
    OC2CONbits.ON=1;
    TRISBbits.TRISB14 = 0;
    ANSELBbits.ANSB14 = 0;
    RPB14R = 0xB; //0xC for OC1, 0xB for OC2
}

void Timer2Setup(){
    //PR2 = (int)(((float)(TMR_TIME * PB_FRQ) /256) +0.5);//sets period register
    PR2=(int)((PB_CLK)/(4*11000)-1);
    TMR2 =0; //starts timer 2 at 0
    T2CONbits.TCKPS =2;   //sets  prescaler value to 4       
    T2CONbits.TGATE =0;   //not gated input(the default)            
    T2CONbits.TCS =0;     //PCBLK input (the default)            
    T2CONbits.ON =1;      //turns on timer 1             
    IPC2bits.T2IP =7;     //Sets priority to 7            
    IPC2bits.T2IS =3;     //sets subgroup priority to 3            
    IFS0bits.T2IF =0;     //Clears interrupt flag            
    IEC0bits.T2IE =1;     //Enables interrupt            
    macro_enable_interrupts();     //Enables interrupt at CPU    
}

void configSwitches(){
    TRISFbits.TRISF3 =1;    //SWITCHES
    TRISFbits.TRISF5 =1;
    TRISFbits.TRISF4 =1;
    TRISDbits.TRISD15 =1;
    TRISDbits.TRISD14 =1;
    TRISBbits.TRISB11 =1;
    TRISBbits.TRISB10 =1;
    TRISBbits.TRISB9 =1;
    ANSELBbits.ANSB11 =0;
    ANSELBbits.ANSB10 =0;
    ANSELBbits.ANSB9 =0;
            
}

void configBTNS(){
    TRISFbits.TRISF0=1;  //BTNC
    TRISBbits.TRISB8= 1; //BTNR
    ANSELBbits.ANSB8 = 0;
    
    TRISBbits.TRISB4 = 1;//Microphone
    ANSELBbits.ANSB4 = 1;
}

void initADC(){

	AD1CON1	=	0; 
    AD1CON1bits.SSRC = 7;   // Internal counter ends sampling and starts conversion (auto convert)
    AD1CON1bits.FORM = 0;   // Integer 16-bit
	// Setup for manual sampling
	AD1CSSL	=	0;
	AD1CON3	=	0x0002;     // ADC Conversion Clock Select bits: TAD = 6 TPB
	AD1CON2	=	0;
    AD1CON2bits.VCFG = 0;   // Voltage Reference Configuration bits: VREFH = AVDD and VREFL = AVSS
	// Turn on ADC
    AD1CON1bits.ON = 1;
} 

unsigned int AnalogRead(unsigned char analogPIN){
    int adc_val = 0;
    
    IEC0bits.T2IE = 0;
    AD1CHS = analogPIN << 16;       // AD1CHS<16:19> controls which analog pin goes to the ADC
 
    AD1CON1bits.SAMP = 1;           // Begin sampling
    while( AD1CON1bits.SAMP );      // wait until acquisition is done
    while( ! AD1CON1bits.DONE );    // wait until conversion is done
 
    adc_val = ADC1BUF0;
    IEC0bits.T2IE = 1;
    return adc_val;
}

void initMIC(){
    tris_MIC = 1;     // set AN4 (RB4) as analog input pin 
    ansel_MIC = 1;   // enable analog (set pins as analog)
}