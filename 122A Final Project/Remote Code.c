#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "io.c"
#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"
#include "usart_ATmega1284.h"

enum REMOTE_STATE {R_INIT, R_UPDATE, R_SEND_THROTTLE, R_SEND_STEER} RState;

int UD_Max = 0;
int LR_Max = 0;

int LR_ADC;
int UD_ADC;

int UD_Min = 500;
int LR_Min = 500;

unsigned char UD_Percent = 0;
unsigned char LR_Percent = 0;

char Update[32];
char TempA[5];
char TempB[5];

unsigned char ThrottleContr;
unsigned char Button1;

void adc_init();
uint16_t adc_read(uint8_t ch);

void Remote_Tick() {
    LR_ADC = adc_read(0);
    UD_ADC = adc_read(1);
    Button1 = ~PINA & 4;

    strcpy(Update,"");

    //Transitions
    switch (RState) {
        case R_INIT:
        if (Button1) {
            RState = R_UPDATE;
        } else {
            RState = R_INIT;
        }
        break;

        case R_UPDATE:
        RState = R_SEND_THROTTLE;
        break;

        case R_SEND_THROTTLE:
        RState = R_SEND_STEER;
        break;
		
		case R_SEND_STEER:
        RState = R_UPDATE;
        break;

        default:
        RState = R_UPDATE;
        break;
    }
    //Actions
    
    switch (RState) {
        case R_INIT:
        strcpy(Update,"");
        if(UD_ADC > UD_Max) { UD_Max = UD_ADC; }
        if(LR_ADC > LR_Max) { LR_Max = LR_ADC; }
        if (UD_ADC < UD_Min) { UD_Min = UD_ADC; }
        if (LR_ADC < LR_Min) { LR_Min = LR_ADC; }

        itoa(UD_Max,TempA,10);
        itoa(UD_Min,TempB,10);
        
        strcat(Update,"UD: ");
        strcat(Update,TempA);
        strcat(Update," ");
        strcat(Update,TempB);

        strcpy(TempA,"");
        strcpy(TempB,"");

        itoa(LR_Max,TempA,10);
        itoa(LR_Min,TempB,10);
        
        strcat(Update," LR: ");
        strcat(Update,TempA);
        strcat(Update," ");
        strcat(Update,TempB);

        LCD_DisplayString(1,Update);
        LCD_Cursor(NULL);
        break;
        
        case R_UPDATE:
        strcpy(Update,"");
        strcpy(TempA,"");
        strcpy(TempB,"");

        LR_Percent = (((float)LR_ADC - (float)LR_Min) * 100.0 / ((float)LR_Max - (float)LR_Min));
        UD_Percent = (((float)UD_ADC - (float)UD_Min) * 100.0 / ((float)UD_Max - (float)UD_Min));
        
        itoa(UD_Percent,TempA,10);
        itoa(UD_ADC,TempB,10);

        strcat(Update,"UD: ");
        strcat(Update,TempA);
        strcat(Update," ");
        strcat(Update,TempB);
        
        strcpy(TempA,"");
        strcpy(TempB,"");

        itoa(LR_Percent,TempA,10);
        itoa(LR_ADC,TempB,10);
        
        strcat(Update," LR: ");
        strcat(Update,TempA);
        strcat(Update," ");
        strcat(Update,TempB);
        
        LCD_DisplayString(1,Update);
        LCD_Cursor(NULL);
        break;

        case R_SEND_THROTTLE:
        //Throttle has highest bit set
        if (USART_IsSendReady(0)) {
            ThrottleContr = UD_Percent + 129;
            USART_Send(ThrottleContr, 0);
        }
        break;
		
		case R_SEND_STEER:
        //Steering is percent only
        if (USART_IsSendReady(0)) {
            USART_Send(LR_Percent, 0);
        }
        break;

        default:
        break;
    }
    
}

void RemoteTask()
{
    RState = R_INIT;
    for(;;)
    {
        Remote_Tick();
        vTaskDelay(50);
    }
}

void RemotePulse(unsigned portBASE_TYPE Priority)
{
    xTaskCreate(RemoteTask, (signed portCHAR *)"RemoteTask", configMINIMAL_STACK_SIZE, NULL, Priority, NULL );
}


int main(void)
{
    //PORT A
    DDRA = 0x00; //Configures Port -- 00 for Input, FF for Output
    PORTA = 0xFF; //Initializes the Value in the Port

    //PORT B
    DDRB = 0xFF; //Configures Port -- 00 for Input, FF for Output
    PORTB = 0x00; //Initializes the Value in the Port
    //DDRB = (1 << DDB6) | (1 << DDB7);

    //PORT C
    DDRC = 0xFF; //Configures Port -- 00 for Input, FF for Output
    PORTC = 0x00; //Initializes the Value in the Port

    //PORT D 11000010
    DDRD = 0xC2; //Configures Port -- 00 for Input, FF for Output
    PORTD = 0x00; //Initializes the Value in the Port

    LCD_init();
    adc_init();

    initUSART(0);
    USART_Flush(0);

    RemotePulse(1);
    vTaskStartScheduler();
}


void adc_init()
{
    // AREF = AVcc
    ADMUX = (1<<REFS0);

    // ADC Enable and prescaler of 128
    // 16000000/128 = 125000
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

uint16_t adc_read(uint8_t ch)
{
    // select the corresponding channel 0~7
    // ANDing with ? will always keep the value
    // of ??etween 0 and 7
    ch &= 0b00000111;  // AND operation with 7
    ADMUX = (ADMUX & 0xF8)|ch; // clears the bottom 3 bits before ORing

    // start single convertion
    // write ? to ADSC
    ADCSRA |= (1<<ADSC);

    // wait for conversion to complete
    // ADSC becomes ? again
    // till then, run loop continuously
    while(ADCSRA & (1<<ADSC));

    return (ADC);
}