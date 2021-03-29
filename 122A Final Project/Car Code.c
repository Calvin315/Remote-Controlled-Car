/*
* Control.c
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "io.c"
#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"
#include "usart_ATmega1284.h"

void RC_Calibration();
void PWM_Init();

unsigned char getBit(unsigned char mod, unsigned char bitNumber)
{
    return ((mod & (0x01 << bitNumber)) != 0);
}

unsigned char setBit(unsigned char mod, unsigned char bitNumber, unsigned char value)
{
    return (value ? mod | (0x01 << bitNumber) : mod & ~ (0x01 << bitNumber));
}

//A2 in A3 out
int LeftDist;
int LeftCount;
unsigned char LeftEcho;

//A4 in A5 out
int RightDist;
int RightCount;
unsigned char RightEcho;

//A0 in A1 out
int FrontDist;
int FrontCount;
unsigned char FrontEcho;

int Throttle;
int Steering;

//Debug
int SteeringValue;
int ThrottleValue;

char RecieveCount = 0;

enum CONTR_STATE {C_INIT, C_RECEIVE, C_SERVO, C_THROTTLE} CState;
void Contr_Tick() {
    //Transitions
    switch (CState) {
        case C_INIT:
        CState = C_RECEIVE;
        break;

        case C_RECEIVE:
        if (RecieveCount == 2)
        {
            CState = C_SERVO;
            RecieveCount = 0;
            } else {
            CState = C_RECEIVE;
        }
        break;
        
        case C_SERVO:
        CState = C_THROTTLE;
        break;
        
        case C_THROTTLE:
        CState = C_RECEIVE;
        break;
        
        default:
        CState = C_RECEIVE;
        break;
    }
    //Actions
    switch (CState) {
        case C_INIT:
        break;

        case C_RECEIVE:
        if (USART_HasReceived(0)) {
            unsigned char temp = USART_Receive(0);
            
            
            if (temp > 129) {
                Throttle = temp - 129;
            } else {
                Steering = temp;
            }
            
        }
        RecieveCount++;
        break;
        
        case C_SERVO:
        //2000 Left
        //1000 Right
        
        //Right is 100%
        //Left is 0%

        if ( Steering > 100)
        {
            Steering = 100;
        }

        if ( Steering < 0)
        {
            Steering = 0;
        }

        SteeringValue = (100 - Steering) * 10 + 1000;

        if (~PINA & 4)
        {
            if(SteeringValue > 1500) {
                SteeringValue = 1500;
            }

            PORTB= setBit(PORTB,1,1);
        } else {
            PORTB= setBit(PORTB,1,0);
        }

        if (~PINA & 16)
        {
            if(SteeringValue < 1500) {
                SteeringValue = 1500;
            }

            PORTB= setBit(PORTB,2,1);
        } else {
            PORTB= setBit(PORTB,2,0);
        }

        OCR3A = SteeringValue;
        break;
        
        case C_THROTTLE:
        if ( (Throttle > 47) && (Throttle < 53) )
        {
            ThrottleValue = 1500;
            } else {
            ThrottleValue = (Throttle * 10 + 1000);
        }
        
        if (~PINA & 1)
        {
            if(Throttle > 1500) {
                Throttle = 1500;
            }
            
            PORTB= setBit(PORTB,0,1);
        } else {
            PORTB= setBit(PORTB,0,0);
        }
        OCR3B = ThrottleValue;
        break;
        
        default:
        break;
    }

    char ControlStr[32] = "Thr: ";
    char tempContr[6];
    char tempSteer[6];
    //itoa(SteeringValue, tempSteer, 10);
    //itoa(ThrottleValue, tempContr, 10);
    itoa(Steering, tempSteer, 10);
    itoa(Throttle, tempContr, 10);

    strcat(ControlStr,tempContr);
    strcat(ControlStr," Str: ");
    strcat(ControlStr,tempSteer);

    LCD_DisplayString(1,ControlStr);
}

void ContrTask()
{
    CState = C_INIT;
    for(;;)
    {
        Contr_Tick();
        vTaskDelay(50);
    }
}

void ContrPulse(unsigned portBASE_TYPE Priority)
{
    xTaskCreate(ContrTask, (signed portCHAR *)"RemoteTask", configMINIMAL_STACK_SIZE, NULL, Priority, NULL );
}

enum LSTATES {L_INIT, L_START, L_TIMER, L_CALC } LState;
void Left_Tick() {
    //Grabbing input from Echo Pin
    LeftEcho = ~PINA & 4;
    //Transitions
    switch(LState){
        case L_INIT:
        LState = L_START;
        break;

        case L_START:
        if ( (~PINA & 4) == 1) {
            LState = L_TIMER;
        } else {     
            LState = L_START;
        }
        break;

        case L_TIMER:
        if(~PINA & 4 == 0) {
            LState = L_CALC;
        } else if (LeftCount > 2000) {
            LState = L_START;
        } else {
            LState = L_TIMER;
        }
        break;

        case  L_CALC:
        LState = L_START;
        break;

        default:
        break;
    }

    //Actions
    switch(LState){
        case L_INIT:
        break;

        case L_START:
        PORTA = setBit(PORTA, 3, 1);
        delay_ms(10);
        PORTA = setBit(PORTA, 3, 0);
        delay_ms(2);
        LeftCount = 0;
        break;

        case L_TIMER:
        if ((~PINA & 4) == 1)
        {
            LeftCount++;

            char echoHold[3];
            char DistHold[32];
            itoa(LeftCount, DistHold,10);
            strcat(DistHold, " INCR ");
            itoa(LeftEcho, echoHold, 10);
            strcat(DistHold, echoHold);
            LCD_DisplayString(1,DistHold);
        }
        break;

        case  L_CALC:
        //LeftDist = ((float)LeftCount*13503.9/1000.0)/2;
        break;

        default:
        break;
    }
}

void LeftTask()
{
    LState = L_INIT;
    for(;;)
    {
        Left_Tick();
        vTaskDelay(1);
    }
}

void LeftPulse(unsigned portBASE_TYPE Priority)
{
    xTaskCreate(LeftTask, (signed portCHAR *)"RemoteTask", configMINIMAL_STACK_SIZE, NULL, Priority, NULL );
}


int main(void)
{

    //PORT A 00101010
    DDRA = 0x2A; //Configures Port -- 00 for Input, FF for Output
    PORTA = 0x00; //Initializes the Value in the Port

    //PORT B
    DDRB = 0xFF; //Configures Port -- 00 for Input, FF for Output
    PORTB = 0x00; //Initializes the Value in the Port
    //DDRB = (1 << DDB6) | (1 << DDB7);

    //PORT C
    DDRC = 0xFF; //Configures Port -- 00 for Input, FF for Output
    PORTC = 0x00; //Initializes the Value in the Port

    //PORT D
    DDRD = 0xC2; //Configures Port -- 00 for Input, FF for Output
    PORTD = 0x00; //Initializes the Value in the Port

    LCD_init();
    PWM_Init();
    RC_Calibration();

    initUSART(0);
    USART_Flush(0);

    ContrPulse(1);
    //LeftPulse(1);
    vTaskStartScheduler();
}

void PWM_Init() {

    TCCR3A = (1 << COM3A1) | (1 << COM3B1) | (1 << WGM31);
    TCCR3B = (1 << CS31) | (1 << WGM33) | (1 << WGM32);

    ICR3 = 0x4E20;
};

void RC_Calibration() {
    //Set Max
    OCR3B = 2000;
    delay_ms(3000);

    //Set Min
    OCR3B = 1000;
    delay_ms(3000);

    //Set Neutral
    OCR3B = 1500;
    delay_ms(3000);
}

