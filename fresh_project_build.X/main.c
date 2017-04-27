/*******************************************************************************
 Explorer 16 Demo Main File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    Explorer 16 Demo Main File.

  Description: 
    This is the main file for the Explorer 16 Demo. It contains the main demo
    flow and calls processor specific files for processor specific
    implementations.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************


#define FOSC (7370000ULL)
#define FCY (FOSC/2)

#include <libpic30.h>
#include "mcc_generated_files/mcc.h"
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdbool.h>

#define ONE_VOLT 310
#define ONE_TENTH_VOLT 31
#define ONE_HUNDREDTH_VOLT 3

// state declarations
#define IDLE 1
#define CHECK_BEER 2
#define LOADING 3
#define LAUNCH_PREP 4
#define LAUNCH 5

// limit switch "button style" port declarations
#define beer_PORT PORTDbits.RD11
#define beer_TRIS TRISDbits.TRISD11
/*
// LED 1
#define led1_PORT PORTEbits.RE0
#define led1_TRIS TRISEbits.TRISE0
// LED 2
#define led2_PORT PORTEbits.RE1
#define led2_TRIS TRISEbits.TRISE1
// LED 3
#define led3_PORT PORTEbits.RE2
#define led3_TRIS TRISEbits.TRISE2
// LED 4
#define led4_PORT PORTFbits.RF1
#define led4_TRIS TRISFbits.TRISF1
// LED 5
#define led5_PORT PORTEbits.RE4
#define led5_TRIS TRISEbits.TRISE4
 */
// h bridge enable
#define hb_enable_PORT PORTDbits.RD1
#define hb_enable_TRIS TRISDbits.TRISD1
// h bridge port 1
#define hb1_PORT PORTDbits.RD2
#define hb1_TRIS TRISDbits.TRISD2
// h bridge port 2
#define hb2_PORT PORTFbits.RF0
#define hb2_TRIS TRISFbits.TRISF0
// solenoid
#define solenoid_PORT PORTEbits.RE4
#define solenoid_TRIS TRISEbits.TRISE4
// pump
#define pump_PORT PORTEbits.RE3
#define pump_TRIS TRISEbits.TRISE3

// *****************************************************************************
// *****************************************************************************
// Section: File Scope Variables and Functions
// *****************************************************************************
// *****************************************************************************

void pwm_testing ( void );
void limit_testing (void);
void UART_testing();
void transmit(uint8_t * buffer, int buf_length);
void adc_testing(void);
float get_sample();
float pressure_to_voltage(float voltage);
float distance_to_pressure(int distance, int distance_decimal);


// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************
int main ( void )
{
    /* Call the System Initialize routine*/
    SYSTEM_Initialize();
    
    // initialize the tristate buffers
    beer_TRIS = 1; // input to micro
    /*
    led1_TRIS = 0; // output from micro
    led1_PORT = 0;
    led2_TRIS = 0; // output from micro
    led2_PORT = 0;
    led3_TRIS = 0; // output from micro
    led3_PORT = 0;
    led4_TRIS = 0; // output from micro
    led4_PORT = 0;
    led5_TRIS = 0; // output from micro
    led5_PORT = 0;
    */
    hb_enable_TRIS = 0;
    hb_enable_PORT = 0; // disable
    hb1_TRIS = 0;
    hb1_PORT = 1; // start with retraction
    hb2_TRIS = 0;
    hb2_PORT = 0; // start with retraction
    solenoid_TRIS = 0;
    solenoid_PORT = 0;
    pump_TRIS = 0;
    pump_PORT = 0;
    
    // variables
    int numBytesWritten;
    int pumpswitch = 1;
    int num_beers = 0;
    int i; // for loopies
    int angle, distance, distance_decimal;
    int buf_length = 6; // number of characters to be read
    unsigned int numBytesRead; // local variables to grab values in debugger
    uint8_t buffer[buf_length]; // buffer to read to
    char char_buffer[buf_length]; // buffer to character conversion
    uint8_t in1_string[8] = "\nINSIDE1";
    uint8_t in2_string[8] = "\nINSIDE2";
    uint8_t in3_string[8] = "\nINSIDE3";
    uint8_t in4_string[8] = "\nINSIDE4";
    uint8_t in5_string[8] = "\nINSIDE5";
    uint8_t confirm[8] = "\nLAUNCH!";
    uint8_t no_beer[8] = "\nNO BEER";
    uint8_t loaded[8] = "\nLOADED1";
    // ADC variables
    float voltage, target_voltage, pressure;
    // loop initializations
    int state = IDLE;
    int n_state = IDLE;
    
    // send launcher to Load
    OC1_Start();
    OC1_Load();
    __delay_ms(3000);
    // send launcher to Load

    // THROW AWAY FIRST ADC VALUE
    voltage = get_sample();
    // THROW AWAY FIRST ADC VALUE

    numBytesWritten = UART1_WriteBuffer(in1_string , 8);
    while (1)
    {
        state = n_state;
        switch (state)
        {
            case IDLE :
                if (beer_PORT == 1)
                { // keep track of beers put in
                    numBytesWritten = UART1_WriteBuffer(loaded, 8);
                    num_beers++;
                    __delay_ms(2000); // 1000ms delay to avoid counting a beer multiple times
                }
                if(UART1_ReceiveBufferIsFull())
                {
                    // turn off pump
                    pump_PORT = 0;
                    // buffer is full, read data buffer
                    numBytesRead = UART1_ReadBuffer(buffer, buf_length);
                    // translate buffer bytes into chars
                    for (i=0; i < buf_length; i++) {char_buffer[i] = (char) (buffer[i]);}
                    // set next state
                    n_state = CHECK_BEER;
                    // data conversion for angle and distance
                    if ((char_buffer[0] == 'A') && (char_buffer[3] == 'D'))
                    { // first byte should be an 'a' for ANGLE
                        if (char_buffer[1] == '+')
                        { // second byte is a '+', make angle integer
                            angle = buffer[2];
                        } else if (char_buffer[1] == '-')
                        { // second byte is a '-', make angle integer
                            angle = (buffer[2])*(-1);
                        } else
                        { // Error w/ UART data, did not find '+/-' in byte 2
                            n_state = IDLE; // data error, go back to IDLE
                        }
                        // grab the distance and decimal nums
                        distance = buffer[4];
                        distance_decimal = buffer[5];
                    } else
                    { // Error w/ UART data, did not find 'A' and 'D' in correct spots
                        n_state = IDLE; // data error, go back to IDLE
                    }
                } else
                { // didnt get anything, stay here and try to pump to 40 psi
                    // sample initial value
                    voltage = get_sample();
                    // convert distance to pressure
                    pressure = distance_to_pressure(distance, distance_decimal);
                    // get target voltage value
                    target_voltage = pressure_to_voltage((float) 37.5);
                    if ((voltage < target_voltage) && pumpswitch)
                    { // turn on pump if not at 40
                        pump_PORT = 1;
                    } else
                    { // turn off pump if not at 40
                        pumpswitch = 0; // do not let it pump up again
                        pump_PORT = 0;
                    }
                    n_state = IDLE;
                }
                break;
                
            case CHECK_BEER :
                if (num_beers > 0) {
                    n_state = LOADING;
                } else {
                    n_state = IDLE; // go to idle and wait for beers to be loaded and another dab
                }
                break;
                // if beer -> LOADING
                // else -> IDLE && print NO BEER, DAB AGAIN W/ BEER
                
            case LOADING :
                //send launcher to Load
                OC1_Load();
                __delay_ms(3000);
                //send launcher to Load
                /// REGULATE PRESSURE BEFORE CAN LOAD
                // sample initial value
                voltage = get_sample();
                // convert distance to pressure
                pressure = distance_to_pressure(distance, distance_decimal);
                // get target voltage value
                target_voltage = pressure_to_voltage((float) pressure);
                while (voltage > target_voltage)
                { // lower pressure if too high
                    solenoid_PORT = 1;
                    __delay_ms(25);
                    solenoid_PORT = 0;
                    __delay_ms(100);
                    voltage = get_sample(); // sample pressure sensor
                }
                while (voltage < target_voltage)
                { // raise pressure if too low
                    // turn on pump
                    pump_PORT = 1;
                    voltage = get_sample(); // sample pressure sensor
                }
                // turn off the pump
                pump_PORT = 0;
                /// REGULATE PRESSURE BEFORE CAN LOAD
                // ACTIVATE DILDO
                hb1_PORT = 0; // push
                hb2_PORT = 1; // push
                hb_enable_PORT = 1; // enable
                __delay_ms(500); // wait
                hb_enable_PORT = 0;
                hb1_PORT = 1; // pull
                hb2_PORT = 0; // pull
                hb_enable_PORT = 1;
                __delay_ms(3000);
                hb_enable_PORT = 0; // disable
                // ACTIVATE DILDO
                n_state = LAUNCH_PREP;
                break;
                // move servo to release beer
                // move servo back to stop next beer
                // if launcher limit switch pressed -> LAUNCH_PREP
                // else -> LOADING
                
            case LAUNCH_PREP :
                // move launcher to firing position
                OC1_Position(angle);
                __delay_ms(3000);
                // move launcher to firing position
                n_state = LAUNCH;
                // turn on the air pump
                // if at any point we get the pressure reading we need (turn off air pump) && were in the right position, BREAK
                // goto -> LAUNCHING
                break;
            
            case LAUNCH :
                solenoid_PORT = 1;
                __delay_ms(75);
                solenoid_PORT = 0;
                __delay_ms(500);
                num_beers--; // launched a beer
                // CONFIRM WITH LAPTOP
                OC1_Load();
                __delay_ms(3000);
                //send launcher to Load
                pumpswitch = 1;
                n_state = IDLE;
                break;
                
            default :
                state = IDLE;
                break;
        }
    }
    return (0);
}

float distance_to_pressure(int distance, int distance_decimal)
{
    if (distance < 1)
    {
        if (distance_decimal < 5)
        {
            return (25.0);
        } else
        {
            return (30.0);
        }
    } else if (distance < 2)
    {
        if (distance_decimal < 5)
        {
            return (32.5);
        } else
        {
            return (35.0);
        }
    } else if (distance < 3)
    {
        if (distance_decimal < 5)
        {
            return (37.5);
        } else
        {
            return (40.0);
        }
    } else if (distance < 4)
    {
        if (distance_decimal < 5)
        {
            return (42.5);
        } else
        {
            return (45.0);
        }
    } else
    {
        return (45);
//        if (distance_decimal < 5)
//        {
//            return (45.0);
//        } else
//        {
//            return (47.5);
//        }
    }
}

float get_sample()
{
    uint16_t sum = 0;
    uint16_t conversion;
    float voltage;
    int i;
    for (i=0;i<10;i++)
    {
        // sample value
        ADC1_Start();
        __delay_ms(1); //Provide Delay
        ADC1_Stop();
        while(!ADC1_IsConversionComplete()) {}
        sum += ADC1_ConversionResultGet();
        // sample value
    }
    conversion = sum / 10;
    voltage = ((float)((float)(3.3 * (float)conversion)/1023));
    return voltage;
}

float pressure_to_voltage(float pressure)
{
    float voltage;
    voltage = ((4.975 * ((0.0012858 * (pressure * 6.894757293168361)) + 0.04)) + 0.598381);
    return voltage;
}

void adc_testing()
{
    //int i;
    uint16_t conversion;
    uint8_t adc_string[8] = "\nVALUE=X";
    // THROW AWAY FIRST VALUE
    while (1)
    {
        // sample for 1 ms
        ADC1_Start();
        //Provide Delay
        __delay_ms(1);
        ADC1_Stop();
        // sample for 1 ms
        while(!ADC1_IsConversionComplete()) {}
        conversion = ADC1_ConversionResultGet();
        adc_string[7] = (uint8_t) conversion;
        UART1_WriteBuffer(adc_string , 8);
        __delay_ms(1000);
        // break
    }
}

void UART_testing()
{
    int i;
    ///char uart_string[] = "UART READY";
    //char uart_string1[] = "GOT 0x41";
    //char uart_string2[] = "READ 2";
    ///LCD_ClearScreen();
    ///LCD_PutString((char*)&uart_string,sizeof(uart_string)-1); // UART ready
    // number of characters to be read
    int buf_length = 5;
    // local variables to grab values in debugger
    unsigned int numBytesRead;
    // buffer to read to
    uint8_t buffer[buf_length];
    // initialize buffer to 0
    for (i=0; i < buf_length; i++) {buffer[i] = 0;}
    char data_buffer[buf_length];
    do
    {
        if(UART1_ReceiveBufferIsFull())
        {
            // buffer is full, read data buffer
            numBytesRead = UART1_ReadBuffer(buffer, buf_length);
            for (i=0; i < buf_length; i++) {data_buffer[i] = (char) (buffer[i]);}
            ///LCD_ClearScreen();
            ///LCD_PutString((char*)&data_buffer, buf_length);
        }
    } while (1);
    
    return;
}

void pwm_testing ( void )
{
    //OC1_Load();
    //OC1_Neutral();
    //OC1_MaxRight();
    OC1_MaxLeft();
    while(1) {}
    return;
}

void limit_testing ( void )
{
    ///char limit_string1[] = "LIMIT SWITCH PRESSED";
    ///char limit_string2[] = "LIMIT SWITCH NOT PRESSED";
    ///char limit_string3[] = "READY FOR TESTING";
    ///LCD_ClearScreen();
    ///LCD_PutString((char*)&limit_string3,sizeof(limit_string3)-1);
    int flag = 0;
    OC1_Start();
    do
    {
        if (beer_PORT == 1)
        {
            if (flag == 0)
            {
               ///LCD_ClearScreen();
               ///LCD_PutString((char*)&limit_string1,sizeof(limit_string1)-1);
               flag = 1;
               OC1_180(); // move servo to 180 when limit switch is pressed
            }
        } else
        {
            if (flag == 1)
            {
               ///LCD_ClearScreen();
               ///LCD_PutString((char*)&limit_string2,sizeof(limit_string2)-1);
               flag = 0;
               OC1_0(); // move servo back to 0 when limit switch is released
            }
        }
    }while(1);
    return;
}
