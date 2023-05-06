// Analog Example
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL with LCD/Temperature Sensor
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz
// Stack:           4096 bytes (needed for sprintf)

// Hardware configuration:
// LM60 Temperature Sensor:
//   AIN3/PE0 is driven by the sensor
//   (V = 424mV + 6.25mV / degC with +/-2degC uncalibrated error)
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "clock.h"
#include "wait.h"
#include "uart0.h"
#include "adc0.h"
#include "gpio.h"
#include "nvic.h"
#include "tm4c123gh6pm.h"
#include "math.h"

#define AIN0 PORTE,3
#define AIN1 PORTE,2
#define AIN2 PORTE,1
#define AIN4 PORTD,2

#define AIN_MUX_SSO 0x421421
#define SENSORS_COUNT 3
#define TH 500

volatile uint16_t ADC0_SS0_SAMPLES[SENSORS_COUNT];
uint8_t FIR_INPUT_DATA_ARRAY_INDEX;
uint8_t down_sample_by_10_flag = 0;
char str[100];
float p2_p3_back_off_value = 0;

uint16_t temp = 0;
uint16_t fir_x, fir_y, fir_z = 0;
uint8_t t1x_flag = 0, t2x_flag = 0, t3x_flag = 0, event_counter = 0;
uint32_t time_diff_counter = 0, t1x = 0, t2x = 0, t3x = 0, hold_off_value = 300000, hold_off_prev_value = 300000; //approx 2sec
uint16_t first_peak_value = 0, aoa = 0;
uint8_t back_off_value = 80,failed_event_flag = 0 ,fail_flag = 0;
uint8_t average_flag = 0, tdoa_flag = 0, aoa_always_flag = 0, aoa_flag = 0,time_constant = 10,time_constant_prev = 10;
float bKV_p2 = 0.8,bKV_p2_prev = 0.8;
float angle = 0;
float DAC_Value_volts = 0;
float SPL_Value = 0,K=0.022;
#define Time_Out_in_ISR_count 1000 //ususally more than 300 us is a failed event

//define sensors events
typedef enum
{
    EVENT_MIC_1, EVENT_MIC_2, EVENT_MIC_3, NO_EVENT

} Event_State;

Event_State Detection = NO_EVENT, p1x_flag = NO_EVENT, p2x_flag = NO_EVENT,
        p3x_flag = NO_EVENT;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0 | SYSCTL_RCGCGPIO_R1
            | SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R3 | SYSCTL_RCGCGPIO_R4
            | SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    //select the pins as analog inputs
    // selectPinAnalogInput(AIN0);
    selectPinAnalogInput(AIN1);
    selectPinAnalogInput(AIN2);
    selectPinAnalogInput(AIN4);

    //select the auxilary fuction
    // setPinAuxFunction(AIN0, GPIO_PCTL_PE3_AIN0);
    setPinAuxFunction(AIN1, GPIO_PCTL_PE2_AIN1);
    setPinAuxFunction(AIN2, GPIO_PCTL_PE1_AIN2);
    setPinAuxFunction(AIN4, GPIO_PCTL_PD3_AIN4);

    enableNvicInterrupt(INT_ADC0SS0);
    setNvicInterruptPriority(INT_ADC0SS0, 1);
}

void Adc0_SS0_ISR(void)
{

    time_diff_counter = time_diff_counter + 1;
    // collect samples from SS0 FIFO
    ADC0_SS0_SAMPLES[0] = ADC0_SSFIFO0_R;
    ADC0_SS0_SAMPLES[1] = ADC0_SSFIFO0_R;
    ADC0_SS0_SAMPLES[2] = ADC0_SSFIFO0_R;
    temp = ADC0_SSFIFO0_R;
    temp = ADC0_SSFIFO0_R;
    temp = ADC0_SSFIFO0_R;

    if (FIR_INPUT_DATA_ARRAY_INDEX == time_constant)
    {
        FIR_INPUT_DATA_ARRAY_INDEX = 0;
        down_sample_by_10_flag = 1;
    }
    else
        FIR_INPUT_DATA_ARRAY_INDEX = FIR_INPUT_DATA_ARRAY_INDEX + 1;

    ADC0_ISC_R = ADC_ISC_IN0; // clear SSO interrupt flag
}

void AOA_calc()
{
    switch (Detection)
    {
    case EVENT_MIC_1:
        if (event_counter < 2 && (time_diff_counter < Time_Out_in_ISR_count))
        {
            failed_event_flag = 0   ;
            if (p2x_flag == NO_EVENT)
            {
                if ((ADC0_SS0_SAMPLES[1] > bKV_p2 * first_peak_value))
                {
                    Detection = EVENT_MIC_2;
                    t2x = time_diff_counter;
                    event_counter = event_counter + 1;
                    t2x_flag = 1;
                    p2x_flag = Detection;
                }
                else if ((ADC0_SS0_SAMPLES[2] > bKV_p2 * first_peak_value))
                {
                    Detection = EVENT_MIC_3;
                    t3x = time_diff_counter;
                    event_counter = event_counter + 1;
                    t3x_flag = 1;
                    p2x_flag = Detection;
                }
                else
                {
                    Detection = EVENT_MIC_1;
                }
            }
            else if (p3x_flag == NO_EVENT)
            {
                if ((ADC0_SS0_SAMPLES[1] > (bKV_p2 - 0.05) * first_peak_value)
                        && (t2x_flag == 0))
                {
                    Detection = EVENT_MIC_2;
                    t2x = time_diff_counter;
                    event_counter = event_counter + 1;
                    t2x_flag = 1;
                    p3x_flag = Detection;
                }
                else if ((ADC0_SS0_SAMPLES[2]
                        > (bKV_p2 - 0.05) * first_peak_value)
                        && (t3x_flag == 0))
                {
                    Detection = EVENT_MIC_3;
                    t3x = time_diff_counter;
                    event_counter = event_counter + 1;
                    t3x_flag = 1;
                    p3x_flag = Detection;
                }
                else
                {
                    Detection = EVENT_MIC_1;
                }

            }
        }
        else
        {
            Detection = NO_EVENT;
            if(event_counter !=2)
            {
                failed_event_flag = 1;
            }
            else
            {
                failed_event_flag = 1;
            }
        }
        break;

    case EVENT_MIC_2:
        if (event_counter < 2 && (time_diff_counter < Time_Out_in_ISR_count))
        {
            failed_event_flag = 0;
            if (p2x_flag == NO_EVENT)
            {
                if ((ADC0_SS0_SAMPLES[0] > bKV_p2 * first_peak_value))
                {
                    Detection = EVENT_MIC_1;
                    t1x = time_diff_counter;
                    event_counter = event_counter + 1;
                    t1x_flag = 1;
                    p2x_flag = Detection;
                }
                else if ((ADC0_SS0_SAMPLES[2] > bKV_p2 * first_peak_value))
                {
                    Detection = EVENT_MIC_3;
                    t3x = time_diff_counter;
                    event_counter = event_counter + 1;
                    t3x_flag = 1;
                    p2x_flag = Detection;
                }
                else
                {
                    Detection = EVENT_MIC_2;
                }
            }
            else if (p3x_flag == NO_EVENT)
            {
                if ((ADC0_SS0_SAMPLES[0] > (bKV_p2 - 0.05) * first_peak_value)
                        && (t1x_flag == 0))
                {
                    Detection = EVENT_MIC_1;
                    t1x = time_diff_counter;
                    event_counter = event_counter + 1;
                    t1x_flag = 1;
                    p3x_flag = Detection;
                }
                else if ((ADC0_SS0_SAMPLES[2]
                        > (bKV_p2 - 0.05) * first_peak_value)
                        && (t3x_flag == 0))
                {
                    Detection = EVENT_MIC_3;
                    t3x = time_diff_counter;
                    event_counter = event_counter + 1;
                    t3x_flag = 1;
                    p3x_flag = Detection;
                }
                else
                {
                    Detection = EVENT_MIC_2;
                }

            }
        }
        else
        {
            Detection = NO_EVENT;
            if(event_counter !=2)
            {
                failed_event_flag = 1;
            }
            else
            {
                failed_event_flag = 1;
            }
        }
        break;

    case EVENT_MIC_3:
        if (event_counter < 2 && (time_diff_counter < Time_Out_in_ISR_count))
        {
            failed_event_flag = 0   ;
            if (p2x_flag == NO_EVENT)
            {
                if ((ADC0_SS0_SAMPLES[0] > bKV_p2 * first_peak_value))
                {
                    Detection = EVENT_MIC_1;
                    t1x = time_diff_counter;
                    event_counter = event_counter + 1;
                    t1x_flag = 1;
                    p2x_flag = Detection;
                }
                else if ((ADC0_SS0_SAMPLES[1] > bKV_p2 * first_peak_value))
                {
                    Detection = EVENT_MIC_2;
                    t2x = time_diff_counter;
                    event_counter = event_counter + 1;
                    t2x_flag = 1;
                    p2x_flag = Detection;
                }
                else
                {
                    Detection = EVENT_MIC_3;
                }
            }
            else if (p3x_flag == NO_EVENT)
            {
                if ((ADC0_SS0_SAMPLES[0] > (bKV_p2 - 0.05) * first_peak_value)
                        && (t1x_flag == 0))
                {
                    Detection = EVENT_MIC_1;
                    t1x = time_diff_counter;
                    event_counter = event_counter + 1;
                    t1x_flag = 1;
                    p3x_flag = Detection;
                }
                else if ((ADC0_SS0_SAMPLES[1]
                        > (bKV_p2 - 0.05) * first_peak_value)
                        && (t2x_flag == 0))
                {
                    Detection = EVENT_MIC_2;
                    t2x = time_diff_counter;
                    event_counter = event_counter + 1;
                    t2x_flag = 1;
                    p3x_flag = Detection;
                }
                else
                {
                    Detection = EVENT_MIC_3;
                }

            }
        }
        else
        {
            Detection = NO_EVENT;
            if(event_counter !=2)
            {
                failed_event_flag = 1;
            }
            else
            {
                failed_event_flag = 1;
            }
        }
        break;
    }

}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    uint16_t sum_x = 0, sum_y = 0, sum_z = 0; // total fits in 16b since 12b adc output x 16 samples
    uint8_t index = 0;
    uint8_t i;
    uint16_t x[16], y[16], z[16];
    USER_DATA data;

    // Initialize hardware
    initHw();
    initUart0();
    initAdc0Ss0();
    enableSS0Interrupt();

    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);

    // Use AIN4,AIN2,AIN1,AIN0 input
    setAdc0Ss0Mux(AIN_MUX_SSO);

    // Clear FIR filter taps
    for (i = 0; i < 16; i++)
    {
        x[i] = 0;
        y[i] = 0;
        z[i] = 0;
    }

    // Endless loop
    while (true)
    {

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////User Interface///////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        if (kbhitUart0())
        {
            //Get the String from user
            getsUart0(&data);

#ifdef DEBUG
                    //print the string
                    putsUart0(data.buffer);
                    putsUart0("\n");
            #endif
            //Parse fields
            parseFields(&data);

#ifdef DEBUG
                    for( ii = 0;ii < data.fieldCount ;ii++){
                        putcUart0(data.fieldType[ii]);
                        putsUart0("\t");
                        putsUart0(&(data.buffer[data.fieldPosition[ii]]));
                        putsUart0("\n");
                    }
            #endif

            //reset
            if (isCommand(&data, "reset", 0))
            {
                NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
            }
            //holdoff
            else if (isCommand(&data, "holdoff", 1))
            {
                if((getFieldInteger(&data, 1)) == -1 )
                {
                    sprintf(str, "holdoff not Updated: %.4i\n", hold_off_value);
                    putsUart0(str);
                }
                else
                {
                    hold_off_value = (uint32_t) getFieldInteger(&data, 1);
                    hold_off_value = (uint32_t) hold_off_value / 6;
                    sprintf(str, "holdoff Updated: %.4i\n", hold_off_value);
                    putsUart0(str);
                }
            }
            //backoff
            else if (isCommand(&data, "backoff", 1))
            {
                    if((getFieldInteger(&data, 1)) == -1 )
                    {
                        sprintf(str, "backoff not Updated: %.4f\n", bKV_p2);
                        putsUart0(str);
                    }
                    else
                    {
                        back_off_value = (uint32_t) getFieldInteger(&data, 1);
                        bKV_p2 = (float) back_off_value / 100;
                        sprintf(str, "backoff Updated: %.4f\n", bKV_p2);
                        putsUart0(str);

                    }
            }
            //average
            else if (isCommand(&data, "average", 0))
            {
                average_flag = 1;
                sprintf(str, "requested average:           \n");
                putsUart0(str);
            }
            //aoa
            else if (isCommand(&data, "aoa", 0))
            {
                aoa_flag = 1;
                sprintf(str, "requested angle of arrival:           \n");
                putsUart0(str);
            }
            //aoa always
            else if (isCommand(&data, "aoaalways", 0))
            {
                aoa_always_flag = 1;
                sprintf(str, "aoa always is set :           \n");
                putsUart0(str);
            }
            //tdoa
            else if (isCommand(&data, "tdoa", 1))
            {
                char *state = getFieldString(&data, 1);
                int s = strcmp(state,"ON");
                if(s==0)
                {
                    tdoa_flag = 1;
                    sprintf(str, "tdoa flag  ON :           \n");
                    putsUart0(str);
                }
                else
                {
                    tdoa_flag = 0;
                    sprintf(str, "tdoa flag  OFF:           \n");
                    putsUart0(str);
                }
            }
            //fail command
            else if (isCommand(&data, "fail", 1))
            {
                char *state = getFieldString(&data, 1);
                int s = strcmp(state, "ON");
                if (s == 0)
                {
                    fail_flag = 1;
                    sprintf(str, "fail flag ON:           \n");
                    putsUart0(str);
                }
                else
                {
                    fail_flag = 0;
                    sprintf(str, "fail flag OFF:           \n");
                    putsUart0(str);
                }
            }
            else if (isCommand(&data, "tc", 1))
            {
                    if((getFieldInteger(&data, 1)) == -1 )
                    {
                        sprintf(str, "time constant is not updated :  %.4i\n", time_constant);
                        putsUart0(str);
                    }
                    else
                    {
                        time_constant = (uint32_t) getFieldInteger(&data, 1);
                        sprintf(str, "time constant is updated :           \n");
                        putsUart0(str);

                    }
            }

        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////MATH///////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // FIR sliding average filter with circular addressing
        if (down_sample_by_10_flag == 1)
        {

            down_sample_by_10_flag = 0;
            // FIR sliding average filter with circular addressing

            sum_x -= x[index];
            sum_x += ADC0_SS0_SAMPLES[0];
            x[index] = ADC0_SS0_SAMPLES[0];

            fir_x = ADC0_SS0_SAMPLES[0];        //sum_x/16  ;

            sum_y -= y[index];
            sum_y += ADC0_SS0_SAMPLES[1];
            y[index] = ADC0_SS0_SAMPLES[1];

            fir_y = ADC0_SS0_SAMPLES[1];        //sum_y/16  ;

            sum_z -= z[index];
            sum_z += ADC0_SS0_SAMPLES[2];
            z[index] = ADC0_SS0_SAMPLES[2];

            fir_z = ADC0_SS0_SAMPLES[2];        //sum_z/16  ;

            index = (index + 1) & 15;

            //printing the average value

            if (average_flag)
            {
                average_flag = 0;
                sprintf(str, "DAC Values and SPL values:           \n");
                putsUart0(str);

                DAC_Value_volts = (float) (ADC0_SS0_SAMPLES[0] * 3.3) / 4096;
                SPL_Value = 20 * log10(DAC_Value_volts / 0.00631) - 44 + 94
                        - 40;

                sprintf(str, "MIC1_DAC:           %.4f\n", DAC_Value_volts);
                putsUart0(str);

                sprintf(str, "MIC1_SPL:           %.4f\n", SPL_Value);
                putsUart0(str);

                DAC_Value_volts = (float) (ADC0_SS0_SAMPLES[1] * 3.3) / 4096;
                SPL_Value = 20 * log10(DAC_Value_volts / 0.00631) - 44 + 94
                        - 40;

                sprintf(str, "MIC2_DAC:           %.4f\n", DAC_Value_volts);
                putsUart0(str);

                sprintf(str, "MIC2_SPL:           %.4f\n", SPL_Value);
                putsUart0(str);

                DAC_Value_volts = (float) (ADC0_SS0_SAMPLES[2] * 3.3) / 4096;
                SPL_Value = 20 * log10(DAC_Value_volts / 0.00631) - 44 + 94
                        - 40;

                sprintf(str, "MIC3_DAC:           %.4f\n", DAC_Value_volts);
                putsUart0(str);

                sprintf(str, "MIC3_SPL:           %.4f\n", SPL_Value);
                putsUart0(str);

            }

            if (ADC0_SS0_SAMPLES[0] > TH)
            {
                t1x = 0;
                Detection = EVENT_MIC_1;
                time_diff_counter = 0;
                event_counter = 0;
                //peak flags
                p1x_flag = Detection;
                p2x_flag = NO_EVENT;
                p3x_flag = NO_EVENT;
                //mic flags
                t1x_flag = 1;
                t2x_flag = 0;
                t3x_flag = 0;
                first_peak_value = fir_x;
                /*sprintf(str, "first event on MIC 1:           \n");
                putsUart0(str);*/

            }
            else if (ADC0_SS0_SAMPLES[1] > TH)
            {
                t2x = 0;
                Detection = EVENT_MIC_2;
                time_diff_counter = 0;
                event_counter = 0;
                //peak flags
                p1x_flag = Detection;
                p2x_flag = NO_EVENT;
                p3x_flag = NO_EVENT;
                //mic flags
                t1x_flag = 0;
                t2x_flag = 1;
                t3x_flag = 0;
                first_peak_value = fir_y;
               /* sprintf(str, "first event on MIC 2:           \n");
                putsUart0(str);*/
            }
            else if (ADC0_SS0_SAMPLES[2] > TH)
            {
                t3x = 0;
                Detection = EVENT_MIC_3;
                time_diff_counter = 0;
                event_counter = 0;
                //peak flags
                p1x_flag = Detection;
                p2x_flag = NO_EVENT;
                p3x_flag = NO_EVENT;
                //mic flags
                t1x_flag = 0;
                t2x_flag = 0;
                t3x_flag = 1;
                first_peak_value = fir_z;
               /* sprintf(str, "first event on MIC 3:           \n");
                putsUart0(str);*/
            }

            while (Detection != NO_EVENT)
            {
                AOA_calc();
            }

            if((fail_flag == 1) && (failed_event_flag == 1))
            {
                failed_event_flag = 0   ;
                if(t1x_flag)
                {
                    sprintf(str, "MIC 1 received the peak:           \n");
                    putsUart0(str);
                }
                else
                {
                    sprintf(str, "MIC 1 didnt received the peak:           \n");
                    putsUart0(str);

                }


                if(t2x_flag)
                {
                    sprintf(str, "MIC 2 received the peak:           \n");
                    putsUart0(str);
                }
                else
                {
                    sprintf(str, "MIC 2 didnt received the peak:           \n");
                    putsUart0(str);

                }


                if(t3x_flag)
                {
                    sprintf(str, "MIC 3 received the peak:           \n");
                    putsUart0(str);
                }
                else
                {
                    sprintf(str, "MIC 3 didnt received the peak:           \n");
                    putsUart0(str);

                }

            }

            if (event_counter == 2)
            {
                event_counter = 0;        //change this later
                //peak 1 print
                if (p1x_flag == EVENT_MIC_1)
                {
                    angle = 60 * (int) ((t3x - t2x)) * K;

                    if (tdoa_flag == 1 || aoa_always_flag == 1)
                    {
                        sprintf(str, "t1x:           %4i\n", t1x);
                        putsUart0(str);
                    }
                }
                else if (p1x_flag == EVENT_MIC_2)
                {
                    angle = 60 * (int) (t1x - t3x) * K;

                    if (tdoa_flag == 1 || aoa_always_flag == 1)
                    {
                        sprintf(str, "t2x:           %4i\n", t2x);
                        putsUart0(str);
                    }
                }
                else if (p1x_flag == EVENT_MIC_3)
                {
                    angle = 60 * (int) (t2x - t1x) * K;            //1/30

                    if (tdoa_flag == 1 || aoa_always_flag == 1)
                    {
                        sprintf(str, "t3x:           %4i\n", t3x);
                        putsUart0(str);
                    }
                }

                if (tdoa_flag == 1 || aoa_always_flag == 1)
                {
                    //peak 2 print
                    if (p2x_flag == EVENT_MIC_1)
                    {
                        sprintf(str, "t1x:           %4i\n", t1x);
                        putsUart0(str);
                    }
                    else if (p2x_flag == EVENT_MIC_2)
                    {
                        sprintf(str, "t2x:           %4i\n", t2x);
                        putsUart0(str);
                    }
                    else if (p2x_flag == EVENT_MIC_3)
                    {
                        sprintf(str, "t3x:           %4i\n", t3x);
                        putsUart0(str);
                    }

                    //peak 3 print
                    if (p3x_flag == EVENT_MIC_1)
                    {
                        sprintf(str, "t1x:           %4i\n", t1x);
                        putsUart0(str);
                    }
                    else if (p3x_flag == EVENT_MIC_2)
                    {
                        sprintf(str, "t2x:           %4i\n", t2x);
                        putsUart0(str);
                    }
                    else if (p3x_flag == EVENT_MIC_3)
                    {
                        sprintf(str, "t3x:           %4i\n", t3x);
                        putsUart0(str);
                    }

                    tdoa_flag = 0;


                    if (aoa_always_flag)
                    {
                        aoa_flag = 0;
                        if (p1x_flag == EVENT_MIC_1)
                        {
                             sprintf(str, "angle W.R.T to MIC 1:           \n");
                             putsUart0(str);
                        }
                        else if (p1x_flag == EVENT_MIC_2)
                        {
                             sprintf(str, "angle W.R.T to MIC 2:           \n");
                             putsUart0(str);
                        }
                        else if (p1x_flag == EVENT_MIC_3)
                        {
                             sprintf(str, "angle W.R.T to MIC 3:           \n");
                             putsUart0(str);
                        }
                        sprintf(str, "angle is :           %.4f\n", angle);
                        putsUart0(str);
                    }

                }

                while (time_diff_counter < hold_off_value) ; //hold_off_value = hold_off_value + time_diff_counter(This should be actual one )

                sprintf(str, "H_OFF\n");
                putsUart0(str);
                //event counter

            }            //event counter

            if (aoa_flag)
            {
                aoa_flag = 0;
                if (p1x_flag == EVENT_MIC_1)
                {
                     sprintf(str, "angle W.R.T to MIC 1:           \n");
                     putsUart0(str);
                }
                else if (p1x_flag == EVENT_MIC_2)
                {
                     sprintf(str, "angle W.R.T to MIC 2:           \n");
                     putsUart0(str);
                }
                else if (p1x_flag == EVENT_MIC_3)
                {
                     sprintf(str, "angle W.R.T to MIC 3:           \n");
                     putsUart0(str);
                }
                sprintf(str, "angle is :           %.4f\n", angle);
                putsUart0(str);
            }

        }            //downsample if ending

    }            //while ending
}
