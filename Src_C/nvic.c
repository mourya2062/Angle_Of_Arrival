// NVIC Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration: -

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include "nvic.h"
#include "tm4c123gh6pm.h"

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
//The NVIC in Cortex -M processors assigns the first 16(0 to 15) for internal use.
//So peripheral vector number starts from 16 but interrupt number(Bit in interrupt Register) starts from 0
//In total we have 4 registers for interrupt enable  and 4 registers for interrupt disable
void enableNvicInterrupt(uint8_t vectorNumber)
{
    volatile uint32_t* p = (uint32_t*) &NVIC_EN0_R; //assigning pointer p with address of EN0 register
    vectorNumber -= 16;                             //subtracting 16 from the vector number to get the interrupt number
    p += vectorNumber >> 5;                         //incrementing the pointer p if the interrupt number is greater than 31,incrementing value = interrupt_number/32
    *p = 1 << (vectorNumber & 31);                  //setting the interrupt number bit to enable the interrupt
}

void disableNvicInterrupt(uint8_t vectorNumber)
{
    volatile uint32_t* p = (uint32_t*) &NVIC_DIS0_R;
    vectorNumber -= 16;
    p += vectorNumber >> 5;
    *p = 1 << (vectorNumber & 31);
}

void setNvicInterruptPriority(uint8_t vectorNumber, uint8_t priority)
{
    volatile uint32_t* p = (uint32_t*) &NVIC_PRI0_R;
    vectorNumber -= 16;
    uint32_t shift = 5 + (vectorNumber & 3) * 8;
    p += vectorNumber >> 2;
    *p &= ~(7 << shift);
    *p |= priority << shift;
}

