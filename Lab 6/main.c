//Galib Raid
//1001925128

#include <stdio.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "wait.h"
#include "uart0.h"

#define MEAS_LR         (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 0*4))) //PB0
#define MEAS_C          (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 1*4))) //PB1
#define LOWSIDE_R       (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4))) //PC4
#define HIGHSIDE_R      (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4))) //PC5
#define INTEGRATE       (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 3*4))) //PB3
#define PUSH_BUTTON     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4))) //PF4
#define RED_LED         (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4))) //PF1
#define GREEN_LED       (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) //PF2

// Port masks
#define MEAS_LR_MASK    0x01  // PB0
#define MEAS_C_MASK     0x02  // PB1
#define LOWSIDE_R_MASK  0x10  // PC4
#define HIGHSIDE_R_MASK 0x20  // PC5
#define INTEGRATE_MASK 0x08  // PB3
#define PUSH_BUTTON_MASK 0x10 // PF4
#define RED_LED_MASK    0x02  // PF1
#define GREEN_LED_MASK  0x04  // PF2
#define COMP_C0_POS (1 << 6) // Bit 6 for PC6
#define COMP_C0_NEG (1 << 7) // Bit 7 for PC7
#define MIN_RESISTANCE 10       // 10 ohms
#define MAX_RESISTANCE 1000000  // 1 mega-ohm
#define MIN_CAPACITANCE 0.00001  // 10 nF
#define MAX_CAPACITANCE 0.001    // 1000 µF
#define MIN_INDUCTANCE 0.00001  // 10 µH
#define MAX_INDUCTANCE 10       // 10 H


// Global variables for measurements
float R = 0, L = 0, C = 0;

uint32_t elapsed_time = 0;
bool measurement_complete = false;

// Initialize Hardware
void initHW(void)
{
    // Initialize clock to 40Mhz
    initSystemClockTo40Mhz();

    // Enable clocks for ports B, C, and F
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1 | SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    // Set direction of GPIO ports B, C, and F
    GPIO_PORTB_DIR_R |= MEAS_LR_MASK | MEAS_C_MASK | INTEGRATE_MASK;
    GPIO_PORTC_DIR_R |= LOWSIDE_R_MASK | HIGHSIDE_R_MASK ;
    GPIO_PORTF_DIR_R |= RED_LED_MASK | GREEN_LED_MASK;

    // Enable GPIO pins as digital I/Os
    GPIO_PORTB_DEN_R |= MEAS_LR_MASK | MEAS_C_MASK | INTEGRATE_MASK;
    GPIO_PORTC_DEN_R |= LOWSIDE_R_MASK | HIGHSIDE_R_MASK ;
    GPIO_PORTF_DEN_R |= RED_LED_MASK | GREEN_LED_MASK | PUSH_BUTTON_MASK;
}

void initComp(void)
{
    // Enable clock for comparator and port C
    SYSCTL_RCGCACMP_R |= SYSCTL_RCGCACMP_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;  // Enable clock for port C
    _delay_cycles(3);

    // Configure PC6 (C0+) and PC7 (C0-) for comparator function
    GPIO_PORTC_AFSEL_R |= COMP_C0_POS | COMP_C0_NEG; // Enable alternate function for PC6 and PC7
    GPIO_PORTC_DEN_R &= ~(COMP_C0_POS | COMP_C0_NEG); // Disable digital function for PC6 and PC7
    GPIO_PORTC_AMSEL_R |= COMP_C0_POS | COMP_C0_NEG; // Enable analog function for PC6 and PC7

    // Configure internal reference (VREF) for C0+
    COMP_ACREFCTL_R |= COMP_ACREFCTL_EN;      // Enable resistor ladder
    COMP_ACREFCTL_R &= ~COMP_ACREFCTL_RNG;    // Set range to low voltage (0-3V)
    COMP_ACREFCTL_R |= COMP_ACREFCTL_VREF_M;  // Set desired VREF (e.g., 2.469V)

    // Set C0+ to VREF and C0- to PC7
    COMP_ACCTL0_R |= COMP_ACCTL0_ASRCP_REF;   // Set C0+ to VREF
    COMP_ACCTL0_R &= ~COMP_ACCTL0_ASRCP_PIN; // Ensure C0+ is not using an external pin

    // Additional configurations for comparator
    COMP_ACCTL0_R |= COMP_ACCTL0_CINV | COMP_ACCTL0_ISLVAL; // Configure output logic and interrupt

    // Allow comparator to stabilize
    waitMicrosecond(1000000);

    // Enable interrupt for Comparator 0
    NVIC_EN0_R = 1 << (INT_COMP0 - 16);
}


void initTimer(void)
{
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    _delay_cycles(3);

    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // Turn off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // Configure as 32-bit timer
    TIMER1_TAMR_R |= TIMER_TAMR_TACDIR | TIMER_TAMR_TAMR_1_SHOT; // Config for 1 shot and count up
    TIMER1_TAV_R = 0; // Reset the timer
}

void CompISR(void)
{
    elapsed_time = TIMER1_TAV_R;  // Read timer
    GREEN_LED = 1;  //LED for debug
    LOWSIDE_R = 1; //Discharge
    COMP_ACMIS_R |= COMP_ACMIS_IN0; // Clear interrupt flag
    measurement_complete = 1;
}


void waitSW1Press(void)
{
    while(PUSH_BUTTON);
}
void autoDetectComponent()
{
    // Step 1: Measure electrical properties
    // Use the global variables R, C, and L here directly
    bool isResistor = (R > MIN_RESISTANCE && R < MAX_RESISTANCE);
    bool isCapacitor = (C > MIN_CAPACITANCE && C < MAX_CAPACITANCE);
    bool isInductor = (L > MIN_INDUCTANCE && L < MAX_INDUCTANCE);

    // Step 2: Identify component
    if (isResistor && !isCapacitor && !isInductor)
    {
        printf("Detected component: Resistor\n");
    }
    else if (isCapacitor && !isResistor && !isInductor)
    {
        printf("Detected component: Capacitor\n");
    }
    else if (isInductor && !isResistor && !isCapacitor)
    {
        printf("Detected component: Inductor\n");
    }
    else
    {
        printf("Error: Unable to identify component.\n");
    }
}



int main(void)
{
    initHW();
    initTimer();
    initComp();
    initUart0();
    float R,L,C;
    char str[80];
    setUart0BaudRate(115200, 40e6);
    while(true)
    {
        putsUart0("Enter r,c,l,a To Measurement..... \n");
        putcUart0('>');
        char ch = getcUart0();

        //Measure R
        if (ch == 'r')
        {
            INTEGRATE = 1;
            MEAS_LR = 1;
            LOWSIDE_R =1;               //Discharge
            waitMicrosecond(1000000);
            TIMER1_TAV_R = 0;
            measurement_complete = false;
            COMP_ACINTEN_R |= COMP_ACINTEN_IN0;
            LOWSIDE_R = 0;              //Charge
            TIMER1_CTL_R |= TIMER_CTL_TAEN;
            putsUart0("\nCalculating ...\n");
            while(!measurement_complete);
            RED_LED = 0;
            R = ((float)elapsed_time/40000000.0)/(1.379*0.000001);
            snprintf(str, sizeof(str), "Resistance =   %f Ohm\n", R);
            putsUart0(str);
        }

        //Measure C
        if (ch == 'c')
        {
            MEAS_C = 1;
            HIGHSIDE_R = 1;
            LOWSIDE_R =1;               //Discharge
            waitMicrosecond(2000000);
            LOWSIDE_R = 0;              //Charge
            TIMER1_TAV_R = 0;
            measurement_complete = 0;
            COMP_ACINTEN_R |= COMP_ACINTEN_IN0;
            TIMER1_CTL_R |= TIMER_CTL_TAEN;
            putsUart0("\nCalculating Capacitor ...\n");
            while(!measurement_complete);    //wait until it true
            RED_LED = 0;
            C = ((float)elapsed_time/40000000.0)/(1.379*100000);
            snprintf(str, sizeof(str), "Capacitance =   %lf F\n", C);
            putsUart0(str);
        }

        //Measure L
        if (ch == 'l')
        {
            HIGHSIDE_R = 0;
            MEAS_C =0;
            INTEGRATE =0;
            LOWSIDE_R = 0;
            measurement_complete = 0;
            LOWSIDE_R = 1;
            COMP_ACMIS_R = COMP_ACMIS_IN0;
            MEAS_LR = 1 ;                   //charge
            TIMER1_TAV_R = 0;
            TIMER1_CTL_R |= TIMER_CTL_TAEN;
            COMP_ACINTEN_R |= COMP_ACINTEN_IN0;
            putsUart0("\nCalculating  ...\n");
            while(!measurement_complete);
            RED_LED = 0;
            L = ((float)elapsed_time/40000000.0)*33/1.379;  //(t/40MHz)
            L = L / 10000000.0;
            LOWSIDE_R = 0;
            MEAS_LR = 0;
            snprintf(str, sizeof(str), "Inductance =   %f L\n", L);
            putsUart0(str);
        }

        // Auto-Detect Component
        if (ch == 'a')
        {
            putsUart0("\nAuto-detecting component...\n");
            autoDetectComponent();
        }
    }
}
