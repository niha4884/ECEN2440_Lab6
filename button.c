/*
 * button.c
 *
 *  Created on: Sep 30, 2020
 *      Author: Arielle Blum
 */

#include "button.h"

/* Configure the S1 Button */
void config_button(void)
{
    //Set up Port 1 Pin 1
    P1->DIR &= ~BIT1;
    P1->OUT |= BIT1;
    P1->REN |= BIT1;
    //Configure the interrupt for P1.1
    P1->IES |= BIT1;
    P1->IFG = 0;        //Clear flag
    P1->IE  |= BIT1;    //Enable Interrupt for Port 1
}

/* Configure NVIC for Interrupt Source */
void config_nvic_button(void)
{
    //enable interrupts
    NVIC_EnableIRQ(PORT1_IRQn);//configure the NVIC for the button interrupt source
}


