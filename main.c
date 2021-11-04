/*
 * main.c
 *
 *  Created on: Jun 16, 2020
 *      Author: Tyler Davidson
 *     Updated: Aug, 13, 2020
 *    Modified: Nov 3, 2020
 * Modified by: Nicholas, William, and Jiamiao
 *
 */

#include "msp.h"
#include "i2c.h"
#include "stIMU.h"
#include "stdio.h"
#include "button.h"

/**
 * main.c
 */

volatile int state = 0;

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    //disable interrupts
    __disable_irq();

    config_button();
    config_nvic_button();

    //enable interrupts
    __enable_irq();

    I2C_OPEN_STRUCT_TypeDef i2c_open_struct;

    i2c_open_struct.uca10 = 0;                              //no self 10-bit Address
    i2c_open_struct.ucsla10 = 0;                            //no 10-bit Addressing
    i2c_open_struct.ucmm = 0;                               //No Multi-Controller
    i2c_open_struct.ucmst = EUSCI_B_CTLW0_MST;              //Controller Mode
    i2c_open_struct.ucsselx = EUSCI_B_CTLW0_SSEL__SMCLK;    //SMCLK to be selected (3MHz)
    i2c_open_struct.ucbrx = 30;                             //Prescaler for Selected Clock.
                                                            //(100kHz)
     //This sets up the I2C driver to operate with the correct settings.
     //EUSCI_B0 uses P1.7 as SCL and P1.6 as SDA
     i2c_open(EUSCI_B0, &i2c_open_struct);

     config_LIS3MDL();

     int16_t mx, my, mz, rx, ry, rz;
     uint8_t data;
     int i;

     // Pointer to an int
     int *adjusted_data;

     //Neutral Position
     while(state == 0)
     {
         //delay
         for(i=0; i<200000; i++);
         i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &data, 2, LIS3MDL_MAG_STATUS_REG);
         printf("status 1: %X\n",data);
         //delay
         for(i=0; i<1000; i++);
         mx = read_magnetometer_x();
         my = read_magnetometer_y();
         mz = read_magnetometer_z();


         // ------------ Test line to set sensitivity ---------
         // Parameter can be adjusted to test different full scale values

         set_magnetometer_sensitivity(LIS3MDL_MAG_FS_4GA);

         // ----------------------------------------------------

         printf("x: %d\n", mx);
         printf("y: %d\n", my);
         printf("z: %d\n", mz);
         printf("\n");
         printf("\n");

         // Call function and display the adjusted data from get_axis_data()
         adjusted_data = get_axis_data();

         printf("Scaled x: %d\n", *(adjusted_data+0));
         printf("Scaled y: %d\n", *(adjusted_data+1));
         printf("Scaled z: %d\n", *(adjusted_data+2));

         printf("\n");
         printf("\n");

         // Populate values to check that ratio of adjusted sensitivity is correct
         rx = *(adjusted_data+0) / mx;
         ry = *(adjusted_data+1) / my;
         rz = *(adjusted_data+2) / mz;


         // Print expected ratio and calculated ratio to make sure get_axis_data() scales correctly
         printf("Expected ratio with  %d\n", get_magnetometer_sensitivity());
         printf("Scaled x ratio: %d\n", rx);
         printf("Scaled y ratio: %d\n", ry);
         printf("Scaled z ratio: %d\n", rz);

         printf("\n");
         printf("\n");




         __nop();
         __nop();
         __nop();
         __nop();
         __nop();
         __nop();
         __nop();
         __nop();
     }
     // State is switched in the Port 1 IRQ Handler
     while(state == 1){
         //after the button is pressed, the IMU will no longer be communicating with the MSP
         //Will only communicate when you want to exchange information so we are not waiting for information or polliing
    }
}

/* Port1 ISR */
void PORT1_IRQHandler(void)
{
    volatile uint32_t j;

    //Check flag
    if(P1->IFG & BIT1)
        //Change State to stop reading from the IMU
        state = 1;

    // Delay for switch debounce, can use __no_operation() instead if you want!
    for(j = 0; j < 100000; j++)

    //end of interrupt, clear the flag
    P1->IFG &= ~BIT1;
}
