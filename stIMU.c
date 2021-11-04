/*
 * stIMU.c
 *
 *  Created on: Aug 2, 2020
 *      Author: Ariel Getter
 *       https://github.com/STMicroelectronics/STM32CubeL4/blob/master/Drivers/BSP/Components/lis3mdl/lis3mdl.c
 *    Modified: Nov 3, 2020
 * Modified by: Nicholas, William, and Jiamiao
 */

#include "stIMU.h"

const int16_t sens4g = 6842;

/****** TOP MAGNETOMETER ******/
void config_LIS3MDL(void){
        //SEND WHO AM I
        //printf to console for debugging
       int i;
       for (i=0; i<10000;i++); // Start-up time
       printf("Connecting to LIS3MDL...\n");
       for(i=0; i<100000; i++); // Time delay for start up
       printf("...\n");
       for(i=0; i<100; i++); // Time delay for start up
       printf("...\n");
       for(i=0; i<100; i++);// Time delay for start up

       uint8_t whoami;
       i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &whoami, 1, LIS3MDL_MAG_WHO_AM_I_REG);
       i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &whoami, 1, LIS3MDL_MAG_WHO_AM_I_REG);
       printf("Access Address: ");
       printf("%X\n", whoami); //for debug - print address in hex
       if(whoami == LIS3MDL_MAG_WHO_AM_I){
           printf("MAGNETOMETER Accessed @ Address: %X\n",whoami); //for debug - print address in hex
       }

       uint8_t array[2];
       array[0] = LIS3MDL_MAG_CTRL_REG1;
       array[1] = CTR_REG1_SET;
       i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, WRITE, array, 2, LIS3MDL_MAG_CTRL_REG1);
       __nop();
       __nop();
       __nop();
       __nop();
       i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &whoami, 1, LIS3MDL_MAG_CTRL_REG1);
       i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &whoami, 1, LIS3MDL_MAG_CTRL_REG1);
       printf("Reading LIS3MDL_MAG_CTRL_REG1 (3E): %X\n", whoami);

       array[0] = LIS3MDL_MAG_CTRL_REG2;
       array[1] = CTR_REG2_SET;
       i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, WRITE, array, 2, LIS3MDL_MAG_CTRL_REG2);

       __nop();
       __nop();
       __nop();
       __nop();
       i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &whoami, 1, LIS3MDL_MAG_CTRL_REG2);
       i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &whoami, 1, LIS3MDL_MAG_CTRL_REG2);
       printf("Reading LIS3MDL_MAG_CTRL_REG2 (60): %X\n", whoami);

       array[0] = LIS3MDL_MAG_CTRL_REG3;
       array[1] = CTR_REG3_SET;
       i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, WRITE, array, 2, LIS3MDL_MAG_CTRL_REG3);
       __nop();
       __nop();
       __nop();
       __nop();
       i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &whoami, 1, LIS3MDL_MAG_CTRL_REG3);
       i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &whoami, 1, LIS3MDL_MAG_CTRL_REG3);
       printf("Reading LIS3MDL_MAG_CTRL_REG3 (00): %X\n", whoami);

       array[0] = LIS3MDL_MAG_CTRL_REG4;
       array[1] = CTR_REG4_SET;
       i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, WRITE, array, 2, LIS3MDL_MAG_CTRL_REG4);

       __nop();
       __nop();
       __nop();
       __nop();
       i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &whoami, 1, LIS3MDL_MAG_CTRL_REG4);
       i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &whoami, 1, LIS3MDL_MAG_CTRL_REG4);
       printf("Reading LIS3MDL_MAG_CTRL_REG4 (0C): %X\n", whoami);

       array[0] = LIS3MDL_MAG_CTRL_REG5;
       array[1] = CTR_REG5_SET;
       i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, WRITE, array, 2, LIS3MDL_MAG_CTRL_REG5);
       __nop();
       __nop();
       __nop();
       __nop();
       i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &whoami, 1, LIS3MDL_MAG_CTRL_REG5);
       i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &whoami, 1, LIS3MDL_MAG_CTRL_REG5);
       printf("Reading LIS3MDL_MAG_CTRL_REG4 (40): %X\n", whoami);
       printf("...\n");
}
int16_t read_magnetometer_x(void){

        int8_t mxh, mxl;
        int16_t mx;
        const int16_t sens4g = 6842;

        mx = 0x0000;

        i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &mxh, 1, LIS3MDL_MAG_OUTX_H);
        i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &mxh, 1, LIS3MDL_MAG_OUTX_H);

        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();

        i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &mxl, 1, LIS3MDL_MAG_OUTX_L);
        i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &mxl, 1, LIS3MDL_MAG_OUTX_L);

        mx = mxh << 8;
        mx = mx + mxl;

        return mx;
};
int16_t read_magnetometer_y(void){
        int8_t myh, myl;
        int16_t my;
        const int16_t sens4g = 6842;

        my = 0x0000;
        i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &myh, 1, LIS3MDL_MAG_OUTY_H);
        i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &myh, 1, LIS3MDL_MAG_OUTY_H);

        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();

        i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &myl, 1, LIS3MDL_MAG_OUTY_L);
        i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &myl, 1, LIS3MDL_MAG_OUTY_L);
	
        my = myh << 8;
        my = (my + myl);

        return my;
	
};
int16_t read_magnetometer_z(void){
	    int8_t mzh,mzl;
	    int16_t mz;
	    const int16_t sens4g = 6842;

	    mz = 0x0000;
	    i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &mzh, 1, LIS3MDL_MAG_OUTZ_H);
	    i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &mzh, 1, LIS3MDL_MAG_OUTZ_H);

	       __nop();
	       __nop();
	       __nop();
	       __nop();
	       __nop();
	       __nop();
	       __nop();
	       __nop();

	    i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &mzl, 1, LIS3MDL_MAG_OUTZ_L);
	    i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &mzl, 1, LIS3MDL_MAG_OUTZ_L);

	    mz = mzh << 8;
	    mz = mz + mzl;
	
	    return mz;
	
};
/****** BOTTOM MAGNETOMETER ******/

void set_magnetometer_sensitivity(LIS3MDL_MAG_FS_t x)
{

   // Create array of wanted data for struct
    uint8_t array[2];
    array[0] = LIS3MDL_MAG_CTRL_REG2;
    array[1] = x;

    // Load data into struct
    i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, WRITE, array, 2, LIS3MDL_MAG_CTRL_REG2);


}

uint8_t get_magnetometer_sensitivity(void)
{
    uint8_t FS_Selection = 0x00;
    uint8_t FS;

    i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &FS, 1, LIS3MDL_MAG_CTRL_REG2);
    i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &FS, 1, LIS3MDL_MAG_CTRL_REG2);

    FS_Selection = FS << 8;


    uint8_t sensitivity;


    if (FS_Selection == LIS3MDL_MAG_FS_4GA)
    {
        sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_4G;
    }
    if (FS_Selection == LIS3MDL_MAG_FS_8GA)
    {
        sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_8G;
    }
    if (FS_Selection == LIS3MDL_MAG_FS_12GA)
    {
        sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_12G;
    }
    if (FS_Selection == LIS3MDL_MAG_FS_16GA)
    {
        sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_16G;
    }
    return sensitivity;
}

int * get_axis_data(void)
{
    static int p[3];

    // Calculate scaled values
    p[0] = get_magnetometer_sensitivity() * read_magnetometer_x();
    p[1] = get_magnetometer_sensitivity() * read_magnetometer_y();
    p[2] = get_magnetometer_sensitivity() * read_magnetometer_z();


    return p;

}
