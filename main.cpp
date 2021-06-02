/*
 * Copyright (c) 2014-2020 Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "rtos.h"
#include "math.h"

Thread thread_01;
Thread thread_02;

I2C i2c0(PTC9, PTC8);   //i2C for BMP180
I2C i2c1(PTC11, PTC10); //i2C for MAX30102

Serial pc(USBTX, USBRX);

const int MAX30102_Slave_Address = 0xAE;  // 8 bit I2C address 1110 111w/r READ
const int BMP180_Slave_Address = 0xEE;    // 8 bit I2C address 1010 111w/r READ

char cmd[2];

/* -------------------------------------------------------------------------- */
/* --------------------------------- CMP180 --------------------------------- */
// --------------------------------- VARIABLES
char BMP180_ID[1];
float AC1;
float AC2;
float AC3;
float AC4;
float AC5;
float AC6;
float B1;
float B2;
float MB;
float MC;
float MD;
float UT;
float UP;
float T;
/* -------------------------------------------------------------------------- */
/* -------------------------------- MAX30102 -------------------------------- */
// -------------------------------- VARIABLES
int SMP_AVE,FIFO_ROLLOVER,FIFO_A_FULL;
int ModeConfig;
int SpO2_ADC_RGE,SpO2_SR,LED_PW;
int LED1_PA, LED2_PA;
int Slot1,Slot2,Slot3,Slot4;
char Temperature_Config[3];
char FIFO_WR_PTR[2],OVF_COUNTER[2],FIFO_RD_PTR[2];
char LED01[3];
long LED01_ADC, LED02_ADC;
/* -------------------------------------------------------------------------- */


/* -------------------------------------------------------------------------- */
/* --------------------------------- CMP180 --------------------------------- */
// --------------------------------- Functions
void BMP180_get_ID() {
    
    cmd[0] = 0xD0;  //Register for 'BMP180 ID'
    i2c0.write(BMP180_Slave_Address, cmd, 1);
    i2c0.read(BMP180_Slave_Address, BMP180_ID, 1);
    printf("%d \r\n ",BMP180_ID[0]);
}

void Calibration_Data() {
    
    short AC1_s;
    short AC2_s;
    short AC3_s;
    unsigned short AC4_s;
    unsigned short AC5_s;
    unsigned short AC6_s;
    short B1_s;
    short B2_s;
    short MB_s;
    short MC_s;
    short MD_s;
    char calib_frame[22];
    
    cmd[0] = 0xAA;  //Register 'Get calibration from data AC1'
    i2c0.write(BMP180_Slave_Address, cmd, 1);
    i2c0.read(BMP180_Slave_Address, calib_frame, 22);
    
    AC1_s =  (calib_frame[0]<<8) | calib_frame[1];
    AC1 = AC1_s;
    printf("AC1: %f \r\n ",AC1);
    AC2_s =  ( (calib_frame[2])<<8 | calib_frame[3] );
    AC2 = AC2_s;
    printf("AC2: %f \r\n ",AC2);
    AC3_s =  ( (calib_frame[4])<<8 | calib_frame[5] );
    AC3 = AC3_s;
    printf("AC3: %f \r\n ",AC3);
    AC4_s =  ( (calib_frame[6])<<8 | calib_frame[7] );
    AC4 = AC4_s;
    printf("AC4: %f \r\n ",AC4);
    AC5_s =  ( (calib_frame[8])<<8 | calib_frame[9] );
    AC5 = AC5_s;
    printf("AC5: %f \r\n ",AC5);
    AC6_s =  ( (calib_frame[10])<<8 | calib_frame[11] );
    AC6 = AC6_s;
    printf("AC6: %f \r\n ",AC6);
    B1_s =  ( (calib_frame[12])<<8 | calib_frame[13] );
    B1 = B1_s;
    printf("B1: %f \r\n ",B1);
    B2_s =  ( (calib_frame[14])<<8 | calib_frame[15] );
    B2 = B2_s;
    printf("B2: %f \r\n ",B2);
    MB_s =  ( (calib_frame[16])<<8 | calib_frame[17] );
    MB = MB_s;
    printf("MB: %f \r\n ",MB);
    MC_s =  ( (calib_frame[18])<<8 | calib_frame[19] );
    MC = MC_s;
    printf("MC: %f \r\n ",MC);
    MD_s =  ( (calib_frame[20])<<8 | calib_frame[21] );
    MD = MD_s;
    printf("MD: %f \r\n ",MD);

}

/*void Uncompensated_Temperature(){
    The code was here now is into Calculated_Temperature function.
}*/

void Uncompensated_Pressure(){
    
    char Uncomp_Pressure[3];
    
    cmd[0] = 0xF4;  //Register for Control ADC
    cmd[1] = 0x34;  //Write 'Start Single Pressure Meansurement'
    i2c0.write(BMP180_Slave_Address, cmd, 2);
    wait_us(4500);
    cmd[0] = 0xF6;  //Register 'Get from ADC Conversion MSB'
    i2c0.write(BMP180_Slave_Address, cmd, 1);
    i2c0.read(BMP180_Slave_Address, Uncomp_Pressure, 3);
    UP = (Uncomp_Pressure[0]<<16) | (Uncomp_Pressure[1]<<8) | Uncomp_Pressure[2];
}

void Calculated_Temperature(){
    
    float X1;
    float X2;
    float B5;
    char Uncomp_Temperature[2];
    
    while(true) {
        cmd[0] = 0xF4;  //Register for Control ADC
        cmd[1] = 0x2E;  //Write 'Start Temp Meansurement'
        i2c0.write(BMP180_Slave_Address, cmd, 2);
        wait_us(4500);
        cmd[0] = 0xF6;  //Register 'Get from ADC Conversion MSB'
        i2c0.write(BMP180_Slave_Address, cmd, 1);
        i2c0.read(BMP180_Slave_Address, Uncomp_Temperature, 2);
        UT = (Uncomp_Temperature[0] << 8) | Uncomp_Temperature[1];
    
        X1 = ((UT - AC6)*AC5)/pow(2.0,15.0);
        X2 = (MC*pow(2.0,11.0))/(X1+MD);
        B5 = X1 + X2;
        T = ((B5 + 8.0)/pow(2.0,4.0))*0.1;
        //printf("Temperature: %0.2f \r\n ",T);
    }
}
// --------------------------------- Functions
/* --------------------------------- CMP180 --------------------------------- */
/* -------------------------------------------------------------------------- */


/* -------------------------------------------------------------------------- */
/* -------------------------------- MAX30102 -------------------------------- */
// -------------------------------- Functions
void Sample_Configuration(){
    //----------------------------------------------------------------------
    //  Sample averagin (SMP_AVE) [b7 - b6 - b5]
    //      value                   samples
    //      000                         1 (no averagin)
    //      001                         2
    //      010                         4
    //      011                         8
    //      100                         16
    //      101                         32
    //      110                         32
    //      111                         32
    //
    //  FIFO rolls on full (FIFO_ROLLOVER) [b4]
    //      This bit controls the behavior of the FIFO when the FIFO
    //      becomes completely filled with data. If FIFO_ROLLOVER_EN 
    //      is set (1), the FIFO address rolls over to zero and the FIFO
    //      continues to fill with new data. If the bit is not set (0), then 
    //      the FIFO is not updated until FIFO_DATA is read or the WRITE/READ
    //      pointer positions are changed.
    //
    //  FIFO Almost full value (FIFO_A_FULL) [b3 - b2 - b1 - b0]
    //      This register sets the number of data samples (3 bytes/sample)
    //      remaining in the FIFO when the interrupt i issued. For 
    //      example, if this field is set to 0x0, the interrupt is issued 
    //      when there is 0 data samples remaining in the FIFO (all 32 
    //      FIFO words have unread data). Furthermore, if this field is set
    //      to 0xF, the interrupt is issued when 15 data samples are 
    //      remaining in the FIFO (17 FIFO data samples have unread data).
    //----------------------------------------------------------------------
    int frame;
    
    SMP_AVE = 0x05;       //modified 19/05/2021 was 010
    FIFO_ROLLOVER = 0x01; //modified 19/05/2021 12:56 was 1
    FIFO_A_FULL = 0x00;
    
    SMP_AVE = SMP_AVE << 5;
    FIFO_ROLLOVER = FIFO_ROLLOVER << 4;
    frame = SMP_AVE | FIFO_ROLLOVER | FIFO_A_FULL;
    
    cmd[0] = 0x08; //Register for 'FIFO configuration'
    cmd[1] = frame;
    i2c1.write(MAX30102_Slave_Address, cmd, 2);   
}

void Mode_Configuration() {
    //----------------------------------------------------------------------
    // 0b00000010 = 0x02: Heart Rate Mode (Red LED only)
    // 0b00000011 = 0x03: SpO2 mode (Red LED and IR LED)
    // 0b00000111 = 0x07: Multi-LED mode (Red LED and IR LED)
    // 0b01000000 = 0x40: Reset all configuration, threshold and data registers
    //----------------------------------------------------------------------
    ModeConfig = 0x02;
    
    cmd[0] = 0x09;  //Register for 'Mode configuration'
    cmd[1] = ModeConfig;
    i2c1.write(MAX30102_Slave_Address, cmd, 2);
}

void SpO2_Configuration(){
    //----------------------------------------------------------------------
    //  SpO2 ADC range control (SpO2_ADC_RGE) [b6 - b5]
    //      00
    //          LSB Size (pA)  : 7.81
    //          Full Scale (nA): 2048
    //      01
    //          LSB Size (pA)  : 15.63
    //          Full Scale (nA): 4096
    //      10
    //          LSB Size (pA)  : 31.25
    //          Full Scale (nA): 8192
    //      11
    //          LSB Size (pA)  : 62.5
    //          Full Scale (nA): 16384
    //
    //  SpO2 sample rate control (SpO2_SR) [b4 - b3- b2]
    //      Value        Samples per second
    //      000                 50
    //      001                 100
    //      010                 200
    //      011                 400
    //      100                 800
    //      101                 1000
    //      110                 1600
    //      111                 3200
    //
    //  LED Pulse with control (LED_PW) [b1 - b0]
    //      00
    //          Pulse with (us)      : 69 (68.95)
    //          ADC resolution (bits): 15
    //      01
    //          Pulse with (us)      : 118 (117.78)
    //          ADC resolution (bits): 16
    //      10
    //          Pulse with (us)      : 215 (215.44)
    //          ADC resolution (bits): 17
    //      11
    //          Pulse with (us)      : 411 (410.75)
    //          ADC resolution (bits): 18
    //----------------------------------------------------------------------
    int frame;
    
    SpO2_ADC_RGE = 0x03;
    SpO2_SR      = 0x06;
    LED_PW       = 0x02;
    
    SpO2_ADC_RGE = SpO2_ADC_RGE << 5;
    SpO2_SR = SpO2_SR << 2;
    frame = SpO2_ADC_RGE | SpO2_SR | LED_PW;
    
    cmd[0] = 0x0A;  //Register for 'SpO2 configuration'
    cmd[1] = frame;
    i2c1.write(MAX30102_Slave_Address, cmd, 2);
}

void LED_Current_Control() {
    //----------------------------------------------------------------------
    //  LED current control (LEDx_PA) [7:0]
    //      0x00: 0.0 mA
    //      0x01: 0.2 mA
    //      0x02: 0.4 mA
    //      ...
    //      0x0F: 3.0 mA
    //      ...
    //      0x1F: 6.2 mA
    //      ...
    //      0x3F: 12.6 mA
    //      ...
    //      0x7F: 25.4 mA
    //      ...
    //      0xFF: 51.0 mA
    //----------------------------------------------------------------------
    LED1_PA = 0x5F;
    LED2_PA = 0x5F;
    
    cmd[0] = 0x0C;  //Register for 'LED current control - LED1_PA'
    cmd[1] = LED1_PA;     //Sending the current for LED1_PA
    i2c1.write(MAX30102_Slave_Address, cmd, 2);
    cmd[0] = 0x0D;  //Register for 'LED current control - LED2_PA'
    cmd[1] = LED2_PA;     //Sending the current for LED2_PA
    i2c1.write(MAX30102_Slave_Address, cmd, 2);
}

void MultiLED_Mode_Control_Slots(){
    //----------------------------------------------------------------------
    //  LED Mode control registers (SLOTx)  [SLOT1: b2 - b1 - b0]
    //                                      [SLOT2: b6 - b5 - b4]
    //                                      [SLOT3: b2 - b1 - b0]
    //                                      [SLOT4: b6 - b5 - b4]
    //
    //  SLOTx         Witch LED is Active         LED pulse amplitude seggion
    //  000       Note, time slot is disabled               N/A, OFF
    //  001                 LED1 (Red)                      LED1_PA[7:0]
    //  010                 LED2 (IR)                       LED2_PA[7:0]
    //  011 - 100           NONE                            N/A, OFF
    //  101 - 111           RESERVERD                       RESERVED
    //----------------------------------------------------------------------
    int slots_1_2, slots_3_4;
    
    Slot1 = 0x01;
    Slot2 = 0x02;
    Slot3 = 0x00;
    Slot4 = 0x00;
    
    slots_1_2 = (Slot2<<4) | (Slot1);
    slots_3_4 = (Slot4<<4) | (Slot3);
    
    cmd[0] = 0x11;      //Register for 'Slot1 and Slot2'
    cmd[1] = slots_1_2;
    i2c1.write(MAX30102_Slave_Address, cmd, 2);
    cmd[0] = 0x12;      //Register for 'Slot3 and Slot4'
    cmd[1] = slots_3_4;
    i2c1.write(MAX30102_Slave_Address, cmd, 2);
}

void Temperature_Data(){
    //----------------------------------------------------------------------
    //  Die teperature integer (TINT) [b7 - b0] READ ONLY
    //      b7      :   sign (+ or -)
    //      b6 - b0 :   integer value
    //
    //  Die temperature fraction (TFRAC) [b3 - b0] READ ONLY
    //      Read the fraction value (after decimal dot)
    //
    //  Die temperature config (TEMP_EN) [b0]
    //      When this set, initialise a single temperature reading
    //      from de temperature sensor.  This bit clears automatically
    //      back to zero at the conclusion of the temperature reading
    //      when the bit is set to one.
    //----------------------------------------------------------------------

  /* cmd[0] = 0x1F;  //Register for 'Die temperature integer'
                    //Register for 'Die temperature fraction'
                    //Register for 'Die temperature config'
    i2c1.write(MAX30102_Slave_Address, cmd, 1);
    i2c1.read(MAX30102_Slave_Address, Temperature_Config, 3);
    
    if (Temperature_Config[2] == 0){
        cmd[0] = 0x21;  //Register for 'Die temperature config'
        cmd[1] = 0x01;  //Set 0x21 for initiates a single temperature reading
        i2c1.write(MAX30102_Slave_Address, cmd, 2);
        
        float temp_integer  = Temperature_Config[0];
        float temp_fraction = Temperature_Config[1];
        float temperature = (temp_integer) + (temp_fraction*0.0625);
    } */
}

void Clear_FIFO(){
    //----------------------------------------------------------------------
    //  Page 15 of manual MAX30102 recomends to set 0x00 all FIFO
    //  before reading new datas.
    //----------------------------------------------------------------------
    FIFO_WR_PTR[0] = 0x04;  //Register for 'FIFO write pointer'
    FIFO_WR_PTR[1] = 0x00;  //value will be set for register
    i2c1.write(MAX30102_Slave_Address, FIFO_WR_PTR, 2);
    
    OVF_COUNTER[0] = 0x05;  //Register for 'Overflow counter'
    OVF_COUNTER[1] = 0x00;  //value will be set for register
    i2c1.write(MAX30102_Slave_Address, OVF_COUNTER, 2);
    
    FIFO_RD_PTR[0] = 0x06;  //Register for 'FIFO read pointer'
    FIFO_RD_PTR[1] = 0x00;  //value will be set for register
    i2c1.write(MAX30102_Slave_Address, FIFO_RD_PTR, 2);
}

void Reading_Heart_Rate(){
    //----------------------------------------------------------------------
    //  This function try out mensure the heart rate
    //  page 16 of manual mentions a example how we can read FIFO DATA
    //----------------------------------------------------------------------
    char LED_Sample[6];
    char Read_PTR[1];
    char Write_PTR[1];
    int i;
    int j;
    int num_available_samples;
    int num_samples_to_read;
    
    while(true){

        cmd[0] = 0x04;
        i2c1.write(MAX30102_Slave_Address, cmd, 1);
        i2c1.read(MAX30102_Slave_Address, Write_PTR, 1);
        cmd[0] = 0x06;
        i2c1.write(MAX30102_Slave_Address, cmd, 1);
        i2c1.read(MAX30102_Slave_Address, Read_PTR, 1);
    
        num_available_samples = Write_PTR - Read_PTR;
        num_samples_to_read   = num_available_samples; 
        
        
        
        for (i = 0 ; i <= num_samples_to_read ; i++)
        {   for (j = 0 ; i < 6 ; i++)
            {LED_Sample[j]=0;}
            cmd[0] = 0x07;  //Register for 'FIFO_DATA'
            i2c1.write(MAX30102_Slave_Address, cmd, 1);
            i2c1.read(MAX30102_Slave_Address, LED_Sample, 3);
                        
            LED01_ADC = (LED_Sample[0]<<16) | (LED_Sample[1]<<8) | (LED_Sample[2]);
            LED01_ADC = LED01_ADC & 0xFFFF;
            printf("%d \r\n ",LED01_ADC);
            //LED02_ADC = (LED_Sample[3]<<16) | (LED_Sample[4]<<8) | (LED_Sample[5]);
            //LED02_ADC = LED02_ADC & 0xFFFF;
            //printf("%d \r\n ",LED02_ADC);
        }
        Thread::wait(5);
    }
}
// -------------------------------- Functions
/* -------------------------------- MAX30102 -------------------------------- */
/* -------------------------------------------------------------------------- */


int main()
{
    pc.baud(115200);
    
    /* --------------------------------------------------------------- BMP180 */
    Calibration_Data();
    Uncompensated_Pressure();
    /* ---------------------------------------------------------------------- */
    
    
    /* ------------------------------------------------------------- MAX30102 */
    //Reset all configuration, threshold and data registers
    cmd[0] = 0x09;  //Register for 'Mode configuration'
    cmd[1] = 0x40;  //Reset all values, threshold and data registers
    i2c1.write(MAX30102_Slave_Address, cmd, 2);
    
    //Set the default values
    Sample_Configuration();
    Mode_Configuration();
    SpO2_Configuration();
    LED_Current_Control();
    MultiLED_Mode_Control_Slots();
    Temperature_Data();
    Clear_FIFO();   //Reset the FIFO before we begin checking the sensor
    /* ---------------------------------------------------------------------- */
    
    
        // -------------------------------------------------------------- BMP180
        thread_01.start(Calculated_Temperature);
        // ---------------------------------------------------------------------
        
        
        // ------------------------------------------------------------ MAX30102
        thread_02.start(Reading_Heart_Rate);
        // ---------------------------------------------------------------------
    
    while (1) {
                                          
                                  /* OTHER CODE */
        // ---------------------------------------------------------------------
        //printf("\t Temperature: %0.2f \t Heart Rate: %d \r\n ",T,LED01_ADC);
        //printf("%d\r\n ",LED02_ADC);
        // ---------------------------------------------------------------------
    }
}