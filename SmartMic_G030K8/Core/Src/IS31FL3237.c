/*
 * IS31FL3237.c
 *
 *  Created on: 2021年10月21日
 *      Author: Hal.Tsai
 */

//----------Header----------
#ifndef __IS31FL3237_C__
#define __IS31FL3237_C__

//----------Private Include----------
#include "IS31FL3237.h"

//----------System Include----------
#include "main.h"

//----------System Define----------
typedef union {
    unsigned char u8Byte;
    struct{
        unsigned bit0:1;
        unsigned bit1:1;
        unsigned bit2:1;
        unsigned bit3:1;
        unsigned bit4:1;
        unsigned bit5:1;
        unsigned bit6:1;
        unsigned bit7:1;
    } sb;
} xu_flag;

//----------Private Variable----------

//----------Global Variable----------
extern I2C_HandleTypeDef hi2c1;

//----------Private Function----------
unsigned char I2C_WriteByte(int DevAddr, unsigned char RegAddr, unsigned char u8Byte);

//----------Global Function----------
extern HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);

//----------Code Implement----------
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
void Hal_LED_Driver_Initial(void) {

    //Configure Global power
    if(1) {
        static const unsigned char LED_Driver_All_Power = 50;  //MIN:0, MAX:255

        //LED Driver 1
        I2C_WriteByte(AD_SEL1_LOW,  0x6E, LED_Driver_All_Power);    //GCC

        //LED Driver 2
        I2C_WriteByte(AD_SEL1_HIGH, 0x6E, LED_Driver_All_Power);    //GCC
    }

    //Configure each LED power, Min:0x00, Max:0xFF
    if(1) {
        static const unsigned char LED_Driver_Each_Power =  50;  //MIN:0, MAX:255
        static const unsigned char LED_array_Power_R     = 100;  //MIN:0, MAX:255
        static const unsigned char LED_array_Power_G     = 180;  //MIN:0, MAX:255

        static const unsigned char LED_Power_R           =  24;  //MIN:0, MAX:255
        static const unsigned char LED_Power_G           =   3;  //MIN:0, MAX:255
        static const unsigned char LED_Power_B           =  12;  //MIN:0, MAX:255

        I2C_WriteByte(AD_SEL1_LOW,  Power_array_LED2_no1 , LED_array_Power_G);      //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  Power_array_LED2_no2 , LED_array_Power_G);      //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  Power_array_LED2_no3 , LED_array_Power_G);      //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  Power_array_LED2_no4 , LED_array_Power_G);      //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  Power_array_LED2_no5 , LED_array_Power_G);      //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  Power_array_LED2_no6 , LED_Driver_Each_Power);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  Power_array_LED2_no7 , LED_Driver_Each_Power);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  Power_array_LED2_no8 , LED_array_Power_R);      //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  Power_array_LED2_no9 , LED_array_Power_R);      //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  Power_array_LED2_no10, LED_array_Power_R);      //write all scaling

        I2C_WriteByte(AD_SEL1_LOW,  Power_array_LED1_no1 , LED_array_Power_G);      //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  Power_array_LED1_no2 , LED_array_Power_G);      //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  Power_array_LED1_no3 , LED_array_Power_G);      //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  Power_array_LED1_no4 , LED_array_Power_G);      //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  Power_array_LED1_no5 , LED_array_Power_G);      //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  Power_array_LED1_no6 , LED_Driver_Each_Power);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  Power_array_LED1_no7 , LED_Driver_Each_Power);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  Power_array_LED1_no8 , LED_array_Power_R);      //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  Power_array_LED1_no9 , LED_array_Power_R);      //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  Power_array_LED1_no10, LED_array_Power_R);      //write all scaling

        I2C_WriteByte(AD_SEL1_LOW,  Power_LED9_R         , LED_Power_R);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  Power_LED9_G         , LED_Power_G);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  Power_LED9_B         , LED_Power_B);  //write all scaling

        //SW6
        I2C_WriteByte(AD_SEL1_LOW,  Power_LED10_R        , LED_Power_R);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  Power_LED10_G        , LED_Power_G);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  Power_LED10_B        , LED_Power_B);  //write all scaling

        I2C_WriteByte(AD_SEL1_LOW,  Power_LED11_R        , LED_Power_R);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  Power_LED11_G        , LED_Power_G);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  Power_LED11_B        , LED_Power_B);  //write all scaling

        I2C_WriteByte(AD_SEL1_LOW,  Power_LED12_R        , LED_Power_R);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  Power_LED12_G        , LED_Power_G);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  Power_LED12_B        , LED_Power_B);  //write all scaling

        //SW5
        I2C_WriteByte(AD_SEL1_HIGH, Power_LED13_R        , LED_Power_R);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, Power_LED13_B        , LED_Power_B);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, Power_LED13_G        , LED_Power_G);  //write all scaling

        I2C_WriteByte(AD_SEL1_HIGH, Power_LED14_R        , LED_Power_R);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, Power_LED14_B        , LED_Power_B);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, Power_LED14_G        , LED_Power_G);  //write all scaling

        I2C_WriteByte(AD_SEL1_HIGH, Power_LED17_R        , LED_Power_R);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, Power_LED17_B        , LED_Power_B);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, Power_LED17_G        , LED_Power_G);  //write all scaling

        //SW4
        I2C_WriteByte(AD_SEL1_HIGH, Power_LED15_R        , LED_Power_R);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, Power_LED15_B        , LED_Power_B);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, Power_LED15_G        , LED_Power_G);  //write all scaling

        I2C_WriteByte(AD_SEL1_HIGH, Power_LED16_R        , LED_Power_R);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, Power_LED16_B        , LED_Power_B);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, Power_LED16_G        , LED_Power_G);  //write all scaling

        I2C_WriteByte(AD_SEL1_HIGH, Power_LED18_R        , LED_Power_R);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, Power_LED18_B        , LED_Power_B);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, Power_LED18_G        , LED_Power_G);  //write all scaling

        //SW3
        I2C_WriteByte(AD_SEL1_HIGH, Power_LED19_R        , LED_Power_R);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, Power_LED19_B        , LED_Power_B);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, Power_LED19_G        , LED_Power_G);  //write all scaling

        I2C_WriteByte(AD_SEL1_HIGH, Power_LED20_R        , LED_Power_R);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, Power_LED20_B        , LED_Power_B);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, Power_LED20_G        , LED_Power_G);  //write all scaling

        I2C_WriteByte(AD_SEL1_HIGH, Power_LED21_R        , LED_Power_R);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, Power_LED21_B        , LED_Power_B);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, Power_LED21_G        , LED_Power_G);  //write all scaling
    }

    //Clear all PWM H/L value
    if(1) {
        static const unsigned char LED_Driver_PWM_Value = 0;  //MIN:0, MAX:255

        I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED2_no1 , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED2_no2 , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED2_no3 , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED2_no4 , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED2_no5 , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED2_no6 , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED2_no7 , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED2_no8 , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED2_no9 , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED2_no10, LED_Driver_PWM_Value);  //write all scaling

        I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED1_no1 , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED1_no2 , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED1_no3 , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED1_no4 , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED1_no5 , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED1_no6 , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED1_no7 , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED1_no8 , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED1_no9 , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED1_no10, LED_Driver_PWM_Value);  //write all scaling

        I2C_WriteByte(AD_SEL1_LOW,  PWM_LED9_R         , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  PWM_LED9_G         , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  PWM_LED9_B         , LED_Driver_PWM_Value);  //write all scaling

        //SW6
        I2C_WriteByte(AD_SEL1_LOW,  PWM_LED10_R        , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  PWM_LED10_G        , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  PWM_LED10_B        , LED_Driver_PWM_Value);  //write all scaling

        I2C_WriteByte(AD_SEL1_LOW,  PWM_LED11_R        , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  PWM_LED11_G        , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  PWM_LED11_B        , LED_Driver_PWM_Value);  //write all scaling

        I2C_WriteByte(AD_SEL1_LOW,  PWM_LED12_R        , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  PWM_LED12_G        , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_LOW,  PWM_LED12_B        , LED_Driver_PWM_Value);  //write all scaling

        //SW5
        I2C_WriteByte(AD_SEL1_HIGH, PWM_LED13_R        , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, PWM_LED13_B        , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, PWM_LED13_G        , LED_Driver_PWM_Value);  //write all scaling

        I2C_WriteByte(AD_SEL1_HIGH, PWM_LED14_R        , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, PWM_LED14_B        , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, PWM_LED14_G        , LED_Driver_PWM_Value);  //write all scaling

        I2C_WriteByte(AD_SEL1_HIGH, PWM_LED17_R        , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, PWM_LED17_B        , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, PWM_LED17_G        , LED_Driver_PWM_Value);  //write all scaling

        //SW4
        I2C_WriteByte(AD_SEL1_HIGH, PWM_LED15_R        , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, PWM_LED15_B        , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, PWM_LED15_G        , LED_Driver_PWM_Value);  //write all scaling

        I2C_WriteByte(AD_SEL1_HIGH, PWM_LED16_R        , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, PWM_LED16_B        , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, PWM_LED16_G        , LED_Driver_PWM_Value);  //write all scaling

        I2C_WriteByte(AD_SEL1_HIGH, PWM_LED18_R        , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, PWM_LED18_B        , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, PWM_LED18_G        , LED_Driver_PWM_Value);  //write all scaling

        //SW3
        I2C_WriteByte(AD_SEL1_HIGH, PWM_LED19_R        , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, PWM_LED19_B        , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, PWM_LED19_G        , LED_Driver_PWM_Value);  //write all scaling

        I2C_WriteByte(AD_SEL1_HIGH, PWM_LED20_R        , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, PWM_LED20_B        , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, PWM_LED20_G        , LED_Driver_PWM_Value);  //write all scaling

        I2C_WriteByte(AD_SEL1_HIGH, PWM_LED21_R        , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, PWM_LED21_B        , LED_Driver_PWM_Value);  //write all scaling
        I2C_WriteByte(AD_SEL1_HIGH, PWM_LED21_G        , LED_Driver_PWM_Value);  //write all scaling
    }

    //Save Power and PWM value
    I2C_WriteByte(AD_SEL1_LOW,  0x49, 0x00);   //update PWM & congtrol register
    I2C_WriteByte(AD_SEL1_HIGH, 0x49, 0x00);   //update PWM & congtrol register

    //Configure 16MHz, 8bits PWM, Normal power on mode
    I2C_WriteByte(AD_SEL1_LOW,  0x00, 0x01);   //normal operation
    I2C_WriteByte(AD_SEL1_HIGH, 0x00, 0x01);   //normal operation
}
/*----------------------------------------------------------------------------*/
void Task_LED_Driver(void) {
    char isUpdate = 0;

    if(1) {  //array LED
        static xu_arrayLED arrayLED_Save[2] = {0};

        if(arrayLED_Save[0].u16Byte != arrayLED[0]) {
            arrayLED_Save[0].u16Byte = arrayLED[0];

            isUpdate = 1;

            I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED1_no1 , (arrayLED_Save[0].sb.bit0)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED1_no2 , (arrayLED_Save[0].sb.bit1)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED1_no3 , (arrayLED_Save[0].sb.bit2)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED1_no4 , (arrayLED_Save[0].sb.bit3)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED1_no5 , (arrayLED_Save[0].sb.bit4)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED1_no6 , (arrayLED_Save[0].sb.bit5)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED1_no7 , (arrayLED_Save[0].sb.bit6)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED1_no8 , (arrayLED_Save[0].sb.bit7)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED1_no9 , (arrayLED_Save[0].sb.bit8)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED1_no10, (arrayLED_Save[0].sb.bit9)? 100:0 );  //write all scaling
        }

        if(arrayLED_Save[1].u16Byte != arrayLED[1]) {
            arrayLED_Save[1].u16Byte = arrayLED[1];

            isUpdate = 1;

            I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED2_no1 , (arrayLED_Save[1].sb.bit0)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED2_no2 , (arrayLED_Save[1].sb.bit1)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED2_no3 , (arrayLED_Save[1].sb.bit2)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED2_no4 , (arrayLED_Save[1].sb.bit3)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED2_no5 , (arrayLED_Save[1].sb.bit4)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED2_no6 , (arrayLED_Save[1].sb.bit5)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED2_no7 , (arrayLED_Save[1].sb.bit6)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED2_no8 , (arrayLED_Save[1].sb.bit7)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED2_no9 , (arrayLED_Save[1].sb.bit8)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_LOW,  PWM_array_LED2_no10, (arrayLED_Save[1].sb.bit9)? 100:0 );  //write all scaling
        }
    }

    if(1) {
        static xu_flag LED9_Save = {0};

        if(LED9_Save.u8Byte != LED9) {
            LED9_Save.u8Byte = LED9;

            isUpdate = 1;

            I2C_WriteByte(AD_SEL1_LOW, PWM_LED9_R          , (LED9_Save.sb.bit0)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_LOW, PWM_LED9_G          , (LED9_Save.sb.bit1)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_LOW, PWM_LED9_B          , (LED9_Save.sb.bit2)? 100:0 );  //write all scaling
        }
    }

    if(1) {
        static xu_flag SW3_LED19_Save = {0};
        static xu_flag SW3_LED20_Save = {0};
        static xu_flag SW3_LED21_Save = {0};

        if(SW3_LED19_Save.u8Byte != SW3_LED19) {
            SW3_LED19_Save.u8Byte = SW3_LED19;

            isUpdate = 1;

            I2C_WriteByte(AD_SEL1_HIGH, PWM_LED19_R        , (SW3_LED19_Save.sb.bit0)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_HIGH, PWM_LED19_G        , (SW3_LED19_Save.sb.bit1)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_HIGH, PWM_LED19_B        , (SW3_LED19_Save.sb.bit2)? 100:0 );  //write all scaling
        }

        if(SW3_LED20_Save.u8Byte != SW3_LED20) {
            SW3_LED20_Save.u8Byte = SW3_LED20;

            isUpdate = 1;

            I2C_WriteByte(AD_SEL1_HIGH, PWM_LED20_R        , (SW3_LED20_Save.sb.bit0)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_HIGH, PWM_LED20_G        , (SW3_LED20_Save.sb.bit1)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_HIGH, PWM_LED20_B        , (SW3_LED20_Save.sb.bit2)? 100:0 );  //write all scaling
        }

        if(SW3_LED21_Save.u8Byte != SW3_LED21) {
            SW3_LED21_Save.u8Byte = SW3_LED21;

            isUpdate = 1;

            I2C_WriteByte(AD_SEL1_HIGH, PWM_LED21_R        , (SW3_LED21_Save.sb.bit0)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_HIGH, PWM_LED21_G        , (SW3_LED21_Save.sb.bit1)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_HIGH, PWM_LED21_B        , (SW3_LED21_Save.sb.bit2)? 100:0 );  //write all scaling
        }
    }

    if(1) {
        static xu_flag SW4_LED15_Save = {0};
        static xu_flag SW4_LED16_Save = {0};
        static xu_flag SW4_LED18_Save = {0};

        if(SW4_LED15_Save.u8Byte != SW4_LED15) {
            SW4_LED15_Save.u8Byte = SW4_LED15;

            isUpdate = 1;

            I2C_WriteByte(AD_SEL1_HIGH, PWM_LED15_R        , (SW4_LED15_Save.sb.bit0)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_HIGH, PWM_LED15_G        , (SW4_LED15_Save.sb.bit1)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_HIGH, PWM_LED15_B        , (SW4_LED15_Save.sb.bit2)? 100:0 );  //write all scaling
        }

        if(SW4_LED16_Save.u8Byte != SW4_LED16) {
            SW4_LED16_Save.u8Byte = SW4_LED16;

            isUpdate = 1;

            I2C_WriteByte(AD_SEL1_HIGH, PWM_LED16_R        , (SW4_LED16_Save.sb.bit0)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_HIGH, PWM_LED16_G        , (SW4_LED16_Save.sb.bit1)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_HIGH, PWM_LED16_B        , (SW4_LED16_Save.sb.bit2)? 100:0 );  //write all scaling
        }

        if(SW4_LED18_Save.u8Byte != SW4_LED18) {
            SW4_LED18_Save.u8Byte = SW4_LED18;

            isUpdate = 1;

            I2C_WriteByte(AD_SEL1_HIGH, PWM_LED18_R        , (SW4_LED18_Save.sb.bit0)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_HIGH, PWM_LED18_G        , (SW4_LED18_Save.sb.bit1)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_HIGH, PWM_LED18_B        , (SW4_LED18_Save.sb.bit2)? 100:0 );  //write all scaling
        }
    }

    if(1) {
        static xu_flag SW5_LED13_Save = {0};
        static xu_flag SW5_LED14_Save = {0};
        static xu_flag SW5_LED17_Save = {0};

        if(SW5_LED13_Save.u8Byte != SW5_LED13) {
            SW5_LED13_Save.u8Byte = SW5_LED13;

            isUpdate = 1;

            I2C_WriteByte(AD_SEL1_HIGH, PWM_LED13_R        , (SW5_LED13_Save.sb.bit0)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_HIGH, PWM_LED13_G        , (SW5_LED13_Save.sb.bit1)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_HIGH, PWM_LED13_B        , (SW5_LED13_Save.sb.bit2)? 100:0 );  //write all scaling
        }

        if(SW5_LED14_Save.u8Byte != SW5_LED14) {
            SW5_LED14_Save.u8Byte = SW5_LED14;

            isUpdate = 1;

            I2C_WriteByte(AD_SEL1_HIGH, PWM_LED14_R        , (SW5_LED14_Save.sb.bit0)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_HIGH, PWM_LED14_G        , (SW5_LED14_Save.sb.bit1)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_HIGH, PWM_LED14_B        , (SW5_LED14_Save.sb.bit2)? 100:0 );  //write all scaling
        }

        if(SW5_LED17_Save.u8Byte != SW5_LED17) {
            SW5_LED17_Save.u8Byte = SW5_LED17;

            isUpdate = 1;

            I2C_WriteByte(AD_SEL1_HIGH, PWM_LED17_R        , (SW5_LED17_Save.sb.bit0)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_HIGH, PWM_LED17_G        , (SW5_LED17_Save.sb.bit1)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_HIGH, PWM_LED17_B        , (SW5_LED17_Save.sb.bit2)? 100:0 );  //write all scaling
        }
    }

    if(1) {
        static xu_flag SW6_LED10_Save = {0};
        static xu_flag SW6_LED11_Save = {0};
        static xu_flag SW6_LED12_Save = {0};

        if(SW6_LED10_Save.u8Byte != SW6_LED10) {
            SW6_LED10_Save.u8Byte = SW6_LED10;

            isUpdate = 1;

            I2C_WriteByte(AD_SEL1_LOW,  PWM_LED10_R        , (SW6_LED10_Save.sb.bit0)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_LOW,  PWM_LED10_G        , (SW6_LED10_Save.sb.bit1)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_LOW,  PWM_LED10_B        , (SW6_LED10_Save.sb.bit2)? 100:0 );  //write all scaling
        }

        if(SW6_LED11_Save.u8Byte != SW6_LED11) {
            SW6_LED11_Save.u8Byte = SW6_LED11;

            isUpdate = 1;

            I2C_WriteByte(AD_SEL1_LOW,  PWM_LED11_R        , (SW6_LED11_Save.sb.bit0)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_LOW,  PWM_LED11_G        , (SW6_LED11_Save.sb.bit1)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_LOW,  PWM_LED11_B        , (SW6_LED11_Save.sb.bit2)? 100:0 );  //write all scaling
        }

        if(SW6_LED12_Save.u8Byte != SW6_LED12) {
            SW6_LED12_Save.u8Byte = SW6_LED12;

            isUpdate = 1;

            I2C_WriteByte(AD_SEL1_LOW,  PWM_LED12_R        , (SW6_LED12_Save.sb.bit0)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_LOW,  PWM_LED12_G        , (SW6_LED12_Save.sb.bit1)? 100:0 );  //write all scaling
            I2C_WriteByte(AD_SEL1_LOW,  PWM_LED12_B        , (SW6_LED12_Save.sb.bit2)? 100:0 );  //write all scaling
        }
    }

    if(isUpdate) {
        isUpdate = 0;

        //Set PWM and Power value
        I2C_WriteByte(AD_SEL1_LOW,  0x49, 0x00);  //update PWM & congtrol registers
        I2C_WriteByte(AD_SEL1_HIGH, 0x49, 0x00);  //update PWM & congtrol registers

        //delay_ms(10);  //polling this api per 10ms
    }
}
/*----------------------------------------------------------------------------*/
unsigned char I2C_WriteByte(int DevAddr, unsigned char RegAddr, unsigned char u8Byte) {
    unsigned char u8Data[2] = {0};
    u8Data[0] = RegAddr;
    u8Data[1] = u8Byte;

    HAL_I2C_Master_Transmit(&hi2c1, DevAddr, u8Data, 2, 0xFFFFFFFF);

    return TRUE;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

#endif
