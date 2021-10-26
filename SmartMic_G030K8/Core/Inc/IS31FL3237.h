/*
 * IS31FL3237.h
 *
 *  Created on: 2021年10月21日
 *      Author: Hal.Tsai
 */

//----------Header----------
#ifndef __IS31FL3237_H__
#define __IS31FL3237_H__

//----------Private Include----------

//----------System Include----------

//----------System Define----------
typedef enum {
    FALSE = 0,
    TRUE  = 1,

    AD_SEL1_LOW     = 0x68,
    AD_SEL1_HIGH    = 0x6E,

    signle_LED_rgb_R = 0x01,
    signle_LED_rgb_G = 0x02,
    signle_LED_rgb_B = 0x04,

    End_Of_xe_IS31FL3237_I2C
} xe_IS31FL3237;

typedef union {
    unsigned short u16Byte;
    struct{
        unsigned bit0:1;
        unsigned bit1:1;
        unsigned bit2:1;
        unsigned bit3:1;
        unsigned bit4:1;
        unsigned bit5:1;
        unsigned bit6:1;
        unsigned bit7:1;
        unsigned bit8:1;
        unsigned bit9:1;
        unsigned bit10:1;
        unsigned bit11:1;
        unsigned bit12:1;
        unsigned bit13:1;
        unsigned bit14:1;
        unsigned bit15:1;
    } sb;
} xu_arrayLED;

typedef enum {
    Power_array_LED2_no1  = 0x4A,
    Power_array_LED2_no2  = 0x4B,
    Power_array_LED2_no3  = 0x4C,
    Power_array_LED2_no4  = 0x4D,
    Power_array_LED2_no5  = 0x4E,
    Power_array_LED2_no6  = 0x4F,
    Power_array_LED2_no7  = 0x50,
    Power_array_LED2_no8  = 0x51,
    Power_array_LED2_no9  = 0x52,
    Power_array_LED2_no10 = 0x53,

    Power_array_LED1_no1  = 0x54,
    Power_array_LED1_no2  = 0x55,
    Power_array_LED1_no3  = 0x56,
    Power_array_LED1_no4  = 0x57,
    Power_array_LED1_no5  = 0x58,
    Power_array_LED1_no6  = 0x59,
    Power_array_LED1_no7  = 0x5A,
    Power_array_LED1_no8  = 0x5B,
    Power_array_LED1_no9  = 0x5C,
    Power_array_LED1_no10 = 0x5D,

    Power_LED9_R          = 0x67,
    Power_LED9_G          = 0x68,
    Power_LED9_B          = 0x69,

    //SW6
    Power_LED10_R         = 0x5E,
    Power_LED10_G         = 0x5F,
    Power_LED10_B         = 0x60,

    Power_LED11_R         = 0x61,
    Power_LED11_G         = 0x62,
    Power_LED11_B         = 0x63,

    Power_LED12_R         = 0x64,
    Power_LED12_G         = 0x65,
    Power_LED12_B         = 0x66,
} xe_Power_AD_SEL1_LOW;

typedef enum {
    //SW5
    Power_LED13_R         = 0x4A,
    Power_LED13_B         = 0x4B,
    Power_LED13_G         = 0x4C,

    Power_LED14_R         = 0x4D,
    Power_LED14_B         = 0x4E,
    Power_LED14_G         = 0x4F,

    Power_LED17_R         = 0x50,
    Power_LED17_B         = 0x51,
    Power_LED17_G         = 0x52,

    //SW4
    Power_LED15_R         = 0x53,
    Power_LED15_B         = 0x54,
    Power_LED15_G         = 0x55,

    Power_LED16_R         = 0x56,
    Power_LED16_B         = 0x57,
    Power_LED16_G         = 0x58,

    Power_LED18_R         = 0x59,
    Power_LED18_B         = 0x5A,
    Power_LED18_G         = 0x5B,

    //SW3
    Power_LED19_R         = 0x5C,
    Power_LED19_B         = 0x5D,
    Power_LED19_G         = 0x5E,

    Power_LED20_R         = 0x5F,
    Power_LED20_B         = 0x60,
    Power_LED20_G         = 0x61,

    Power_LED21_R         = 0x62,
    Power_LED21_B         = 0x63,
    Power_LED21_G         = 0x64,
} xe_Power_AD_SEL1_HIGH;

typedef enum {
    PWM_array_LED2_no1  =  1,
    PWM_array_LED2_no2  =  3,
    PWM_array_LED2_no3  =  5,
    PWM_array_LED2_no4  =  7,
    PWM_array_LED2_no5  =  9,
    PWM_array_LED2_no6  = 11,
    PWM_array_LED2_no7  = 13,
    PWM_array_LED2_no8  = 15,
    PWM_array_LED2_no9  = 17,
    PWM_array_LED2_no10 = 19,

    PWM_array_LED1_no1  = 21,
    PWM_array_LED1_no2  = 23,
    PWM_array_LED1_no3  = 25,
    PWM_array_LED1_no4  = 27,
    PWM_array_LED1_no5  = 29,
    PWM_array_LED1_no6  = 31,
    PWM_array_LED1_no7  = 33,
    PWM_array_LED1_no8  = 35,
    PWM_array_LED1_no9  = 37,
    PWM_array_LED1_no10 = 39,

    PWM_LED9_R          = 59,
    PWM_LED9_G          = 61,
    PWM_LED9_B          = 63,

    //SW6
    PWM_LED10_R         = 41,
    PWM_LED10_G         = 43,
    PWM_LED10_B         = 45,

    PWM_LED11_R         = 47,
    PWM_LED11_G         = 49,
    PWM_LED11_B         = 51,

    PWM_LED12_R         = 53,
    PWM_LED12_G         = 55,
    PWM_LED12_B         = 57,
} xe_PWM_AD_SEL1_LOW;

typedef enum {
    //SW5
    PWM_LED13_R         =  1,
    PWM_LED13_B         =  3,
    PWM_LED13_G         =  5,

    PWM_LED14_R         =  7,
    PWM_LED14_B         =  9,
    PWM_LED14_G         = 11,

    PWM_LED17_R         = 13,
    PWM_LED17_B         = 15,
    PWM_LED17_G         = 17,

    //SW4
    PWM_LED15_R         = 19,
    PWM_LED15_B         = 21,
    PWM_LED15_G         = 23,

    PWM_LED16_R         = 25,
    PWM_LED16_B         = 27,
    PWM_LED16_G         = 29,

    PWM_LED18_R         = 31,
    PWM_LED18_B         = 33,
    PWM_LED18_G         = 35,

    //SW3
    PWM_LED19_R         = 37,
    PWM_LED19_B         = 39,
    PWM_LED19_G         = 41,

    PWM_LED20_R         = 43,
    PWM_LED20_B         = 45,
    PWM_LED20_G         = 47,

    PWM_LED21_R         = 49,
    PWM_LED21_B         = 51,
    PWM_LED21_G         = 53,
} xe_PWM_AD_SEL1_HIGH;


//----------Private Variable----------

//----------Global Variable----------

//----------Private Function----------

//----------Global Function----------

//----------Code Implement----------
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#ifdef __IS31FL3237_C__
           unsigned short arrayLED[2] = {0};

           unsigned char LED9      = {0};

           unsigned char SW3_LED19 = {0};
           unsigned char SW3_LED20 = {0};
           unsigned char SW3_LED21 = {0};

           unsigned char SW4_LED15 = {0};
           unsigned char SW4_LED16 = {0};
           unsigned char SW4_LED18 = {0};

           unsigned char SW5_LED13 = {0};
           unsigned char SW5_LED14 = {0};
           unsigned char SW5_LED17 = {0};

           unsigned char SW6_LED10 = {0};
           unsigned char SW6_LED11 = {0};
           unsigned char SW6_LED12 = {0};


           void delay_ms(unsigned int t);
           void Hal_LED_Driver_Initial(void);
           void Task_LED_Driver(void);
           void IS31FL3237_mode1(void);

#else
    extern unsigned short arrayLED[];

    extern unsigned char LED9;

    extern unsigned char SW3_LED19;
    extern unsigned char SW3_LED20;
    extern unsigned char SW3_LED21;

    extern unsigned char SW4_LED15;
    extern unsigned char SW4_LED16;
    extern unsigned char SW4_LED18;

    extern unsigned char SW5_LED13;
    extern unsigned char SW5_LED14;
    extern unsigned char SW5_LED17;

    extern unsigned char SW6_LED10;
    extern unsigned char SW6_LED11;
    extern unsigned char SW6_LED12;


    extern void delay_ms(unsigned int t);
    extern void Hal_LED_Driver_Initial(void);
    extern void Task_LED_Driver(void);
    extern void IS31FL3237_mode1(void);

#endif

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

#endif
