/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Display an string on TFT LCD panel via SPI interface.
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define IO_LED          PA10_NS
#define IO_LED1         PA10_NS
#define IO_LED2         PA11_NS


#define SPI_LCD_PORT  SPI1_NS //SPI2

#define ILI9341_RESET   PA9_NS
#define ILI9341_DC      PA8_NS
#define ILI9341_LED     PC11_NS

#define GPIO_SPI1_SS        PH9_NS
#define GPIOPORT_SPI1_SS    PH_NS
#define PINMASK_SPI1_SS     BIT9


extern uint8_t Font8x16[];

#define White           0xFFFF
#define Black           0x0000
#define Blue            0x001F
#define Blue2           0x051F
#define Red             0xF800
#define Magenta         0xF81F
#define Green           0x07E0
#define Cyan            0x7FFF
#define Yellow          0xFFE0


uint32_t PllClock = 64000000;
uint32_t CyclesPerUs = 64;

uint32_t __PC(void)
{
    return 0;
}

void Delay(TIMER_T *t, uint32_t us)
{
    CLK_SysTickDelay(us);
}


uint8_t LCD_ReadReg(uint8_t u8Comm)
{
    SPI_ClearRxFIFO(SPI_LCD_PORT);

    ILI9341_DC = 0;

    SPI_WRITE_TX(SPI_LCD_PORT, u8Comm);
    SPI_WRITE_TX(SPI_LCD_PORT, 0x00);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_LCD_PORT));

    SPI_READ_RX(SPI_LCD_PORT);

    return (SPI_READ_RX(SPI_LCD_PORT));
}

void LCD_WriteCommand(uint8_t u8Comm)
{
    ILI9341_DC = 0;

    SPI_WRITE_TX(SPI_LCD_PORT, u8Comm);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_LCD_PORT));
}

void LCD_WriteData(uint8_t u8Data)
{
    ILI9341_DC = 1;

    SPI_WRITE_TX(SPI_LCD_PORT, u8Data);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_LCD_PORT));
}


void ILI9341_LCD_SetAddress(uint32_t x1, uint32_t x2, uint32_t y1, uint32_t y2)
{
    if(x1 >= 240)
        x1 = 239;
    if(x2 >= 240)
        x2 = 239;
    if(y1 >= 320)
        y1 = 319;
    if(y2 >= 320)
        y2 = 319;

    LCD_WriteCommand(0x2a);
    LCD_WriteData(x1 >> 8);
    LCD_WriteData(x1);
    LCD_WriteData(x2 >> 8);
    LCD_WriteData(x2);

    LCD_WriteCommand(0x2b);
    LCD_WriteData(y1 >> 8);
    LCD_WriteData(y1);
    LCD_WriteData(y2 >> 8);
    LCD_WriteData(y2);
}

void ILI9341_LCD_PutChar8x16(uint16_t x, uint16_t y, uint8_t c, uint32_t fColor, uint32_t bColor)
{
    uint32_t i, j;
    for(i = 0; i < 16; i++)
    {
        uint8_t m = Font8x16[c * 16 + i];
        ILI9341_LCD_SetAddress(x + i, x + i, y, y + 7);
        LCD_WriteCommand(0x2c);

        for(j = 0; j < 8; j++)
        {
            if((m & 0x01) == 0x01)
            {
                LCD_WriteData(fColor >> 8);
                LCD_WriteData(fColor);
            }
            else
            {
                LCD_WriteData(bColor >> 8);
                LCD_WriteData(bColor);
            }
            m >>= 1;
        }
    }
}

void ILI9341_LCD_PutString(uint16_t x, uint16_t y, char *s, uint32_t fColor, uint32_t bColor)
{
    uint8_t l = 0;
    while(*s)
    {
        if(*s < 0x80)
        {
            ILI9341_LCD_PutChar8x16(x, 312 - y - l * 8, *s, fColor, bColor);
            s++;
            l++;
        }
    }
}


void ILI9341_LCD_Fill(uint16_t x, uint16_t y, uint16_t x1, uint16_t y1, uint32_t bColor)
{
    uint32_t j;

    ILI9341_LCD_SetAddress(x, x1 - 1, y, y1 - 1);

    //for(i=0;i<x1-x;i++)
    {
        LCD_WriteCommand(0x2c);

        for(j = 0; j < (x1 - x) * (y1 - y); j++)
        {
            LCD_WriteData(bColor >> 8);
            LCD_WriteData(bColor);
        }
    }
}

void ILI9341_LCD_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    SCU_SET_PNSSET(SPI1_Attr);
    SCU_SET_IONSSET(SCU_IONSSET_PA_Msk|SCU_IONSSET_PC_Msk|SCU_IONSSET_PH_Msk);

    /* Enable SPI1 Clock */
    CLK->APBCLK0 |= CLK_APBCLK0_SPI1CKEN_Msk;
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI1SEL_Msk)) | CLK_CLKSEL2_SPI1SEL_PLL;


    /* Configure SPI3 as a master, MSB first, 8-bit transaction, SPI Mode-0 timing, clock is 4MHz */
    SPI_Open(SPI_LCD_PORT, SPI_MASTER, SPI_MODE_0, 8, 4000000);

    /* Configure SPI1 as a low level active device. */
    SPI_EnableAutoSS(SPI_LCD_PORT, SPI_SS, SPI_SS_ACTIVE_LOW);

    /* Start SPI */
    SPI_ENABLE(SPI_LCD_PORT);



    /* Setup SPI1 multi-function pins */
    SYS->GPE_MFPL &= ~(SYS_GPE_MFPL_PE0MFP_Msk       | SYS_GPE_MFPL_PE1MFP_Msk);
    SYS->GPE_MFPL |=  (SYS_GPE_MFPL_PE0MFP_SPI1_MOSI | SYS_GPE_MFPL_PE1MFP_SPI1_MISO);

    SYS->GPH_MFPH &= ~(SYS_GPH_MFPH_PH8MFP_Msk | SYS_GPH_MFPH_PH9MFP_Msk);
    SYS->GPH_MFPH |=  (SYS_GPH_MFPH_PH8MFP_SPI1_CLK | SYS_GPH_MFPH_PH9MFP_SPI1_SS);

    /* Set IO to high slew rate */
    PE_NS->SLEWCTL |= 3;
    PH_NS->SLEWCTL |= (3 << 8);

    /* Configure DC/RESET/LED pins */
    ILI9341_DC = 0;
    ILI9341_RESET = 0;
    ILI9341_LED = 0;

    /* LCD DC/RESET/LED */
    //GPIO_SetMode(PA, BIT8 | BIT9, GPIO_MODE_OUTPUT);
    PA_NS->MODE = (PA_NS->MODE & (~(0xf << 8*2))) | (0x5 << 8*2);
    //GPIO_SetMode(PC, BIT11, GPIO_MODE_OUTPUT);
    PC_NS->MODE = (PC_NS->MODE & (~(0x3 << 11*2))) | (0x1 << 11*2);

    /* cs pin */
    //GPIO_SetMode(GPIOPORT_SPI1_SS, PINMASK_SPI1_SS, GPIO_MODE_OUTPUT);
    PH_NS->MODE = (PH_NS->MODE & (~(0x3 << 9*2))) | (0x1 << 9*2);

    
    /* Configure LCD */
    ILI9341_DC = 1;

    ILI9341_RESET = 0;
    Delay(TIMER0, 20000);

    ILI9341_RESET = 1;
    Delay(TIMER0, 40000);

    LCD_WriteCommand(0xCB);
    LCD_WriteData(0x39);
    LCD_WriteData(0x2C);
    LCD_WriteData(0x00);
    LCD_WriteData(0x34);
    LCD_WriteData(0x02);

    LCD_WriteCommand(0xCF);
    LCD_WriteData(0x00);
    LCD_WriteData(0xC1);
    LCD_WriteData(0x30);

    LCD_WriteCommand(0xE8);
    LCD_WriteData(0x85);
    LCD_WriteData(0x00);
    LCD_WriteData(0x78);

    LCD_WriteCommand(0xEA);
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);

    LCD_WriteCommand(0xED);
    LCD_WriteData(0x64);
    LCD_WriteData(0x03);
    LCD_WriteData(0x12);
    LCD_WriteData(0x81);

    LCD_WriteCommand(0xF7);
    LCD_WriteData(0x20);

    LCD_WriteCommand(0xC0);
    LCD_WriteData(0x23);

    LCD_WriteCommand(0xC1);
    LCD_WriteData(0x10);

    LCD_WriteCommand(0xC5);
    LCD_WriteData(0x3e);
    LCD_WriteData(0x28);

    LCD_WriteCommand(0xC7);
    LCD_WriteData(0x86);

    LCD_WriteCommand(0x36);
    LCD_WriteData(0x48);

    LCD_WriteCommand(0x3A);
    LCD_WriteData(0x55);

    LCD_WriteCommand(0xB1);
    LCD_WriteData(0x00);
    LCD_WriteData(0x18);

    LCD_WriteCommand(0xB6);
    LCD_WriteData(0x08);
    LCD_WriteData(0x82);
    LCD_WriteData(0x27);

    LCD_WriteCommand(0xF2);
    LCD_WriteData(0x00);

    LCD_WriteCommand(0x26);
    LCD_WriteData(0x01);

    LCD_WriteCommand(0xE0);
    LCD_WriteData(0x0F);
    LCD_WriteData(0x31);
    LCD_WriteData(0x2B);
    LCD_WriteData(0x0C);
    LCD_WriteData(0x0E);
    LCD_WriteData(0x08);
    LCD_WriteData(0x4E);
    LCD_WriteData(0xF1);
    LCD_WriteData(0x37);
    LCD_WriteData(0x07);
    LCD_WriteData(0x10);
    LCD_WriteData(0x03);
    LCD_WriteData(0x0E);
    LCD_WriteData(0x09);
    LCD_WriteData(0x00);

    LCD_WriteCommand(0xE1);
    LCD_WriteData(0x00);
    LCD_WriteData(0x0E);
    LCD_WriteData(0x14);
    LCD_WriteData(0x03);
    LCD_WriteData(0x11);
    LCD_WriteData(0x07);
    LCD_WriteData(0x31);
    LCD_WriteData(0xC1);
    LCD_WriteData(0x48);
    LCD_WriteData(0x08);
    LCD_WriteData(0x0F);
    LCD_WriteData(0x0C);
    LCD_WriteData(0x31);
    LCD_WriteData(0x36);
    LCD_WriteData(0x0F);

    LCD_WriteCommand(0x11);
    Delay(TIMER0, 60000);

    LCD_WriteCommand(0x29);    //Display on

    ILI9341_LED = 1;
}

#if 0
int mainx(void)
{
    char buf[255] = {0};
    int32_t i;
    extern void VectorRemap(void);

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init UART to 115200-8n1 for print message */
    //UART_Open(UART0, 115200);

    printf("+-------------------------------------------+\n");
    printf("|       NuEdu-FingerPrint Demo Board        |\n");
    printf("+-------------------------------------------+\n");


    // LED
    //GPIO_SetMode(PC, BIT14, GPIO_MODE_OUTPUT);
    //IO_LED = 0;
    //GPIO_SetMode(PA, BIT14 | BIT15, GPIO_MODE_OUTPUT);
    //IO_LED1 = 1;
    //IO_LED2 = 0;


    /* Configure SPI3 as a master, MSB first, 8-bit transaction, SPI Mode-0 timing, clock is 4MHz */
    SPI_Open(SPI_LCD_PORT, SPI_MASTER, SPI_MODE_0, 8, 4000000);

    /* Configure SPI1 as a low level active device. */
    SPI_EnableAutoSS(SPI_LCD_PORT, SPI_SS, SPI_SS_ACTIVE_LOW);

    /* Start SPI */
    SPI_ENABLE(SPI_LCD_PORT);

    /* Init LCD */
    ILI9341_LCD_Init();

    /*
        PD4 for counter input. Support 0 ~ 6MHz
        PC2 for PWM output (duty cycle 50%)

        Variable resistor is used to control PWM output frequency.
    */

    ILI9341_LCD_Fill(0, 0, 240, 320, Black);

    ILI9341_LCD_PutString(0, 0, "LCD Display Demo ...", Blue, Black);
    ILI9341_LCD_PutString(16, 0, "Nuvoton Technology Corp.", Red, Black);
    ILI9341_LCD_PutString(32, 0, "Hello World!", Yellow, Black);
    ILI9341_LCD_PutString(48, 0, ">>> M2351 <<<", Green, Black);



    /* Show the String on the screen */
    i = 0;
    while(1)
    {
        /* Show a counter on LCD */
        sprintf((char *)buf, "%5d    ", i++);
        ILI9341_LCD_PutString(64, 0, buf, Cyan, Black);

        /* Toggle LED */
        IO_LED ^= 1;
        IO_LED1 ^= 1;
        IO_LED2 ^= 1;

        CLK_SysTickDelay(150000);

    }
    while(1);
}
#endif
