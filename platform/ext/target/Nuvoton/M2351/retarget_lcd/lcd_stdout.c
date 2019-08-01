/*
 * Copyright (c) 2017-2018 ARM Limited
 *
 * Licensed under the Apace License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apace.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "uart_stdout.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "Driver_USART.h"
#include "target_cfg.h"

#include "NuMicro.h"


#define LCD_ON          0

#define White           0xFFFF
#define Black           0x0000
#define Blue            0x001F
#define Blue2           0x051F
#define Red             0xF800
#define Magenta         0xF81F
#define Green           0x07E0
#define Cyan            0x7FFF
#define Yellow          0xFFE0

extern void ILI9341_LCD_Init(void);
extern void ILI9341_LCD_Fill(uint16_t x, uint16_t y, uint16_t x1, uint16_t y1, uint32_t bColor);

//CWS
#if 1
#define ASSERT_HIGH(X)
__attribute__((noreturn))
void exit(int x)
{
    while(1);
}

#else
#define ASSERT_HIGH(X)  assert(X == ARM_DRIVER_OK)
#endif




/* Imports USART driver */
extern ARM_DRIVER_USART TFM_DRIVER_STDIO;

/* Struct FILE is implemented in stdio.h. Used to redirect printf to
 * TFM_DRIVER_STDIO
 */
FILE __stdout;

static void uart_putc(unsigned char c)
{
#if LCD_ON    
    extern void ILI9341_LCD_PutString(uint16_t x, uint16_t y, char *s, uint32_t fColor, uint32_t bColor);
    static char buf[255];
    static int32_t buf_idx = 0;
    static int32_t line_idx = 0;
    int32_t i,j,k;
    int32_t color;
#endif
    
    
    int32_t ret = ARM_DRIVER_OK;

    ret = TFM_DRIVER_STDIO.Send(&c, 1);
    ASSERT_HIGH(ret);

#if LCD_ON

    if(line_idx > 14)
    {
        line_idx = 0;
        /* Clean screen */
        ILI9341_LCD_Fill(0, 0, 240, 320, Black);
        
    }
    
    color = White;
    if((c != '\n') && (c != '\r'))
    {
        buf[buf_idx++] = c;
    }
    else if(c == '\n')
        ;
    else
    {
        buf[buf_idx] = 0;
        k = buf_idx;
        // Parse color control
        if(buf[0] == 0x1b)
        {
            for(i=0;i<255;i++)
            {
                if(buf[i] == 0x6d)
                {
                    if(buf[i-1] == '2')
                    {
                        color = Green;
                    }
                    else if(buf[i-1] == '7')
                    {
                        color = Yellow;
                    }


                
                    k = 0;
                    for(j=i+1;j<255;j++)
                    {
                        buf[k++] = buf[j];
                        if(buf[j] == 0)
                            break;
                    }
                    break;
                }
            }
        }
        
        if(k < 40 &&  0)
        {
            for(i=k-1;i<40;i++)
                buf[i] = ' ';
            buf[i] = 0;
        }
        
        ILI9341_LCD_PutString(line_idx*16, 0, buf, color, Black);
        buf_idx = 0;
        line_idx++;
        
    }
    
#endif
}

/* Redirects printf to TFM_DRIVER_STDIO in case of ARMCLANG*/
#if defined(__ARMCC_VERSION)
/* __ARMCC_VERSION is only defined starting from Arm compiler version 6 */
int fputc(int ch, FILE *f)
{
    /* Send byte to USART */
    uart_putc(ch);

    /* Return character written */
    return ch;
}
#elif defined(__GNUC__)
/* Redirects printf to TFM_DRIVER_STDIO in case of GNUARM */
int _write(int fd, char *str, int len)
{
    int i;

    for (i = 0; i < len; i++) {
        /* Send byte to USART */
        uart_putc(str[i]);
    }

    /* Return the number of characters written */
    return len;
}
#endif

void stdio_init(void)
{
    
    int32_t ret = ARM_DRIVER_OK;
    ret = TFM_DRIVER_STDIO.Initialize(NULL);
    ASSERT_HIGH(ret);

    ret = TFM_DRIVER_STDIO.Control(ARM_USART_MODE_ASYNCHRONOUS, 115200);
    ASSERT_HIGH(ret);

#if LCD_ON
    /* Init LCD */
    ILI9341_LCD_Init();

    /* Clean screen */
    ILI9341_LCD_Fill(0, 0, 240, 320, Black);
#endif    
}

void stdio_uninit(void)
{
    int32_t ret = ARM_DRIVER_OK;
    ret = TFM_DRIVER_STDIO.Uninitialize();
    ASSERT_HIGH(ret);
}

