/*
 * Copyright (c) 2019, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "platform_retarget.h"
#include "platform_retarget_dev.h"
#include "timer_cmsdk.h"
#include "tfm_plat_test.h"

#include "smm_mps2.h"
#include "NuMicro.h"

#define TIMER_RELOAD_VALUE (16*1024*1024)
#define USERLED_MASK       (0x3)
#define MPS2_USERPB0_BASE  (0x50302008)
#define MPS2_USERPB0_MASK  (0x1)

void tfm_plat_test_wait_user_button_pressed(void)
{
#ifdef M2351
    while(PB0);
#else    
    volatile uint32_t *p_btn = (volatile uint32_t *) MPS2_USERPB0_BASE;

    /* Wait until user button 0 is pressed */
    while (!(*p_btn & MPS2_USERPB0_MASK)) {
      ;
    }
#endif    
}

void tfm_plat_test_wait_user_button_released(void)
{
#ifdef M2351
    while(!PB0);
#else    
    volatile uint32_t *p_btn = (volatile uint32_t *) MPS2_USERPB0_BASE;

    /* Wait until user button 0 is released */
    while ((*p_btn & MPS2_USERPB0_MASK)) {
      ;
    }
#endif    
}

uint32_t tfm_plat_test_get_led_status(void)
{
#ifdef M2351
    return PA10;
#else    
    
    struct arm_mps2_fpgaio_t *fpgaio = SEC_MPS2_FPGAIO;
    return  fpgaio->LED;
#endif    
}

void tfm_plat_test_set_led_status(uint32_t status)
{
#ifdef M2351
    PA10 = status;
#else    
    struct arm_mps2_fpgaio_t *fpgaio = SEC_MPS2_FPGAIO;
    fpgaio->LED = status;
#endif    
    
}

uint32_t tfm_plat_test_get_userled_mask(void)
{
    return USERLED_MASK;
}

void tfm_plat_test_secure_timer_start(void)
{
#ifdef M2351
    if((CLK->APBCLK0 & CLK_APBCLK0_TMR0CKEN_Msk) == 0)
    {
        CLK->APBCLK0 |= CLK_APBCLK0_TMR0CKEN_Msk;
    
        CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_TMR0SEL_Msk)) | CLK_CLKSEL1_TMR0SEL_HIRC;
        
        
        CLK->APBCLK0 |= CLK_APBCLK0_TMR2CKEN_Msk;
    
        CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_TMR2SEL_Msk)) | CLK_CLKSEL1_TMR2SEL_HIRC;
        
    }

    TIMER0->CTL = TIMER_CTL_CNTEN_Msk | TIMER_CTL_INTEN_Msk | TIMER_PERIODIC_MODE | (12-1);
    
    TIMER0->CMP = 12000000/12;
    
    NVIC_EnableIRQ(TMR0_IRQn);
    

#else    
    if (!cmsdk_timer_is_initialized(&CMSDK_TIMER0_DEV_S)) {
        cmsdk_timer_init(&CMSDK_TIMER0_DEV_S);
    }
    cmsdk_timer_set_reload_value(&CMSDK_TIMER0_DEV_S, TIMER_RELOAD_VALUE);
    cmsdk_timer_enable(&CMSDK_TIMER0_DEV_S);
    cmsdk_timer_enable_interrupt(&CMSDK_TIMER0_DEV_S);
#endif    
}

void tfm_plat_test_secure_timer_stop(void)
{
#ifdef M2351
    TIMER0->CTL = 0;
#else    
    cmsdk_timer_disable(&CMSDK_TIMER0_DEV_S);
    cmsdk_timer_disable_interrupt(&CMSDK_TIMER0_DEV_S);
    cmsdk_timer_clear_interrupt(&CMSDK_TIMER0_DEV_S);
#endif    
}

void tfm_plat_test_non_secure_timer_start(void)
{
#ifdef M2351
    TIMER2_NS->CTL = TIMER_CTL_CNTEN_Msk | TIMER_CTL_INTEN_Msk | TIMER_PERIODIC_MODE | (12-1);
    
    TIMER2_NS->CMP = 12000000/12;
    
    NVIC_EnableIRQ(TMR2_IRQn);
    

#else    
    if (!cmsdk_timer_is_initialized(&CMSDK_TIMER1_DEV_NS)) {
        cmsdk_timer_init(&CMSDK_TIMER1_DEV_NS);
    }
    cmsdk_timer_set_reload_value(&CMSDK_TIMER1_DEV_NS, TIMER_RELOAD_VALUE);
    cmsdk_timer_enable(&CMSDK_TIMER1_DEV_NS);
    cmsdk_timer_enable_interrupt(&CMSDK_TIMER1_DEV_NS);
#endif    
}

void tfm_plat_test_non_secure_timer_stop(void)
{
#ifdef M2351
    TIMER2->CTL = 0;
	TIMER2->INTSTS = 1;
#else    
    cmsdk_timer_disable(&CMSDK_TIMER1_DEV_NS);
    cmsdk_timer_disable_interrupt(&CMSDK_TIMER1_DEV_NS);
    cmsdk_timer_clear_interrupt(&CMSDK_TIMER1_DEV_NS);
#endif    
}
