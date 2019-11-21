/*
 * Copyright (c) 2018 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cmsis.h"
#include "target_cfg.h"
#include "Driver_MPC.h"
#include "platform_retarget_dev.h"
#include "region_defs.h"
#include "tfm_secure_api.h"
#include "tfm_plat_defs.h"


/* Macros to pick linker symbols */
#define REGION(a, b, c) a##b##c
#define REGION_NAME(a, b, c) REGION(a, b, c)
#define REGION_DECLARE(a, b, c) extern uint32_t REGION_NAME(a, b, c)

/* The section names come from the scatter file */
REGION_DECLARE(Load$$LR$$, LR_NS_PARTITION, $$Base);
REGION_DECLARE(Load$$LR$$, LR_VENEER, $$Base);
REGION_DECLARE(Load$$LR$$, LR_VENEER, $$Limit);
#ifdef BL2
REGION_DECLARE(Load$$LR$$, LR_SECONDARY_PARTITION, $$Base);
#endif /* BL2 */

const struct memory_region_limits memory_regions = {
    .non_secure_code_start =
        (uint32_t)&REGION_NAME(Load$$LR$$, LR_NS_PARTITION, $$Base) +
        BL2_HEADER_SIZE,

    .non_secure_partition_base =
        (uint32_t)&REGION_NAME(Load$$LR$$, LR_NS_PARTITION, $$Base),

    .non_secure_partition_limit =
        (uint32_t)&REGION_NAME(Load$$LR$$, LR_NS_PARTITION, $$Base) +
        NS_PARTITION_SIZE - 1,

    .veneer_base =
        (uint32_t)&REGION_NAME(Load$$LR$$, LR_VENEER, $$Base),

    .veneer_limit =
        (uint32_t)&REGION_NAME(Load$$LR$$, LR_VENEER, $$Limit),

#ifdef BL2
    .secondary_partition_base =
        (uint32_t)&REGION_NAME(Load$$LR$$, LR_SECONDARY_PARTITION, $$Base),

    .secondary_partition_limit =
        (uint32_t)&REGION_NAME(Load$$LR$$, LR_SECONDARY_PARTITION, $$Base) +
        SECONDARY_PARTITION_SIZE - 1,
#endif /* BL2 */
};

/* Allows software, via SAU, to define the code region as a NSC */
#define NSCCFG_CODENSC  1

/* Import MPC driver */
extern ARM_DRIVER_MPC Driver_SRAM1_MPC, Driver_SRAM2_MPC;

/* Define Peripherals NS address range for the platform */
#define PERIPHERALS_BASE_NS_START (0x50000000)
#define PERIPHERALS_BASE_NS_END   (0x5FFFFFFF)

/* Enable system reset request for CPU 0 */
#define ENABLE_CPU0_SYSTEM_RESET_REQUEST (1U << 4U)

/* To write into AIRCR register, 0x5FA value must be write to the VECTKEY field,
 * otherwise the processor ignores the write.
 */
#define SCB_AIRCR_WRITE_MASK ((0x5FAUL << SCB_AIRCR_VECTKEY_Pos))

/* Debug configuration flags */
#define SPNIDEN_SEL_STATUS (0x01u << 7)
#define SPNIDEN_STATUS     (0x01u << 6)
#define SPIDEN_SEL_STATUS  (0x01u << 5)
#define SPIDEN_STATUS      (0x01u << 4)
#define NIDEN_SEL_STATUS   (0x01u << 3)
#define NIDEN_STATUS       (0x01u << 2)
#define DBGEN_SEL_STATUS   (0x01u << 1)
#define DBGEN_STATUS       (0x01u << 0)

#define All_SEL_STATUS (SPNIDEN_SEL_STATUS | SPIDEN_SEL_STATUS | \
                        NIDEN_SEL_STATUS | DBGEN_SEL_STATUS)

struct tfm_spm_partition_platform_data_t tfm_peripheral_std_uart = {
        UART0_BASE_NS,
        UART0_BASE_NS + 0xFFF,
        PPC_SP_DO_NOT_CONFIGURE,
        -1
};

struct tfm_spm_partition_platform_data_t tfm_peripheral_uart1 = {
        UART1_BASE_S,
        UART1_BASE_S + 0xFFF,
        PPC_SP_APB_PPC_EXP1,
        CMSDK_UART1_APB_PPC_POS
};

struct tfm_spm_partition_platform_data_t tfm_peripheral_fpga_io = {
        MPS2_IO_FPGAIO_BASE_S,
        MPS2_IO_FPGAIO_BASE_S + 0xFFF,
        PPC_SP_APB_PPC_EXP2,
        CMSDK_FPGA_IO_PPC_POS
};

struct tfm_spm_partition_platform_data_t tfm_peripheral_timer0 = {
        CMSDK_TIMER0_BASE_S,
        CMSDK_TIMER1_BASE_S - 1,
        PPC_SP_APB_PPC0,
        CMSDK_TIMER0_APB_PPC_POS
};
enum tfm_plat_err_t enable_fault_handlers(void)
{
    /* Fault handles enable registers are not present in a baseline
     * implementation
     */
    return TFM_PLAT_ERR_SUCCESS;
}

enum tfm_plat_err_t system_reset_cfg(void)
{
#ifndef M2351
    struct sysctrl_t *sysctrl = (struct sysctrl_t *)CMSDK_SYSCTRL_BASE_S;
    uint32_t reg_value = SCB->AIRCR;

    /* Enable system reset request for CPU 0, to be triggered via
     * NVIC_SystemReset function.
     */
    sysctrl->resetmask |= ENABLE_CPU0_SYSTEM_RESET_REQUEST;
#else
    uint32_t reg_value = SCB->AIRCR;
#endif

    /* Clear SCB_AIRCR_VECTKEY value */
    reg_value &= ~(uint32_t)(SCB_AIRCR_VECTKEY_Msk);

    /* Enable system reset request for the secure world only */
    reg_value |= (uint32_t)(SCB_AIRCR_WRITE_MASK | SCB_AIRCR_SYSRESETREQS_Msk);

    SCB->AIRCR = reg_value;
    return TFM_PLAT_ERR_SUCCESS;
}

enum tfm_plat_err_t init_debug(void)
{
    /* Set UART0 to NS for debug message */
    SCU_SET_PNSSET(UART0_Attr);

    return TFM_PLAT_ERR_SUCCESS;
}

/*----------------- NVIC interrupt target state to NS configuration ----------*/
enum tfm_plat_err_t nvic_interrupt_target_state_cfg(void)
{
    /* Target every interrupt to NS; unimplemented interrupts will be WI */
    for (uint8_t i=0; i<sizeof(NVIC->ITNS)/sizeof(NVIC->ITNS[0]); i++) {
        NVIC->ITNS[i] = 0xFFFFFFFF;
    }

#ifndef M2351
    /* Make sure that MPC and PPC are targeted to S state */
    NVIC_ClearTargetState(MPC_IRQn);
    NVIC_ClearTargetState(PPC_IRQn);
#endif    

#ifdef SECURE_UART1
    /* UART1 is a secure peripheral, so its IRQs have to target S state */
    NVIC_ClearTargetState(UARTRX1_IRQn);
    NVIC_ClearTargetState(UARTTX1_IRQn);
    NVIC_ClearTargetState(UART1_IRQn);
#endif
    return TFM_PLAT_ERR_SUCCESS;
}

/*----------------- NVIC interrupt enabling for S peripherals ----------------*/
enum tfm_plat_err_t nvic_interrupt_enable(void)
{
#ifndef M2351
    struct spctrl_def* spctrl = CMSDK_SPCTRL;
    int32_t ret = ARM_DRIVER_OK;

    /* MPC interrupt enabling */
    ret = Driver_SRAM1_MPC.EnableInterrupt();
    if (ret != ARM_DRIVER_OK) {
        return TFM_PLAT_ERR_SYSTEM_ERR;
    }
    ret = Driver_SRAM2_MPC.EnableInterrupt();
    if (ret != ARM_DRIVER_OK) {
        return TFM_PLAT_ERR_SYSTEM_ERR;
    }
    NVIC_EnableIRQ(MPC_IRQn);

    /* PPC interrupt enabling */
    /* Clear pending PPC interrupts */
    /* In the PPC configuration function, we have used the Non-Secure
     * Privilege Control Block to grant unprivilged NS access to some
     * peripherals used by NS. That triggers a PPC0 exception as that
     * register is meant for NS privileged access only. Clear it here
     */
    spctrl->secppcintclr = CMSDK_APB_PPC0_INT_POS_MASK;

    /* Enable PPC interrupts for APB PPC */
    spctrl->secppcinten |= CMSDK_APB_PPC0_INT_POS_MASK |
                           CMSDK_APB_PPC1_INT_POS_MASK |
                           CMSDK_APB_PPCEXP0_INT_POS_MASK |
                           CMSDK_APB_PPCEXP1_INT_POS_MASK |
                           CMSDK_APB_PPCEXP2_INT_POS_MASK |
                           CMSDK_APB_PPCEXP3_INT_POS_MASK;
    NVIC_EnableIRQ(PPC_IRQn);
#endif    
    return TFM_PLAT_ERR_SUCCESS;
}

/*------------------- SAU/IDAU configuration functions -----------------------*/

struct sau_cfg_t {
    uint32_t RNR;
    uint32_t RBAR;
    uint32_t RLAR;
};

const struct sau_cfg_t sau_cfg[] = {
    {
        TFM_NS_REGION_CODE,
        ((uint32_t)&REGION_NAME(Load$$LR$$, LR_NS_PARTITION, $$Base)),
        ((uint32_t)&REGION_NAME(Load$$LR$$, LR_NS_PARTITION, $$Base) +
        NS_PARTITION_SIZE - 1)
    },
    {
        TFM_NS_REGION_DATA,
        NS_DATA_START,
        NS_DATA_LIMIT
    },
    {
        TFM_NS_REGION_VENEER,
        (uint32_t)&REGION_NAME(Load$$LR$$, LR_VENEER, $$Base),
        (uint32_t)&REGION_NAME(Load$$LR$$, LR_VENEER, $$Limit)
    },
    {
        TFM_NS_REGION_PERIPH_1,
        PERIPHERALS_BASE_NS_START,
#ifdef SECURE_UART1
        (UART1_BASE_NS - 1)
    },
    {
        TFM_NS_REGION_PERIPH_2,
        UART2_BASE_NS,
#endif
        PERIPHERALS_BASE_NS_END
    }
#ifdef BL2
    ,
    {
        TFM_NS_SECONDARY_IMAGE_REGION,
        (uint32_t)&REGION_NAME(Load$$LR$$, LR_SECONDARY_PARTITION, $$Base),
        (uint32_t)&REGION_NAME(Load$$LR$$, LR_SECONDARY_PARTITION, $$Base) +
        SECONDARY_PARTITION_SIZE - 1
    }
#endif
};

void sau_and_idau_cfg(void)
{
    /* Enables SAU */
    TZ_SAU_Enable();

    /* Configures SAU regions to be non-secure */
    SAU->RNR  = TFM_NS_REGION_CODE;
    SAU->RBAR = (memory_regions.non_secure_partition_base
                & SAU_RBAR_BADDR_Msk);
    SAU->RLAR = (memory_regions.non_secure_partition_limit
                & SAU_RLAR_LADDR_Msk)
                | SAU_RLAR_ENABLE_Msk;


    SAU->RNR  = TFM_NS_REGION_DATA;
    SAU->RBAR = (NS_DATA_START & SAU_RBAR_BADDR_Msk);
    SAU->RLAR = (NS_DATA_LIMIT & SAU_RLAR_LADDR_Msk) | SAU_RLAR_ENABLE_Msk;

    /* Configures veneers region to be non-secure callable */
    SAU->RNR  = TFM_NS_REGION_VENEER;
    SAU->RBAR = (memory_regions.veneer_base  & SAU_RBAR_BADDR_Msk);
    SAU->RLAR = (memory_regions.veneer_limit & SAU_RLAR_LADDR_Msk)
                | SAU_RLAR_ENABLE_Msk
                | SAU_RLAR_NSC_Msk;

    /* Configure the peripherals space */
    /* Only UART1 is configured as a secure peripheral */
    SAU->RNR  = TFM_NS_REGION_PERIPH_1;
    SAU->RBAR = (PERIPHERALS_BASE_NS_START & SAU_RBAR_BADDR_Msk);

#ifdef SECURE_UART1
    /* To statically configure a peripheral range as secure, close NS peripheral
     * region before range, and open a new NS region after the reserved space.
     */
    SAU->RLAR = ((UART1_BASE_NS-1) & SAU_RLAR_LADDR_Msk)
                | SAU_RLAR_ENABLE_Msk;

    SAU->RNR  = TFM_NS_REGION_PERIPH_2;
    SAU->RBAR = (UART2_BASE_NS & SAU_RBAR_BADDR_Msk);
#endif

    SAU->RLAR = (PERIPHERALS_BASE_NS_END & SAU_RLAR_LADDR_Msk)
                | SAU_RLAR_ENABLE_Msk;

#ifdef BL2
    /* Secondary image partition */
    SAU->RNR  = TFM_NS_SECONDARY_IMAGE_REGION;
    SAU->RBAR = (memory_regions.secondary_partition_base  & SAU_RBAR_BADDR_Msk);
    SAU->RLAR = (memory_regions.secondary_partition_limit & SAU_RLAR_LADDR_Msk)
                | SAU_RLAR_ENABLE_Msk;
#endif /* BL2 */

#ifndef M2351
    /* Allows SAU to define the code region as a NSC */
    struct spctrl_def* spctrl = CMSDK_SPCTRL;
    spctrl->nsccfg |= NSCCFG_CODENSC;
#endif    

    
}


void ppc_configure_to_non_secure(enum ppc_bank_e bank, uint16_t pos)
{
#ifndef M2351
    /* Setting NS flag for peripheral to enable NS access */
    struct spctrl_def* spctrl = CMSDK_SPCTRL;
    ((uint32_t*)&(spctrl->ahbnsppc0))[bank] |= (1U << pos);
#endif    
}

void ppc_configure_to_secure(enum ppc_bank_e bank, uint16_t pos)
{
#ifndef M2351
    /* Clear NS flag for peripheral to prevent NS access */
    struct spctrl_def* spctrl = CMSDK_SPCTRL;
    ((uint32_t*)&(spctrl->ahbnsppc0))[bank] &= ~(1U << pos);
#endif    
}

void ppc_en_secure_unpriv(enum ppc_bank_e bank, uint16_t pos)
{
#ifndef M2351
    struct spctrl_def* spctrl = CMSDK_SPCTRL;
    ((uint32_t*)&(spctrl->ahbspppc0))[bank] |= (1U << pos);
#endif    
}

void ppc_clr_secure_unpriv(enum ppc_bank_e bank, uint16_t pos)
{
#ifndef M2351
    struct spctrl_def* spctrl = CMSDK_SPCTRL;
    ((uint32_t*)&(spctrl->ahbspppc0))[bank] &= ~(1U << pos);
#endif    
}

void ppc_clear_irq(void)
{
#ifndef M2351
    struct spctrl_def* spctrl = CMSDK_SPCTRL;
    /* Clear APB PPC EXP2 IRQ */
    spctrl->secppcintclr = CMSDK_APB_PPCEXP2_INT_POS_MASK;
#endif    
}


#ifdef M2351


typedef struct {
    char *name;
    uint32_t u32Addr;
    uint8_t u8NSIdx;
} IP_T;

IP_T ip_tbl[] = {
{"SYS",SYS_BASE,0},
{"CLK",CLK_BASE,0},
{"INT",INT_BASE,0},
{"GPIOA",GPIOA_BASE,224+0},
{"GPIOB",GPIOB_BASE,224+1},
{"GPIOC",GPIOC_BASE,224+2},
{"GPIOD",GPIOD_BASE,224+3},
{"GPIOE",GPIOE_BASE,224+4},
{"GPIOF",GPIOF_BASE,224+5},
{"GPIOG",GPIOG_BASE,224+6},
{"GPIOH",GPIOH_BASE,224+7},
{"GPIO_DBCTL",GPIO_DBCTL_BASE,0},
{"PA",GPIO_PIN_DATA_BASE       ,224+0},
{"PB",GPIO_PIN_DATA_BASE+16*4  ,224+0},
{"PC",GPIO_PIN_DATA_BASE+2*16*4,224+0},
{"PD",GPIO_PIN_DATA_BASE+3*16*4,224+0},
{"PE",GPIO_PIN_DATA_BASE+4*16*4,224+0},
{"PF",GPIO_PIN_DATA_BASE+5*16*4,224+0},
{"PG",GPIO_PIN_DATA_BASE+6*16*4,224+0},
{"PH",GPIO_PIN_DATA_BASE+7*16*4,224+0},
{"PDMA0",PDMA0_BASE,0},
{"PDMA1",PDMA1_BASE,PDMA1_Attr},
{"USBH",USBH_BASE,USBH_Attr},
{"FMC",FMC_BASE,0},
{"SDH0",SDH0_BASE,SDH0_Attr},
{"EBI",EBI_BASE,EBI_Attr},
{"SCU",SCU_BASE,0},
{"CRC",CRC_BASE,CRC_Attr},
{"CRPT",CRPT_BASE,CRPT_Attr},
{"WDT",WDT_BASE,0},
{"WWDT",WWDT_BASE,0},
{"RTC",RTC_BASE,RTC_Attr},
{"EADC",EADC_BASE,EADC_Attr},
{"ACMP01",ACMP01_BASE,ACMP01_Attr},
{"DAC0",DAC0_BASE,DAC_Attr},
{"DAC1",DAC1_BASE,DAC_Attr},
{"I2S0",I2S0_BASE,I2S0_Attr},
{"OTG",OTG_BASE,OTG_Attr},
{"TMR01",TMR01_BASE,0},
{"TMR23",TMR23_BASE,TMR23_Attr},
{"EPWM0",EPWM0_BASE,EPWM0_Attr},
{"EPWM1",EPWM1_BASE,EPWM1_Attr},
{"BPWM0",BPWM0_BASE,BPWM0_Attr},
{"BPWM1",BPWM1_BASE,BPWM1_Attr},
{"QSPI0",QSPI0_BASE,QSPI0_Attr},
{"SPI0",SPI0_BASE,SPI0_Attr},
{"SPI1",SPI1_BASE,SPI1_Attr},
{"SPI2",SPI2_BASE,SPI2_Attr},
{"SPI3",SPI3_BASE,SPI3_Attr},
{"UART0",UART0_BASE,UART0_Attr},
{"UART1",UART1_BASE,UART1_Attr},
{"UART2",UART2_BASE,UART2_Attr},
{"UART3",UART3_BASE,UART3_Attr},
{"UART4",UART4_BASE,UART4_Attr},
{"UART5",UART5_BASE,UART5_Attr},
{"I2C0",I2C0_BASE,I2C0_Attr},
{"I2C1",I2C1_BASE,I2C1_Attr},
{"I2C2",I2C2_BASE,I2C2_Attr},
{"SC0",SC0_BASE,SC0_Attr},
{"SC1",SC1_BASE,SC1_Attr},
{"SC2",SC2_BASE,SC2_Attr},
{"CAN0",CAN0_BASE,CAN0_Attr},
{"QEI0",QEI0_BASE,QEI0_Attr},
{"QEI1",QEI1_BASE,QEI1_Attr},
{"ECAP0",ECAP0_BASE,ECAP0_Attr},
{"ECAP1",ECAP1_BASE,ECAP1_Attr},
{"TRNG",TRNG_BASE,TRNG_Attr},
{"USBD",USBD_BASE,USBD_Attr},
{"USCI0",USCI0_BASE, USCI0_Attr},
{"USCI1",USCI1_BASE, USCI1_Attr},
{0,USCI1_BASE+4096, 0},
};

#define LF  "\r\n"

uint32_t ProcessHardFault(uint32_t lr, uint32_t msp, uint32_t psp)
{
    extern void SCU_IRQHandler();
    uint32_t *sp;
    int32_t i;
    uint32_t inst, addr,taddr,tdata;
    int32_t secure;
    uint32_t rm,rn,rt, imm5, imm8;
    int32_t eFlag;
    uint8_t idx, bit;
    int32_t s;

    /* Check the used stack */
    secure = (lr & 0x40ul)?1:0;
    if(secure)
    {
        /* Secure stack used */
        if(lr & 4UL)
        {
            sp = (uint32_t *)psp;
        }
        else
        {
            sp = (uint32_t *)msp;
        }
    
    }
#if defined (__ARM_FEATURE_CMSE) &&  (__ARM_FEATURE_CMSE == 3)    
    else
    {
        /* Non-secure stack used */
        if(lr & 4)
            sp = (uint32_t *)__TZ_get_PSP_NS();
        else
            sp = (uint32_t *)__TZ_get_MSP_NS();
    
    }
#endif    
    
    /*
        r0  = sp[0]
        r1  = sp[1]
        r2  = sp[2]
        r3  = sp[3]
        r12 = sp[4]
        lr  = sp[5]
        pc  = sp[6]
        psr = sp[7]
    */
    
    printf("!!---------------------------------------------------------------!!"LF);
    printf("                       <<< HardFault >>>"LF);
    /* Get the instruction caused the hardfault */
    addr = sp[6];
    inst = M16(addr);
    eFlag = 0;
    if((!secure) && ((addr & NS_OFFSET) == 0) )
    {
        printf("  Non-secure CPU try to fetch secure code in 0x%x"LF, addr);
        printf("  Try to check NSC region or SAU settings."LF);
        
        eFlag = 1;
    }else if(inst == 0xBEAB)
    {
        printf("  [0x%08x] 0x%04x Execute BKPT without ICE connected"LF, addr, inst);
        eFlag = 2;
    }    
    else if((inst >> 12) == 5)
    {
        eFlag = 3;
        /* 0101xx Load/store (register offset) on page C2-327 of armv8m ref */
        rm = (inst >> 6) & 0x7;
        rn = (inst >> 3) & 0x7;
        rt = inst & 0x7;
        
        taddr = sp[rn] + sp[rm];
        tdata = sp[rt];
        if(rn == rt)
        {
            printf("  [0x%08x] 0x%04x %s R%d [0x%x]"LF,addr, inst, 
            (inst&BIT11)?"LDR":"STR",rt, taddr);
        }
        else
        {
            printf("  [0x%08x] 0x%04x %s 0x%x [0x%x]"LF,addr, inst, 
            (inst&BIT11)?"LDR":"STR",tdata, taddr);
        }
        
    }
    else if((inst >> 13) == 3)
    {
        eFlag = 3;
        /* 011xxx    Load/store word/byte (immediate offset) on page C2-327 of armv8m ref */
        imm5 = (inst >> 6) & 0x1f;
        rn = (inst >> 3) & 0x7;
        rt = inst & 0x7;
        
        taddr = sp[rn] + imm5;
        tdata = sp[rt];
        if(rt == rn)
        {
            printf("  [0x%08x] 0x%04x %s R%d [0x%x]"LF,addr, inst, 
            (inst&BIT11)?"LDR":"STR",rt, taddr);
        }
        else
        {
            printf("  [0x%08x] 0x%04x %s 0x%x [0x%x]"LF,addr, inst, 
            (inst&BIT11)?"LDR":"STR",tdata, taddr);
        }
    }
    else if((inst >> 12) == 8)
    {
        eFlag = 3;
        /* 1000xx    Load/store halfword (immediate offset) on page C2-328 */
        imm5 = (inst >> 6) & 0x1f;
        rn = (inst >> 3) & 0x7;
        rt = inst & 0x7;
        
        taddr = sp[rn] + imm5;
        tdata = sp[rt];
        if(rt == rn)
        {
            printf("  [0x%08x] 0x%04x %s R%d [0x%x]"LF,addr, inst, 
            (inst&BIT11)?"LDR":"STR",rt, taddr);
        }
        else
        {
            printf("  [0x%08x] 0x%04x %s 0x%x [0x%x]"LF,addr, inst, 
            (inst&BIT11)?"LDR":"STR",tdata, taddr);
        }
        
    }
    else if((inst >> 12) == 9)
    {
        eFlag = 3;
        /* 1001xx    Load/store (SP-relative) on page C2-328 */
        imm8 = inst & 0xff;
        rt = (inst >> 8) & 0x7;
        
        taddr = sp[6] + imm8;
        tdata = sp[rt];
        printf("  [0x%08x] 0x%04x %s 0x%x [0x%x]"LF,addr, inst, 
        (inst&BIT11)?"LDR":"STR",tdata, taddr);
    }
    else
    {
        eFlag = 4;
        printf("  [0x%08x] 0x%04x Unexpected instruction"LF, addr, inst);
    }
    
    if(eFlag == 3)
    {
        /* It is LDR/STR hardfault */
        if(!secure) 
        {
            /* It is happened in Nonsecure code */
            
            for(i=0;i< sizeof(ip_tbl)-1;i++)
            {
                /* Case 1: Nonsecure code try to access secure IP. It also causes SCU violation */
                if((taddr >= ip_tbl[i].u32Addr) && (taddr < (ip_tbl[i+1].u32Addr)))
                {
                    idx = ip_tbl[i].u8NSIdx;
                    bit = idx & 0x1f;
                    idx = idx >> 5;
                    s = (SCU->PNSSET[idx] >> bit) & 1ul;
                    printf("  Illegal access to %s %s in Nonsecure code."LF,(s)?"Nonsecure":"Secure", ip_tbl[i].name);
                    break;
                }
                
                /* Case 2: Nonsecure code try to access Nonsecure IP but the IP is secure IP */
                if((taddr >= (ip_tbl[i].u32Addr+NS_OFFSET)) && (taddr < (ip_tbl[i+1].u32Addr+NS_OFFSET)))
                {
                    idx = ip_tbl[i].u8NSIdx;
                    bit = idx & 0x1f;
                    idx = idx >> 5;
                    s = (SCU->PNSSET[idx] >> bit) & 1ul;
                    printf("  Illegal access to %s %s in Nonsecure code."LF"It may be set as secure IP here.LF",(s)?"Nonsecure":"Secure", ip_tbl[i].name);
                    break;
                }
            }
        }
        else
        {
            /* It is happened in secure code */
            
            
            if(taddr > NS_OFFSET)
            {
                /* Case 3: Secure try to access secure IP through Nonsecure address. It also causes SCU violation */
                for(i=0;i< sizeof(ip_tbl)-1;i++)
                {
                    if((taddr >= (ip_tbl[i].u32Addr+NS_OFFSET)) && (taddr < (ip_tbl[i+1].u32Addr+NS_OFFSET)))
                    {
                        idx = ip_tbl[i].u8NSIdx;
                        bit = idx & 0x1f;
                        idx = idx >> 5;
                        s = (SCU->PNSSET[idx] >> bit) & 1ul;
                        printf("  Illegal to use Nonsecure address to access %s %s in Secure code"LF,(s)?"Nonsecure":"Secure", ip_tbl[i].name);
                        break;
                    }
                }
            }
        
        
        }
    }
    
    printf("!!---------------------------------------------------------------!!"LF);
    
    /* Or *sp to remove compiler warning */
    while(1U|*sp){}
    
    return lr;
}



#endif