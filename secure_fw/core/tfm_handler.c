/*
 * Copyright (c) 2017-2019, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdio.h>
#include <string.h>

#include "secure_utilities.h"
#include "tfm_svc.h"
#include "tfm_secure_api.h"
#include "region_defs.h"
#include "tfm_api.h"
#include "tfm_internal.h"
#include "tfm_memory_utils.h"
#ifdef TFM_PSA_API
#include <stdbool.h>
#include "tfm_svcalls.h"
#endif

/* This SVC handler is called when a secure partition requests access to a
 * buffer area
 */
extern int32_t tfm_core_set_buffer_area_handler(const uint32_t args[]);
#ifdef TFM_PSA_API
extern void tfm_psa_ipc_request_handler(const uint32_t svc_args[]);
#endif

struct tfm_fault_context_s {
    uint32_t R0;
    uint32_t R1;
    uint32_t R2;
    uint32_t R3;
    uint32_t R12;
    uint32_t LR;
    uint32_t ReturnAddress;
    uint32_t RETPSR;
} tfm_fault_context;

#if defined(__ARM_ARCH_8M_MAIN__)
/**
 * \brief Overwrites default Secure fault handler.
 */
void SecureFault_Handler(void)
{
    /* figure out context from which we landed in fault handler */
    uint32_t lr = __get_LR();
    uint32_t sp;

    if (lr & EXC_RETURN_SECURE_STACK) {
        if (lr & EXC_RETURN_STACK_PROCESS) {
            sp = __get_PSP();
        } else {
            sp = __get_MSP();
        }
    } else {
        if (lr & EXC_RETURN_STACK_PROCESS) {
            sp =  __TZ_get_PSP_NS();
        } else {
            sp = __TZ_get_MSP_NS();
        }
    }

    /* Only save the context if sp is valid */
    if ((sp >=  S_DATA_START &&
         sp <=  (S_DATA_LIMIT - sizeof(tfm_fault_context)) + 1) ||
        (sp >= NS_DATA_START &&
         sp <= (NS_DATA_LIMIT - sizeof(tfm_fault_context)) + 1)) {
        tfm_memcpy(&tfm_fault_context,
                   (const void *)sp,
                   sizeof(tfm_fault_context));
    }

    LOG_MSG("Oops... Secure fault!!! You're not going anywhere!");
    while (1) {
        ;
    }
}
#elif defined(__ARM_ARCH_8M_BASE__)
/**
 * \brief Overwrites default Hard fault handler.
 *
 * In case of a baseline implementation fault conditions that would generate a
 * SecureFault in a mainline implementation instead generate a Secure HardFault.
 */
# ifndef M2351 
void HardFault_Handler(void)
{
    /* In a baseline implementation there is no way, to find out whether this is
     * a hard fault triggered directly, or another fault that has been
     * escalated.
     */

    while (1) {
        ;
    }
}
# else

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

#define LF	"\r\n"

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
        /* 011xxx	 Load/store word/byte (immediate offset) on page C2-327 of armv8m ref */
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
        /* 1000xx	 Load/store halfword (immediate offset) on page C2-328 */
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
        /* 1001xx	 Load/store (SP-relative) on page C2-328 */
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



# endif

#else
#error "Unsupported ARM Architecture."
#endif

#if defined(__ARM_ARCH_8M_MAIN__)
__attribute__((naked)) void SVC_Handler(void)
{
    __ASM volatile(
    "TST     lr, #4\n"  /* Check store SP in thread mode to r0 */
    "IT      EQ\n"
    "BXEQ    lr\n"
    "MRS     r0, PSP\n"
    "MOV     r1, lr\n"
    "BL      SVCHandler_main\n"
    "BX      r0\n"
    );
}
#elif defined(__ARM_ARCH_8M_BASE__)
__attribute__((naked)) void SVC_Handler(void)
{
    __ASM volatile(
    ".syntax unified\n"
    "MOVS    r0, #4\n"  /* Check store SP in thread mode to r0 */
    "MOV     r1, lr\n"
    "TST     r0, r1\n"
    "BEQ     handler\n"
    "MRS     r0, PSP\n"  /* Coming from thread mode */
    "B sp_stored\n"
    "handler:\n"
    "BX      lr\n"  /* Coming from handler mode */
    "sp_stored:\n"
    "MOV     r1, lr\n"
    "BL      SVCHandler_main\n"
    "BX      r0\n"
    );
}
#else
#error "Unsupported ARM Architecture."
#endif

uint32_t SVCHandler_main(uint32_t *svc_args, uint32_t lr)
{
    uint8_t svc_number;
    /*
     * Stack contains:
     * r0, r1, r2, r3, r12, r14 (lr), the return address and xPSR
     * First argument (r0) is svc_args[0]
     */
    if (lr & EXC_RETURN_SECURE_STACK) {
        /* SV called directly from secure context. Check instruction for
         * svc_number
         */
        svc_number = ((uint8_t *)svc_args[6])[-2];
    } else {
        /* Secure SV executing with NS return.
         * NS cannot directly trigger S SVC so this should not happen
         * FixMe: check for security implications
         */
        return lr;
    }
    switch (svc_number) {
#ifdef TFM_PSA_API
    case TFM_SVC_IPC_REQUEST:
        tfm_psa_ipc_request_handler(svc_args);
        break;
    case TFM_SVC_SCHEDULE:
    case TFM_SVC_EXIT_THRD:
    case TFM_SVC_PSA_FRAMEWORK_VERSION:
    case TFM_SVC_PSA_VERSION:
    case TFM_SVC_PSA_CONNECT:
    case TFM_SVC_PSA_CALL:
    case TFM_SVC_PSA_CLOSE:
    case TFM_SVC_PSA_WAIT:
    case TFM_SVC_PSA_GET:
    case TFM_SVC_PSA_SET_RHANDLE:
    case TFM_SVC_PSA_READ:
    case TFM_SVC_PSA_SKIP:
    case TFM_SVC_PSA_WRITE:
    case TFM_SVC_PSA_REPLY:
    case TFM_SVC_PSA_NOTIFY:
    case TFM_SVC_PSA_CLEAR:
    case TFM_SVC_PSA_EOI:
        svc_args[0] = SVC_Handler_IPC(svc_number, svc_args, lr);
        break;
#else
    case TFM_SVC_SFN_REQUEST:
        lr = tfm_core_partition_request_svc_handler(svc_args, lr);
        break;
    case TFM_SVC_SFN_RETURN:
        lr = tfm_core_partition_return_handler(lr);
        break;
    case TFM_SVC_VALIDATE_SECURE_CALLER:
        tfm_core_validate_secure_caller_handler(svc_args);
        break;
    case TFM_SVC_GET_CALLER_CLIENT_ID:
        tfm_core_get_caller_client_id_handler(svc_args);
        break;
    case TFM_SVC_SPM_REQUEST:
        tfm_core_spm_request_handler((struct tfm_exc_stack_t *)svc_args);
        break;
    case TFM_SVC_MEMORY_CHECK:
        tfm_core_memory_permission_check_handler(svc_args);
        break;
    case TFM_SVC_SET_SHARE_AREA:
        tfm_core_set_buffer_area_handler(svc_args);
        break;
#endif
    case TFM_SVC_PRINT:
        printf("\e[1;34m[Sec Thread] %s\e[0m\r\n", (char *)svc_args[0]);
        break;
    case TFM_SVC_GET_BOOT_DATA:
        tfm_core_get_boot_data_handler(svc_args);
        break;
    default:
        LOG_MSG("Unknown SVC number requested!");
        break;
    }

    return lr;
}

void tfm_access_violation_handler(void)
{
    while (1) {
        ;
    }
}
