/*
 * Copyright (c) 2018-2019, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

/* NOTE: This API should be implemented by platform vendor. For the security of
 * the secure storage system's and the bootloader's rollback protection etc. it
 * is CRITICAL to use a internal (in-die) persistent memory for multiple time
 * programmable (MTP) non-volatile counters or use a One-time Programmable (OTP)
 * non-volatile counters solution.
 *
 * AN521 does not have any available MTP or OTP non-volatile counters, so a
 * software dummy implementation has been implemented in this case. The current
 * implementation is not resistant to asynchronous power failures and should
 * not be used in production code. It is exclusively for testing purposes.
 */

#include "platform/include/tfm_plat_nv_counters.h"

#include <limits.h>
#include "Driver_Flash.h"
#include "flash_layout.h"
#include "NuMicro.h"

/* Compilation time checks to be sure the defines are well defined */
#ifndef TFM_NV_COUNTERS_AREA_ADDR
#error "TFM_NV_COUNTERS_AREA_ADDR must be defined in flash_layout.h"
#endif

#ifndef TFM_NV_COUNTERS_AREA_SIZE
#error "TFM_NV_COUNTERS_AREA_SIZE must be defined in flash_layout.h"
#endif

#ifndef TFM_NV_COUNTERS_SECTOR_ADDR
#error "TFM_NV_COUNTERS_SECTOR_ADDR must be defined in flash_layout.h"
#endif

#ifndef TFM_NV_COUNTERS_SECTOR_SIZE
#error "TFM_NV_COUNTERS_SECTOR_SIZE must be defined in flash_layout.h"
#endif

#ifndef FLASH_DEV_NAME
#error "FLASH_DEV_NAME must be defined in flash_layout.h"
#endif
/* End of compilation time checks to be sure the defines are well defined */

#define OTP_NUM_OFFSET   OTP_NV_COUNTER_BASE
#define SECTOR_OFFSET    0
#define NV_COUNTER_SIZE  sizeof(uint8_t)
#define INIT_VALUE_SIZE  NV_COUNTER_SIZE
#define NV_COUNTERS_AREA_OFFSET (TFM_NV_COUNTERS_AREA_ADDR - \
                                 TFM_NV_COUNTERS_SECTOR_ADDR)

#define NV_COUNTERS_INITIALIZED 0xC0DE0042

/* Import the CMSIS flash device driver */
extern ARM_DRIVER_FLASH FLASH_DEV_NAME;


uint8_t tfm_plat_otp_counter(enum tfm_nv_counter_t counter_id)
{
    uint32_t u32OtpNum;
    uint32_t otp[2] = { 0 };
    uint8_t i;

    /* OTP OTP_NUM_OFFSET ~ PLAT_NV_COUNTER_MAX(5) is for nv counter. each has 0~64 count*/
    u32OtpNum = OTP_NUM_OFFSET + (uint32_t)counter_id;
    /* Read key hash value from OTP */
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;
    FMC->ISPCMD = FMC_ISPCMD_READ_64;
    FMC->ISPADDR = FMC_OTP_BASE + u32OtpNum * 8UL;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while(FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk) {}

    otp[0] = FMC->MPDAT0;
    otp[1] = FMC->MPDAT1;

    /* Get zero count */
    for(i = 0;i < 64;i++)
    {
        if(otp[i / 32] & (1 << i))
            break;
    }

    return i;
}


enum tfm_plat_err_t tfm_plat_init_nv_counter(void)
{
    /* We use OTP as nv counter. So we don't need to init it */
    return TFM_PLAT_ERR_SUCCESS;
}

enum tfm_plat_err_t tfm_plat_read_nv_counter(enum tfm_nv_counter_t counter_id,
                                             uint32_t size, uint8_t *val)
{

    if(counter_id > PLAT_NV_COUNTER_MAX)
    {
        return TFM_PLAT_ERR_SYSTEM_ERR;
    }

    *val = tfm_plat_otp_counter(counter_id);

    return TFM_PLAT_ERR_SUCCESS;
}

enum tfm_plat_err_t tfm_plat_set_nv_counter(enum tfm_nv_counter_t counter_id,
                                            uint32_t value)
{
    uint32_t orgValue;
    uint32_t otp[2] = { 0xfffffffful, 0xfffffffful };
    int32_t i;
    uint32_t u32OtpNum;


    if(counter_id > PLAT_NV_COUNTER_MAX)
    {
        return TFM_PLAT_ERR_SYSTEM_ERR;
    }

    if(value > 64)
    {
        return TFM_PLAT_ERR_SYSTEM_ERR;
    }

    orgValue = tfm_plat_otp_counter(counter_id);

    if(value != orgValue)
    {
        if(value > orgValue)
        {
            /* Generate the value of otp */
            for(i = 0;i < value;i++)
            {
                otp[i / 32] ^= (1ul << i);
            }

            for(i = 0; i < 2; i++)
            {
                u32OtpNum = OTP_NUM_OFFSET + (uint32_t)counter_id;
                FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;
                FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
                FMC->ISPADDR = FMC_OTP_BASE + u32OtpNum * 8UL;
                FMC->ISPDAT = otp[i];
                FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
                while(FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk) {}

                if(FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
                {
                    FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
                    return TFM_PLAT_ERR_SYSTEM_ERR;
                }
            }
        }
    }

    return TFM_PLAT_ERR_SUCCESS;
}

enum tfm_plat_err_t tfm_plat_increment_nv_counter(
                                           enum tfm_nv_counter_t counter_id)
{
    uint32_t security_cnt;
    enum tfm_plat_err_t err;

    err = tfm_plat_read_nv_counter(counter_id,
                                   sizeof(security_cnt),
                                   (uint8_t *)&security_cnt);
    if (err != TFM_PLAT_ERR_SUCCESS) {
        return err;
    }

    if (security_cnt == 64) {
        return TFM_PLAT_ERR_MAX_VALUE;
    }

    return tfm_plat_set_nv_counter(counter_id, security_cnt + 1u);
}
