/*
 * Copyright (c) 2019, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "platform/include/tfm_attest_hal.h"
#include <stdint.h>
#include "NuMicro.h"

/* Example verification service URL for initial attestation token */
static const char verification_service_url[] = "www.nuvoton.com";

/* Example profile definition document for initial attestation token */
static const char attestation_profile_definition[] = "psa-tfm-profile-1.md";

enum tfm_security_lifecycle_t tfm_attest_hal_get_security_lifecycle(void)
{
    uint32_t u32SCRLOCK, u32ARLOCK;
    uint32_t u32Rotpk0Lock;
    
    /* Eanble FMC */
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;

    /* Check if ROTPK locked status */
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = FMC_OTP_BASE + 0x800;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while(FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk) {}
    u32Rotpk0Lock = FMC->ISPDAT;

    /* Check the SCRLOCK status */
    FMC->ISPADDR = FMC_SCRLOCK_BASE;
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while(FMC->ISPTRG){}
    u32SCRLOCK = FMC->ISPDAT & 0xFFul;

    /* Check the ARLOCK status */
    FMC->ISPADDR = FMC_ARLOCK_BASE;
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while(FMC->ISPTRG) {}
    u32ARLOCK = FMC->ISPDAT & 0xFFul;

    if(u32Rotpk0Lock == 0xfffffffful)
    {
        if((u32SCRLOCK == 0x5a) && (u32ARLOCK == 0x5a))
        {
            /* No ROTPK, No SCRLOCK, No ARLOCK */
            return TFM_SLC_ASSEMBLY_AND_TEST;
        }
        else
        {
            /* No ROTPK but SCRLOCK or ARLOCK */
            return TFM_SLC_UNKNOWN;
        }
    }
    else
    {
        if((u32SCRLOCK == 0x5a) && (u32ARLOCK == 0x5a))
        {
            /* ROTPK lock, no SCRLOCK and no ARLOCK */
            return TFM_SLC_PSA_ROT_PROVISIONING;
        }
        else if((u32SCRLOCK != 0x5a) && (u32ARLOCK == 0x5a))
        {
            /* ROTPK lock, SCRLOCK, No ARLOCK*/
            return TFM_SLC_NON_PSA_ROT_DEBUG;
        }
        else if(u32ARLOCK != 0x5a)
        {
            /* ROTPK lock, ARLOCK */
            return TFM_SLC_SECURED;
        }
        else
        {
            /* Should not be here*/
            return TFM_SLC_UNKNOWN;
        }
    }
}

const char *
tfm_attest_hal_get_verification_service(uint32_t *size)
{
    *size = sizeof(verification_service_url) - 1;

    return verification_service_url;
}

const char *
tfm_attest_hal_get_profile_definition(uint32_t *size)
{
    *size = sizeof(attestation_profile_definition) - 1;

    return attestation_profile_definition;
}
