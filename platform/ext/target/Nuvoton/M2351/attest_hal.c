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
    /* check the lock status */
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;
    FMC->ISPADDR = FMC_SCRLOCK_BASE;
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while(FMC->ISPTRG){}
    u32SCRLOCK = FMC->ISPDAT;
    FMC->ISPADDR = FMC_ARLOCK_BASE;
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while(FMC->ISPTRG) {}
    u32ARLOCK = FMC->ISPDAT;

    if(((FMC->ISPDAT&0xff) != 0x5a) || ((FMC->ISPDAT&0xff) != 0x5a))
        return TFM_SLC_SECURED;
    else
        return TFM_SLC_UNKNOWN;

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
