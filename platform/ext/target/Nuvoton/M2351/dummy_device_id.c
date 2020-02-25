/*
 * Copyright (c) 2018-2019 ARM Limited
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

#include "platform/include/tfm_plat_device_id.h"
#include <stddef.h>
#include "flash_layout.h"
#include "NuMicro.h"

/*
 * NOTE: Functions in this file must be ported per target platform.
 */


extern void ReadOtpHash(uint32_t u32StartOtpNum, uint32_t au32OtpHash[8]);
extern int32_t SHAHash(uint32_t u32Mode, uint32_t *pu32Addr, int32_t size, uint32_t digest[]);

extern const uint8_t  initial_attestation_raw_public_key_hash[];
extern const uint32_t initial_attestation_raw_public_key_hash_size;

/* 
   Update the number from M2351 PSA Level 1 Certificate Number 
   https://www.psacertified.org/products/numicro-m2351-series/
*/
static const uint8_t m2351_ean_13[] = "060456527292810010";

/**
 * \brief Copy the device specific ID to the destination buffer
 *
 * \param[out]  p_dst  Pointer to buffer where to store ID
 * \param[in]   p_src  Pointer to the ID
 * \param[in]   size   Length of the ID
 */
static inline void copy_id(uint8_t *p_dst, const uint8_t *p_src, size_t size)
{
    uint32_t i;

    for (i = size; i > 0; i--) {
        *p_dst = *p_src;
        p_src++;
        p_dst++;
    }
}

/**
 * Instance ID is mapped to EAT Universal Entity ID (UEID)
 * This implementation creates the instance ID as follows:
 *  - byte 0:    0x01 indicates the type of UEID to be GUID
 *  - byte 1-32: Hash of attestation public key. Public key is hashed in raw
 *               format without any encoding.
 */
enum tfm_plat_err_t tfm_plat_get_instance_id(uint32_t *size, uint8_t *buf)
{
    uint8_t *p_dst;
    const uint8_t *p_src = initial_attestation_raw_public_key_hash;
    uint32_t au32OtpHash[8];
    uint32_t au32Hash[8];
    int32_t i;

    if (*size < INSTANCE_ID_MAX_SIZE) {
        return TFM_PLAT_ERR_SYSTEM_ERR;
    }

    /* Calculate HUK key hash */
    SHAHash(SHA_MODE_SHA256, (uint32_t *)initial_attestation_raw_public_key_hash, 32, au32Hash);

    /* Read HUK hash from OTP */
    ReadOtpHash(OTP_IID_HASH_BASE, au32OtpHash);

    /* Check if the key hash matching */
    for(i = 0; i < 8; i++)
    {
        if(au32OtpHash[i] != au32Hash[i])
        {
            /* HUK is not match key hash in OTP */
            return TFM_PLAT_ERR_SYSTEM_ERR;
        }
    }

    buf[0] = 0x01; /* First byte is type byte:  0x01 indicates GUID */
    p_dst = &buf[1];

    copy_id(p_dst, p_src, initial_attestation_raw_public_key_hash_size);

    /* Instance ID size:  1 type byte + size of public key hash */
    *size = initial_attestation_raw_public_key_hash_size + 1;

    return TFM_PLAT_ERR_SUCCESS;
}

enum tfm_plat_err_t tfm_plat_get_implementation_id(uint32_t *size,
                                                   uint8_t  *buf)
{
    int32_t i;
    uint32_t u32ID[4];

    if (*size < 16) {
        return TFM_PLAT_ERR_SYSTEM_ERR;
    }

    /* The implment ID is UID (12 bytes) || PDID (4 bytes) */
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;
    FMC->ISPCMD = FMC_ISPCMD_READ_UID;
    for(i = 0;i < 3;i++)
    {
        FMC->ISPADDR = i*4;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        while(FMC->ISPTRG) {}
        u32ID[i] = FMC->ISPDAT;
    }
    u32ID[3] = SYS->PDID;

    copy_id(buf, (uint8_t *)&u32ID[0], 16);
    *size = 16;

    return TFM_PLAT_ERR_SUCCESS;
}

enum tfm_plat_err_t tfm_plat_get_hw_version(uint32_t *size, uint8_t *buf)
{
    const uint8_t *p_hw_version = m2351_ean_13;
    uint32_t hw_version_size = sizeof(m2351_ean_13) - 1;

    if (*size < hw_version_size) {
        return TFM_PLAT_ERR_SYSTEM_ERR;
    }

    copy_id(buf, p_hw_version, hw_version_size);
    *size = hw_version_size;

    return TFM_PLAT_ERR_SUCCESS;
}
