/*
 * Copyright (c) 2017-2019 ARM Limited
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

#include "platform/include/tfm_plat_crypto_keys.h"
#include <stddef.h>
#include "flash_layout.h"
#include "NuMicro.h"

/* FIXME: Functions in this file should be implemented by platform vendor. For
 * the security of the storage system, it is critical to use a hardware unique
 * key. For the security of the attestation, it is critical to use a unique key
 * pair and keep the private key is secret.
 *
 * AN519 does not have any available hardware unique key engine, so a
 * software stub has been implemented in this case.
 */

#define TFM_KEY_LEN_BYTES  16

__ALIGNED(4)
static const uint8_t sample_tfm_key[TFM_KEY_LEN_BYTES] = 
{ 0x8C, 0x7B, 0xFE, 0xD8, 0x86, 0x36, 0x15, 0x00, 0x43, 0x7A, 0x85, 0xBD, 0x66, 0x81, 0x08, 0x60 };

extern const enum ecc_curve_t initial_attestation_curve_type;
extern const uint8_t  initial_attestation_private_key[];
extern const uint32_t initial_attestation_private_key_size;
extern const uint8_t  initial_attestation_public_x_key[];
extern const uint32_t initial_attestation_public_x_key_size;
extern const uint8_t  initial_attestation_public_y_key[];
extern const uint32_t initial_attestation_public_y_key_size;

void ReadOtpHash(uint32_t u32StartOtpNum, uint32_t au32OtpHash[8])
{
    int32_t i;
    uint32_t u32OtpNum;

    /* Read key hash value from OTP */
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;
    i = 0;
    for(u32OtpNum = u32StartOtpNum; u32OtpNum < u32StartOtpNum + 4; u32OtpNum++)
    {
        FMC->ISPCMD = FMC_ISPCMD_READ_64;
        FMC->ISPADDR = FMC_OTP_BASE + u32OtpNum * 8UL;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        while(FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk) {}
        au32OtpHash[i] = FMC->MPDAT0;
        au32OtpHash[i + 1] = FMC->MPDAT1;
        i += 2;
    }
}


/* SHA256 by hardware */
int32_t SHAHash(uint32_t u32Mode, uint32_t *pu32Addr, int32_t size, uint32_t digest[])
{
	int32_t i;
	int32_t n;

	/* Enable CRYPTO */
	CLK->AHBCLK |= CLK_AHBCLK_CRPTCKEN_Msk;

	/* Init SHA */
	CRPT->HMAC_CTL = (u32Mode << CRPT_HMAC_CTL_OPMODE_Pos) | CRPT_HMAC_CTL_INSWAP_Msk | CRPT_HMAC_CTL_OUTSWAP_Msk;
	CRPT->HMAC_DMACNT = size;

	/* Calculate SHA */
	while (size > 0)
	{
		if (size <= 4)
		{
			CRPT->HMAC_CTL |= CRPT_HMAC_CTL_DMALAST_Msk;
		}

		/* Trigger to start SHA processing */
		CRPT->HMAC_CTL |= CRPT_HMAC_CTL_START_Msk;

		/* Waiting for SHA data input ready */
		while ((CRPT->HMAC_STS & CRPT_HMAC_STS_DATINREQ_Msk) == 0);

		/* Input new SHA date */
		CRPT->HMAC_DATIN = *pu32Addr;
		pu32Addr++;
		size -= 4;
	}

	/* Waiting for calculation done */
	while (CRPT->HMAC_STS & CRPT_HMAC_STS_BUSY_Msk);

	/* return SHA results */
	n = 0;
	if (u32Mode == SHA_MODE_SHA1)
		n = 5;
	else if (u32Mode == SHA_MODE_SHA224)
		n = 7;
	else if (u32Mode == SHA_MODE_SHA256)
		n = 8;
	else if (u32Mode == SHA_MODE_SHA384)
		n = 12;

	for (i = 0; i < n; i++)
		digest[i] = CRPT->HMAC_DGST[i];

	return 0;
}


/**
 * \brief Copy the key to the destination buffer
 *
 * \param[out]  p_dst  Pointer to buffer where to store the key
 * \param[in]   p_src  Pointer to the key
 * \param[in]   size   Length of the key
 */
static inline void copy_key(uint8_t *p_dst, const uint8_t *p_src, size_t size)
{
    uint32_t i;

    for (i = size; i > 0; i--) {
        *p_dst = *p_src;
        p_src++;
        p_dst++;
    }
}

enum tfm_plat_err_t tfm_plat_get_crypto_huk(uint8_t *key, uint32_t size)
{
	int32_t i;
	uint32_t hash[8];
	uint32_t otp[8];

    if(size > TFM_KEY_LEN_BYTES) {
        return TFM_PLAT_ERR_SYSTEM_ERR;
    }
	
	/* Calculate HUK key hash */
	SHAHash(SHA_MODE_SHA256, (uint32_t *)sample_tfm_key, 16, hash);
        
    /* Read HUK hash from OTP */
    ReadOtpHash(OTP_HUK_HASH_BASE, otp);

	/* Check if the key hash matching */
	for (i = 0;i < 8;i++)
	{
		if (otp[i] != hash[i])
		{
			/* HUK is not match key hash in OTP */
			return TFM_PLAT_ERR_SYSTEM_ERR;
		}
	}

	/* Return HUK */
    copy_key(key, sample_tfm_key, size);

    return TFM_PLAT_ERR_SUCCESS;
}

enum tfm_plat_err_t
tfm_plat_get_initial_attest_key(uint8_t          *key_buf,
                                uint32_t          size,
                                struct ecc_key_t *ecc_key,
                                enum ecc_curve_t *curve_type)
{
    uint8_t *key_dst;
    const uint8_t *key_src;
    uint32_t key_size;
    uint32_t full_key_size = initial_attestation_private_key_size  +
                             initial_attestation_public_x_key_size +
                             initial_attestation_public_y_key_size;
    uint32_t au32OtpHash[8];
    uint32_t au32Hash[8];
    int32_t i;

    if (size < full_key_size) {
        return TFM_PLAT_ERR_SYSTEM_ERR;
    }

    /* Set the EC curve type which the key belongs to */
    *curve_type = initial_attestation_curve_type;

    /* Calculate key hash */
    SHAHash(SHA_MODE_SHA256, (uint32_t *)initial_attestation_private_key, 32, au32Hash);

    /* Get IAK Hash from OTP */
    ReadOtpHash(OTP_IAK_HASH_BASE, au32OtpHash);

    /* Check the key hash with OTP */
    for(i = 0; i < 8; i++)
    {
        if(au32OtpHash[i] != au32Hash[i])
        {
            return TFM_PLAT_ERR_SYSTEM_ERR;
        }
    }


    /* Copy the private key to the buffer, it MUST be present */
    key_dst  = key_buf;
    key_src  = initial_attestation_private_key;
    key_size = initial_attestation_private_key_size;
    copy_key(key_dst, key_src, key_size);
    ecc_key->priv_key = key_dst;
    ecc_key->priv_key_size = key_size;

    /* Copy the x-coordinate of public key to the buffer, it MIGHT be present */
    if (initial_attestation_public_x_key_size != 0) {
        key_dst  = key_dst + key_size;
        key_src  = initial_attestation_public_x_key;
        key_size = initial_attestation_public_x_key_size;
        copy_key(key_dst, key_src, key_size);
        ecc_key->pubx_key = key_dst;
        ecc_key->pubx_key_size = key_size;
    } else {
        ecc_key->pubx_key = NULL;
        ecc_key->pubx_key_size = 0;
    }

    /* Copy the y-coordinate of public key to the buffer, it MIGHT be present */
    if (initial_attestation_public_y_key_size != 0) {
        key_dst  = key_dst + key_size;
        key_src  = initial_attestation_public_y_key;
        key_size = initial_attestation_public_y_key_size;
        copy_key(key_dst, key_src, key_size);
        ecc_key->puby_key = key_dst;
        ecc_key->puby_key_size = key_size;
    } else {
        ecc_key->puby_key = NULL;
        ecc_key->puby_key_size = 0;
    }

    return TFM_PLAT_ERR_SUCCESS;
}
