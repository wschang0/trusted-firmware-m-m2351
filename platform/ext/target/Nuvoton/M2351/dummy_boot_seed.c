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

#include "platform/include/tfm_plat_boot_seed.h"
#include "NuMicro.h"

/*!
 * \def BOOT_SEED
 *
 * \brief Fixed value for boot seed used for test.
 */
#define BOOT_SEED   0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, \
                    0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF, \
                    0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, \
                    0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF

//static const uint8_t boot_seed[BOOT_SEED_SIZE] = {BOOT_SEED};

enum tfm_plat_err_t tfm_plat_get_boot_seed(uint32_t size, uint8_t *buf)
{
    /* FixMe: - This getter function must be ported per target platform.
     *        - Platform service shall provide an API to further interact this
     *          getter function to retrieve the boot seed.
     */

    uint32_t i;
	uint32_t u32Reg;

    if (size != BOOT_SEED_SIZE) {
        return TFM_PLAT_ERR_SYSTEM_ERR;
    }

	/* Basic Configuration */
	CLK->APBCLK0 |= CLK_APBCLK0_RTCCKEN_Msk;
	CLK->APBCLK1 |= CLK_APBCLK1_TRNGCKEN_Msk;
	RTC->LXTCTL |= RTC_LXTCTL_LIRC32KEN_Msk | RTC_LXTCTL_C32KS_Msk;

	SYS->IPRST1 |= SYS_IPRST1_TRNGRST_Msk;
	SYS->IPRST1 ^= SYS_IPRST1_TRNGRST_Msk;

	TRNG->ACT |= TRNG_ACT_ACT_Msk;
	/* Waiting for ready */
	while ((TRNG->CTL & TRNG_CTL_READY_Msk) == 0);

	TRNG->CTL = (9 << TRNG_CTL_CLKP_Pos);

	u32Reg = TRNG->CTL;
	for (i = 0;i < size;i++)
	{
		TRNG->CTL = TRNG_CTL_TRNGEN_Msk | u32Reg;
		while ((TRNG->CTL&TRNG_CTL_DVIF_Msk) == 0);
		buf[i] = TRNG->DATA;
	}

    return TFM_PLAT_ERR_SUCCESS;
}
