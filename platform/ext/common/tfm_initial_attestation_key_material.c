/*
 * Copyright (c) 2019, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdint.h>
#include "platform/include/tfm_plat_defs.h"
#include "platform/include/tfm_plat_crypto_keys.h"
#include "NuMicro.h"

/*
 * This file contains the hard coded version of the ECDSA P-256 key pair in:
 * platform/common/tfm_initial_attestation_key.pem
 *
 * This key is used to sign the initial attestation token.
 * The key pair is stored in raw format, without any encoding(ASN.1, COSE).
 *
 * This ECDSA P-256 key is the construction of:
 *   - private key:      32 bytes
 *   - public key:
 *       - X-coordinate: 32 bytes
 *       - Y-coordinate: 32 bytes
 *
 * The hash of the raw public key (H(0x04 || X || Y)) is also included, because
 * it is used as an instance ID. It is a unique identifier of the device
 * instance.
 *
 * Instance ID is mapped to:
 *   - UEID in the EAT token
 *
 *   #######  DO NOT USE THIS KEY IN PRODUCTION #######
 */

/* Type of the EC curve which the key belongs to */
TFM_LINK_SET_RO_IN_PARTITION_SECTION("TFM_SP_INITIAL_ATTESTATION")
const enum ecc_curve_t initial_attestation_curve_type = P_256;

/* Initial attestation private key in raw format, without any encoding.
 * It belongs to the ECDSA P-256 curve.
 * It MUST present on the device-
 */
TFM_LINK_SET_RO_IN_PARTITION_SECTION("TFM_SP_INITIAL_ATTESTATION")
__ALIGNED(4)
const uint8_t initial_attestation_private_key[] = 
{
    0x2E, 0xF9, 0xB1, 0xAD, 0xB5, 0xCF, 0x49, 0x74,
    0xBE, 0x5D, 0x88, 0x64, 0xB2, 0xB3, 0x4D, 0xBA,
    0xA3, 0xBA, 0x86, 0x2D, 0x67, 0x67, 0xAE, 0x46,
    0xB2, 0x56, 0xEF, 0xEC, 0xFF, 0x80, 0x66, 0x83
};

TFM_LINK_SET_RO_IN_PARTITION_SECTION("TFM_SP_INITIAL_ATTESTATION")
const uint32_t initial_attestation_private_key_size =
        sizeof(initial_attestation_private_key);

/* Initial attestation x-coordinate of the public key in raw format,
 * without any encoding.
 * It belongs to the ECDSA P-256 curve.
 * It MIGHT be present on the device.
 */
TFM_LINK_SET_RO_IN_PARTITION_SECTION("TFM_SP_INITIAL_ATTESTATION")
const uint8_t initial_attestation_public_x_key[] =
{
    0xAA, 0x26, 0x0A, 0x44, 0xA5, 0xAB, 0x68, 0x69,
    0x6A, 0xC5, 0xD0, 0x7E, 0x6E, 0x01, 0x06, 0xC2,
    0x11, 0x29, 0x6D, 0x9D, 0x5B, 0x4B, 0x16, 0x38,
    0x9F, 0x1A, 0x56, 0x45, 0x04, 0x01, 0x16, 0xF3
};

TFM_LINK_SET_RO_IN_PARTITION_SECTION("TFM_SP_INITIAL_ATTESTATION")
const uint32_t initial_attestation_public_x_key_size =
        sizeof(initial_attestation_public_x_key);

/* Initial attestation y-coordinate of the public key in raw format,
 * without any encoding.
 * It belongs to the ECDSA P-256 curve.
 * It MIGHT be present on the device.
 */
TFM_LINK_SET_RO_IN_PARTITION_SECTION("TFM_SP_INITIAL_ATTESTATION")
const uint8_t initial_attestation_public_y_key[] =
{
    0x1F, 0xAF, 0xB8, 0x03, 0xEB, 0xB9, 0xBC, 0xBA,
    0xFD, 0x29, 0x89, 0xB8, 0xA7, 0xC0, 0x39, 0xBA,
    0x19, 0x37, 0x68, 0xC1, 0x26, 0xA5, 0x21, 0x6E,
    0x9C, 0x38, 0x40, 0x50, 0xC4, 0x2F, 0x3F, 0x63
};

TFM_LINK_SET_RO_IN_PARTITION_SECTION("TFM_SP_INITIAL_ATTESTATION")
const uint32_t initial_attestation_public_y_key_size =
        sizeof(initial_attestation_public_y_key);

/* Hash (SHA256) of initial attestation public key.
 * Byte string representation of ECC public key according to
 * psa_export_public_key() in interface/include/psa/crypto.h:
 * 0x04 || X_coord || Y_coord
 */
TFM_LINK_SET_RO_IN_PARTITION_SECTION("TFM_SP_INITIAL_ATTESTATION")
__ALIGNED(4)
const uint8_t initial_attestation_raw_public_key_hash[] =
{
    0x22, 0x66, 0x59, 0x60, 0x4D, 0xDB, 0x7C, 0x25, 
    0x4A, 0x41, 0x72, 0x00, 0x0F, 0x0C, 0xCE, 0xF4, 
    0x77, 0x6F, 0x2F, 0xD5, 0x47, 0x0A, 0xE4, 0x0F, 
    0x0D, 0x5F, 0x3A, 0x21, 0x2E, 0xBE, 0x46, 0x7F, 
};

TFM_LINK_SET_RO_IN_PARTITION_SECTION("TFM_SP_INITIAL_ATTESTATION")
const uint32_t initial_attestation_raw_public_key_hash_size =
        sizeof(initial_attestation_raw_public_key_hash);
