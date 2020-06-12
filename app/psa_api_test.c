/*
 * Copyright (c) 2019, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include "psa/protected_storage.h"



#include "psa_api_test.h"
#include "tfm_nspm_api.h"
#include "tfm_integ_test.h"

#if 1

#define TEST_UID_1      2U
#define UID_BASE_VALUE  0


 /* enums */
typedef enum {
    NONSECURE = 0x0,
    SECURE = 0x1,
} security_t;




#define TEST_BUFF_SIZE 16

static uint8_t write_buff[TEST_BUFF_SIZE] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                             0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };
static uint8_t read_buff[TEST_BUFF_SIZE] = { 0 };
static uint8_t buff1[TEST_BUFF_SIZE] = { 0 };
//static uint8_t read_buff2[14] = { 0 };
static struct psa_ps_info_t info = {0};
static int32_t test_i = 0;



////////////////////////////////////////////////////////////////////////////////////////

static int32_t sst_calls_get_call(psa_ps_uid_t p_uid)
{
    uint32_t status = 0;
    size_t p_data_length = 0;


    /*1) set() a UID */
    status = psa_ps_set(p_uid, TEST_BUFF_SIZE, write_buff, 0);


    //2) uid = 0, crash
    status = psa_ps_get(0, 3, 4, read_buff);

    //3) uid = null, crash
    status = psa_ps_get(NULL, 3, 4, read_buff);

    //4) uid = 5, crash, uid = 7, crash
    status = psa_ps_get(7, 0, 4, read_buff);

    //11) p_data = null, data_length > 0, crash
    uint8_t *read_buff1 = NULL;
    status = psa_ps_get(p_uid, 0, 1, read_buff1);

    //12) p_data = buff[data_length-1]
    uint8_t *read_buff2 = NULL;
    read_buff2 = (uint8_t *)malloc(14);
    p_data_length = 15;
    status = psa_ps_get(p_uid, 0, p_data_length, read_buff2);

    /*13) remove() the set UID */
    status = psa_ps_remove(p_uid);

    return PSA_PS_SUCCESS;
}

/***********************************************************************/
//set

static int32_t sst_calls_set_call(psa_ps_uid_t p_uid)
{
    uint32_t status = 0;

    //15) 0	16	write_buff	PSA_PS_FLAG_NONE
    status = psa_ps_set(0, TEST_BUFF_SIZE, write_buff, PSA_PS_FLAG_NONE);

    //remove
    status = psa_ps_remove(0);

    //15) null	16	write_buff	PSA_PS_FLAG_NONE
    status = psa_ps_set(NULL, TEST_BUFF_SIZE, write_buff, PSA_PS_FLAG_NONE);

    //remove
    status = psa_ps_remove(NULL);

    //15) 6	514	buff[514]	PSA_PS_FLAG_NONE
    status = psa_ps_set(p_uid, 514, buff1, PSA_PS_FLAG_NONE);

    //remove
    status = psa_ps_remove(p_uid);

    //15) 6	-2	write_buff	PSA_PS_FLAG_NONE, crash
    status = psa_ps_set(p_uid, -2, write_buff, PSA_PS_FLAG_NONE);

    //remove
    status = psa_ps_remove(p_uid);

    //19) 6	2	null	PSA_PS_FLAG_NONE, crash
    status = psa_ps_set(p_uid, 2, NULL, PSA_PS_FLAG_NONE);

    //remove
    status = psa_ps_remove(p_uid);

    return PSA_PS_SUCCESS;
}

/***********************************************************************/
//get_info

static int32_t sst_calls_get_info_call(psa_ps_uid_t p_uid)
{
    uint32_t status = 0;

    //25) set
    status = psa_ps_set(p_uid, 16, write_buff, 0);

    //26) 0 info, crash
    status = psa_ps_get_info(0, &info);

    //27) null info, crash
    status = psa_ps_get_info(NULL, &info);

    //28) 8 info, crash
    status = psa_ps_get_info(8, NULL);

    //29) -2 info, crash
    status = psa_ps_get_info(-2, &info);

    //30) 6 NULL, VAL_PS_GET_INFO, PSA_PS_ERROR_INVALID_ARGUMENT
    status = psa_ps_get_info(p_uid, NULL);

    //remove
    status = psa_ps_remove(p_uid);

    return PSA_PS_SUCCESS;
}

/************************************************************************************/
//removo
static int32_t sst_calls_remove_call(psa_ps_uid_t p_uid)
{
    uint32_t status;

    //set
    status = psa_ps_set(p_uid, 16, write_buff, 0);

    //uid = 0
    status = psa_ps_remove(0);

    //uid = NULL
    status = psa_ps_remove(NULL);

    return PSA_PS_SUCCESS;
}

int32_t psa_sst_uid_parameter_fuzz(security_t caller)
{
    int32_t test_status;
    psa_ps_uid_t uid = UID_BASE_VALUE + 6;
    test_i = 1;

    //get
    test_status = sst_calls_get_call(uid);
    if(test_status != PSA_PS_SUCCESS)
        return test_status;
    //set
    test_status = sst_calls_set_call(uid);
    if(test_status != PSA_PS_SUCCESS)
        return test_status;

    //get_info
    test_status = sst_calls_get_info_call(uid);
    if(test_status != PSA_PS_SUCCESS)
        return test_status;

    //remove
    test_status = sst_calls_remove_call(uid);
    if(test_status != PSA_PS_SUCCESS)
        return test_status;

    return PSA_PS_SUCCESS;
}


#endif


/**
 * \brief This symbol is the entry point provided by the PSA API compliance
 *        test libraries
 */
extern void val_entry(void);

__attribute__((noreturn))
void psa_api_test(void *arg)
{
    psa_ps_status_t status;

    UNUSED_VARIABLE(arg);


    printf("My ps test ....\r\n");
    /* Set with no data and no flags and a valid UID */
    status = psa_sst_uid_parameter_fuzz(NONSECURE);
    if(status != PSA_PS_SUCCESS) {
        printf("Set should not fail with valid UID");
    }
    else
    {
        printf("ps set ok!\r\n");
    }



#ifdef TFM_NS_CLIENT_IDENTIFICATION
    tfm_nspm_register_client_id();
#endif /* TFM_NS_CLIENT_IDENTIFICATION */

    //val_entry();

    for (;;) {
    }
}
