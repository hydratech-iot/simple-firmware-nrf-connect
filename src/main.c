/*
 * File Name: system_manager.h
 *
 * Author: Thuan Le (thuanle@hydratech-iot.com)
 *
 * Description: Main
 *
 * Copyright 2024, HydraTech. All rights reserved.
 * You may use this file only in accordance with the license, terms, conditions,
 * disclaimers, and limitations in the end user license agreement accompanying
 * the software package with which this file was provided.
 */

/* Includes ----------------------------------------------------------- */
#include "base_type.h"
#include "system_manager.h"

/* Private defines ---------------------------------------------------- */
#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
LOG_MODULE_REGISTER(main_log);

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
int main(void)
{
    LOG_INF("=================================================================");
    LOG_INF("=================== BEGIN OF APPLICATION  =======================");

    system_manager_init();

    return 0;
}

/* Private function --------------------------------------------------- */
void assert_failed(char *file, uint32_t line)
{
    char err_msg_long[200] = "@";
    char *err_msg;
    uint32_t err_msg_length;
    uint32_t fn_length;

    err_msg = err_msg_long;
    err_msg_length = sizeof(err_msg_long);

    fn_length = (err_msg_length - 20); // ", 1234567"

    // Make error string ------------------------------------ {
    if (strlen((char *)file) > fn_length)
    {
        file += ((uint32_t)strlen((char *)file) - fn_length);
        strcat(err_msg, "...");
    }
    sprintf(err_msg + strlen(err_msg), "%s, line: %u", file, line);
    err_msg[err_msg_length - 1] = 0;
    // ------------------------------------------------------ }
    LOG_ERR("This is an error file %s", err_msg);
}

/* End of file -------------------------------------------------------- */