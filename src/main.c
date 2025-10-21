/*
 * File Name: system_manager.h
 *
 * Author: Thuan Le
 *
 * Description: Main
 *
 */

/* Includes ----------------------------------------------------------- */
#include "base_type.h"
#include "bsp_io.h"
#include "bsp_spi.h"
#include "system_manager.h"

/* Private defines ---------------------------------------------------- */
#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
LOG_MODULE_REGISTER(main_log);

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
#define SYSTEM_MANAGER_THREAD_STACK_SIZE (8192)

/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
K_THREAD_STACK_DEFINE(system_manager_stack, SYSTEM_MANAGER_THREAD_STACK_SIZE);
static struct k_thread system_manager_thread;
static k_tid_t system_manager_thread_id;

/* Private function prototypes ---------------------------------------- */
static void system_manager_task(void *p1, void *p2, void *p3);

/* Function definitions ----------------------------------------------- */
int main(void)
{
    LOG_INF("=================================================================");
    LOG_INF("=================== BEGIN OF APPLICATION  =======================");

    system_manager_init();

    // Create the thread
    system_manager_thread_id = k_thread_create(&system_manager_thread,                      //!< Pointer to initialize thread
                                               system_manager_stack,                        //!< Pointer to stack space
                                               K_THREAD_STACK_SIZEOF(system_manager_stack), //!< Stack size in bytes
                                               system_manager_task,                         //!< Entry thread function
                                               NULL,                                        //!< 1st entry point parameter
                                               NULL,                                        //!< 2nd entry point parameter
                                               NULL,                                        //!< 3rd entry point parameter
                                               K_PRIO_COOP(4),                              //!< Thread priority
                                               0,                                           //!< Thread option
                                               K_NO_WAIT);                                  //!< Scheduling delay (in milliseconds)

    k_thread_name_set(&system_manager_thread, "system_manager");

    return 0;
}

uint8_t tx_data[1] = {0xAB};

static void system_manager_task(void *p1, void *p2, void *p3)
{
    while (1)
    {
        bsp_io_write_specific_pin(IO_PIN_SPI_CS, 0); // CS low
        bsp_spi_2_transmit(tx_data, sizeof(tx_data));
        bsp_io_write_specific_pin(IO_PIN_SPI_CS, 1); // CS high
    }
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