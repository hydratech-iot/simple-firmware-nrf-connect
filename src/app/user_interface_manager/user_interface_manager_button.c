/*
 * File Name: user_interface_manager_button.c
 *
 * Author: Thuan Le
 *
 * Description: Button Manager
 *
 */

/* Includes ----------------------------------------------------------------- */
#include "system_manager.h"
#include "user_interface_manager_button.h"
#include "base_board_defs.h"
#include "bsp_io.h"
#include "bsp_timer.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(user_interface_manager_button, CONFIG_LOG_DEFAULT_LEVEL);

#define BUTTON_MANAGER_THREAD_STACK_SIZE (2048)

#define BTN_DEBOUNCE_CLICK_MS           (50)
#define BTN_POWER_DEBOUNCE_HOLD_MAX_MS  (23000)
#define BTN_POWER_LONG_PRESSED_MS       (20000)

/* Private enumerate/structure ---------------------------------------------- */
/* Private Constants -------------------------------------------------------- */

/* Private variables -------------------------------------------------------- */
K_THREAD_STACK_DEFINE(button_manager_stack, BUTTON_MANAGER_THREAD_STACK_SIZE);
static struct k_thread button_manager_thread;
static k_tid_t button_manager_thread_id;

static struct gpio_dt_spec io_button = IO_BUTTON;
static struct gpio_callback io_button_cb_data;

/* Private prototypes ------------------------------------------------------- */
static void user_interface_manager_button_task(void *p1, void *p2, void *p3);
static void button_interrupt_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

K_SEM_DEFINE(btn_sem, 0, 1);

static button_event_cb_t btn_event_callback;

/* Private macros ----------------------------------------------------------- */
/* Public APIs -------------------------------------------------------------- */
void user_interface_manager_button_init(button_event_cb_t callback)
{
    btn_event_callback = callback;

    bsp_io_attach_interrupt(io_button, &io_button_cb_data, button_interrupt_callback, GPIO_INT_EDGE_FALLING);

    button_manager_thread_id = k_thread_create(&button_manager_thread,                      //!< Pointer to initialize thread
                                               button_manager_stack,                        //!< Pointer to stack space
                                               K_THREAD_STACK_SIZEOF(button_manager_stack), //!< Stack size in bytes
                                               user_interface_manager_button_task,          //!< Entry thread function
                                               NULL,                                        //!< 1st entry point parameter
                                               NULL,                                        //!< 2nd entry point parameter
                                               NULL,                                        //!< 3rd entry point parameter
                                               K_PRIO_COOP(4),                              //!< Thread priority
                                               0,                                           //!< Thread option
                                               K_NO_WAIT);                                  //!< Scheduling delay (in milliseconds)

    k_thread_name_set(&button_manager_thread, "user_interface_manager_button");
}

/* Private functions -------------------------------------------------------- */
static void user_interface_manager_button_task(void *p1, void *p2, void *p3)
{
    int64_t start_time = 0;

    while (1)
    {
        if (k_sem_take(&btn_sem, K_FOREVER) != 0)
        {
            continue;
        }

        start_time = k_uptime_get();
        k_sleep(K_MSEC(BTN_DEBOUNCE_CLICK_MS));

        while ((k_uptime_get() - start_time) < BTN_POWER_DEBOUNCE_HOLD_MAX_MS)
        {
            if (bsp_io_read_specific_pin(IO_PIN_BUTTON) == 0) // Button press
            {
                if (k_uptime_get() - start_time > BTN_POWER_LONG_PRESSED_MS)
                {
                    LOG_INF("Long press detected");
                    btn_event_callback(BTM_EVT_LONG_PRESS);
                    break;
                }
            }
            else if (bsp_io_read_specific_pin(IO_PIN_BUTTON) == 1) // Button release
            {
                btn_event_callback(BTN_EVT_CLICK);
                break;
            }
            else
            {
                // Nothing to do here
            }
        }
    }
}

static void button_interrupt_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_sem_give(&btn_sem);
}

/* End of file -------------------------------------------------------------- */
