/*
 * File Name: bsp_io.h
 *
 * Author: Thuan Le
 *
 * Description:
 *
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_io.h"

/* Private defines ---------------------------------------------------- */
#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
LOG_MODULE_REGISTER(bsp_io);

#define IO_INFO(_pin, _io_dt) [_pin] = {    \
        .pin   = _pin,                      \
        .io_dt = _io_dt                     \
}

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
    io_pin_t pin;
    struct gpio_dt_spec io_dt;
}
io_info_t;

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static const io_info_t IO_TABLE[IO_PIN_MAX] =
{
    //        +=============================+=======================+
    //        | IO PIN                      | Device tree           |
    //        |                             |                       |
    //        +-----------------------------+-----------------------+
     IO_INFO ( IO_PIN_BUTTON                , IO_BUTTON             )
    ,IO_INFO ( IO_PIN_LED                   , IO_LED                )
    //        +=============================+=======================+
};

/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
void bsp_io_init(void)
{
    // Input setting
    bsp_io_set_config_pin(&IO_TABLE[IO_PIN_BUTTON].io_dt, GPIO_INPUT);

    // Output setting
    bsp_io_set_config_pin(&IO_TABLE[IO_PIN_LED].io_dt, GPIO_OUTPUT);

    // Turn off the LED
    bsp_io_write_specific_pin(IO_PIN_LED, 0);
}

void bsp_io_write_specific_pin(io_pin_t pin, bool level)
{
    bsp_io_write_pin(&IO_TABLE[pin].io_dt, level);
}

bool bsp_io_read_specific_pin(io_pin_t pin)
{
    return (bool)bsp_io_read_pin(&IO_TABLE[pin].io_dt);
}

base_status_t bsp_io_attach_interrupt(struct gpio_dt_spec gpio_dev, struct gpio_callback *gpio_callback_data, gpio_callback_handler_t gpio_callback_handler, gpio_flags_t flags)
{
  if (!device_is_ready(gpio_dev.port))
  {
    return BS_ERROR;
  }

  if (bsp_io_set_config_pin(&gpio_dev, GPIO_INPUT) != 0)
  {
    return BS_ERROR;
  }

  if (gpio_pin_interrupt_configure_dt(&gpio_dev, flags) != 0)
  {
    return BS_ERROR;
  }

  gpio_init_callback(gpio_callback_data, gpio_callback_handler, BIT(gpio_dev.pin));
  gpio_add_callback(gpio_dev.port, gpio_callback_data);

  return BS_OK;
}

/* End of file -------------------------------------------------------- */
