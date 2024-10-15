cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(salus)

target_sources(app PRIVATE src/main.c)

zephyr_library_sources(
                        # Drivers
                        src/hw/components/driver/as6221/as6221.c
                        src/hw/components/driver/bmi323/bmi323.c
                        src/hw/components/driver/bmi323/bmi3.c
                        src/hw/components/driver/bmi323/common/common.c
                        src/hw/components/driver/max17048/max17048.c
                        src/hw/components/driver/max86176/max86176.c
                        src/hw/components/driver/one_button/one_button.c
                        src/hw/components/driver/pca9540/pca9540.c
                        src/hw/components/driver/ssd1306/ssd1306.c
                        src/hw/components/driver/stts22h/stts22h.c
                        src/hw/components/driver/stwlc38/stwlc38.c

                        # BSP
                        src/hw/bsp/bsp_afe.c
                        src/hw/bsp/bsp_bm.c
                        src/hw/bsp/bsp_i2c_mul.c
                        src/hw/bsp/bsp_i2c.c
                        src/hw/bsp/bsp_imu.c
                        src/hw/bsp/bsp_io.c
                        src/hw/bsp/bsp_led.c
                        src/hw/bsp/bsp_nvs.c
                        src/hw/bsp/bsp_oled.c
                        src/hw/bsp/bsp_power.c
                        src/hw/bsp/bsp_pwm.c
                        src/hw/bsp/bsp_spi.c
                        src/hw/bsp/bsp_temp.c
                        src/hw/bsp/bsp_timer.c
                        src/hw/bsp/bsp_wireless_charger.c
                        
                        # Middlewares
                        src/middlewares/font/font.c

                        # Algorithms
                        src/algorithms/madgwick/madgwick_ahrs.c
                        src/algorithms/ppg/heart_rate_analyzer.c

                        # BLE
                        src/ble/ble_peripheral.c

                        # Application
                        src/app/ble_manager/ble_manager.c
                        src/app/configuration_manager/configuration_manager.c
                        src/app/network_manager/network_manager.c
                        src/app/power_manager/power_manager.c
                        src/app/sensor_manager/sensor_manager.c
                        src/app/system_manager/system_manager.c
                        src/app/user_interface_manager/user_interface_manager_led.c
                        src/app/user_interface_manager/user_interface_manager_button.c
                        src/app/user_interface_manager/user_interface_manager_oled.c
)

# Include directories
zephyr_library_include_directories(.)
zephyr_library_include_directories(./src)

# Drivers
zephyr_library_include_directories(./src/hw/components/driver/as6221)
zephyr_library_include_directories(./src/hw/components/driver/bmi323)
zephyr_library_include_directories(./src/hw/components/driver/bmi323/common)
zephyr_library_include_directories(./src/hw/components/driver/max17048)
zephyr_library_include_directories(./src/hw/components/driver/max86176)
zephyr_library_include_directories(./src/hw/components/driver/one_button)
zephyr_library_include_directories(./src/hw/components/driver/pca9540)
zephyr_library_include_directories(./src/hw/components/driver/ssd1306)
zephyr_library_include_directories(./src/hw/components/driver/stts22h)
zephyr_library_include_directories(./src/hw/components/driver/stwlc38)

# BSP
zephyr_library_include_directories(./src/hw/bsp)

# Middlewares
zephyr_library_include_directories(./src/middlewares/font)
zephyr_library_include_directories(./src/middlewares/media/header)

# Algorithms
zephyr_library_include_directories(./src/algorithms/madgwick)
zephyr_library_include_directories(./src/algorithms/ppg)
zephyr_library_include_directories(./src/algorithms/ppg/include)

# BLE
zephyr_library_include_directories(./src/ble)

# Application
zephyr_library_include_directories(./src/app/ble_manager)
zephyr_library_include_directories(./src/app/configuration_manager)
zephyr_library_include_directories(./src/app/network_manager)
zephyr_library_include_directories(./src/app/power_manager)
zephyr_library_include_directories(./src/app/sensor_manager)
zephyr_library_include_directories(./src/app/system_manager)
zephyr_library_include_directories(./src/app/user_interface_manager)

# Common
zephyr_library_include_directories(./src/common)

# Create a wrapper CMake library that our app can link with
add_library(mylib_lib STATIC IMPORTED GLOBAL)

set(MYLIB_LIB_DIR     ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithms/ppg)
set_target_properties(mylib_lib PROPERTIES IMPORTED_LOCATION     ${MYLIB_LIB_DIR}/libbiolib.a)

target_link_libraries(app PUBLIC mylib_lib)
