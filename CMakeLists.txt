cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(Snitch)

target_sources(app PRIVATE src/main.c)

zephyr_library_sources(
                        # BSP
                        src/hw/bsp/bsp_io.c
                        
                        # BLE
                        src/ble/ble_peripheral.c

                        # Application
                        src/app/ble_manager/ble_manager.c
                        src/app/network_manager/network_manager.c
                        src/app/system_manager/system_manager.c
                        src/app/user_interface_manager/user_interface_manager_button.c
)

# Include directories
zephyr_library_include_directories(.)
zephyr_library_include_directories(./src)

# BSP
zephyr_library_include_directories(./src/hw/bsp)

# BLE
zephyr_library_include_directories(./src/ble)

# Application
zephyr_library_include_directories(./src/app/ble_manager)
zephyr_library_include_directories(./src/app/network_manager)
zephyr_library_include_directories(./src/app/system_manager)
zephyr_library_include_directories(./src/app/user_interface_manager)

# Common
zephyr_library_include_directories(./src/common)