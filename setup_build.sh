#! /bin/bash
################################################################################
# Environment (DO NOT MODIFY)                                                  #
################################################################################
CWD=$(pwd)
################################################################################

TARGET_NAME="customer"
BOARD="nrf52840dk_nrf52840"
CONF_FILE_PATH="config/prj.conf"
CMAKE_OPTION="\
-DHW_VERSION:STRING=1.0 \
"

# Create build config (DO NOT MODIFY)
FULL_CONF_DIR=${CWD}'/build/'${TARGET_NAME}
FULL_CONF_FILE=${CWD}'/'${CONF_FILE_PATH}
BOARD_ROOT=${CWD}'/board_config/'${TARGET_NAME}

echo -e '\n---------------------------------------------------------------------------'
echo -e 'Generating build config: '${TARGET_NAME}'\n'
rm -rf ${FULL_CONF_DIR}
cmake '-B'${FULL_CONF_DIR} '-S'${CWD} -GNinja -DBOARD=${BOARD} -DBOARD_ROOT:STRING=${BOARD_ROOT} -DCONF_FILE:STRING=${FULL_CONF_FILE} ${CMAKE_OPTION}
################################################################################