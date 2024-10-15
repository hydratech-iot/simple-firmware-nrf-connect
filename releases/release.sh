FIRMWARE_VERSION=v1.0.1

BUILD_PATH=../build/customer/zephyr/
BUILD_FIMRWARE_PATH=$BUILD_PATH/nrf_sample.hex

FINAL_FIMRWARE_PATH=$FIRMWARE_VERSION/customer_firmware_$FIRMWARE_VERSION.hex

rm -R $FIRMWARE_VERSION
mkdir $FIRMWARE_VERSION

cp $BUILD_FIMRWARE_PATH $FINAL_FIMRWARE_PATH