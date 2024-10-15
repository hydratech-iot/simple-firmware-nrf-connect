/*
* File Name: bsp_imu.c
* 
* Author: Thuan Le (thuanle@hydratech-iot.com)
* 
* Description: Board Support Package for BMI323
* 
* Copyright 2024, HydraTech. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*/

/* Public includes ---------------------------------------------------------- */
#include "bsp_imu.h"
#include "bsp_i2c.h"
#include "common.h"
#include <math.h>

/* Private includes --------------------------------------------------------- */
/* Private defines ---------------------------------------------------------- */
LOG_MODULE_REGISTER(bsp_imu, CONFIG_LOG_DEFAULT_LEVEL);

#define BMI323_FIFO_RAW_DATA_BUFFER_SIZE  UINT16_C(2048)
#define BMI323_FIFO_RAW_DATA_USER_LENGTH  UINT16_C(2048)

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH                     (9.80665f)

/*! Setting a watermark level in FIFO */
/*!
 * Calculation:
 * fifo_watermark_level = 120, accel_frame_len = 6, gyro_frame_len = 6
 * fifo_accel_frame_count = ((120 * 2) / (6 + 6)) = 20 frames
 */
#define BMI2_FIFO_WATERMARK_LEVEL         (120)

/* Private enumerate/structure ---------------------------------------------- */
/* Private macros ----------------------------------------------------------- */
/* Public variables --------------------------------------------------------- */
/* Private variables -------------------------------------------------------- */
static struct bmi3_dev dev;

/* Initialize FIFO frame structure */
static struct bmi3_fifo_frame fifoframe = { 0 };

static uint16_t watermark = 0;

/* Array of accelerometer frames */
/* Calculation for frame count:
 * Total frame count = FIFO water-mark size(in bytes) / Total accel frames
 *                   = (240 / 6) = 20 frames
 */
static struct bmi3_fifo_sens_axes_data fifo_accel_data[20];

/* Array of gyroscope frames */
/* Calculation for frame count:
 * Total frame count = FIFO water-mark size(in bytes) / Total gyro frames
 *                   = (240 / 6) = 20 frames
 */
static struct bmi3_fifo_sens_axes_data fifo_gyro_data[20];

/* Number of accel, gyro and temperature frames to be extracted from FIFO
 * Calculation:
 * fifo_watermark_level = 240(in bytes), accel_frame_len = 6, gyro_frame_len = 6
 * fifo_frame_length = (240 / (6 + 6)) = 20 frames
 */
uint16_t fifo_frame_length = 20;

/* Number of bytes of FIFO data */
uint8_t fifo_data[BMI323_FIFO_RAW_DATA_BUFFER_SIZE] = {0};

/* Private prototypes ------------------------------------------------------- */
/*!
 *  @brief This internal API is used to set configurations for FIFO, accelerometer and gyroscope.
 *
 *  @param[in] dev       : Structure instance of bmi3_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_sensor_fifo_config(struct bmi3_dev *dev);

/*!
 *  @brief This internal function converts lsb to meter per second squared for 16 bit accelerometer for
 *  range 2G, 4G, 8G or 16G.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] g_range   : Gravity range.
 *  @param[in] bit_width : Resolution for accel.
 *
 *  @return Accel values in meter per second squared.
 */
static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width);

/*!
 *  @brief This function converts lsb to degree per second for 16 bit gyro at
 *  range 125, 250, 500, 1000 or 2000dps.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] dps       : Degree per second.
 *  @param[in] bit_width : Resolution for gyro.
 *
 *  @return Degree per second.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);

/* Public implementations --------------------------------------------------- */
base_status_t bsp_imu_init(void)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Function to select interface between SPI and I2C, according to that the device structure gets updated.
     * Interface reference is given as a parameter
     * For I2C : BMI3_I2C_INTF
     * For SPI : BMI3_SPI_INTF
     */
    rslt = bmi3_interface_init(&dev, BMI3_I2C_INTF);
    if (BMI3_OK != rslt)
    {
        LOG_INF("IMU (BMI323) init fail");
        return BS_ERROR;
    }

    LOG_INF("IMU (BMI323) init success");
    bmi3_error_codes_print_result("bmi3_interface_init", rslt);

    /* Initialize BMI323 */
    rslt = bmi323_init(&dev);
    bmi3_error_codes_print_result("bmi323_init", rslt);

    /* Set the accelerometer, gyroscope and FIFO configurations */
    rslt = set_sensor_fifo_config(&dev);
    bmi3_error_codes_print_result("set_sensor_fifo_config", rslt);

    /* Set the water-mark level */
    fifoframe.wm_lvl = BMI2_FIFO_WATERMARK_LEVEL;

    rslt = bmi323_set_fifo_wm(fifoframe.wm_lvl, &dev);
    bmi3_error_codes_print_result("bmi323_set_fifo_wm", rslt);

    rslt = bmi323_get_fifo_wm(&watermark, &dev);
    bmi3_error_codes_print_result("bmi323_get_fifo_wm", rslt);

    /* Update FIFO structure */
    /* Mapping the buffer to store the FIFO data. */
    fifoframe.data = fifo_data;

    /* Length of FIFO frame. */
    fifoframe.length = BMI323_FIFO_RAW_DATA_USER_LENGTH;

    return BS_OK;
}

base_status_t bsp_imu_get_data(imu_data_t *imu_data, uint16_t *fifo_frames)
{
    int8_t rslt;
    uint16_t int_status = 0;
    uint16_t fifo_length = 0;
    float x = 0, y = 0, z = 0;

    /* Read FIFO data on interrupt. */
    rslt = bmi323_get_int1_status(&int_status, &dev);
    bmi3_error_codes_print_result("bmi323_get_int1_status", rslt);

    /* To check the status of FIFO water-mark interrupt. */
    if ((rslt == BMI323_OK) && (int_status & BMI3_INT_STATUS_FWM))
    {
#if (CONFIG_DEBUG_LOG_ENABLE)
        LOG_INF("Water-mark interrupt occurred");
        LOG_INF("FIFO water-mark level is in word: %d", watermark);
#endif // CONFIG_DEBUG_LOG_ENABLE

        rslt = bmi323_get_fifo_length(&fifoframe.available_fifo_len, &dev);
        bmi3_error_codes_print_result("bmi323_get_fifo_length", rslt);

        /* Convert available fifo length from word to byte */
        fifo_length = (uint16_t)(fifoframe.available_fifo_len * 2);

        fifoframe.length = fifo_length + dev.dummy_byte;

#if (CONFIG_DEBUG_LOG_ENABLE)
        LOG_INF("FIFO length in words : %d", fifoframe.available_fifo_len);
        LOG_INF("FIFO data bytes available : %d ", fifo_length);
        LOG_INF("FIFO data bytes requested : %d ", fifoframe.length);
#endif // CONFIG_DEBUG_LOG_ENABLE

        /* Read FIFO data */
        rslt = bmi323_read_fifo_data(&fifoframe, &dev);
        bmi3_error_codes_print_result("bmi323_read_fifo_data", rslt);

        if (rslt == BMI323_OK)
        {

            /* Parse the FIFO data to extract accelerometer data from the FIFO buffer */
            (void)bmi323_extract_accel(fifo_accel_data, &fifoframe, &dev);

#if (CONFIG_DEBUG_LOG_ENABLE)
            LOG_INF("Requested accelerometer data frames before parsing: %d", fifo_frame_length);
            LOG_INF("Parsed accelerometer data frames: %d", fifoframe.avail_fifo_accel_frames);
            LOG_INF("Accel data in LSB units and Gravity data in m/s^2");
            LOG_INF("ACCEL_DATA_SET, Acc_Raw_X, Acc_Raw_Y, Acc_Raw_Z, Acc_ms2_X, Acc_ms2_Y, Acc_ms2_Z");
#endif // CONFIG_DEBUG_LOG_ENABLE

            /* Print the parsed accelerometer data from the FIFO buffer */
            for (uint16_t idx = 0; idx < fifoframe.avail_fifo_accel_frames; idx++)
            {
                /* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range. */
                x = lsb_to_mps2(fifo_accel_data[idx].x, 2, dev.resolution);
                y = lsb_to_mps2(fifo_accel_data[idx].y, 2, dev.resolution);
                z = lsb_to_mps2(fifo_accel_data[idx].z, 2, dev.resolution);

                imu_data[idx].accel_raw.x = fifo_accel_data[idx].x;
                imu_data[idx].accel_raw.y = fifo_accel_data[idx].y;
                imu_data[idx].accel_raw.z = fifo_accel_data[idx].z;

                imu_data[idx].accel_mps2.x = x;
                imu_data[idx].accel_mps2.y = y;
                imu_data[idx].accel_mps2.z = z;

#if (CONFIG_DEBUG_LOG_ENABLE)
                /* Print the data in m/s2. */
                LOG_INF("%d, %d, %d, %d, %4.2f, %4.2f, %4.2f",
                       idx,
                       fifo_accel_data[idx].x,
                       fifo_accel_data[idx].y,
                       fifo_accel_data[idx].z,
                       x,
                       y,
                       z);
#endif // CONFIG_DEBUG_LOG_ENABLE
            }

            /* Parse the FIFO data to extract gyroscope data from the FIFO buffer */
            (void)bmi323_extract_gyro(fifo_gyro_data, &fifoframe, &dev);

#if (CONFIG_DEBUG_LOG_ENABLE)
            LOG_INF("Requested gyro data frames before parsing: %d", fifo_frame_length);
            LOG_INF("Parsed gyroscope data frames: %d", fifoframe.avail_fifo_gyro_frames);
            LOG_INF("Gyro data in LSB units and degrees per second");
            LOG_INF("GYRO_DATA_SET, Gyr_Raw_X, Gyr_Raw_Y, Gyr_Raw_Z, Gyr_dps_X, Gyr_dps_Y, Gyr_dps_Z");
#endif // CONFIG_DEBUG_LOG_ENABLE

            /* Print the parsed gyroscope data from the FIFO buffer */
            for (uint16_t idx = 0; idx < fifoframe.avail_fifo_gyro_frames; idx++)
            {
                /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
                x = lsb_to_dps(fifo_gyro_data[idx].x, (float)2000, dev.resolution);
                y = lsb_to_dps(fifo_gyro_data[idx].y, (float)2000, dev.resolution);
                z = lsb_to_dps(fifo_gyro_data[idx].z, (float)2000, dev.resolution);

                imu_data[idx].gyro_raw.x = fifo_gyro_data[idx].x;
                imu_data[idx].gyro_raw.y = fifo_gyro_data[idx].y;
                imu_data[idx].gyro_raw.z = fifo_gyro_data[idx].z;

                imu_data[idx].gyro_dps.x = x;
                imu_data[idx].gyro_dps.y = y;
                imu_data[idx].gyro_dps.z = z;

#if (CONFIG_DEBUG_LOG_ENABLE)
                /* Print the data in dps. */
                LOG_INF("%d, %d, %d, %d, %4.2f, %4.2f, %4.2f",
                       idx,
                       fifo_gyro_data[idx].x,
                       fifo_gyro_data[idx].y,
                       fifo_gyro_data[idx].z,
                       x,
                       y,
                       z);
#endif // CONFIG_DEBUG_LOG_ENABLE
            }
        }

        *fifo_frames = fifoframe.avail_fifo_accel_frames;

        return BS_OK;
    }

    return BS_ERROR;
}

/* Private implementations -------------------------------------------------- */
/*!
 * @brief This internal API is used to set configurations for accelerometer, gyroscope and FIFO.
 */
static int8_t set_sensor_fifo_config(struct bmi3_dev *dev)
{
    /* Status of API are returned to this variable. */
    int8_t rslt;
    struct bmi3_map_int map_int = { 0 };
    struct bmi3_sens_config config[2];

    /* Array to define set FIFO flush */
    uint8_t data[2] = { BMI323_ENABLE, 0 };

    /* Configure the type of feature. */
    config[0].type = BMI323_ACCEL;
    config[1].type = BMI323_GYRO;

    /* NOTE: The user can change the following configuration parameters according to their requirement. */
    /* Accel configuration settings. */
    /* Output Data Rate. By default ODR is set as 50Hz for accelerometer. */
    config[0].cfg.acc.odr = BMI3_ACC_ODR_50HZ;

    /* The Accel bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR. */
    config[0].cfg.acc.bwp = BMI3_ACC_BW_ODR_QUARTER;

    /* Set number of average samples for accel. */
    config[0].cfg.acc.avg_num = BMI3_ACC_AVG64;

    /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
    config[0].cfg.acc.range = BMI3_ACC_RANGE_2G;

    /* To enable the accelerometer set the power mode to normal mode */
    config[0].cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;

    /* Gyro configuration settings. */
    /* Output data Rate. Default ODR is 50Hz, setting to 50Hz. */
    config[1].cfg.gyr.odr = BMI3_GYR_ODR_50HZ;

    /*  The Gyroscope bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR
     *  Value   Name      Description
     *    0   odr_half   BW = gyr_odr/2
     *    1  odr_quarter BW = gyr_odr/4
     */
    config[1].cfg.gyr.bwp = BMI3_GYR_BW_ODR_HALF;

    /* Value    Name    Description
     *  000     avg_1   No averaging; pass sample without filtering
     *  001     avg_2   Averaging of 2 samples
     *  010     avg_4   Averaging of 4 samples
     *  011     avg_8   Averaging of 8 samples
     *  100     avg_16  Averaging of 16 samples
     *  101     avg_32  Averaging of 32 samples
     *  110     avg_64  Averaging of 64 samples
     */
    config[1].cfg.gyr.avg_num = BMI3_GYR_AVG4;

    /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps. */
    config[1].cfg.gyr.range = BMI3_GYR_RANGE_125DPS;

    /* To enable the gyroscope set the power mode to normal mode */
    config[1].cfg.gyr.gyr_mode = BMI3_GYR_MODE_NORMAL;

    /* Set new configurations */
    rslt = bmi323_set_sensor_config(config, 2, dev);
    bmi3_error_codes_print_result("bmi323_set_sensor_config", rslt);

    /* To enable the accelerometer, gyroscope in FIFO conf addr */
    rslt = bmi323_set_fifo_config(BMI3_FIFO_ACC_EN | BMI3_FIFO_GYR_EN, BMI323_ENABLE, dev);
    bmi3_error_codes_print_result("bmi323_set_fifo_config", rslt);

    /* Set the FIFO flush in FIFO control register to clear the FIFO data */
    rslt = bmi323_set_regs(BMI3_REG_FIFO_CTRL, data, 2, dev);
    bmi3_error_codes_print_result("bmi323_set_regs", rslt);

    /* Map the FIFO water-mark interrupt to INT1 */
    /* Note: User can map the interrupt to INT1 or INT2 */
    map_int.fifo_watermark_int = BMI3_INT1;

    /* Map the interrupt configuration */
    rslt = bmi323_map_interrupt(map_int, dev);
    bmi3_error_codes_print_result("bmi323_map_interrupt", rslt);

    return rslt;
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}

/* End of file -------------------------------------------------------------- */
