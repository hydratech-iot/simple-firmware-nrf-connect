/*
 * File Name: sensor_manager.c
 *
 * Author: Thuan Le (thuanle@hydratech-iot.com)
 *
 * Description: Sensor Manager
 *
 * Copyright 2024, HydraTech. All rights reserved.
 * You may use this file only in accordance with the license, terms, conditions,
 * disclaimers, and limitations in the end user license agreement accompanying
 * the software package with which this file was provided.
 */

/* Public includes ---------------------------------------------------------- */
#include "sensor_manager.h"
#include "protocol.h"
#include "ble_peripheral.h"
#include "heart_rate_analyzer.h"
#include "spo2_analyzer.h"
#include "bsp_afe.h"
#include "bsp_imu.h"
#include "base_board_defs.h"
#include "bsp_timer.h"
#include "bsp_bm.h"
#include "sqi_cal.h"
#include "spo2.h"
#include "ppg_data.h"
#include "system_manager.h"

/* Private includes --------------------------------------------------------- */
LOG_MODULE_REGISTER(sensor_manager, CONFIG_LOG_DEFAULT_LEVEL);

/* Private defines ---------------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------------- */
/* Private macros ----------------------------------------------------------- */

/* Public variables --------------------------------------------------------- */
/* Private variables -------------------------------------------------------- */
volatile bool afe_data_ready = false;

static uint16_t afe_sample_cnt;
static uint16_t afe_sample_success_cnt;
static uint16_t afe_sample_error_cnt;

static uint16_t ble_msg_index = 0;

static packet_ppg_data_t ppg_pkt = {
    .header.cmd       = PACKET_CMD_PPG_STREAMING_DATA,
    .header.msg_index = 0,
    .header.len       = 0,
    .data             = {0},
};

static packet_imu_data_t imu_pkt = {
    .header.cmd       = PACKET_CMD_IMU_STREAMING_DATA,
    .header.msg_index = 0,
    .header.len       = 0,
    .data             = {0},
};

static packet_temperature_data_t temperature_pkt = {
    .header.cmd       = PACKET_CMD_TEMPERATURE_DATA,
    .header.msg_index = 0,
    .header.len       = 0,
    .value            = {0},
};

static packet_ppg_phys_data_t ppg_phys_pkt = {
    .header.cmd       = PACKET_CMD_PPG_PHYSICAL_DATA,
    .header.msg_index = 0,
    .header.len       = 0,
    .spo2             = 0,
    .heart_rate       = 0,
};

static bool sensor_manager_enable = false;

static uint8_t heart_rate = 0;
static measured_values_t measured_value;
float resample_signal[LEN_SIG_128HZ];

#if (0)
20240816_184944_ppg.csv
static int32_t ppg_green_test_data[] = {326080, 326369, 326640, 326883, 326905, 327049, 327373, 327398, 327526, 327619, 327823, 328041, 328337, 328484, 328876, 329157, 329365, 329686, 331759, 330320, 330683, 330869, 331319, 331496, 331866, 332094, 332330, 332688, 332815, 333062, 333573, 333569, 334048, 334371, 334583, 335054, 335329, 335539, 335798, 336152, 336449, 336502, 336810, 337139, 337253, 337499, 337658, 337762, 337772, 337362, 338144, 336451, 335160, 333888, 332496, 330917, 329322, 327848, 326558, 325366, 324567, 325447, 323110, 324355, 322536, 322333, 322498, 322185, 322149, 322178, 322147, 322122, 322091, 322154, 322203, 322497, 322836, 323266, 325371, 324297, 324701, 325131, 325317, 325606, 325835, 325988, 326009, 326021, 326232, 327830, 326366, 326584, 326666, 326887, 326937, 327353, 327531, 327812, 328062, 328298, 328492, 328869, 329037, 329482, 329790, 330171, 330462, 330752, 330975, 331244, 331787, 333641, 332921, 332765, 332796, 333028, 334944, 333504, 335177, 333937, 334170, 334408, 334685, 334895, 335038, 335962, 335435, 337403, 335870, 335904, 335854, 335700, 335315, 334571, 333688, 332470, 330994, 329372, 327898, 326446, 324924, 323722, 322586, 321566, 321084, 320381, 321470, 319596, 319198, 318866, 318567, 318520, 318185, 319696, 318019, 317882, 317907, 318077, 318147, 318453, 318885, 320527, 319547, 319915, 320160, 320653, 320839, 321138, 321125, 321510, 321667, 323454, 323549, 323663, 322071, 322235, 322357, 323615, 322841, 323072, 323868, 323631, 324090, 324542, 324953, 325263, 325680, 326033, 326434, 326936, 327701, 327978, 327898, 329818, 328529, 328756, 329106, 329452, 331301, 329936, 330445, 330779, 331144, 333148, 331792, 332065, 332420, 332700, 332904, 333239, 333533, 333762, 334070, 335757, 334465, 334376, 334185, 333676, 332805, 331705, 330395, 328849, 328939, 326237, 324278, 322985, 323288, 320903, 320058, 319380, 318719, 318363, 318061, 317721, 317393, 317251, 317435, 316913, 316806, 317376, 316744, 318424, 316961, 317224, 317612, 318033, 318484, 318905, 319200, 319741, 320135, 320471, 320721, 320840, 321163, 321282, 321413, 321667, 322409, 321944, 322143, 323864, 322636, 322927, 323117, 323371, 323746, 324071, 324407, 324775, 325233, 325711, 326077, 326540, 326991, 327359, 327835, 328267, 330204, 329111, 329473, 329682, 330120, 332390, 330649, 330926, 331183, 331454, 331795, 332054, 332474, 332760, 334820, 333459, 333703, 334053, 334358, 334719, 334879, 335121, 335397, 335402, 336893, 334915, 334164, 333320, 332105, 330614, 329463, 327498, 325890, 324506, 323389, 322288, 321430, 322341, 320377, 320038, 320194, 319530, 319400, 319261, 320279, 319135, 319037, 318891, 318878, 318890, 319052, 319190, 319354, 319718, 320078, 321880, 320794, 321010, 322855, 321459, 321683, 321820, 322047, 322011, 322049, 322091, 322223, 322313, 322471, 322587, 322879, 323124, 323229, 323665, 323894, 324230, 324559, 324830, 325105, 325524, 327242, 327785, 326362, 326670, 327005, 327309, 327619, 327910, 327976, 328382, 328644, 328860, 329103, 329364, 329607, 329834, 330072, 330352, 332055, 330667, 330672, 330818, 331092, 331134, 331480, 331747, 332004, 332226, 332460, 333993, 332268, 331745, 332473, 329759, 328101, 326472, 324883, 322893, 321376, 319768, 318500, 317207, 316026, 314985, 314164, 313381, 314595, 313929, 311983, 312104, 311789, 312090, 311970, 312077, 312157, 312179, 314000, 312712, 314469, 313231, 313628, 314037, 314094, 315571, 315086, 315295, 315464, 317411, 317383, 316087, 316158, 316189, 316473, 316732, 317071, 317210, 317756, 317812, 318149, 319974, 318542, 318891, 319168, 319495, 319593, 320004, 320282, 320629, 322508, 321208, 321635, 322017, 322328, 322720, 322962, 323229, 323500, 323758, 324019, 324498, 324600, 324698, 326600, 325220, 325497, 325769, 327917, 326290, 326581, 326772, 327043, 327257, 327496, 327388, 329384, 327194, 326681, 325651, 324685, 323199, 321565, 319753, 317963, 316240, 314736, 313256, 312102, 312196, 310293, 309755, 309505, 308986, 308866, 310095, 308573, 308526, 308512, 308557, 308511, 308842, 309100, 309337, 309917, 310593, 311365, 312062, 312985, 313804, 315819, 315310, 315946, 316330, 316655, 316943, 317168, 317269, 318949, 317793, 318033, 318198, 318367, 318650, 319028, 320651, 319419, 319621, 319933, 321560, 320379, 320574, 322311, 321083, 321287, 321624, 321826, 322145, 322496, 322761, 323118, 323474, 323769, 324109, 324392, 324745, 326509, 325014, 325352, 327015, 326973, 327693, 326207, 326330, 326602, 326796, 327101, 327220, 327427, 327582, 327873, 328188, 328410, 328390, 328623, 328636, 330119, 327881, 326975, 325732, 324097, 323663, 320301, 318160, 316391, 314586, 312878, 311355, 309945, 308696, 307738, 307001, 306296, 305861, 305435, 305274, 305213, 304964, 304855, 304953, 304839, 304784, 304880, 304994, 306016, 307146, 305939, 306346, 306887, 307207, 307476, 308007, 308283, 308496, 308728, 308894, 309026, 309106, 309129, 309192, 309137, 309247, 309267, 309307, 309560, 309445, 309872, 310096, 310295, 311042, 310810, 311143, 311413, 311809, 311988, 312487, 312853, 313193, 313649, 314052, 314229, 314779, 316511, 316933, 315825, 316148, 316603, 316947, 317285, 317641, 318048, 318320, 318708, 318995, 320796, 319796, 320020, 320376, 320656, 320998, 321373, 321612, 321901, 322043, 322135, 322089, 321761, 320927, 320154, 318910, 317426, 315765, 313975, 313692, 310697, 309318, 308048, 308473, 306114, 305315, 304745, 304292, 303955, 305617, 303599, 303558, 303941, 303652, 303645, 303700, 303751, 303957, 304051, 304326, 304584, 304957, 305381, 305742, 306146, 306493, 308243, 307228, 307319, 307675, 307869, 308037, 308150, 308301, 308191, 308563, 308755, 308928, 309200, 309470, 309847, 310104, 310381, 310756, 311085, 311487, 311635, 312228, 312628, 313035, 313288, 313885, 314248, 314611, 314973, 315379, 315696, 315937, 316177, 316547, 316888, 318628, 317307, 317533, 317701, 317911, 318186, 318425, 318638, 318901, 319276, 319521, 319816, 320167, 320494, 320754, 321041, 321097, 321115, 320928, 322064, 319467, 320093, 316879, 317149, 314030, 312495, 311017, 309807, 308689, 309114, 308348, 306139, 305702, 305216, 305189, 304542, 304595, 304659, 304709, 304819, 304866, 304957, 305038, 305262, 305529, 305855, 306288, 306469, 308618, 307497, 307851, 308241, 308507, 308767, 309094, 309259, 309492, 309729, 309804, 309985, 311631, 310257, 310480, 310623, 310840, 311136, 311376, 311653, 311938, 312321, 312659, 313095, 313371, 313692, 315775, 314410, 314857, 315269, 315489, 316032, 316439, 316769, 317201, 317605, 317977, 318332, 318686, 319015, 319137, 319616, 319888, 320233, 320483, 320793, 321164, 321498, 321661, 321917, 322637, 322497, 322707, 322962, 322961, 324586, 322913, 322567, 321856, 320613, 319462, 318109, 316503, 314849, 313328, 311795, 310541, 309501, 308735, 307788, 307086, 306638, 306338, 305936, 306019, 305918, 305880, 305742, 305874, 307310, 306034, 306239, 306478, 306640, 307128, 307473, 307780, 308426, 308852, 309176, 309763, 309915, 310373, 310602, 310860, 311120, 311237, 311451, 311803, 313309, 312224, 312429, 312734, 313125, 313419, 313612, 314221, 314490, 314690, 315173, 316962, 315851, 316267, 316669, 316991, 317320, 317845, 318230, 318759, 319118, 319550, 319922, 320273, 320594, 320960, 321219, 323224, 321900, 322183, 323962, 322730, 323134, 323488, 323818, 324181, 324533, 324822, 326577, 327005, 325787, 326120, 326390, 326633, 326699, 326569, 326036, 325249, 324352, 323063, 321591, 320108, 318647, 317221, 315872, 314808, 313897, 313059, 312526, 311974, 311700, 311325, 311009, 312478, 310937, 310827, 311435, 311133, 311261, 311350, 311589, 311913, 313739, 312762, 313106, 313629, 315366, 314348, 314933, 315267, 315337, 315831, 315966, 316381, 316513, 316703, 316872, 317021, 317229, 319056, 319200, 317871, 318165, 318435, 319557, 319213, 319460, 321390, 320014, 320473, 320812, 321233, 321754, 322124, 322523, 323086, 325144, 323997, 324472, 324671, 325292, 325687, 326064, 326353, 326736, 327107, 327471, 327720, 328082, 329692, 328600, 328888, 330670, 329464, 331389, 330011, 331935, 330537, 332250, 331031, 331220, 331384, 331457, 332967, 331135, 330334, 329516, 328410, 328689, 325494, 323951, 322434, 321161, 319785, 318883, 318005, 317255, 316728, 316252, 315929, 317400, 315430, 315293, 315304, 315400, 315132, 315455, 315549, 315745, 317479, 316246, 317342, 316967, 317266, 317784, 318241, 318804, 321248, 319663, 320026, 321825, 320628, 320959, 322697, 321295, 321524, 321648, 321903, 322175, 322449, 322710, 324913, 323427, 323861, 324287, 326290, 325104, 325528, 325915, 326404, 326909, 327272, 327661, 329248, 328482, 328811, 329227, 329470, 329766, 330171, 330326, 330569, 330910, 331168, 332949, 331648, 331880, 332076, 332224, 332507, 332640, 332782, 332833, 333117, 333360, 335036, 333625, 334817, 333512, 333267, 332713, 331949, 330854, 329527, 328193, 326781, 326842, 324002, 322818, 321940, 321177, 320711, 320340, 320752, 320015, 320036, 320153, 320276, 322210, 320602, 320801, 321132, 321337, 321665, 323487, 322446, 322805, 323339, 323919, 324350, 324744, 325166, 325516, 326579, 327538, 326418, 326542, 326566, 328420, 326974, 327056, 327105, 327277, 327437, 327645, 327909, 328196, 328461, 330061, 329083, 329449, 329763, 330089, 330506, 330857, 331110, 331437, 331845, 332169, 332466, 332760, 333028, 333188, 333459, 335242, 333924, 334127, 334434, 335667, 334980, 336871, 335611, 335816, 336367, 336552, 336836, 337169, 337514, 337818, 337929, 338051, 337812, 337309, 336620, 335570, 334272, 332928, 331317, 329887, 328346, 327077, 326736, 325228, 324640, 324127, 323738, 323659, 323449, 323524, 323448, 323428, 323695, 323835, 325327, 324030, 324176, 324338, 324571, 324880, 325213, 325544, 325920, 326337, 326660, 326960, 327137, 327379, 327483, 327550, 327707, 327647, 327726, 327781, 327751, 328076, 328078, 328277, 328483, 328679, 328852, 329097, 329308, 331468, 329903, 332019, 330568, 330877, 331208, 331475, 331910, 333776, 332632, 332943, 333266, 333553, 333950, 334053, 334892, 334641, 334873, 334982, 335331, 335603, 335894, 336158, 336491, 336755, 336702, 337197, 337320, 337430, 337391, 337295, 338588, 336342, 335453, 334418, 332888, 331339, 329592, 327965, 326234, 324741, 323341, 321943, 323034, 320419, 319668, 319154, 318602, 318191, 317838, 317621, 317340, 317243, 316994, 317027, 316965, 316994, 317123, 319218, 317603};
#endif

/* Private prototypes ------------------------------------------------------- */
static void sensor_manager_temperature_task(void);

static void sensor_manager_afe_interrupt_callback(uint8_t pin);

static base_status_t sensor_manager_process_afe_data(device_mode_t mode);

static int create_a_signed_number(int number, int bit);

/* Public implementations --------------------------------------------------- */
void sensor_manager_init(void)
{
    sensor_manager_afe_init();

    bsp_imu_init();

    bsp_temp_init();
}

void sensor_manager_afe_init(void)
{
    bsp_afe_init(sensor_manager_afe_interrupt_callback);
    bsp_afe_pll_enable(true);
    sensor_manager_start();
}

void sensor_manager_task(device_mode_t mode)
{
    ppg_phys_data_t ppg_phys;
    sensor_manager_afe_task(mode, &ppg_phys);

    imu_phys_data_t imu_phys;
    sensor_manager_imu_task(mode, &imu_phys);

    sensor_manager_temperature_task();
}

void sensor_manager_start(void)
{
    sensor_manager_enable = true;
    LOG_INF("Start collect data");
    bsp_afe_ppg_enable(AFE_PPG_CHANNEL_1, true);
}

void sensor_manager_stop(void)
{
    sensor_manager_enable = false;
    LOG_INF("Stop collect data");
    bsp_afe_ppg_enable(AFE_PPG_CHANNEL_1, false);
}

/* Private implementations -------------------------------------------------- */
base_status_t sensor_manager_afe_task(device_mode_t mode, ppg_phys_data_t *ppg_phys)
{
    static int32_t afe_data_buf[256] = {0};
    base_status_t ret = BS_ERROR;

    if (afe_data_ready)
    {
        afe_data_ready = false;

        bsp_afe_read_data(afe_data_buf, &afe_sample_cnt);

        memcpy(ppg_pkt.data, afe_data_buf, sizeof (afe_data_buf));

#if (CONFIG_DEBUG_LOG_ENABLE)
        LOG_INF("Sample count: %d", afe_sample_cnt);
#endif // CONFIG_DEBUG_LOG_ENABLE

        if (afe_sample_cnt == AFE_FIFO_THRESHOLD)
        {
            afe_sample_success_cnt++;

#if (CONFIG_DEBUG_LOG_ENABLE)
            LOG_INF("AFE get sample success: %d, failed: %d \r\n", afe_sample_success_cnt, afe_sample_error_cnt);
            NRF_LOG_PROCESS();
#endif // CONFIG_DEBUG_LOG_ENABLE
        }
        else
        {
            afe_sample_error_cnt++;

#if (CONFIG_DEBUG_LOG_ENABLE)
            LOG_INF("AFE get sample failed: %d, FIFO count: %d \r\n", afe_sample_error_cnt, afe_sample_cnt);
            NRF_LOG_PROCESS();
#endif // CONFIG_DEBUG_LOG_ENABLE
        }

        ret = sensor_manager_process_afe_data(mode);

        if (mode == DEVICE_MODE_STREAMING)
        {
            // Send data to BLE
            ppg_pkt.header.msg_index = ble_msg_index++;
            ppg_pkt.header.len       = afe_sample_cnt * 4;
            ble_peripheral_send_data((uint8_t *)&ppg_pkt, ppg_pkt.header.len + sizeof(packet_hdr_t));

            if (ret == BS_OK)
            {
                ppg_phys_pkt.header.len = sizeof(packet_ppg_phys_data_t) - sizeof(packet_hdr_t);
                ppg_phys_pkt.heart_rate = heart_rate;
                // ppg_phys_pkt.spo2       = measured_value.spo2;
                ble_peripheral_send_data((uint8_t *)&ppg_phys_pkt, sizeof(packet_ppg_phys_data_t));

                return BS_OK;
            }
        }
        else
        {
            if (ret == BS_OK)
            {
                // ppg_phys->spo2 = measured_value.spo2;
                ppg_phys->heart_rate = heart_rate;

                return BS_OK;
            }
        }
    }

    return BS_ERROR;
}

base_status_t sensor_manager_imu_task(device_mode_t mode, imu_phys_data_t *imu_phys)
{
    imu_pkt.header.msg_index = ble_msg_index++;
    uint16_t fifo_frames;
    imu_data_t imu_data[100];
    uint16_t size_of_raw_accel_and_gyro = sizeof(int16_t) * 6;

    if (bsp_imu_get_data(imu_data, &fifo_frames) == BS_OK)
    {
        if (mode == DEVICE_MODE_STREAMING)
        {
            for (uint16_t idx = 0; idx < fifo_frames; idx++)
            {
                // Copy raw accel and raw gyro
                memcpy(&imu_pkt.data[idx * 6], &imu_data[idx], size_of_raw_accel_and_gyro);
            }

            imu_pkt.header.len = size_of_raw_accel_and_gyro * fifo_frames;
            ble_peripheral_send_data((uint8_t *)&imu_pkt, imu_pkt.header.len + sizeof(packet_hdr_t));
        }
        else
        {
            // TODO: Process the IMU data
        }

        return BS_OK;
    }

    return BS_ERROR;
}

static void sensor_manager_temperature_task(void)
{
    static uint32_t tick_start = 0;
    static sm_temperature_state_t temp_state = SM_TEMPERATURE_STATE_IDLE;

    switch (temp_state)
    {
    case SM_TEMPERATURE_STATE_IDLE:
    {
        if (bsp_tmr_get_tick_ms() - tick_start >= 5000)
        {
            tick_start = bsp_tmr_get_tick_ms();
            temp_state = SM_TEMPERATURE_STATE_TRIGGER_MEASURE;
        }
        break;
    }

    case SM_TEMPERATURE_STATE_TRIGGER_MEASURE:
    {
        for (uint8_t i = 0; i < TEMPERATURE_MAX; i++)
        {
            bsp_temp_single_shot_set(i);
        }

        temp_state = SM_TEMPERATURE_STATE_MEASURE;
        break;
    }

    case SM_TEMPERATURE_STATE_MEASURE:
    {
        float temp;

        if (bsp_tmr_get_tick_ms() - tick_start >= 50)
        {
            tick_start = bsp_tmr_get_tick_ms();

            for (uint8_t i = 0; i < TEMPERATURE_MAX; i++)
            {
                bsp_temp_celsius_get(i, &temp);
                temperature_pkt.value[i] = temp;

                LOG_INF("Temperature %d equal %f", i, temp);
            }

            temperature_pkt.header.len = sizeof(packet_temperature_data_t) - sizeof(packet_hdr_t);
            ble_peripheral_send_data((uint8_t *)&temperature_pkt, sizeof(packet_temperature_data_t));

            temp_state = SM_TEMPERATURE_STATE_IDLE;
        }
        break;
    }

    default:
        break;
    }
}

static void sensor_manager_afe_interrupt_callback(uint8_t pin)
{
    afe_data_ready = true;
}

static base_status_t sensor_manager_process_afe_data(device_mode_t mode)
{
    float sample;
    uint32_t one_sample;
    afe_ppg_led_t channel;
    bool is_heart_rate_updated = false;
    static int32_t ppg_green[256] = {0};
    uint16_t ppg_green_index = 0;

    for (uint16_t i = 0; i < afe_sample_cnt; i++)
    {
        // Get the FIFO type
        uint16_t fifo_type = ppg_pkt.data[i] >> 20;

        // Remove the Flag
        one_sample = ppg_pkt.data[i] & 0x000FFFFF;
        one_sample = create_a_signed_number(one_sample, 20);
        sample = (float)one_sample / ADC_GAIN;

        if (fifo_type == MAX86176_FIFO_TAG_PPG_MEAS_1)
        {
            channel = PPG_LED_CHANNEL_RED;
            ppg_data_process(sample, channel, resample_signal, &measured_value);
        }
        else if (fifo_type == MAX86176_FIFO_TAG_PPG_MEAS_2)
        {
            channel = PPG_LED_CHANNEL_IR;
            ppg_data_process(sample, channel, resample_signal, &measured_value);
        }
        else if (fifo_type == MAX86176_FIFO_TAG_PPG_MEAS_3)
        {
            channel = PPG_LED_CHANNEL_GREEN;
            ppg_green[ppg_green_index] = one_sample;
            ppg_green_index++;
        }
        else
        {
            return BS_ERROR;
        }
    }

    is_heart_rate_updated = heart_rate_analyzer_runner(ppg_green, ppg_green_index, &heart_rate);
    ppg_green_index = 0;

    if (heart_rate == 0)
    {
        measured_value.spo2 = 0;
    }

    if (is_heart_rate_updated && (measured_value.sqi_red_percent_for_spo2 > THR_PERCENT_SQI))
    {
        LOG_INF("Heart rate: %u (bpm)", heart_rate);
        LOG_INF("Spo2: %f ", measured_value.spo2);

        g_device.ppg_phys_prv.heart_rate = heart_rate;
        g_device.ppg_phys_prv.spo2       = measured_value.spo2;
        return BS_OK;
    }

    if (mode == DEVICE_MODE_STREAMING)
    {
        if (is_heart_rate_updated && (measured_value.sqi_red_percent_for_spo2 < THR_PERCENT_SQI))
        {
            heart_rate = 0;
            measured_value.spo2 = 0;
            return BS_OK;
        }
    }

    return BS_ERROR;
}

static int create_a_signed_number(int number, int bit)
{
  // Maximum positive value for a signed number
  int max_value = (1 << (bit - 1)) - 1;

  // Converting it to a decimal value
  int decimal_value = (number <= max_value) ? number : number - (1 << bit);

  return decimal_value;
}

/* End of file -------------------------------------------------------------- */