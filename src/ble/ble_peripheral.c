/*
 * File Name: ble_peripheral.c
 *
 * Author: Thuan Le
 *
 * Description:
 *
 */

/* Includes ----------------------------------------------------------- */
#include "ble_peripheral.h"
#include "base_include.h"
#include "network_manager.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(ble_peripheral, CONFIG_LOG_DEFAULT_LEVEL);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private Constants -------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, NULL, 0),
};

static struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

static struct bt_conn *current_conn;

static ble_receive_cb_t ble_receive_callback;
static ble_connection_status_cb_t ble_connection_status_callback;

/* Private function prototypes ---------------------------------------- */
static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
                          uint16_t len)
{
    char addr[BT_ADDR_LE_STR_LEN] = {0};

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

    ble_receive_callback((uint8_t *)data, len);

    LOG_INF("Received data from: %s", addr);
}

static struct bt_nus_cb nus_cb = {
    .received = bt_receive_cb,
};

static void exchange_func(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
    /**
     * @note Checks if the MTU exchange is successful.
     * If successful, logs a message, updates the connection pointer, and notifies the connected event.
     * Otherwise, disconnects the connection, logs an error message, and notifies the disconnection event.
     */
    if (!err)
    {
        LOG_INF("MTU exchange done");
    }
    else
    {
        bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        LOG_ERR("MTU exchange failed (err %" PRIu8 ")", err);
    }
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    if (err)
    {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Connected %s", addr);
    ble_connection_status_callback(BLE_CONNECTED);

    current_conn = bt_conn_ref(conn);

    if (err)
    {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }

    static struct bt_gatt_exchange_params exchange_params;
    exchange_params.func = exchange_func;
    err = bt_gatt_exchange_mtu(conn, &exchange_params);
    if (err)
    {
        LOG_ERR("MTU exchange failed (err %d)", err);
        bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Disconnected: %s (reason %u)", addr, reason);
    ble_connection_status_callback(BLE_DISCONNECTED);

    if (current_conn)
    {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static const char *ble_peripheral_device_info_get_device_name(void)
{
    static char ble_dev_name[128];
    char mac_add[20];

    // Get MAC address
    ble_peripheral_get_mac_address(mac_add);

    strcpy(ble_dev_name, DEVICE_NAME);
    strcat(ble_dev_name, mac_add);

    return ble_dev_name;
}

/* Function definitions ------------------------------------------------ */
void ble_peripheral_init(ble_receive_cb_t receive_cb, ble_connection_status_cb_t connection_status_cb)
{
    int err;

    err = bt_enable(NULL);
    if (err)
    {

        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }

    LOG_INF("Bluetooth initialized");
    ble_receive_callback = receive_cb;
    ble_connection_status_callback = connection_status_cb;

    if (IS_ENABLED(CONFIG_SETTINGS))
    {
        settings_load();
    }

    err = bt_nus_init(&nus_cb);
    if (err)
    {
        LOG_ERR("Failed to initialize UART service (err: %d)", err);
        return;
    }

    ad[1].data = ble_peripheral_device_info_get_device_name();
    ad[1].data_len = strlen(ad[1].data);
    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
                          ARRAY_SIZE(sd));
    if (err)
    {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }
}

void ble_peripheral_deinit(void)
{
    bt_disable();
}

void ble_peripheral_send_data(const uint8_t *data, uint16_t len)
{
    if (!current_conn)
    {
        return;
    }

    bt_nus_send(current_conn, data, len);
}

void ble_peripheral_get_mac_address(char *mac_addr)
{
    bt_addr_le_t mac = {0};
    size_t count = 1;

    bt_id_get(&mac, &count);

    sprintf(mac_addr, "%02X-%02X-%02X-%02X-%02X-%02X",
            mac.a.val[5], mac.a.val[4], mac.a.val[3], mac.a.val[2], mac.a.val[1], mac.a.val[0]);

    LOG_INF("Advertising as %s\n", mac_addr);
}

/* Private function definitions ---------------------------------------- */
/* Command Process Function -------------------------------------------- */
/* End of file --------------------------------------------------------- */
