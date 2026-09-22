#include "ble_telemetry.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if IS_ENABLED(CONFIG_BT)
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/settings/settings.h>

/* Custom Service 128-bit UUID definition */
#define BT_UUID_PM100_VAL \
    BT_UUID_128_ENCODE(0xe20a1a00, 0x473b, 0x4444, 0x9f6d, 0xbe083a8bd92d)
static struct bt_uuid_128 pm100_svc_uuid = BT_UUID_INIT_128(BT_UUID_PM100_VAL);

/* Custom Characteristics 128-bit UUIDs */
#define BT_UUID_PWM_VAL \
    BT_UUID_128_ENCODE(0xe20a1a03, 0x473b, 0x4444, 0x9f6d, 0xbe083a8bd92d)
static struct bt_uuid_128 pwm_chrc_uuid = BT_UUID_INIT_128(BT_UUID_PWM_VAL);

#define BT_UUID_TEAM_NAME_VAL \
    BT_UUID_128_ENCODE(0xe20a1a04, 0x473b, 0x4444, 0x9f6d, 0xbe083a8bd92d)
static struct bt_uuid_128 team_name_chrc_uuid = BT_UUID_INIT_128(BT_UUID_TEAM_NAME_VAL);

#define BT_UUID_TEAM_NUM_VAL \
    BT_UUID_128_ENCODE(0xe20a1a05, 0x473b, 0x4444, 0x9f6d, 0xbe083a8bd92d)
static struct bt_uuid_128 team_num_chrc_uuid = BT_UUID_INIT_128(BT_UUID_TEAM_NUM_VAL);

#define BT_UUID_WHITE_BLINK_VAL \
    BT_UUID_128_ENCODE(0xe20a1a06, 0x473b, 0x4444, 0x9f6d, 0xbe083a8bd92d)
static struct bt_uuid_128 white_blink_chrc_uuid = BT_UUID_INIT_128(BT_UUID_WHITE_BLINK_VAL);

#define BT_UUID_POWER_VAL \
    BT_UUID_128_ENCODE(0xe20a1a08, 0x473b, 0x4444, 0x9f6d, 0xbe083a8bd92d)
static struct bt_uuid_128 power_chrc_uuid = BT_UUID_INIT_128(BT_UUID_POWER_VAL);

#define BT_UUID_CTRL_STATE_VAL \
    BT_UUID_128_ENCODE(0xe20a1a07, 0x473b, 0x4444, 0x9f6d, 0xbe083a8bd92d)
static struct bt_uuid_128 ctrl_state_chrc_uuid = BT_UUID_INIT_128(BT_UUID_CTRL_STATE_VAL);

/* Thread-safe local BLE data caches */
static uint16_t g_ble_voltage = 0;   // In Millivolts (mV)
static uint16_t g_ble_current = 0;   // In Milliamperes (mA)
static uint32_t g_ble_energy = 0;    // In Joules (J)
static uint16_t g_ble_pwms[3] = {1000, 1000, 1000}; // Array of [PWM_in, PWM_out, PWM_ctrl] in us
static uint32_t g_ble_power = 0;     // Peak power since boot, in Milliwatts (mW)
static uint8_t g_ble_control_state = 0; // enum ctrl_state

/* Dynamic Device Name Buffer */
static char g_dynamic_name[45] = "PM100-PowerLimiter";

/* Notification Subscription Config flags */
static bool g_notify_voltage = false;
static bool g_notify_current = false;
static bool g_notify_energy = false;
static bool g_notify_uptime = false;
static bool g_notify_pwms = false;
static bool g_notify_power = false;
static bool g_notify_control_state = false;

/* GATT Attribute Read Callbacks (Protected with Encryption Permission) */

static ssize_t read_voltage(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			    void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &g_ble_voltage, sizeof(g_ble_voltage));
}

static ssize_t read_current(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			    void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &g_ble_current, sizeof(g_ble_current));
}

static ssize_t read_energy(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			   void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &g_ble_energy, sizeof(g_ble_energy));
}

static ssize_t read_uptime(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			   void *buf, uint16_t len, uint16_t offset)
{
    uint32_t uptime_ms = (uint32_t)k_uptime_get();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &uptime_ms, sizeof(uptime_ms));
}

static ssize_t read_pwms(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, g_ble_pwms, sizeof(g_ble_pwms));
}

static ssize_t read_power(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			  void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &g_ble_power, sizeof(g_ble_power));
}

static ssize_t read_control_state(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				  void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &g_ble_control_state, sizeof(g_ble_control_state));
}

static ssize_t read_team_name(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			      void *buf, uint16_t len, uint16_t offset)
{
    char local_name[32];
    k_mutex_lock(&g_config_mutex, K_FOREVER);
    strncpy(local_name, g_config.team_name, sizeof(local_name) - 1);
    local_name[sizeof(local_name) - 1] = '\0';
    k_mutex_unlock(&g_config_mutex);

    return bt_gatt_attr_read(conn, attr, buf, len, offset, local_name, strlen(local_name));
}

static ssize_t read_team_number(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				void *buf, uint16_t len, uint16_t offset)
{
    uint32_t number;
    k_mutex_lock(&g_config_mutex, K_FOREVER);
    number = g_config.team_number;
    k_mutex_unlock(&g_config_mutex);

    return bt_gatt_attr_read(conn, attr, buf, len, offset, &number, sizeof(number));
}

/* GATT Attribute Write Callback for LED White Blink Trigger */
static ssize_t write_white_blink(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				 const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    if (offset != 0 || len != 1) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    uint8_t val = ((const uint8_t *)buf)[0];
    if (val != 0) {
        g_blink_start_time = k_uptime_get();
        LOG_INF("BLE White Blink Command received. Overriding LED to BLINK state for 5 seconds.");
    }

    return len;
}

/* CCC Descriptor Configuration Change Callbacks */

static void voltage_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    g_notify_voltage = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("BLE Voltage notification state changed: %s", g_notify_voltage ? "enabled" : "disabled");
}

static void current_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    g_notify_current = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("BLE Current notification state changed: %s", g_notify_current ? "enabled" : "disabled");
}

static void energy_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    g_notify_energy = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("BLE Energy notification state changed: %s", g_notify_energy ? "enabled" : "disabled");
}

static void uptime_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    g_notify_uptime = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("BLE Uptime notification state changed: %s", g_notify_uptime ? "enabled" : "disabled");
}

static void pwm_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    g_notify_pwms = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("BLE PWM Signals notification state changed: %s", g_notify_pwms ? "enabled" : "disabled");
}

static void power_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    g_notify_power = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("BLE Power notification state changed: %s", g_notify_power ? "enabled" : "disabled");
}

static void control_state_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    g_notify_control_state = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("BLE Control State notification state changed: %s", g_notify_control_state ? "enabled" : "disabled");
}

/* Custom PM100 GATT Service Declaration - All characteristics require Encryption */
BT_GATT_SERVICE_DEFINE(pm100_svc,
    BT_GATT_PRIMARY_SERVICE(&pm100_svc_uuid),

    /* 1. Voltage Characteristic (Standard 16-bit UUID 0x2B18, ENCRYPT permissions) */
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_16(0x2B18),
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT, read_voltage, NULL, NULL),
    BT_GATT_CCC(voltage_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    /* 2. Current Characteristic (Standard 16-bit UUID 0x2B17, ENCRYPT permissions) */
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_16(0x2B17),
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT, read_current, NULL, NULL),
    BT_GATT_CCC(current_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    /* 3. Energy Characteristic (Standard 16-bit UUID 0x2B06, ENCRYPT permissions) */
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_16(0x2B06),
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT, read_energy, NULL, NULL),
    BT_GATT_CCC(energy_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    /* 4. Uptime Characteristic (Standard 16-bit UUID 0x2A2B, ENCRYPT permissions) */
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_16(0x2A2B),
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT, read_uptime, NULL, NULL),
    BT_GATT_CCC(uptime_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    /* 5. PWM Signals Characteristic (Custom 128-bit UUID, ENCRYPT permissions) */
    BT_GATT_CHARACTERISTIC(&pwm_chrc_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT, read_pwms, NULL, NULL),
    BT_GATT_CCC(pwm_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    /* 6. Read-Only Team Name Characteristic (Custom 128-bit UUID, ENCRYPT permissions) */
    BT_GATT_CHARACTERISTIC(&team_name_chrc_uuid.uuid,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ_ENCRYPT, read_team_name, NULL, NULL),

    /* 7. Read-Only Team Number Characteristic (Custom 128-bit UUID, ENCRYPT permissions) */
    BT_GATT_CHARACTERISTIC(&team_num_chrc_uuid.uuid,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ_ENCRYPT, read_team_number, NULL, NULL),

    /* 8. Write-Only White Blink Command (Custom 128-bit UUID, ENCRYPT permissions) */
    BT_GATT_CHARACTERISTIC(&white_blink_chrc_uuid.uuid,
        BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_WRITE_ENCRYPT, NULL, write_white_blink, NULL),

    /* 9. Peak Power Characteristic (Custom 128-bit UUID, ENCRYPT permissions) */
    BT_GATT_CHARACTERISTIC(&power_chrc_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT, read_power, NULL, NULL),
    BT_GATT_CCC(power_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    /* 10. Control State Characteristic (Custom 128-bit UUID, ENCRYPT permissions) */
    BT_GATT_CHARACTERISTIC(&ctrl_state_chrc_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT, read_control_state, NULL, NULL),
    BT_GATT_CCC(control_state_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* Security Passkey Callback / Authentication Interface */
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
    LOG_INF("BLE Security Passkey pairing initiated. Enter passcode display: %04u", passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
    LOG_INF("BLE Security Pairing cancelled by peer.");
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
    LOG_INF("BLE Pairing complete. Bonded: %s", bonded ? "yes" : "no");
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
    LOG_ERR("BLE Pairing failed: reason %u", (unsigned int)reason);
}

static struct bt_conn_auth_cb auth_cb = {
    .passkey_display = auth_passkey_display,
    .cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb auth_info_cb = {
    .pairing_complete = pairing_complete,
    .pairing_failed = pairing_failed,
};

/* BLE Connection Callbacks */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("BLE Connection failed: error %u", err);
    } else {
        LOG_INF("BLE Client Connected successfully.");
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("BLE Client Disconnected (reason %u).", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

/* BLE Dynamic Advertising Data */
static struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, g_dynamic_name, 18), // Starts with default name, updated dynamically
};

static void bt_ready(int err)
{
    if (err) {
        LOG_ERR("Bluetooth initialization failed: error %d", err);
        return;
    }

    LOG_INF("Bluetooth Stack initialized successfully.");

    /* 1. Register security authorization callback */
    err = bt_conn_auth_cb_register(&auth_cb);
    if (err) {
        LOG_ERR("Failed to register BLE security callbacks: error %d", err);
    }

    err = bt_conn_auth_info_cb_register(&auth_info_cb);
    if (err) {
        LOG_ERR("Failed to register BLE security info callbacks: error %d", err);
    }

    /* 2. Configure active pairing password (passkey) loaded from NVS PIN_code */
    unsigned int passkey_pin = 1234; // Safe fallback
    k_mutex_lock(&g_config_mutex, K_FOREVER);
    passkey_pin = (unsigned int)strtoul(g_config.PIN_code, NULL, 10);
    k_mutex_unlock(&g_config_mutex);

    err = bt_passkey_set(passkey_pin);
    if (err) {
        LOG_ERR("Failed to set BLE pairing passkey: error %d", err);
    } else {
        LOG_INF("BLE Security PIN set successfully to '%04u'. Link Encryption active.", passkey_pin);
    }

    /* 3. Load BLE settings from persistent storage (restores bonds/identities) */
    if (IS_ENABLED(CONFIG_SETTINGS)) {
        err = settings_load();
        if (err) {
            LOG_ERR("Failed to load BLE settings: error %d", err);
        } else {
            LOG_INF("BLE settings loaded successfully (bonding keys restored).");
        }
    }

    /* 4. Compose dynamic device name: "<team_name>_<team_number>_<MAC_last_4>" */
    bt_addr_le_t addrs[1];
    size_t count = 1;
    bt_id_get(addrs, &count);
    uint8_t *mac = addrs[0].a.val;

    k_mutex_lock(&g_config_mutex, K_FOREVER);
    snprintf(g_dynamic_name, sizeof(g_dynamic_name), "%s_%u_%02X%02X",
             g_config.team_name, g_config.team_number, mac[1], mac[0]);
    k_mutex_unlock(&g_config_mutex);

    LOG_INF("BLE compiled dynamic device name: '%s'", g_dynamic_name);

    /* Update dynamic name reference and len in advertising struct */
    ad[1].data = (uint8_t *)g_dynamic_name;
    ad[1].data_len = strlen(g_dynamic_name);

    /* 5. Begin Conn advertising */
    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Failed to start BLE advertising: error %d", err);
        return;
    }

    LOG_INF("BLE Secure Advertising started successfully as '%s'.", g_dynamic_name);
}

int ble_telemetry_init(void)
{
    LOG_INF("Initializing Bluetooth Subsystem...");

    int err = bt_enable(bt_ready);
    if (err) {
        LOG_ERR("bt_enable returned error %d", err);
        return err;
    }

    return 0;
}

void ble_telemetry_update(const struct system_telemetry *telem)
{
    if (!telem) {
        return;
    }

    /* Update internal representations with SI scaling and safe integers */
    g_ble_voltage = (uint16_t)(telem->voltage_v * 1000.0f); // V -> mV
    g_ble_current = (uint16_t)(telem->current_a * 1000.0f); // A -> mA
    g_ble_energy = (uint32_t)telem->total_consumption_j;   // J -> raw J
    g_ble_pwms[0] = (uint16_t)telem->pwm_input_us;
    g_ble_pwms[1] = (uint16_t)telem->pwm_output_us;
    g_ble_pwms[2] = (uint16_t)telem->pwm_control_us;
    g_ble_power = (uint32_t)(telem->power_w * 1000.0f);   // Peak W -> mW
    g_ble_control_state = (uint8_t)telem->state;

    /* Issue notifications dynamically to paired peers if subscribed */
    if (g_notify_voltage) {
        bt_gatt_notify(NULL, &pm100_svc.attrs[2], &g_ble_voltage, sizeof(g_ble_voltage));
    }
    if (g_notify_current) {
        bt_gatt_notify(NULL, &pm100_svc.attrs[5], &g_ble_current, sizeof(g_ble_current));
    }
    if (g_notify_energy) {
        bt_gatt_notify(NULL, &pm100_svc.attrs[8], &g_ble_energy, sizeof(g_ble_energy));
    }
    if (g_notify_uptime) {
        uint32_t uptime_ms = (uint32_t)telem->time_ms;
        bt_gatt_notify(NULL, &pm100_svc.attrs[11], &uptime_ms, sizeof(uptime_ms));
    }
    if (g_notify_pwms) {
        bt_gatt_notify(NULL, &pm100_svc.attrs[14], g_ble_pwms, sizeof(g_ble_pwms));
    }
    if (g_notify_power) {
        bt_gatt_notify(NULL, &pm100_svc.attrs[23], &g_ble_power, sizeof(g_ble_power));
    }
    if (g_notify_control_state) {
        bt_gatt_notify(NULL, &pm100_svc.attrs[26], &g_ble_control_state, sizeof(g_ble_control_state));
    }
}

#else /* IS_ENABLED(CONFIG_BT) == false */

/* Compile-ready stub fallback implementation when BLE is turned off in configs */

int ble_telemetry_init(void)
{
    LOG_WRN("Bluetooth BLE not enabled in build (CONFIG_BT=n). BLE service disabled.");
    return 0;
}

void ble_telemetry_update(const struct system_telemetry *telem)
{
    // No-op stub
}

#endif
