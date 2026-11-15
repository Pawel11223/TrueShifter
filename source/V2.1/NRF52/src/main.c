#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/addr.h>

#include <string.h>

/* ==========================================================================
 * Version / hardware
 * ========================================================================== */

#define TRUESHIFTER_VERSION  "2.1.0"
#define TRUESHIFTER_HARDWARE "nRF52840"

/* ==========================================================================
 * UUIDs (same as in ESP32 project)
 * ========================================================================== */

#define BH_UUID_ENCODE(w32) \
	BT_UUID_128_ENCODE(w32, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e)

static struct bt_uuid_128 bh_svc_uuid = BT_UUID_INIT_128(BH_UUID_ENCODE(0x6e400001));
static struct bt_uuid_128 bh_char_motor_uuid = BT_UUID_INIT_128(BH_UUID_ENCODE(0x6e400002));
static struct bt_uuid_128 bh_char_serial_uuid = BT_UUID_INIT_128(BH_UUID_ENCODE(0x6e400003));
static struct bt_uuid_128 bh_char_config_uuid = BT_UUID_INIT_128(BH_UUID_ENCODE(0x6e400005));
static struct bt_uuid_128 bh_char_version_uuid = BT_UUID_INIT_128(BH_UUID_ENCODE(0x6e400007));
static struct bt_uuid_128 bh_char_battery_uuid = BT_UUID_INIT_128(BH_UUID_ENCODE(0x6e400008));
static struct bt_uuid_128 bh_char_stable_uuid = BT_UUID_INIT_128(BH_UUID_ENCODE(0x6e40000a));
static struct bt_uuid_128 bh_char_monitor_uuid = BT_UUID_INIT_128(BH_UUID_ENCODE(0x6e40000b));
static struct bt_uuid_128 bh_char_ath_uuid = BT_UUID_INIT_128(BH_UUID_ENCODE(0x6e40000c));
static struct bt_uuid_128 ts_char_config_uuid = BT_UUID_INIT_128(BH_UUID_ENCODE(0x6e4000fe));

/* ==========================================================================
 * bHaptics -> TrueGear mapping (40 motors in 4 groups of 10)
 * ========================================================================== */

static const uint8_t bh_to_tg_group[40] = {
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
	3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
	4, 4, 4, 4, 4, 4, 4, 4, 4, 4
};

static const uint16_t bh_to_tg_mask[40] = {
	0x8000, 0x0400, 0x4000, 0x0200, 0x2000, 0x0100, 0x1000, 0x0080, 0x0800, 0x0040,
	0x8000, 0x0400, 0x4000, 0x0200, 0x2000, 0x0100, 0x1000, 0x0080, 0x0800, 0x0040,
	0x8000, 0x0400, 0x4000, 0x0200, 0x2000, 0x0100, 0x1000, 0x0080, 0x0800, 0x0040,
	0x8000, 0x0400, 0x4000, 0x0200, 0x2000, 0x0100, 0x1000, 0x0080, 0x0800, 0x0040
};

/* ==========================================================================
 * Shared state + mutexes (equivalents of std::mutex from ESP32)
 * ========================================================================== */

static K_MUTEX_DEFINE(motor_mutex);
static K_MUTEX_DEFINE(state_mutex);
static K_MUTEX_DEFINE(config_mutex);

#define MOTOR_COUNT 40

static uint8_t current_motor_state[MOTOR_COUNT];
static uint8_t last_sent_motor_state[MOTOR_COUNT];
static bool has_new_data;

static uint32_t last_send_time;
static uint32_t last_status_check_time;

#define DEBOUNCE_DELAY_MS  50
#define STATUS_INTERVAL_MS 15000
#define SEND_MIN_INTERVAL  35
#define SEND_REFRESH_MS    120

static uint8_t bh_serial[10] = {
	0xcf, 0xcb, 0x0d, 0x95, 0x5f, 0xf6, 0xee, 0x2c, 0xbd, 0x73
};

static uint8_t bh_config_val[3] = { 100, 0, 0 };
static uint8_t bh_monitor_val;
static uint8_t bh_ath_val[20];
static uint16_t bh_version_val = 0x0502;
static uint16_t bh_battery_val = 50;

static uint8_t bas_level = 100;
static uint8_t vest_intensity = 50;

static bool is_connected_to_truegear;
static bool is_scanning;
static bool do_connect;
static bool tg_connecting;
static bt_addr_le_t truegear_addr;
static bool truegear_addr_valid;

static struct bt_conn *tg_conn;
static struct bt_conn *pc_conn;

static uint16_t tg_svc_start;
static uint16_t tg_svc_end;
static uint16_t tg_rx_handle;
static uint16_t tg_tx_handle;

/* ==========================================================================
 * Status LED (led0 devicetree alias)
 * ========================================================================== */

#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#define HAS_LED 1
#else
#define HAS_LED 0
#endif

/* ==========================================================================
 * Persistent configuration (Preferences/NVS -> Zephyr settings)
 * ========================================================================== */

static int ts_settings_set(const char *name, size_t len,
			   settings_read_cb read_cb, void *cb_arg)
{
	if (strcmp(name, "vest_int") != 0) {
		return -ENOENT;
	}

	if (len < 1) {
		return -EINVAL;
	}

	uint8_t val;
	int rc = read_cb(cb_arg, &val, sizeof(val));

	if (rc < 0) {
		return rc;
	}

	k_mutex_lock(&config_mutex, K_FOREVER);
	vest_intensity = (val > 100) ? 100 : val;
	k_mutex_unlock(&config_mutex);

	printk("[NVS] Vest Power: %u%%\n", vest_intensity);
	return 0;
}

static int ts_settings_get(const char *key, char *val, int val_len_max)
{
	if (strcmp(key, "vest_int") != 0) {
		return -ENOENT;
	}

	if (val_len_max < 1) {
		return -EINVAL;
	}

	k_mutex_lock(&config_mutex, K_FOREVER);
	val[0] = (char)vest_intensity;
	k_mutex_unlock(&config_mutex);

	return 1;
}

SETTINGS_STATIC_HANDLER_DEFINE(ts, "ts", ts_settings_get, ts_settings_set, NULL, NULL);

/* ==========================================================================
 * GATT helpers (value read/write)
 * ========================================================================== */

struct gatt_buf {
	void *data;
	uint16_t len;
};

static ssize_t read_buf(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	const struct gatt_buf *g = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, g->data, g->len);
}

static ssize_t write_buf(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	struct gatt_buf *g = attr->user_data;

	if (offset + len > g->len) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	memcpy((uint8_t *)g->data + offset, buf, len);
	return len;
}

static ssize_t read_str(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	const char *str = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, str, strlen(str));
}

static struct gatt_buf serial_buf = { bh_serial, sizeof(bh_serial) };
static struct gatt_buf config_buf = { bh_config_val, sizeof(bh_config_val) };
static struct gatt_buf ath_buf = { bh_ath_val, sizeof(bh_ath_val) };
static struct gatt_buf version_buf = { &bh_version_val, sizeof(bh_version_val) };
static struct gatt_buf battery_buf = { &bh_battery_val, sizeof(bh_battery_val) };
static struct gatt_buf monitor_buf = { &bh_monitor_val, sizeof(bh_monitor_val) };
static struct gatt_buf bas_buf = { &bas_level, sizeof(bas_level) };

static const char str_bhaptics[] = "bhaptics";
static const char str_model[] = "TactSuitX40";
static const char str_fw_rev[] = TRUESHIFTER_VERSION;
static const char str_hw_rev[] = TRUESHIFTER_HARDWARE;

/* ==========================================================================
 * Data path diagnostics (PC/Android -> TrueGear)
 * ========================================================================== */

static struct {
	uint32_t motor_writes;
	uint32_t motor_bytes;
	uint32_t decode_max;
	uint32_t send_attempts;
	uint32_t send_ok;
	uint32_t send_fail;
	uint32_t skip_no_tg;
	uint32_t skip_not_conn;
	uint32_t skip_state;
	uint16_t last_attr;
	uint16_t last_len;
	uint16_t last_off;
	uint8_t last_flags;
	uint8_t last_raw[8];
	const char *last_char;
} diag;

static void diag_dump(void)
{
	printk("[DIAG] writes=%u bytes=%u decmax=%u | send try=%u ok=%u fail=%u | "
	       "skip tg=%u conn=%u state=%u\n",
	       diag.motor_writes, diag.motor_bytes, diag.decode_max,
	       diag.send_attempts, diag.send_ok, diag.send_fail,
	       diag.skip_no_tg, diag.skip_not_conn, diag.skip_state);
	printk("[DIAG] last write: char=%s len=%u off=%u flags=0x%02x raw=",
	       diag.last_char ? diag.last_char : "?",
	       diag.last_len, diag.last_off, diag.last_flags);
	for (int i = 0; i < 8; i++) {
		printk("%02X ", diag.last_raw[i]);
	}
	printk("\n");
	printk("[DIAG] tg_conn=%p rx=0x%04x tx=0x%04x tg_connected=%d connecting=%d "
	       "pc_conn=%p vest=%u%%\n",
	       (void *)tg_conn, tg_rx_handle, tg_tx_handle,
	       (int)is_connected_to_truegear, (int)tg_connecting,
	       (void *)pc_conn, vest_intensity);
}

static void send_test_vibration(void);

/* ==========================================================================
 * Motor frame writer (equivalent of RxCallbacks::onWrite from ESP32)
 * ========================================================================== */

static uint8_t motor_raw[20];

static ssize_t bh_motor_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			      const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{

	if (flags & BT_GATT_WRITE_FLAG_PREPARE) {
		return 0;
	}

	if (offset >= sizeof(motor_raw)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if ((offset + len) > sizeof(motor_raw)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	memcpy(&motor_raw[offset], buf, len);
	uint16_t filled = offset + len;

	diag.motor_writes++;
	diag.motor_bytes += len;
	diag.last_attr = attr->handle;
	diag.last_len = len;
	diag.last_off = offset;
	diag.last_flags = flags;
	diag.last_char = attr->user_data ? (const char *)attr->user_data : "?";
	memcpy(diag.last_raw, buf, MIN(len, sizeof(diag.last_raw)));

	k_mutex_lock(&motor_mutex, K_FOREVER);
	memset(current_motor_state, 0, sizeof(current_motor_state));

	for (uint16_t i = 0; i < filled; i++) {
		uint8_t byte = motor_raw[i];
		uint8_t m1 = (byte >> 4) & 0x0F;
		uint8_t m2 = byte & 0x0F;

		current_motor_state[i * 2] = (m1 * 100) / 15;
		current_motor_state[i * 2 + 1] = (m2 * 100) / 15;
	}

	int mx = 0;

	for (int i = 0; i < MOTOR_COUNT; i++) {
		if (current_motor_state[i] > mx) {
			mx = current_motor_state[i];
		}
	}
	diag.decode_max = (uint32_t)mx;

	has_new_data = true;
	k_mutex_unlock(&motor_mutex);

	if (diag.motor_writes <= 3 || (diag.motor_writes % 50) == 0) {
		printk("[MOTOR] write #%u char=%s len=%u off=%u flags=0x%02x "
		       "filled=%u decmax=%d\n",
		       diag.motor_writes,
		       attr->user_data ? (const char *)attr->user_data : "?",
		       len, offset, flags, filled, mx);
	}

	return len;
}

/* ==========================================================================
 * Intensity writer (TS_CHAR_CONFIG) + NVS save
 * ========================================================================== */

static ssize_t read_ts_config(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			      void *buf, uint16_t len, uint16_t offset)
{
	uint8_t v;

	k_mutex_lock(&config_mutex, K_FOREVER);
	v = vest_intensity;
	k_mutex_unlock(&config_mutex);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &v, sizeof(v));
}

static ssize_t write_ts_config(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			       const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (len < 1U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint8_t new_intensity = ((const uint8_t *)buf)[0];

	if (new_intensity > 100) {
		new_intensity = 100;
	}

	k_mutex_lock(&config_mutex, K_FOREVER);
	vest_intensity = new_intensity;
	k_mutex_unlock(&config_mutex);

	settings_save_one("ts/vest_int", &new_intensity, sizeof(new_intensity));

	printk("[CONFIG] Vest Power Changed: %u%%\n", new_intensity);
	return len;
}

/* ==========================================================================
 * CCC callbacks (notify/indicate)
 * ========================================================================== */

static void bh_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	printk("[GATT] CCC changed: 0x%04x\n", value);
}

/* ==========================================================================
 * GATT services (peripheral - emulating TactSuitX40)
 * ========================================================================== */

BT_GATT_SERVICE_DEFINE(dis_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_DIS),

	BT_GATT_CHARACTERISTIC(BT_UUID_DIS_MANUFACTURER_NAME,
		BT_GATT_CHRC_READ, BT_GATT_PERM_READ,
		read_str, NULL, (void *)str_bhaptics),

	BT_GATT_CHARACTERISTIC(BT_UUID_DIS_MODEL_NUMBER,
		BT_GATT_CHRC_READ, BT_GATT_PERM_READ,
		read_str, NULL, (void *)str_model),

	BT_GATT_CHARACTERISTIC(BT_UUID_DIS_SERIAL_NUMBER,
		BT_GATT_CHRC_READ, BT_GATT_PERM_READ,
		read_buf, NULL, &serial_buf),

	BT_GATT_CHARACTERISTIC(BT_UUID_DIS_FIRMWARE_REVISION,
		BT_GATT_CHRC_READ, BT_GATT_PERM_READ,
		read_str, NULL, (void *)str_fw_rev),

	BT_GATT_CHARACTERISTIC(BT_UUID_DIS_HARDWARE_REVISION,
		BT_GATT_CHRC_READ, BT_GATT_PERM_READ,
		read_str, NULL, (void *)str_hw_rev),
);

BT_GATT_SERVICE_DEFINE(bas_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_BAS),

	BT_GATT_CHARACTERISTIC(BT_UUID_BAS_BATTERY_LEVEL,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ,
		read_buf, NULL, &bas_buf),
	BT_GATT_CCC(bh_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

BT_GATT_SERVICE_DEFINE(bh_svc,
	BT_GATT_PRIMARY_SERVICE(&bh_svc_uuid),

	BT_GATT_CHARACTERISTIC(&bh_char_motor_uuid.uuid,
		BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
		BT_GATT_PERM_WRITE, NULL, bh_motor_write, (void *)"MOTOR"),

	BT_GATT_CHARACTERISTIC(&bh_char_stable_uuid.uuid,
		BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
		BT_GATT_PERM_WRITE, NULL, bh_motor_write, (void *)"STABLE"),

	BT_GATT_CHARACTERISTIC(&bh_char_battery_uuid.uuid,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE_WITHOUT_RESP | BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
		read_buf, write_buf, &battery_buf),
	BT_GATT_CCC(bh_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(&bh_char_serial_uuid.uuid,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
		read_buf, write_buf, &serial_buf),

	BT_GATT_CHARACTERISTIC(&bh_char_version_uuid.uuid,
		BT_GATT_CHRC_READ, BT_GATT_PERM_READ,
		read_buf, NULL, &version_buf),

	BT_GATT_CHARACTERISTIC(&bh_char_config_uuid.uuid,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
		read_buf, write_buf, &config_buf),

	BT_GATT_CHARACTERISTIC(&ts_char_config_uuid.uuid,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
		read_ts_config, write_ts_config, NULL),

	BT_GATT_CHARACTERISTIC(&bh_char_monitor_uuid.uuid,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP |
		BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_INDICATE | BT_GATT_CHRC_BROADCAST,
		BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
		read_buf, write_buf, &monitor_buf),
	BT_GATT_CCC(bh_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(&bh_char_ath_uuid.uuid,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
		read_buf, write_buf, &ath_buf),
);

/* ==========================================================================
 * TrueGear client - discovery + subscription
 * ========================================================================== */

enum disc_state {
	DISC_IDLE,
	DISC_SERVICE,
	DISC_CHARS,
};

static enum disc_state disc_st = DISC_IDLE;
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_discover_params sub_disc_params;
static struct bt_gatt_subscribe_params subscribe_params;

static uint8_t tg_notify_func(struct bt_conn *conn,
			      struct bt_gatt_subscribe_params *params,
			      const void *data, uint16_t length)
{
	const uint8_t *pdata = data;

	if (!data) {
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}

	if (length == 20 && pdata[0] == 0x68 && pdata[1] == 0x68 && pdata[3] == 0x81) {
		int vest_voltage = (pdata[9] << 8) | pdata[10];
		int percentage;

		if (vest_voltage >= 4150) {
			percentage = 100;
		} else if (vest_voltage <= 3300) {
			percentage = 0;
		} else {
			percentage = (int)(((vest_voltage - 3300) * 100) / 850);
		}

		if (percentage > 100) {
			percentage = 100;
		}
		if (percentage < 0) {
			percentage = 0;
		}

		printk("[CLIENT] BATTERY: %d%% (%d mV)\n", percentage, vest_voltage);

		bh_battery_val = (uint16_t)percentage;
		bt_gatt_notify_uuid(NULL, &bh_char_battery_uuid.uuid, NULL,
				    &bh_battery_val, sizeof(bh_battery_val));
	}

	return BT_GATT_ITER_CONTINUE;
}

static void tg_subscribe_cb(struct bt_conn *conn, uint8_t err,
			    struct bt_gatt_subscribe_params *params)
{
	if (err) {
		printk("[CLIENT] Subscribe failed (err %u)\n", err);
		return;
	}

	printk("[CLIENT] Subskrypcja TX zarejestrowana.\n");

	k_mutex_lock(&state_mutex, K_FOREVER);
	is_connected_to_truegear = true;
	k_mutex_unlock(&state_mutex);

	last_status_check_time = k_uptime_get_32();

	send_test_vibration();
}

static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	int err;

	if (!attr) {
		if (disc_st == DISC_SERVICE) {
			printk("[CLIENT] Service not found\n");
			bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		} else if (disc_st == DISC_CHARS) {
			if (tg_rx_handle && tg_tx_handle) {
				subscribe_params.notify = tg_notify_func;
				subscribe_params.subscribe = tg_subscribe_cb;
				subscribe_params.value = BT_GATT_CCC_NOTIFY;
				subscribe_params.value_handle = tg_tx_handle;
				subscribe_params.ccc_handle = BT_GATT_AUTO_DISCOVER_CCC_HANDLE;
				subscribe_params.end_handle = tg_svc_end;
				subscribe_params.disc_params = &sub_disc_params;

				err = bt_gatt_subscribe(conn, &subscribe_params);
				if (err && err != -EALREADY) {
					printk("[CLIENT] Subscribe failed (err %d)\n", err);
					bt_conn_disconnect(conn,
						BT_HCI_ERR_REMOTE_USER_TERM_CONN);
				} else if (err == -EALREADY) {

					k_mutex_lock(&state_mutex, K_FOREVER);
					is_connected_to_truegear = true;
					k_mutex_unlock(&state_mutex);
					last_status_check_time = k_uptime_get_32();
					send_test_vibration();
				}
			} else {
				printk("[CLIENT] TX characteristic not found!\n");
				bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
			}
		}

		memset(params, 0, sizeof(*params));
		disc_st = DISC_IDLE;
		return BT_GATT_ITER_STOP;
	}

	if (params->type == BT_GATT_DISCOVER_PRIMARY) {
		struct bt_gatt_service_val *svc = attr->user_data;

		tg_svc_start = attr->handle + 1;
		tg_svc_end = svc->end_handle;

		printk("[CLIENT] Service found (handles 0x%04x-0x%04x)\n",
		       tg_svc_start, tg_svc_end);

		disc_st = DISC_CHARS;
		params->uuid = NULL;
		params->type = BT_GATT_DISCOVER_CHARACTERISTIC;
		params->start_handle = tg_svc_start;
		params->end_handle = tg_svc_end;

		err = bt_gatt_discover(conn, params);
		if (err) {
			printk("[CLIENT] Discover failed (err %d)\n", err);
		}
		return BT_GATT_ITER_STOP;
	}

	if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
		struct bt_gatt_chrc *chrc = attr->user_data;

		if (!bt_uuid_cmp(chrc->uuid, &bh_char_motor_uuid.uuid)) {
			tg_rx_handle = chrc->value_handle;
			printk("[CLIENT] RX handle 0x%04x\n", tg_rx_handle);
		} else if (!bt_uuid_cmp(chrc->uuid, &bh_char_serial_uuid.uuid)) {
			tg_tx_handle = chrc->value_handle;
			printk("[CLIENT] TX handle 0x%04x\n", tg_tx_handle);
		}
	}

	return BT_GATT_ITER_CONTINUE;
}

static void start_discovery(struct bt_conn *conn)
{
	int err;

	disc_st = DISC_SERVICE;
	discover_params.uuid = &bh_svc_uuid.uuid;
	discover_params.func = discover_func;
	discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	discover_params.type = BT_GATT_DISCOVER_PRIMARY;

	err = bt_gatt_discover(conn, &discover_params);
	if (err) {
		printk("[CLIENT] Discover failed (err %d)\n", err);
	}
}

/* ==========================================================================
 * Frame translation and sending to TrueGear
 * ========================================================================== */

static void translate_and_send_to_truegear(const uint8_t *motor_data)
{
	if (tg_conn == NULL || tg_rx_handle == 0U) {
		diag.skip_no_tg++;
		return;
	}

	bool connected;

	k_mutex_lock(&state_mutex, K_FOREVER);
	connected = is_connected_to_truegear;
	k_mutex_unlock(&state_mutex);

	if (!connected) {
		diag.skip_not_conn++;
		return;
	}

	uint8_t local_intensity;

	k_mutex_lock(&config_mutex, K_FOREVER);
	local_intensity = vest_intensity;
	k_mutex_unlock(&config_mutex);

	uint16_t duration_millis = 150;
	uint8_t frame[20] = { 0 };

	frame[0] = 0x68;
	frame[1] = 0x68;
	frame[2] = 0x01;
	frame[3] = 0x01;
	frame[4] = 0x00;
	frame[5] = 0x00;
	frame[6] = 0x00;
	frame[7] = (duration_millis >> 8) & 0xFF;
	frame[8] = duration_millis & 0xFF;

	int max_intens = 0;
	uint16_t group1 = 0, group2 = 0, group3 = 0, group4 = 0;

	for (int i = 0; i < MOTOR_COUNT; i++) {
		if (motor_data[i] > max_intens) {
			max_intens = motor_data[i];
		}
		if (motor_data[i] > 0) {
			uint8_t target_group = bh_to_tg_group[i];
			uint16_t target_mask = bh_to_tg_mask[i];

			if (target_group == 1) {
				group1 |= target_mask;
			} else if (target_group == 2) {
				group2 |= target_mask;
			} else if (target_group == 3) {
				group3 |= target_mask;
			} else if (target_group == 4) {
				group4 |= target_mask;
			}
		}
	}

	int intensity = (max_intens * (int)local_intensity * 127) / 10000;

	if (intensity > 255) {
		intensity = 255;
	}
	if (intensity < 0) {
		intensity = 0;
	}

	frame[9] = (uint8_t)intensity;
	frame[10] = (uint8_t)intensity;
	frame[11] = (group1 >> 8) & 0xFF;
	frame[12] = group1 & 0xFF;
	frame[13] = (group2 >> 8) & 0xFF;
	frame[14] = group2 & 0xFF;
	frame[15] = (group3 >> 8) & 0xFF;
	frame[16] = group3 & 0xFF;
	frame[17] = (group4 >> 8) & 0xFF;
	frame[18] = group4 & 0xFF;
	frame[19] = 0x16;

	diag.send_attempts++;

	int err = bt_gatt_write_without_response(tg_conn, tg_rx_handle,
						 frame, sizeof(frame), false);
	if (err) {
		diag.send_fail++;
		printk("[CLIENT] write failed (%d)\n", err);
	} else {
		diag.send_ok++;
	}

	if (diag.send_attempts <= 3 || (diag.send_attempts % 50) == 0) {
		printk("[TX] frame #%u max=%d int=%d g=%04x%04x%04x%04x err=%d\n",
		       diag.send_attempts, max_intens, intensity,
		       group1, group2, group3, group4, err);
	}
}

static struct bt_gatt_write_params test_write_params;
static uint8_t test_frame[20];

static void test_write_cb(struct bt_conn *conn, uint8_t err,
			  struct bt_gatt_write_params *params)
{
	if (err) {
		printk("[TX] TEST write REJECTED by TrueGear: 0x%02x\n", err);
	} else {
		printk("[TX] TEST write ACCEPTED — vest should vibrate now\n");
	}
}

static void send_test_vibration(void)
{
	if (tg_conn == NULL || tg_rx_handle == 0U) {
		printk("[TX] TEST skipped: no tg_conn/rx handle\n");
		return;
	}

	memset(test_frame, 0, sizeof(test_frame));
	test_frame[0] = 0x68;
	test_frame[1] = 0x68;
	test_frame[2] = 0x01;
	test_frame[3] = 0x01;
	test_frame[7] = 0x01;
	test_frame[8] = 0x2C;
	test_frame[9] = 40;
	test_frame[10] = 40;
	test_frame[11] = 0xFF;
	test_frame[12] = 0xFF;
	test_frame[13] = 0xFF;
	test_frame[14] = 0xFF;
	test_frame[15] = 0xFF;
	test_frame[16] = 0xFF;
	test_frame[17] = 0xFF;
	test_frame[18] = 0xFF;
	test_frame[19] = 0x16;

	test_write_params.func = test_write_cb;
	test_write_params.handle = tg_rx_handle;
	test_write_params.offset = 0;
	test_write_params.data = test_frame;
	test_write_params.length = sizeof(test_frame);

	int err = bt_gatt_write(tg_conn, &test_write_params);

	printk("[TX] TEST write queued (err=%d) — handle 0x%04x\n", err, tg_rx_handle);
	if (err) {
		printk("[TX] TEST write queue failed (%d)\n", err);
	}
}

/* ==========================================================================
 * TrueGear scanning and connecting
 * ========================================================================== */

static void start_scan(void);
static void start_advertising(void);

static bool eir_found(struct bt_data *data, void *user_data)
{
	bool *found = user_data;
	char name[32];

	switch (data->type) {
	case BT_DATA_NAME_COMPLETE:
	case BT_DATA_NAME_SHORTENED:
		if (data->data_len >= sizeof(name)) {
			return true;
		}
		memcpy(name, data->data, data->data_len);
		name[data->data_len] = '\0';

		if (strstr(name, "Truegear") != NULL ||
		    strstr(name, "truegear") != NULL) {
			*found = true;
			return false;
		}
		break;
	default:
		break;
	}

	return true;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	bool found = false;

	(void)type;

	bt_data_parse(ad, eir_found, &found);
	if (!found) {
		return;
	}

	if (tg_conn != NULL || is_connected_to_truegear || tg_connecting) {
		return;
	}

	int err = bt_le_scan_stop();

	if (err && err != -EALREADY) {
		printk("[CLIENT] Stop LE scan failed (err %d)\n", err);
	}

	k_mutex_lock(&state_mutex, K_FOREVER);
	is_scanning = false;
	bt_addr_le_copy(&truegear_addr, addr);
	truegear_addr_valid = true;
	do_connect = true;
	k_mutex_unlock(&state_mutex);

	printk("[CLIENT] TrueGear found — connecting...\n");
}

static struct bt_le_scan_param scan_param = {
	.type = BT_LE_SCAN_TYPE_ACTIVE,
	.options = BT_LE_SCAN_OPT_NONE,
	.interval = BT_GAP_SCAN_FAST_INTERVAL,
	.window = BT_GAP_SCAN_FAST_WINDOW,
};

static void start_scan(void)
{
	k_mutex_lock(&state_mutex, K_FOREVER);

	if (is_scanning || is_connected_to_truegear || do_connect || tg_connecting) {
		k_mutex_unlock(&state_mutex);
		return;
	}
	is_scanning = true;
	k_mutex_unlock(&state_mutex);

	int err = bt_le_scan_start(&scan_param, device_found);

	if (err) {
		k_mutex_lock(&state_mutex, K_FOREVER);
		is_scanning = false;
		k_mutex_unlock(&state_mutex);
		if (err != -EALREADY) {
			static uint32_t scan_err_count;

			scan_err_count++;
			if (scan_err_count <= 3 || (scan_err_count % 100) == 0) {
				printk("[CLIENT] Scanning failed to start (err %d) "
				       "[count=%u]\n", err, scan_err_count);
			}
		}
	}
}

static bool connect_to_truegear(const bt_addr_le_t *addr)
{

	if (tg_conn != NULL) {
		printk("[CLIENT] tg_conn already set (%p) — skipping connect\n",
		       (void *)tg_conn);
		k_mutex_lock(&state_mutex, K_FOREVER);
		tg_connecting = false;
		k_mutex_unlock(&state_mutex);
		return true;
	}

	struct bt_conn *new_conn = NULL;

	int err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
				    BT_LE_CONN_PARAM_DEFAULT, &new_conn);

	k_mutex_lock(&state_mutex, K_FOREVER);
	tg_connecting = false;
	k_mutex_unlock(&state_mutex);

	if (err) {
		printk("[CLIENT] Create connection failed (err %d)\n", err);

		k_mutex_lock(&state_mutex, K_FOREVER);
		is_scanning = false;
		k_mutex_unlock(&state_mutex);
		start_scan();
		return false;
	}

	tg_conn = new_conn;
	printk("[CLIENT] Connecting... (tg_conn=%p)\n", (void *)tg_conn);
	return true;
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	char addr_str[BT_ADDR_LE_STR_LEN];
	struct bt_conn_info info;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));

	if (conn_err) {
		printk("Failed to connect to %s (%u)\n", addr_str, conn_err);

		if (conn == tg_conn) {
			bt_conn_unref(tg_conn);
			tg_conn = NULL;
		}

		k_mutex_lock(&state_mutex, K_FOREVER);
		is_scanning = false;
		k_mutex_unlock(&state_mutex);
		start_scan();
		return;
	}

	(void)bt_conn_get_info(conn, &info);

	if (info.role == BT_CONN_ROLE_CENTRAL) {
		printk("[CLIENT] TrueGear connected\n");
		(void)bt_conn_le_param_update(conn, BT_LE_CONN_PARAM(6, 12, 0, 500));
		start_discovery(conn);
	} else {
		printk("Connected to PC/VR/APP\n");
		pc_conn = bt_conn_ref(conn);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr_str[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));
	printk("Disconnected: %s, reason 0x%02x\n", addr_str, reason);

	if (conn == tg_conn) {
		printk("[CLIENT] TrueGear disconnected\n");
		k_mutex_lock(&state_mutex, K_FOREVER);
		is_connected_to_truegear = false;
		k_mutex_unlock(&state_mutex);
		tg_rx_handle = 0;
		tg_tx_handle = 0;
		bt_conn_unref(tg_conn);
		tg_conn = NULL;
	} else {
		printk("Disconnected. Restarting advertising...\n");
		k_mutex_lock(&motor_mutex, K_FOREVER);
		memset(current_motor_state, 0, sizeof(current_motor_state));
		has_new_data = true;
		k_mutex_unlock(&motor_mutex);

		if (pc_conn) {
			bt_conn_unref(pc_conn);
			pc_conn = NULL;
		}

		start_advertising();
	}
}

/* ==========================================================================
 * Pairing callbacks (SMP) - required by Android
 * ========================================================================== */

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr_str[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));
	printk("[SMP] Pairing completed: %s, bonded: %d\n", addr_str, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr_str[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));
	printk("[SMP] Pairing failed: %s, reason %d\n", addr_str, reason);
}

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed,
};

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

/* ==========================================================================
 * Negotiated ATT MTU log (Android usually requests large MTU)
 * ========================================================================== */

static void att_mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	printk("[GATT] ATT MTU updated: tx=%u rx=%u\n", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {
	.att_mtu_updated = att_mtu_updated,
};

/* ==========================================================================
 * Advertising (peripheral - TactSuitX40)
 * ========================================================================== */

static const uint8_t mfg_data[] = { 0x9B, 0x03, 0x00, 0x00 };
static const char device_name[] = "TactSuitX40";

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, sizeof(mfg_data)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BH_UUID_ENCODE(0x6e400001)),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, device_name, sizeof(device_name) - 1),
	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE, 0xFD, 0x01),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x30, 0x15),
};

static void start_advertising(void)
{

	static const struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(
		BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY,
		BT_GAP_ADV_FAST_INT_MIN_1, BT_GAP_ADV_FAST_INT_MAX_1, NULL);

	int err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad),
				  sd, ARRAY_SIZE(sd));
	if (err) {
		if (err != -EALREADY) {
			printk("Advertising failed to start (err %d)\n", err);
		}
	} else {
		printk("Advertising started\n");
	}
}

/* ==========================================================================
 * Fixed public address (equivalent of esp_base_mac_addr_set)
 * ========================================================================== */

#define BH_MAC_MSB 0xE0

static void set_identity_address(void)
{
	bt_addr_le_t addr = {
		.type = BT_ADDR_LE_PUBLIC,
		.a.val = { 0xC3, 0xB2, 0xA1, 0x1B, 0x5A, 0xE0 },
	};
	int err = bt_id_create(&addr, NULL);

	if (err < 0) {

		addr.type = BT_ADDR_LE_RANDOM;
		err = bt_id_create(&addr, NULL);
		if (err < 0) {
			printk("[BLE] bt_id_create failed (%d)\n", err);
			return;
		}
		printk("[BLE] Identity (random static) set\n");
	} else {
		printk("[BLE] Identity (public) set\n");
	}
}

/* ==========================================================================
 * Main loop (equivalent of loop() from ESP32)
 * ========================================================================== */

int main(void)
{
	int err;

	if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
		(void)usb_enable(NULL);
	}

#if HAS_LED
	if (gpio_is_ready_dt(&led)) {
		(void)gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	}
#endif

	k_msleep(1000);

	printk("\n=======================================\n");
	printk("   TrueShifter V%s \n", TRUESHIFTER_VERSION);
	printk("   Board: %s\n", TRUESHIFTER_HARDWARE);
	printk("=======================================\n\n");

	set_identity_address();

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}
	printk("Bluetooth initialized\n");

	(void)settings_load();

	printk("[NVS] Config loaded:\n");
	printk("   Vest Power: %u%% (Stock: 50%%)\n", vest_intensity);
	printk("-----------------------------------------\n");

	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err) {
		printk("[SMP] auth_info_cb_register failed (%d)\n", err);
	}

	bt_gatt_cb_register(&gatt_callbacks);

	start_advertising();
	start_scan();

	printk("[SYSTEM] Server & Client initialized. Starting main loop.\n");

	last_status_check_time = k_uptime_get_32();

	while (1) {

		bool l_do_connect = false;
		bt_addr_le_t l_addr;
		bool has_addr = false;

		k_mutex_lock(&state_mutex, K_FOREVER);
		if (do_connect && truegear_addr_valid) {
			l_do_connect = true;
			bt_addr_le_copy(&l_addr, &truegear_addr);
			has_addr = true;
			do_connect = false;

			tg_connecting = true;
		}
		k_mutex_unlock(&state_mutex);

		if (l_do_connect && has_addr) {
			k_msleep(250);
			connect_to_truegear(&l_addr);
		}

		bool connected_flag;
		bool has_new;

		k_mutex_lock(&state_mutex, K_FOREVER);
		connected_flag = is_connected_to_truegear;
		k_mutex_unlock(&state_mutex);

		k_mutex_lock(&motor_mutex, K_FOREVER);
		has_new = has_new_data;
		k_mutex_unlock(&motor_mutex);

		if (connected_flag && has_new) {
			uint32_t now = k_uptime_get_32();

			if ((now - last_send_time) >= SEND_MIN_INTERVAL) {
				uint8_t local_copy[MOTOR_COUNT];

				k_mutex_lock(&motor_mutex, K_FOREVER);
				memcpy(local_copy, current_motor_state, sizeof(local_copy));
				k_mutex_unlock(&motor_mutex);

				bool state_changed = false;
				bool is_all_zero = true;

				for (int i = 0; i < MOTOR_COUNT; i++) {
					if (local_copy[i] != last_sent_motor_state[i]) {
						state_changed = true;
					}
					if (local_copy[i] > 0) {
						is_all_zero = false;
					}
				}

				bool should_send = false;

				if (state_changed) {
					should_send = true;
				} else if (!is_all_zero && ((now - last_send_time) >= SEND_REFRESH_MS)) {
					should_send = true;
				}

				if (should_send) {
					translate_and_send_to_truegear(local_copy);
					k_mutex_lock(&motor_mutex, K_FOREVER);
					memcpy(last_sent_motor_state, local_copy, sizeof(local_copy));
					has_new_data = false;
					k_mutex_unlock(&motor_mutex);
					last_send_time = now;
				} else {
					k_mutex_lock(&motor_mutex, K_FOREVER);
					has_new_data = false;
					k_mutex_unlock(&motor_mutex);
				}
			}
		}

		uint32_t current_ms = k_uptime_get_32();

		if ((current_ms - last_status_check_time) >= STATUS_INTERVAL_MS) {
			last_status_check_time = current_ms;

			bool should_scan = false;

			k_mutex_lock(&state_mutex, K_FOREVER);
			if (!is_connected_to_truegear && !is_scanning && !do_connect &&
			    !tg_connecting) {
				should_scan = true;
			}
			k_mutex_unlock(&state_mutex);

			if (should_scan) {
				start_scan();
			}
		}

		bool should_scan_now = false;

		k_mutex_lock(&state_mutex, K_FOREVER);
		if (!is_connected_to_truegear && !is_scanning && !do_connect &&
		    !tg_connecting) {
			should_scan_now = true;
		}
		k_mutex_unlock(&state_mutex);

		if (should_scan_now) {
			start_scan();
		}

		{
			static uint32_t last_diag_time;

			if ((current_ms - last_diag_time) >= 10000U) {
				last_diag_time = current_ms;
				diag_dump();
			}
		}

#if HAS_LED
		gpio_pin_toggle_dt(&led);
#endif
		k_msleep(1);
	}

	return 0;
}
