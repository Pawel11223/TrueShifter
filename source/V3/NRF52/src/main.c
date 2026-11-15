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
#include <zephyr/bluetooth/controller.h>

#include <string.h>

/* ==========================================================================
 * Version / hardware
 * ========================================================================== */

#define TRUESHIFTER_VERSION  "3.2.1"
#define TRUESHIFTER_HARDWARE "nRF52840"

#define ID_SUIT  0
#define ID_ARML  1
#define ID_ARMR  2
#define ID_COUNT 3

static uint8_t bh_serial_suit[10] = {
	0xcf, 0xcb, 0x0d, 0x95, 0x5f, 0xf6, 0xee, 0x2c, 0xbd, 0x73
};
static uint8_t bh_serial_arml[10] = {
	0xa0, 0xba, 0x0a, 0xd1, 0xbf, 0x36, 0x11, 0x30, 0xa4, 0xff
};
static uint8_t bh_serial_armr[10] = {
	0xb0, 0x1c, 0xc1, 0xf8, 0xec, 0x12, 0x18, 0x4e, 0x09, 0x77
};

static const char *const id_name[ID_COUNT] = { "SUIT", "ARM_L", "ARM_R" };

/* ==========================================================================
 * UUIDs
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
 * bHaptics -> TrueGear mapping
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
 * Shared state + mutexes
 * ========================================================================== */

static K_MUTEX_DEFINE(motor_mutex);
static K_MUTEX_DEFINE(state_mutex);
static K_MUTEX_DEFINE(config_mutex);

#define MOTOR_COUNT 40
#define ARM_MOTOR_COUNT 3

static uint8_t current_motor_state[MOTOR_COUNT];
static uint8_t last_sent_motor_state[MOTOR_COUNT];
static bool has_new_data;

static uint32_t last_send_time;
static uint32_t last_status_check_time;

static uint32_t last_arm_l_time;
static uint32_t last_arm_r_time;

#define STATUS_INTERVAL_MS 15000
#define SEND_MIN_INTERVAL  35
#define SEND_REFRESH_MS    120

static uint8_t bh_config_suit[3] = { 100, 0, 0 };
static uint8_t bh_config_arml[3] = { 100, 1, 0 };
static uint8_t bh_config_armr[3] = { 100, 2, 0 };

static uint8_t bh_monitor_val;
static uint8_t bh_ath_val[20];
static uint16_t bh_battery_val = 50;
static uint8_t bas_level = 100;

static uint8_t vest_intensity = 50;
static uint8_t ems_intensity = 30;
static uint8_t ems_threshold = 25;
static uint8_t enable_tactosy = 1;

#define EMS_AGG_AVG    0
#define EMS_AGG_MAX    1
#define EMS_AGG_SMART  2

#define EMS_TYPE_ONCE  0
#define EMS_TYPE_CONST 1
#define EMS_TYPE_FADE  2

static uint8_t ems_agg_mode = EMS_AGG_SMART;
static uint8_t ems_pulse_type = EMS_TYPE_FADE;
static uint8_t ems_interval = 2;
static uint8_t ems_duration = 16;
static uint8_t ems_base_power = 2;
static uint8_t ems_peak_mod = 1;
static uint8_t ems_wake = 0;
static uint8_t ts_reset_slot;

enum tx_stage {
	TX_IDLE,
	TX_WAKE,
	TX_EMS,
};

static enum tx_stage tx_stage = TX_IDLE;
static uint8_t ems_frame[20];
static bool ems_test_done;

#define EMS_BMP_A_MASK     0xF000
#define EMS_BMP_B_MASK     0xF000
#define EMS_TARGET_MODE    1

#define TG_TEST_VIBRATION  0

static const uint8_t shake_frame_template[20] = {
	104, 104, 1, 2, 0, 0, 0, 1, 244, 0,
	70, 255, 192, 255, 192, 255, 192, 255, 192, 22
};

static const uint8_t ems_frame_template[20] = {
	104, 104, 1, 17, 22, 2, 191, 3, 232, 1,
	0, 0, 0, 0, 0, 240, 0, 240, 0, 22
};

static bool is_connected_to_truegear;
static bool is_scanning;
static bool do_connect;
static bool tg_connecting;
static bt_addr_le_t truegear_addr;
static bool truegear_addr_valid;

static struct bt_conn *tg_conn;
static struct bt_conn *conn_suit;
static struct bt_conn *conn_arm_l;
static struct bt_conn *conn_arm_r;

static struct bt_le_ext_adv *adv_set[ID_COUNT];

static uint16_t tg_svc_start;
static uint16_t tg_svc_end;
static uint16_t tg_rx_handle;
static uint16_t tg_tx_handle;

/* ==========================================================================
 * Status LED
 * ========================================================================== */

#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#define HAS_LED 1
#else
#define HAS_LED 0
#endif

/* ==========================================================================
 * Connection identification
 * ========================================================================== */

static uint8_t conn_get_id(struct bt_conn *conn)
{
	if (conn == NULL) {
		return ID_SUIT;
	}
	struct bt_conn_info info;
	if (bt_conn_get_info(conn, &info) == 0) {
		return info.id;
	}
	return ID_SUIT;
}

static bool conn_is_arm(struct bt_conn *conn)
{
	uint8_t id = conn_get_id(conn);
	return (id == ID_ARML || id == ID_ARMR);
}

/* ==========================================================================
 * Persistent configuration (NVS)
 * ========================================================================== */

struct ts_setting {
	const char *key;
	uint8_t *var;
	uint8_t max;
};

static const struct ts_setting ts_settings[] = {
	{ "vest_int", &vest_intensity,  100 },
	{ "ems_int",  &ems_intensity,   100 },
	{ "ems_thr",  &ems_threshold,   100 },
	{ "tactosy",  &enable_tactosy,  1   },
	{ "agg",      &ems_agg_mode,    2   },
	{ "type",     &ems_pulse_type,  2   },
	{ "intv",     &ems_interval,    255 },
	{ "dur",      &ems_duration,    255 },
	{ "bpow",     &ems_base_power,  255 },
	{ "pmod",     &ems_peak_mod,    1   },
	{ "wake",     &ems_wake,        1   },
	{ "reset",    &ts_reset_slot,   255 },
};

#define TS_SETTING_COUNT ARRAY_SIZE(ts_settings)

static int ts_settings_set(const char *name, size_t len,
			   settings_read_cb read_cb, void *cb_arg)
{
	uint8_t val;
	int rc;

	if (len < 1) {
		return -EINVAL;
	}

	rc = read_cb(cb_arg, &val, sizeof(val));
	if (rc < 0) {
		return rc;
	}

	for (size_t i = 0; i < TS_SETTING_COUNT; i++) {
		if (strcmp(name, ts_settings[i].key) != 0) {
			continue;
		}

		uint8_t clamped = MIN(val, ts_settings[i].max);

		k_mutex_lock(&config_mutex, K_FOREVER);
		*ts_settings[i].var = clamped;
		k_mutex_unlock(&config_mutex);

		printk("[NVS] %s = %u\n", ts_settings[i].key, clamped);
		return 0;
	}

	return -ENOENT;
}

static int ts_settings_get(const char *key, char *val, int val_len_max)
{
	if (val_len_max < 1) {
		return -EINVAL;
	}

	for (size_t i = 0; i < TS_SETTING_COUNT; i++) {
		if (strcmp(key, ts_settings[i].key) != 0) {
			continue;
		}

		uint8_t v;

		k_mutex_lock(&config_mutex, K_FOREVER);
		v = *ts_settings[i].var;
		k_mutex_unlock(&config_mutex);

		val[0] = (char)v;
		return 1;
	}

	return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(ts, "ts", ts_settings_get, ts_settings_set, NULL, NULL);

/* ==========================================================================
 * GATT helpers
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

static ssize_t read_version(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			    void *buf, uint16_t len, uint16_t offset)
{
	uint8_t id = conn_get_id(conn);

	uint16_t ver = sys_cpu_to_le16((id == ID_SUIT) ? 0x0502 : 0x0102);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &ver, sizeof(ver));
}

static const char str_bhaptics[] = "bhaptics";
static const char str_model_suit[]  = "TactSuitX40";
static const char str_model_arm_l[] = "TactSleeve (L)";
static const char str_model_arm_r[] = "TactSleeve (R)";
static const char str_fw_rev[] = TRUESHIFTER_VERSION;
static const char str_hw_rev[] = TRUESHIFTER_HARDWARE;

static uint8_t *serial_for_conn(struct bt_conn *conn)
{
	uint8_t id = conn_get_id(conn);
	if (id == ID_ARML) {
		return bh_serial_arml;
	}
	if (id == ID_ARMR) {
		return bh_serial_armr;
	}
	return bh_serial_suit;
}

static ssize_t read_model(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			  void *buf, uint16_t len, uint16_t offset)
{
	const char *m;
	uint8_t id = conn_get_id(conn);

	if (id == ID_ARML) {
		m = str_model_arm_l;
	} else if (id == ID_ARMR) {
		m = str_model_arm_r;
	} else {
		m = str_model_suit;
	}

	return bt_gatt_attr_read(conn, attr, buf, len, offset, m, strlen(m));
}

static ssize_t read_serial(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			   void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 serial_for_conn(conn), 10);
}

static ssize_t write_serial(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			    const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	uint8_t *dst = serial_for_conn(conn);

	if (offset + len > 10) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	memcpy(dst + offset, buf, len);
	return len;
}

static uint8_t *config_for_conn(struct bt_conn *conn)
{
	uint8_t id = conn_get_id(conn);
	if (id == ID_ARML) {
		return bh_config_arml;
	}
	if (id == ID_ARMR) {
		return bh_config_armr;
	}
	return bh_config_suit;
}

static ssize_t read_bh_config(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			      void *buf, uint16_t len, uint16_t offset)
{
	uint8_t *cfg = config_for_conn(conn);
	return bt_gatt_attr_read(conn, attr, buf, len, offset, cfg, 3);
}

static ssize_t write_bh_config(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			       const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	uint8_t *cfg = config_for_conn(conn);

	if (offset + len > 3) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	memcpy(cfg + offset, buf, len);
	printk("[CONFIG] bHaptics config updated for %s: intensity=%u position=%u\n",
	       id_name[conn_get_id(conn)], cfg[0], cfg[1]);
	return len;
}

static struct gatt_buf ath_buf = { bh_ath_val, sizeof(bh_ath_val) };
static struct gatt_buf battery_buf = { &bh_battery_val, sizeof(bh_battery_val) };
static struct gatt_buf monitor_buf = { &bh_monitor_val, sizeof(bh_monitor_val) };
static struct gatt_buf bas_buf = { &bas_level, sizeof(bas_level) };

/* ==========================================================================
 * Diagnostics
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
	uint32_t arm_writes;
	uint32_t ems_queued;
	uint32_t ems_dropped_thr;
	uint32_t ems_dropped_off;
	uint32_t ems_str_last;
	uint32_t arm_l_events;
	uint32_t arm_r_events;
	uint16_t ems_raw_last;
	uint32_t ems_bitmap_last;
	uint32_t wake_sent;
	uint32_t ems_tx_ok;
	uint32_t ems_tx_fail;
	uint32_t arm_l_active;
	uint32_t arm_r_active;
} diag;

static const char *tx_stage_name(enum tx_stage st)
{
	switch (st) {
	case TX_WAKE:
		return "WAKE";
	case TX_EMS:
		return "EMS";
	default:
		return "IDLE";
	}
}

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
	printk("[DIAG] ARM/EMS: writes=%u queued=%u drop_thr=%u drop_off=%u "
	       "str=%u current=%u bmp=%08X | evL=%u evR=%u\n",
	       diag.arm_writes, diag.ems_queued, diag.ems_dropped_thr,
	       diag.ems_dropped_off, diag.ems_str_last, diag.ems_raw_last,
	       diag.ems_bitmap_last, diag.arm_l_events, diag.arm_r_events);
	printk("[DIAG] TX: stage=%s wake=%u ems_ok=%u ems_fail=%u | bandL=%u bandR=%u\n",
	       tx_stage_name(tx_stage), diag.wake_sent,
	       diag.ems_tx_ok, diag.ems_tx_fail,
	       diag.arm_l_active, diag.arm_r_active);
	printk("[DIAG] tg_conn=%p rx=0x%04x tx=0x%04x tg_connected=%d connecting=%d\n",
	       (void *)tg_conn, tg_rx_handle, tg_tx_handle,
	       (int)is_connected_to_truegear, (int)tg_connecting);
	printk("[DIAG] conn suit=%p armL=%p armR=%p | vest=%u%% ems=%u%% thr=%u%% "
	       "bpow=%u00 tactosy=%u\n",
	       (void *)conn_suit, (void *)conn_arm_l, (void *)conn_arm_r,
	       vest_intensity, ems_intensity, ems_threshold,
	       ems_base_power, enable_tactosy);
}

static void send_test_ems(void);
#if TG_TEST_VIBRATION
static void send_test_vibration(void);
#endif

/* ==========================================================================
 * Motor frame writer + EMS generator (TactSleeve - 3 motors)
 * ========================================================================== */

static uint8_t motor_raw[20];

static void build_wake_frame(uint8_t *frame)
{
	memcpy(frame, shake_frame_template, 20);
}

struct ems_pulse_params {
	uint8_t type;
	uint16_t start_ms;
	uint16_t end_ms;
	uint8_t interval;
	uint8_t start_pct;
};

static uint8_t ems_aggregate(const uint8_t *m, uint8_t mode)
{
	uint8_t peak = 0;
	uint16_t sum = 0;

	for (int i = 0; i < ARM_MOTOR_COUNT; i++) {
		if (m[i] > peak) {
			peak = m[i];
		}
		sum += m[i];
	}

	uint8_t avg = (uint8_t)(sum / ARM_MOTOR_COUNT);

	switch (mode) {
	case EMS_AGG_MAX:
		return peak;
	case EMS_AGG_SMART:

		return (uint8_t)(((uint16_t)peak + (uint16_t)avg) / 2U);
	default:
		return avg;
	}
}

static void ems_smart_params(const uint8_t *m, struct ems_pulse_params *out,
			     uint8_t strength, uint8_t thr, uint32_t dt_ms)
{
	uint8_t active = 0;

	for (int i = 0; i < ARM_MOTOR_COUNT; i++) {
		if (m[i] >= thr) {
			active++;
		}
	}

	out->start_ms = 0;

	if (dt_ms < 150U) {
		out->type = 0x12;
		out->end_ms = 110;
		out->interval = 2;
		out->start_pct = 45;
		return;
	}

	if (active <= 1) {
		out->type = 0x12;
		out->end_ms = 140 + ((uint16_t)strength * 80U) / 100U;
		out->interval = 2;
		out->start_pct = 35;
	} else if (active >= 3) {
		out->type = 0x12;
		out->end_ms = 250 + ((uint16_t)strength * 150U) / 100U;
		out->interval = 3;
		out->start_pct = 30;
	} else {
		out->type = 0x12;
		out->end_ms = 180 + ((uint16_t)strength * 100U) / 100U;
		out->interval = 3;
		out->start_pct = 35;
	}
}

static void ems_manual_params(struct ems_pulse_params *out)
{
	static const uint8_t type_map[3] = { 0x10, 0x11, 0x12 };
	uint8_t t, dur, iv;

	k_mutex_lock(&config_mutex, K_FOREVER);
	t = ems_pulse_type;
	dur = ems_duration;
	iv = ems_interval;
	k_mutex_unlock(&config_mutex);

	out->type = type_map[MIN(t, 2U)];
	out->start_ms = 0;
	out->end_ms = (uint16_t)dur * 10U;
	out->interval = iv;
	out->start_pct = (out->type == 0x12) ? 35 : 100;
}

static void build_ems_frame(uint8_t *frame, uint32_t current,
			    uint16_t bmp_a, uint16_t bmp_b,
			    const struct ems_pulse_params *pp)
{
	uint32_t start_i = (current * (uint32_t)pp->start_pct) / 100U;
	uint32_t end_i = current;

	if (start_i > 0xFFFFU) {
		start_i = 0xFFFFU;
	}
	if (end_i > 0xFFFFU) {
		end_i = 0xFFFFU;
	}

	memcpy(frame, ems_frame_template, 20);

	frame[3] = pp->type;
	frame[5] = (pp->start_ms >> 8) & 0xFF;
	frame[6] = pp->start_ms & 0xFF;
	frame[7] = (pp->end_ms >> 8) & 0xFF;
	frame[8] = pp->end_ms & 0xFF;
	frame[9] = pp->interval;

	frame[11] = (start_i >> 8) & 0xFF;
	frame[12] = start_i & 0xFF;
	frame[13] = (end_i >> 8) & 0xFF;
	frame[14] = end_i & 0xFF;

	frame[15] = (bmp_a >> 8) & 0xFF;
	frame[16] = bmp_a & 0xFF;
	frame[17] = (bmp_b >> 8) & 0xFF;
	frame[18] = bmp_b & 0xFF;
}

static void queue_ems_frame(const uint8_t *motors, uint8_t strength, uint8_t slider,
			    uint16_t bmp_a, uint16_t bmp_b, uint32_t dt_ms)
{
	uint8_t base100, peak_mod, agg, wake, thr;
	struct ems_pulse_params pp;

	k_mutex_lock(&config_mutex, K_FOREVER);
	base100 = ems_base_power;
	peak_mod = ems_peak_mod;
	agg = ems_agg_mode;
	wake = ems_wake;
	thr = ems_threshold;
	k_mutex_unlock(&config_mutex);

	if (agg == EMS_AGG_SMART) {
		ems_smart_params(motors, &pp, strength, thr, dt_ms);
	} else {
		ems_manual_params(&pp);
	}

	uint32_t base = (uint32_t)base100 * 100U;
	uint32_t current = (base * (100U + 14U * (uint32_t)slider)) / 1000U;

	if (peak_mod) {
		current = (current * (uint32_t)strength) / 100U;
	}

	if (current > 0xFFFFU) {
		current = 0xFFFFU;
	}

	k_mutex_lock(&motor_mutex, K_FOREVER);
	if (tx_stage == TX_IDLE) {
		build_ems_frame(ems_frame, current, bmp_a, bmp_b, &pp);
		tx_stage = wake ? TX_WAKE : TX_EMS;
	} else {

		uint32_t prev_end = ((uint32_t)ems_frame[13] << 8) | ems_frame[14];

		if (current > prev_end) {
			uint32_t start_i = (current * (uint32_t)pp.start_pct) / 100U;

			ems_frame[11] = (start_i >> 8) & 0xFF;
			ems_frame[12] = start_i & 0xFF;
			ems_frame[13] = (current >> 8) & 0xFF;
			ems_frame[14] = current & 0xFF;
		}
		ems_frame[15] |= (bmp_a >> 8) & 0xFF;
		ems_frame[16] |= bmp_a & 0xFF;
		ems_frame[17] |= (bmp_b >> 8) & 0xFF;
		ems_frame[18] |= bmp_b & 0xFF;
	}

	uint8_t snapshot[20];
	memcpy(snapshot, ems_frame, sizeof(snapshot));
	k_mutex_unlock(&motor_mutex);

	diag.ems_queued++;
	diag.ems_str_last = strength;
	diag.ems_raw_last = (uint16_t)current;
	diag.ems_bitmap_last = ((uint32_t)snapshot[15] << 24) |
			       ((uint32_t)snapshot[16] << 16) |
			       ((uint32_t)snapshot[17] << 8) |
			       (uint32_t)snapshot[18];

	if (diag.ems_queued <= 3 || (diag.ems_queued % 50) == 0) {
		printk("[EMS] queue #%u str=%u slider=%u cur=%u typ=%02X dur=%u intv=%u dt=%u "
		       "bmp=%02X%02X%02X%02X | ",
		       diag.ems_queued, strength, slider, current, pp.type,
		       pp.end_ms, pp.interval, dt_ms,
		       snapshot[15], snapshot[16], snapshot[17], snapshot[18]);
		for (int i = 0; i < 20; i++) {
			printk("%02X ", snapshot[i]);
		}
		printk("\n");
	}
}

static void handle_arm_write(struct bt_conn *conn, const uint8_t *payload,
			     uint16_t filled)
{
	uint8_t id = conn_get_id(conn);
	bool is_left = (id == ID_ARML);
	uint8_t motors[ARM_MOTOR_COUNT] = { 0 };

	uint32_t now = k_uptime_get_32();
	uint32_t dt = is_left ? (now - last_arm_l_time) : (now - last_arm_r_time);

	if (is_left) {
		last_arm_l_time = now;
	} else {
		last_arm_r_time = now;
	}

	if (filled == 2) {

		motors[0] = ((payload[0] >> 4) & 0x0F) * 100 / 15;
		motors[1] = (payload[0] & 0x0F) * 100 / 15;
		motors[2] = ((payload[1] >> 4) & 0x0F) * 100 / 15;
	} else if (filled == 3) {

		if (payload[0] > 15 || payload[1] > 15 || payload[2] > 15) {

			motors[0] = MIN(payload[0], 100);
			motors[1] = MIN(payload[1], 100);
			motors[2] = MIN(payload[2], 100);
		} else {

			uint8_t m[6];
			for (int i = 0; i < 3; i++) {
				m[i * 2]     = ((payload[i] >> 4) & 0x0F) * 100 / 15;
				m[i * 2 + 1] = (payload[i] & 0x0F) * 100 / 15;
			}
			motors[0] = MAX(m[0], m[1]);
			motors[1] = MAX(m[2], m[3]);
			motors[2] = MAX(m[4], m[5]);
		}
	} else if (filled >= 6) {

		motors[0] = MAX(payload[0], payload[1]);
		motors[1] = MAX(payload[2], payload[3]);
		motors[2] = MAX(payload[4], payload[5]);
	} else {
		for (uint16_t i = 0; i < MIN(filled, (uint16_t)ARM_MOTOR_COUNT); i++) {
			motors[i] = payload[i];
		}
	}

	diag.arm_writes++;
	if (is_left) {
		diag.arm_l_events++;
	} else {
		diag.arm_r_events++;
	}

	uint8_t thr, inten, tactosy, agg;

	k_mutex_lock(&config_mutex, K_FOREVER);
	thr = ems_threshold;
	inten = ems_intensity;
	tactosy = enable_tactosy;
	agg = ems_agg_mode;
	k_mutex_unlock(&config_mutex);

	if (!tactosy) {
		diag.ems_dropped_off++;
		return;
	}

	uint8_t strength = ems_aggregate(motors, agg);
	diag.ems_str_last = strength;

	if (strength < thr) {
		diag.ems_dropped_thr++;
		return;
	}

	uint16_t bmp_a, bmp_b;

#if EMS_TARGET_MODE
	if (is_left) {
		bmp_a = EMS_BMP_A_MASK;
		bmp_b = 0x0000U;
	} else {
		bmp_a = 0x0000U;
		bmp_b = EMS_BMP_B_MASK;
	}
#else
	bmp_a = EMS_BMP_A_MASK;
	bmp_b = EMS_BMP_B_MASK;
#endif

	queue_ems_frame(motors, strength, inten, bmp_a, bmp_b, dt);

	if (diag.arm_writes <= 3 || (diag.arm_writes % 50) == 0) {
		printk("[EMS] write #%u %s str=%u thr=%u slider=%u agg=%u dt=%u bmp=%04X+%04X\n",
		       diag.arm_writes, is_left ? "ARM_L" : "ARM_R",
		       strength, thr, inten, agg, dt, bmp_a, bmp_b);
	}
}

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

	diag.last_attr = attr->handle;
	diag.last_len = len;
	diag.last_off = offset;
	diag.last_flags = flags;
	diag.last_char = attr->user_data ? (const char *)attr->user_data : "?";
	memcpy(diag.last_raw, buf, MIN(len, sizeof(diag.last_raw)));

	if (conn_is_arm(conn)) {
		handle_arm_write(conn, motor_raw, filled);
		return len;
	}

	diag.motor_writes++;
	diag.motor_bytes += len;

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
 * TS_CHAR_CONFIG (6e4000fe)
 * ========================================================================== */

#define TS_CFG_LEN (int)TS_SETTING_COUNT
#define TS_CFG_RESET_IDX (TS_CFG_LEN - 1)
#define TS_CFG_RESET_MAGIC 0xA5

static void ts_factory_reset(void)
{
	printk("[CONFIG] *** RESET BONDOW BT dla %u tozsamosci ***\n", (uint32_t)ID_COUNT);
	for (uint8_t id = 0; id < ID_COUNT; id++) {
		int err = bt_unpair(id, NULL);
		printk("[CONFIG] bt_unpair(id=%u) -> %d\n", id, err);
	}
}

static ssize_t read_ts_config(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			      void *buf, uint16_t len, uint16_t offset)
{
	uint8_t v[TS_CFG_LEN];

	k_mutex_lock(&config_mutex, K_FOREVER);
	for (int i = 0; i < TS_CFG_LEN; i++) {
		v[i] = *ts_settings[i].var;
	}
	k_mutex_unlock(&config_mutex);

	v[TS_CFG_RESET_IDX] = 0;
	return bt_gatt_attr_read(conn, attr, buf, len, offset, v, sizeof(v));
}

static ssize_t write_ts_config(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			       const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	const uint8_t *p = buf;

	if ((offset + len) > (uint16_t)TS_CFG_LEN) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	if (len == 0U) {
		return 0;
	}

	for (uint16_t i = 0; i < len; i++) {
		uint16_t idx = offset + i;

		if (idx == (uint16_t)TS_CFG_RESET_IDX) {
			if (p[i] == TS_CFG_RESET_MAGIC) {
				ts_factory_reset();
			}
			continue;
		}

		uint8_t clamped = MIN(p[i], ts_settings[idx].max);
		char nvs_name[24];

		k_mutex_lock(&config_mutex, K_FOREVER);
		*ts_settings[idx].var = clamped;
		k_mutex_unlock(&config_mutex);

		snprintk(nvs_name, sizeof(nvs_name), "ts/%s", ts_settings[idx].key);
		int save_err = settings_save_one(nvs_name, &clamped, 1);

		printk("[CONFIG] %s = %u (nvs=%d)\n", ts_settings[idx].key, clamped, save_err);
	}

	return len;
}

static void bh_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	printk("[GATT] CCC changed: 0x%04x\n", value);
}

/* ==========================================================================
 * GATT peripheral services
 * ========================================================================== */

static ssize_t read_gap_name(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			     void *buf, uint16_t len, uint16_t offset)
{
	const char *n;
	uint8_t id = conn_get_id(conn);

	if (id == ID_ARML) {
		n = str_model_arm_l;
	} else if (id == ID_ARMR) {
		n = str_model_arm_r;
	} else {
		n = str_model_suit;
	}

	return bt_gatt_attr_read(conn, attr, buf, len, offset, n, strlen(n));
}

static ssize_t read_gap_appearance(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				   void *buf, uint16_t len, uint16_t offset)
{
	uint8_t id = conn_get_id(conn);

	uint16_t ap_le = sys_cpu_to_le16((id == ID_SUIT) ? 509 : 508);
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &ap_le, sizeof(ap_le));
}

static ssize_t read_gap_ppcp(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			     void *buf, uint16_t len, uint16_t offset)
{
	struct __packed {
		uint16_t min_int;
		uint16_t max_int;
		uint16_t latency;
		uint16_t timeout;
	} ppcp;

	ppcp.min_int = sys_cpu_to_le16(CONFIG_BT_PERIPHERAL_PREF_MIN_INT);
	ppcp.max_int = sys_cpu_to_le16(CONFIG_BT_PERIPHERAL_PREF_MAX_INT);
	ppcp.latency = sys_cpu_to_le16(CONFIG_BT_PERIPHERAL_PREF_LATENCY);
	ppcp.timeout = sys_cpu_to_le16(CONFIG_BT_PERIPHERAL_PREF_TIMEOUT);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &ppcp, sizeof(ppcp));
}

BT_GATT_SERVICE_DEFINE(_2_gap_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_GAP),

	BT_GATT_CHARACTERISTIC(BT_UUID_GAP_DEVICE_NAME,
		BT_GATT_CHRC_READ, BT_GATT_PERM_READ,
		read_gap_name, NULL, NULL),

	BT_GATT_CHARACTERISTIC(BT_UUID_GAP_APPEARANCE,
		BT_GATT_CHRC_READ, BT_GATT_PERM_READ,
		read_gap_appearance, NULL, NULL),

	BT_GATT_CHARACTERISTIC(BT_UUID_GAP_PPCP,
		BT_GATT_CHRC_READ, BT_GATT_PERM_READ,
		read_gap_ppcp, NULL, NULL),
);

BT_GATT_SERVICE_DEFINE(dis_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_DIS),

	BT_GATT_CHARACTERISTIC(BT_UUID_DIS_MANUFACTURER_NAME,
		BT_GATT_CHRC_READ, BT_GATT_PERM_READ,
		read_str, NULL, (void *)str_bhaptics),

	BT_GATT_CHARACTERISTIC(BT_UUID_DIS_MODEL_NUMBER,
		BT_GATT_CHRC_READ, BT_GATT_PERM_READ,
		read_model, NULL, NULL),

	BT_GATT_CHARACTERISTIC(BT_UUID_DIS_SERIAL_NUMBER,
		BT_GATT_CHRC_READ, BT_GATT_PERM_READ,
		read_serial, NULL, NULL),

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
		read_serial, write_serial, NULL),

	BT_GATT_CHARACTERISTIC(&bh_char_version_uuid.uuid,
		BT_GATT_CHRC_READ, BT_GATT_PERM_READ,
		read_version, NULL, NULL),

	BT_GATT_CHARACTERISTIC(&bh_char_config_uuid.uuid,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
		read_bh_config, write_bh_config, NULL),

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
 * TrueGear client
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

static int voltage_to_pct(int mv)
{
	int pct;
	if (mv >= 4150) return 100;
	if (mv <= 3300) return 0;
	pct = (int)(((mv - 3300) * 100) / 850);
	if (pct > 100) pct = 100;
	if (pct < 0) pct = 0;
	return pct;
}

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
		int vest_mv = (pdata[9] << 8) | pdata[10];
		bool left_on = (pdata[11] == 0x01);
		bool right_on = (pdata[15] == 0x01);
		int left_mv = (pdata[13] << 8) | pdata[14];
		int right_mv = (pdata[17] << 8) | pdata[18];

		int percentage = voltage_to_pct(vest_mv);

		if (left_on) diag.arm_l_active++;
		if (right_on) diag.arm_r_active++;

		printk("[CLIENT] BATTERY: %d%% (%d mV)", percentage, vest_mv);
		if (left_on) printk(" | ARM_L: %d mV (%d%%)", left_mv, voltage_to_pct(left_mv));
		if (right_on) printk(" | ARM_R: %d mV (%d%%)", right_mv, voltage_to_pct(right_mv));
		printk("\n");

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

#if TG_TEST_VIBRATION
	send_test_vibration();
#endif
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
					bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
				} else if (err == -EALREADY) {
					k_mutex_lock(&state_mutex, K_FOREVER);
					is_connected_to_truegear = true;
					k_mutex_unlock(&state_mutex);
					last_status_check_time = k_uptime_get_32();
#if TG_TEST_VIBRATION
					send_test_vibration();
#endif
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
 * Sending data to TrueGear
 * ========================================================================== */

static int send_raw_frame(const uint8_t *frame)
{
	if (tg_conn == NULL || tg_rx_handle == 0U) {
		return -ENOTCONN;
	}

	int err = bt_gatt_write_without_response(tg_conn, tg_rx_handle, frame, 20, false);
	if (err) {
		diag.send_fail++;
		printk("[CLIENT] write failed (%d)\n", err);
	} else {
		diag.send_ok++;
	}

	return err;
}

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

	frame[0] = 0x68; frame[1] = 0x68;
	frame[2] = 0x01; frame[3] = 0x01;
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

			if (target_group == 1) group1 |= target_mask;
			else if (target_group == 2) group2 |= target_mask;
			else if (target_group == 3) group3 |= target_mask;
			else if (target_group == 4) group4 |= target_mask;
		}
	}

	int intensity = (max_intens * (int)local_intensity * 127) / 10000;
	if (intensity > 255) intensity = 255;
	if (intensity < 0) intensity = 0;

	frame[9] = (uint8_t)intensity;
	frame[10] = (uint8_t)intensity;
	frame[11] = (group1 >> 8) & 0xFF; frame[12] = group1 & 0xFF;
	frame[13] = (group2 >> 8) & 0xFF; frame[14] = group2 & 0xFF;
	frame[15] = (group3 >> 8) & 0xFF; frame[16] = group3 & 0xFF;
	frame[17] = (group4 >> 8) & 0xFF; frame[18] = group4 & 0xFF;
	frame[19] = 0x16;

	diag.send_attempts++;
	send_raw_frame(frame);
}

#if TG_TEST_VIBRATION
static struct bt_gatt_write_params test_write_params;
static uint8_t test_frame[20];

static void test_write_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params *params)
{
	if (err) printk("[TX] TEST write REJECTED: 0x%02x\n", err);
	else printk("[TX] TEST write ACCEPTED\n");
}

static void send_test_vibration(void)
{
	if (tg_conn == NULL || tg_rx_handle == 0U) return;

	memset(test_frame, 0, sizeof(test_frame));
	test_frame[0] = 0x68; test_frame[1] = 0x68;
	test_frame[2] = 0x01; test_frame[3] = 0x01;
	test_frame[7] = 0x01; test_frame[8] = 0x2C;
	test_frame[9] = 40;   test_frame[10] = 40;
	for (int i = 11; i <= 18; i++) test_frame[i] = 0xFF;
	test_frame[19] = 0x16;

	test_write_params.func = test_write_cb;
	test_write_params.handle = tg_rx_handle;
	test_write_params.offset = 0;
	test_write_params.data = test_frame;
	test_write_params.length = sizeof(test_frame);

	bt_gatt_write(tg_conn, &test_write_params);
}
#endif

static struct bt_gatt_write_params test_ems_params;
static uint8_t test_ems_frame[20];

static void test_ems_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params *params)
{
	if (err) printk("[TX] *** TEST EMS REJECTED: 0x%02x ***\n", err);
	else printk("[TX] *** TEST EMS ACCEPTED ***\n");
}

static void send_test_ems(void)
{
	if (tg_conn == NULL || tg_rx_handle == 0U) return;

	uint8_t slider = 30;
	uint8_t base100;

	k_mutex_lock(&config_mutex, K_FOREVER);
	slider = ems_intensity;
	base100 = ems_base_power;
	k_mutex_unlock(&config_mutex);

	uint32_t current = ((uint32_t)base100 * 100U * (100U + 14U * (uint32_t)slider)) / 1000U;

	struct ems_pulse_params pp = {
		.type = 0x12,
		.start_ms = 0,
		.end_ms = 250,
		.interval = 2,
		.start_pct = 35,
	};

	static const struct {
		uint16_t a;
		uint16_t b;
		const char *label;
	} sweep[3] = {
		{ EMS_BMP_A_MASK, EMS_BMP_B_MASK, "BOTH" },
		{ EMS_BMP_A_MASK, 0x0000U,        "GROUP A (L)" },
		{ 0x0000U,        EMS_BMP_B_MASK, "GROUP B (R)" },
	};

	printk("[TX] TEST EMS SWEEP — current=%u (slider=%u%%, base=%u00)\n",
	       current, slider, base100);

	for (int i = 0; i < 3; i++) {
		build_ems_frame(test_ems_frame, current, sweep[i].a, sweep[i].b, &pp);
		if (i == 0) {
			test_ems_params.func = test_ems_cb;
			test_ems_params.handle = tg_rx_handle;
			test_ems_params.offset = 0;
			test_ems_params.data = test_ems_frame;
			test_ems_params.length = sizeof(test_ems_frame);
			bt_gatt_write(tg_conn, &test_ems_params);
		} else {
			send_raw_frame(test_ems_frame);
		}
		k_msleep(1500);
	}

	last_send_time = k_uptime_get_32();
}

/* ==========================================================================
 * TrueGear scanning
 * ========================================================================== */

static void start_scan(void);
static void start_advertising_id(uint8_t id);
static void stop_advertising_id(uint8_t id);

static bool eir_found(struct bt_data *data, void *user_data)
{
	bool *found = user_data;
	char name[32];

	switch (data->type) {
	case BT_DATA_NAME_COMPLETE:
	case BT_DATA_NAME_SHORTENED:
		if (data->data_len >= sizeof(name)) return true;
		memcpy(name, data->data, data->data_len);
		name[data->data_len] = '\0';

		if (strstr(name, "Truegear") != NULL || strstr(name, "truegear") != NULL) {
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
	if (!found) return;

	if (tg_conn != NULL || is_connected_to_truegear || tg_connecting) return;

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
	.window = 0x0014,
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
	}
}

static bool connect_to_truegear(const bt_addr_le_t *addr)
{
	if (tg_conn != NULL) {
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
		printk("[CLIENT] Create connection failed (%d)\n", err);
		start_scan();
		return false;
	}

	tg_conn = new_conn;
	return true;
}

/* ==========================================================================
 * Connection callbacks
 * ========================================================================== */

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
		start_scan();
		return;
	}

	(void)bt_conn_get_info(conn, &info);

	if (info.role == BT_CONN_ROLE_CENTRAL) {
		printk("[CLIENT] TrueGear connected\n");
		(void)bt_conn_le_param_update(conn, BT_LE_CONN_PARAM(6, 12, 0, 500));
		start_discovery(conn);
		return;
	}

	switch (info.id) {
	case ID_SUIT:
		conn_suit = bt_conn_ref(conn);
		printk("[ID] SUIT connected (%s)\n", addr_str);
		break;
	case ID_ARML:
		conn_arm_l = bt_conn_ref(conn);
		printk("[ID] ARM_L connected (%s)\n", addr_str);
		break;
	case ID_ARMR:
		conn_arm_r = bt_conn_ref(conn);
		printk("[ID] ARM_R connected (%s)\n", addr_str);
		break;
	default:
		return;
	}

	stop_advertising_id(info.id);
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
		return;
	}

	if (conn == conn_suit) {
		printk("[ID] SUIT disconnected.\n");
		k_mutex_lock(&motor_mutex, K_FOREVER);
		memset(current_motor_state, 0, sizeof(current_motor_state));
		has_new_data = true;
		k_mutex_unlock(&motor_mutex);
		bt_conn_unref(conn_suit);
		conn_suit = NULL;
		start_advertising_id(ID_SUIT);
		return;
	}

	if (conn == conn_arm_l) {
		printk("[ID] ARM_L disconnected.\n");
		k_mutex_lock(&motor_mutex, K_FOREVER);
		tx_stage = TX_IDLE;
		k_mutex_unlock(&motor_mutex);
		bt_conn_unref(conn_arm_l);
		conn_arm_l = NULL;
		start_advertising_id(ID_ARML);
		return;
	}

	if (conn == conn_arm_r) {
		printk("[ID] ARM_R disconnected.\n");
		k_mutex_lock(&motor_mutex, K_FOREVER);
		tx_stage = TX_IDLE;
		k_mutex_unlock(&motor_mutex);
		bt_conn_unref(conn_arm_r);
		conn_arm_r = NULL;
		start_advertising_id(ID_ARMR);
		return;
	}
}

/* ==========================================================================
 * SMP Just Works callbacks
 * ========================================================================== */

static void pairing_confirm(struct bt_conn *conn)
{
	bt_conn_auth_pairing_confirm(conn);
}

static enum bt_security_err pairing_accept(struct bt_conn *conn,
					   const struct bt_conn_pairing_feat *const feat)
{
	return BT_SECURITY_ERR_SUCCESS;
}

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

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.pairing_accept = pairing_accept,
	.pairing_confirm = pairing_confirm,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed,
};

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

/* ==========================================================================
 * GATT log
 * ========================================================================== */

static void att_mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	printk("[GATT] ATT MTU updated: tx=%u rx=%u\n", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {
	.att_mtu_updated = att_mtu_updated,
};

static bool gatt_auth_read(struct bt_conn *conn, const struct bt_gatt_attr *attr)
{
	return true;
}

static bool gatt_auth_write(struct bt_conn *conn, const struct bt_gatt_attr *attr)
{
	return true;
}

static struct bt_gatt_authorization_cb gatt_auth_cb = {
	.read_authorize = gatt_auth_read,
	.write_authorize = gatt_auth_write,
};

/* ==========================================================================
 * Advertising (Extended Advertising API)
 * ========================================================================== */

static const uint8_t mfg_data_suit[] = { 0x9B, 0x03, 0x00, 0x00 };
static const uint8_t mfg_data_arml[] = { 0x9B, 0x03, 0x01, 0x00 };
static const uint8_t mfg_data_armr[] = { 0x9B, 0x03, 0x02, 0x00 };

static const struct bt_data ad_suit[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data_suit, sizeof(mfg_data_suit)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BH_UUID_ENCODE(0x6e400001)),
};

static const struct bt_data ad_arm_l[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data_arml, sizeof(mfg_data_arml)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BH_UUID_ENCODE(0x6e400001)),
};

static const struct bt_data ad_arm_r[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data_armr, sizeof(mfg_data_armr)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BH_UUID_ENCODE(0x6e400001)),
};

static const struct bt_data sd_suit[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, "TactSuitX40", 11),
	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE, 0xFD, 0x01),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x30, 0x15),
};

static const struct bt_data sd_arm_l[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, "TactSleeve (L)", 14),
	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE, 0xFC, 0x01),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x30, 0x15),
};

static const struct bt_data sd_arm_r[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, "TactSleeve (R)", 14),
	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE, 0xFC, 0x01),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x30, 0x15),
};

#define ADV_OPTS (BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY)

static const struct bt_le_adv_param adv_params[ID_COUNT] = {
	{
		.id = ID_SUIT,
		.options = ADV_OPTS,
		.interval_min = BT_GAP_ADV_FAST_INT_MIN_1,
		.interval_max = BT_GAP_ADV_FAST_INT_MAX_1,
		.peer = NULL,
	},
	{
		.id = ID_ARML,
		.options = ADV_OPTS,
		.interval_min = BT_GAP_ADV_FAST_INT_MIN_1,
		.interval_max = BT_GAP_ADV_FAST_INT_MAX_1,
		.peer = NULL,
	},
	{
		.id = ID_ARMR,
		.options = ADV_OPTS,
		.interval_min = BT_GAP_ADV_FAST_INT_MIN_1,
		.interval_max = BT_GAP_ADV_FAST_INT_MAX_1,
		.peer = NULL,
	},
};

static void start_advertising_id(uint8_t id)
{
	if (id >= ID_COUNT || adv_set[id] == NULL) return;

	int err = bt_le_ext_adv_start(adv_set[id], BT_LE_EXT_ADV_START_DEFAULT);
	if (!err) {
		printk("[ADV] id%u (%s) started\n", id, id_name[id]);
	}
}

static void stop_advertising_id(uint8_t id)
{
	if (id >= ID_COUNT || adv_set[id] == NULL) return;
	(void)bt_le_ext_adv_stop(adv_set[id]);
	printk("[ADV] id%u (%s) stopped\n", id, id_name[id]);
}

static void start_advertising(void)
{
	for (uint8_t i = 0; i < ID_COUNT; i++) {
		start_advertising_id(i);
	}
}

static void create_advertising_sets(void)
{
	for (uint8_t i = 0; i < ID_COUNT; i++) {
		const struct bt_data *ad_p;
		size_t ad_len;
		const struct bt_data *sd_p;
		size_t sd_len;

		if (i == ID_SUIT) {
			ad_p = ad_suit;
			ad_len = ARRAY_SIZE(ad_suit);
			sd_p = sd_suit;
			sd_len = ARRAY_SIZE(sd_suit);
		} else if (i == ID_ARML) {
			ad_p = ad_arm_l;
			ad_len = ARRAY_SIZE(ad_arm_l);
			sd_p = sd_arm_l;
			sd_len = ARRAY_SIZE(sd_arm_l);
		} else {
			ad_p = ad_arm_r;
			ad_len = ARRAY_SIZE(ad_arm_r);
			sd_p = sd_arm_r;
			sd_len = ARRAY_SIZE(sd_arm_r);
		}

		int err = bt_le_ext_adv_create(&adv_params[i], NULL, &adv_set[i]);
		if (err) {
			printk("[ADV] id%u (%s) create failed (%d)\n", i, id_name[i], err);
			adv_set[i] = NULL;
			continue;
		}

		err = bt_le_ext_adv_set_data(adv_set[i], ad_p, ad_len, sd_p, sd_len);
		if (err) {
			printk("[ADV] id%u (%s) set_data failed (%d)\n", i, id_name[i], err);
		}
	}
}

/* ==========================================================================
 * Sleeve identity creation
 * ========================================================================== */

static void create_arm_identities(void)
{
	static const struct {
		uint8_t val[6];
		const char *desc;
	} rand_ids[2] = {
		{ { 0xD1, 0xB2, 0xA1, 0x1B, 0x5A, 0xF2 }, "ARM_L" },
		{ { 0xD2, 0xB2, 0xA1, 0x1B, 0x5A, 0xF2 }, "ARM_R" },
	};

	for (uint8_t i = 0; i < 2; i++) {
		bt_addr_le_t addr;
		addr.type = BT_ADDR_LE_RANDOM;
		memcpy(addr.a.val, rand_ids[i].val, sizeof(addr.a.val));

		int err = bt_id_create(&addr, NULL);
		if (err < 0) {
			printk("[BLE] Identity %u (%s) bt_id_create failed (%d)\n",
			       i + 1, rand_ids[i].desc, err);
		} else {
			printk("[BLE] Identity %u (%s) set (id=%d, rand-static)\n",
			       i + 1, rand_ids[i].desc, err);
		}
	}
}

/* ==========================================================================
 * Main program
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

	static const uint8_t pub_mac[6] = { 0xC3, 0xB2, 0xA1, 0x1B, 0x5A, 0xE0 };
	bt_ctlr_set_public_addr(pub_mac);

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (%d)\n", err);
		return 0;
	}
	printk("Bluetooth initialized\n");

	create_arm_identities();

	err = settings_subsys_init();
	if (err) {
		printk("[NVS] settings_subsys_init failed (%d)\n", err);
	} else {
		err = settings_load();
		if (err) {
			printk("[NVS] settings_load failed (%d)\n", err);
		}
	}

	printk("[NVS] Config loaded:\n");
	k_mutex_lock(&config_mutex, K_FOREVER);
	for (size_t i = 0; i < TS_SETTING_COUNT; i++) {
		printk("   [%u] %-8s = %u\n", (uint32_t)i, ts_settings[i].key, *ts_settings[i].var);
	}
	k_mutex_unlock(&config_mutex);
	printk("-----------------------------------------\n");

	err = bt_conn_auth_cb_register(&conn_auth_callbacks);
	if (err) printk("[SMP] auth_cb_register failed (%d)\n", err);

	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err) printk("[SMP] auth_info_cb_register failed (%d)\n", err);

	bt_gatt_cb_register(&gatt_callbacks);

	err = bt_gatt_authorization_cb_register(&gatt_auth_cb);
	if (err) printk("[GATT] authorization_cb_register failed (%d)\n", err);

	create_advertising_sets();
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
		k_mutex_lock(&state_mutex, K_FOREVER);
		connected_flag = is_connected_to_truegear;
		k_mutex_unlock(&state_mutex);

		if (connected_flag) {
			uint32_t now = k_uptime_get_32();

			if ((now - last_send_time) >= SEND_MIN_INTERVAL) {
				enum tx_stage st;
				uint8_t local_ems[20];
				uint8_t wake_frame[20];
				uint8_t local_copy[MOTOR_COUNT];
				bool have_motor = false;

				k_mutex_lock(&motor_mutex, K_FOREVER);
				st = tx_stage;
				memcpy(local_ems, ems_frame, sizeof(local_ems));
				if (st == TX_IDLE && has_new_data) {
					memcpy(local_copy, current_motor_state, sizeof(local_copy));
					have_motor = true;
				}
				k_mutex_unlock(&motor_mutex);

				if (st == TX_WAKE) {
					build_wake_frame(wake_frame);
					(void)send_raw_frame(wake_frame);
					diag.wake_sent++;
					k_mutex_lock(&motor_mutex, K_FOREVER);
					if (tx_stage == TX_WAKE) tx_stage = TX_EMS;
					k_mutex_unlock(&motor_mutex);
					last_send_time = now;
				} else if (st == TX_EMS) {
					int err_ems = send_raw_frame(local_ems);
					if (err_ems) diag.ems_tx_fail++;
					else diag.ems_tx_ok++;

					k_mutex_lock(&motor_mutex, K_FOREVER);
					if (tx_stage == TX_EMS) tx_stage = TX_IDLE;
					k_mutex_unlock(&motor_mutex);
					last_send_time = now;
				} else if (have_motor) {
					bool state_changed = false;
					bool is_all_zero = true;

					for (int i = 0; i < MOTOR_COUNT; i++) {
						if (local_copy[i] != last_sent_motor_state[i]) {
							state_changed = true;
						}
						if (local_copy[i] > 0) is_all_zero = false;
					}

					bool should_send = false;
					if (state_changed) should_send = true;
					else if (!is_all_zero && ((now - last_send_time) >= SEND_REFRESH_MS)) {
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
		}

		if (!ems_test_done) {
			bool tg_ready;
			k_mutex_lock(&state_mutex, K_FOREVER);
			tg_ready = is_connected_to_truegear;
			k_mutex_unlock(&state_mutex);

			if (tg_ready && (conn_arm_l != NULL || conn_arm_r != NULL)) {
				ems_test_done = true;
				send_test_ems();
			}
		}

		uint32_t current_ms = k_uptime_get_32();

		if ((current_ms - last_status_check_time) >= STATUS_INTERVAL_MS) {
			last_status_check_time = current_ms;
			bool should_scan = false;

			k_mutex_lock(&state_mutex, K_FOREVER);
			if (!is_connected_to_truegear && !is_scanning && !do_connect && !tg_connecting) {
				should_scan = true;
			}
			k_mutex_unlock(&state_mutex);

			if (should_scan) {
				start_scan();
			}
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
