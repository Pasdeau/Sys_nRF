#include "uart_async_adapter.h"

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <inttypes.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>

#include <bluetooth/services/nus.h>
#include <zephyr/bluetooth/att.h>
#include <dk_buttons_and_leds.h>

#include <zephyr/settings/settings.h>

#include <stdio.h>

#include <zephyr/logging/log.h>

#include "ADS1298.c"

static struct k_work brightness_work;
K_MUTEX_DEFINE(data_mutex);

const static struct device *dev0, *dev1;
#define LEDEXT0_NODE DT_ALIAS(reset)

#define LEDEXT0	DT_GPIO_LABEL(LEDEXT0_NODE, gpios)
#define RESET DT_GPIO_PIN(LEDEXT0_NODE, gpios)

#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000
#define CON_STATUS_LED DK_LED2

#define SW0_NODE DT_ALIAS(sw0)

#define UART_BUF_SIZE 512
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX 100

static K_SEM_DEFINE(ble_init_ok, 0, 1);

#define SLEEP_TIME_MS    1000
#define DEBOUNCE_TIME_MS 200
#define NUM_BUTTONS      5
#define NUM_LEDS         4
#define BRIGHTNESS_STEPS 5

#define BUTTON_NODE(n) DT_ALIAS(swext##n)
#define LED_NODE(n)    DT_ALIAS(ext##n)
#define SHDN_NODE 	   DT_ALIAS(shdn)

#define SPI_AD8403 DT_NODELABEL(spi_ad8403)
const struct device *spi_ad8403;

static const struct gpio_dt_spec shdn_pin = GPIO_DT_SPEC_GET_OR(SHDN_NODE, gpios, {0});

static int led_brightness_levels[NUM_LEDS][BRIGHTNESS_STEPS] = {
    {255, 192, 128, 64, 0},   // LED 0
    {255, 192, 128, 64, 0},   // LED 1
    {255, 192, 128, 64, 0},   // LED 2
    {255, 192, 128, 64, 0}    // LED 3
};

static int current_led = -1;
static int current_brightness[NUM_LEDS] = {0};
static bool is_active = true;
static bool increase_brightness = true;
static uint32_t last_press_time = 0;

struct button {
    const struct gpio_dt_spec spec;
    struct gpio_callback callback;
};

static struct button buttons[NUM_BUTTONS] = {
    {GPIO_DT_SPEC_GET_OR(BUTTON_NODE(0), gpios, {0})},
    {GPIO_DT_SPEC_GET_OR(BUTTON_NODE(1), gpios, {0})},
    {GPIO_DT_SPEC_GET_OR(BUTTON_NODE(2), gpios, {0})},
    {GPIO_DT_SPEC_GET_OR(BUTTON_NODE(3), gpios, {0})},
	{GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0})},
};

static struct gpio_dt_spec leds[NUM_LEDS] = {
    GPIO_DT_SPEC_GET_OR(LED_NODE(0), gpios, {0}),
    GPIO_DT_SPEC_GET_OR(LED_NODE(1), gpios, {0}),
    GPIO_DT_SPEC_GET_OR(LED_NODE(2), gpios, {0}),
    GPIO_DT_SPEC_GET_OR(LED_NODE(3), gpios, {0}),
};

struct k_timer mosfet_timer;
static struct gpio_dt_spec *current_mosfet = NULL;

void shdn_init(void) {
    if (!device_is_ready(shdn_pin.port)) {
        LOG_ERR("SHDN pin not ready!\n");
        return;
    }
    gpio_pin_configure_dt(&shdn_pin, GPIO_OUTPUT_INACTIVE);
}

static void spi_init(void) {
	spi_ad8403 = DEVICE_DT_GET(SPI_AD8403);
    if (!device_is_ready(spi_ad8403)) {
		LOG_ERR("SPI master device not ready!\n");
	}
}

struct spi_cs_control spim_cs = {
	.gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_NODELABEL(spi_cs_ad8403)),
	.delay = 0,
};

struct spi_config spi_ad8403_cfg = {
    .frequency = 4000000,
    .operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
	.slave = 0,
	.cs = &spim_cs,
};

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

static const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart));
static struct k_work_delayable uart_work;

struct uart_data_t {
	void *fifo_reserved;
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
};

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

#if CONFIG_BT_NUS_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
static const struct device *const async_adapter;
#endif

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data) {
	ARG_UNUSED(dev);

	static size_t aborted_len;
	struct uart_data_t *buf;
	static uint8_t *aborted_buf;
	static bool disable_req;

	switch (evt->type) {
	case UART_TX_DONE:
		LOG_DBG("UART_TX_DONE");
		if ((evt->data.tx.len == 0) || (!evt->data.tx.buf)) {
			return;
		}
		if (aborted_buf) {
			buf = CONTAINER_OF(aborted_buf, struct uart_data_t, data);
			aborted_buf = NULL;
			aborted_len = 0;
		} else {
			buf = CONTAINER_OF(evt->data.tx.buf, struct uart_data_t, data);
		}
		k_free(buf);
		buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		if (!buf) {
			return;
		}
		if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS)) {
			LOG_WRN("Failed to send data over UART");
		}
		break;

	case UART_RX_RDY:
		LOG_DBG("UART_RX_RDY");
		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data);
		buf->len += evt->data.rx.len;
		if (disable_req) {
			return;
		}
		if ((evt->data.rx.buf[buf->len - 1] == '\n') || (evt->data.rx.buf[buf->len - 1] == '\r')) {
			disable_req = true;
			uart_rx_disable(uart);
		}
		break;

	case UART_RX_DISABLED:
		LOG_DBG("UART_RX_DISABLED");
		disable_req = false;
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
			k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
			return;
		}
		uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
		break;

	case UART_RX_BUF_REQUEST:
		LOG_DBG("UART_RX_BUF_REQUEST");
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
		}
		break;

	case UART_RX_BUF_RELEASED:
		LOG_DBG("UART_RX_BUF_RELEASED");
		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t, data);
		if (buf->len > 0) {
			k_fifo_put(&fifo_uart_rx_data, buf);
		} else {
			k_free(buf);
		}
		break;

	case UART_TX_ABORTED:
		LOG_DBG("UART_TX_ABORTED");
		if (!aborted_buf) {
			aborted_buf = (uint8_t *)evt->data.tx.buf;
		}
		aborted_len += evt->data.tx.len;
		buf = CONTAINER_OF(aborted_buf, struct uart_data_t, data);
		uart_tx(uart, &buf->data[aborted_len], buf->len - aborted_len, SYS_FOREVER_MS);
		break;
	default:
		break;
	}
}

static void uart_work_handler(struct k_work *item) {
	struct uart_data_t *buf;
	buf = k_malloc(sizeof(*buf));
	if (buf) {
		buf->len = 0;
	} else {
		LOG_WRN("Not able to allocate UART receive buffer");
		k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
		return;
	}
	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
}

static bool uart_test_async_api(const struct device *dev) {
	const struct uart_driver_api *api = (const struct uart_driver_api *)dev->api;
	return (api->callback_set != NULL);
}

static int uart_init(void) {
	int err;
	int pos;
	struct uart_data_t *rx;
	struct uart_data_t *tx;

	if (!device_is_ready(uart)) {
		return -ENODEV;
	}

	if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
		err = usb_enable(NULL);
		if (err && (err != -EALREADY)) {
			LOG_ERR("Failed to enable USB");
			return err;
		}
	}

	rx = k_malloc(sizeof(*rx));
	if (rx) {
		rx->len = 0;
	} else {
		return -ENOMEM;
	}

	k_work_init_delayable(&uart_work, uart_work_handler);


	if (IS_ENABLED(CONFIG_BT_NUS_UART_ASYNC_ADAPTER) && !uart_test_async_api(uart)) {
		/* Implement API adapter */
		uart_async_adapter_init(async_adapter, uart);
		uart = async_adapter;
	}

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		k_free(rx);
		LOG_ERR("Cannot initialize UART callback");
		return err;
	}

	if (IS_ENABLED(CONFIG_UART_LINE_CTRL)) {
		LOG_INF("Wait for DTR");
		while (true) {
			uint32_t dtr = 0;

			uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr);
			if (dtr) {
				break;
			}
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
		}
		LOG_INF("DTR set");
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DCD, 1);
		if (err) {
			LOG_WRN("Failed to set DCD, ret code %d", err);
		}
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DSR, 1);
		if (err) {
			LOG_WRN("Failed to set DSR, ret code %d", err);
		}
	}

	tx = k_malloc(sizeof(*tx));

	if (tx) {
		pos = snprintf(tx->data, sizeof(tx->data),
			       "Starting Nordic UART service example\r\n");

		if ((pos < 0) || (pos >= sizeof(tx->data))) {
			k_free(rx);
			k_free(tx);
			LOG_ERR("snprintf returned %d", pos);
			return -ENOMEM;
		}

		tx->len = pos;
	} else {
		k_free(rx);
		return -ENOMEM;
	}

	err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
	if (err) {
		k_free(rx);
		k_free(tx);
		LOG_ERR("Cannot display welcome message (err: %d)", err);
		return err;
	}

	err = uart_rx_enable(uart, rx->data, sizeof(rx->data), 100);
	if (err) {
		LOG_ERR("Cannot enable uart reception (err: %d)", err);
		/* Free the rx buffer only because the tx buffer will be handled in the callback */
		k_free(rx);
	}
	return err;
}

static void connected(struct bt_conn *conn, uint8_t err) {
	char addr[BT_ADDR_LE_STR_LEN];
	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);
	current_conn = bt_conn_ref(conn);
	dk_set_led_on(CON_STATUS_LED);

	// ===== Active 2M PHY ===== //
    struct bt_conn_le_phy_param phy_2m = {
        .options = 0,
        .pref_tx_phy = BT_GAP_LE_PHY_2M,
        .pref_rx_phy = BT_GAP_LE_PHY_2M,
    };

    int phy_err = bt_conn_le_phy_update(conn, &phy_2m);
    if (phy_err) {
        LOG_ERR("Failed to set 2M PHY (err %d)", phy_err);
    } else {
        LOG_INF("2M PHY enabled at connection");
    }
}

static void toggle_ble_phy(void) {
	LOG_INF("toggle_ble_phy() called!\n");

    static bool is_phy_2m = true;  // 记录当前的 PHY 状态
    int err;

    if (!current_conn) {
        LOG_INF("No active BLE connection\n");
        return;
    }

    struct bt_conn_le_phy_param phy_param;

    if (is_phy_2m) {
        // 切换到 1M PHY
        phy_param.pref_tx_phy = BT_GAP_LE_PHY_1M;
        phy_param.pref_rx_phy = BT_GAP_LE_PHY_1M;
        LOG_INF("Switching to 1M PHY\n");
    } else {
        // 切换到 2M PHY
        phy_param.pref_tx_phy = BT_GAP_LE_PHY_2M;
        phy_param.pref_rx_phy = BT_GAP_LE_PHY_2M;
        LOG_INF("Switching to 2M PHY\n");
    }

    phy_param.options = 0;

    err = bt_conn_le_phy_update(current_conn, &phy_param);
    if (err) {
        LOG_ERR("Failed to update PHY (err %d)\n", err);
    } else {
        is_phy_2m = !is_phy_2m;  // 切换状态
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Disconnected: %s (reason %u)", addr, reason);
	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}
	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
		dk_set_led_off(CON_STATUS_LED);
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err) {
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	if (!err) {
		LOG_INF("Security changed: %s level %u", addr, level);
	} else {
		LOG_WRN("Security failed: %s level %u err %d", addr, level, err);
	}
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};

#if defined(CONFIG_BT_NUS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey) {
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Passkey for %s: %06u", addr, passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey) {
	char addr[BT_ADDR_LE_STR_LEN];
	auth_conn = bt_conn_ref(conn);
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Passkey for %s: %06u", addr, passkey);
	LOG_INF("Press Button 1 to confirm, Button 2 to reject.");
}

static void auth_cancel(struct bt_conn *conn) {
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Pairing cancelled: %s", addr);
}

static void pairing_complete(struct bt_conn *conn, bool bonded) {
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason) {
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Pairing failed conn: %s, reason %d", addr, reason);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data, uint16_t len) {
	int err;
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	LOG_INF("Received data from: %s", addr);

	for (uint16_t pos = 0; pos != len;) {
		struct uart_data_t *tx = k_malloc(sizeof(*tx));

		if (!tx) {
			LOG_WRN("Not able to allocate UART send data buffer");
			return;
		}

		/* Keep the last byte of TX buffer for potential LF char. */
		size_t tx_data_size = sizeof(tx->data) - 1;

		if ((len - pos) > tx_data_size) {
			tx->len = tx_data_size;
		} else {
			tx->len = (len - pos);
		}

		memcpy(tx->data, &data[pos], tx->len);

		pos += tx->len;

		/* Append the LF character when the CR character triggered
		 * transmission from the peer.
		 */
		if ((pos == len) && (data[len - 1] == '\r')) {
			tx->data[tx->len] = '\n';
			tx->len++;
		}

		err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
		if (err) {
			k_fifo_put(&fifo_uart_tx_data, tx);
		}
	}
}

static struct bt_nus_cb nus_cb = { .received = bt_receive_cb, };

void set_shdn(bool enable) {
    gpio_pin_set(shdn_pin.port, shdn_pin.pin, enable ? 1 : 0);
    LOG_INF("SHDN %s\n", enable ? "disabled" : "enabled");
}

int set_pot(int address, int value) {
    uint16_t data = ((address & 0x03) << 8) | (value & 0xFF);
    uint8_t tx_buffer[2] = {data >> 8, data & 0xFF};

    const struct spi_buf tx_buf = {
        .buf = tx_buffer,
        .len = sizeof(tx_buffer),
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    set_shdn(false);

    int error = spi_write(spi_ad8403, &spi_ad8403_cfg, &tx);
    if (error != 0) {
        LOG_ERR("SPI transceive error: %i\n", error);
        return error;
    } else {
        LOG_INF("SPI write successful\n");
        return 0;
    }
}

void mosfet_blink_callback(struct k_timer *timer_id) {
    static bool mosfet_on = false;

    if (current_mosfet != NULL) {
        gpio_pin_set(current_mosfet->port, current_mosfet->pin, mosfet_on ? 0 : 1);
        mosfet_on = !mosfet_on;
    }
}

void turn_on_led(int led_index) {
    for (int i = 0; i < NUM_LEDS; i++) {
        gpio_pin_set(leds[i].port, leds[i].pin, (i == led_index) ? 0 : 1);
    }
}

void update_led_brightness() {
    int level = led_brightness_levels[current_led][current_brightness[current_led]];
    set_pot(current_led, level);
}

void brightness_work_handler(struct k_work *work) {
    if (is_active) {
        int level;
        if (increase_brightness) {
            current_brightness[current_led] = (current_brightness[current_led] + 1) % BRIGHTNESS_STEPS;
        } else {
            current_brightness[current_led] = (current_brightness[current_led] - 1 + BRIGHTNESS_STEPS) % BRIGHTNESS_STEPS;
        }
        level = led_brightness_levels[current_led][current_brightness[current_led]];

        if (set_pot(current_led, level) == 0) {
            LOG_INF("Updated brightness for LED %d to %d\n", current_led, level);
        }
    }
}

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    uint32_t current_time = k_uptime_get_32();
    int action = 0;

    if (current_time - last_press_time < DEBOUNCE_TIME_MS) {
        return;
    }
    last_press_time = current_time;

    if (pins & BIT(buttons[0].spec.pin)) {
        if (is_active) {
            current_led = (current_led + 1) % NUM_LEDS;

            switch (current_led) {
                case 0: current_mosfet = &leds[0]; break;
                case 1: current_mosfet = &leds[1]; break;
                case 2: current_mosfet = &leds[2]; break;
                case 3: current_mosfet = &leds[3]; break;
            }

            k_timer_stop(&mosfet_timer);
            gpio_pin_set(current_mosfet->port, current_mosfet->pin, 1);
			// k_timer_start(&mosfet_timer, K_MSEC(20), K_MSEC(20));
            action = 0;
        }
    } else if (pins & BIT(buttons[1].spec.pin)) {
        increase_brightness = true;
        k_work_submit(&brightness_work); // 提交到工作队列
        action = 1;
    } else if (pins & BIT(buttons[2].spec.pin)) {
        increase_brightness = false;
        k_work_submit(&brightness_work); // 提交到工作队列
        action = 2;
    } else if (pins & BIT(buttons[3].spec.pin)) {
        is_active = !is_active;
        action = 3;
    } else if (pins & BIT(buttons[4].spec.pin)) {
        LOG_INF("PHY CHANGE button detected!\n");
        toggle_ble_phy();
        action = 4;
	}

    if (is_active) {
        turn_on_led(current_led);

        LOG_INF("Button %d pressed: ", action);
        LOG_INF("Current LED is %d\n", current_led);
        LOG_INF("Current brightness level: %d\n", led_brightness_levels[current_led][current_brightness[current_led]]);
    }
}

void force_led_on(int led_index) {
    if (led_index >= 0 && led_index < NUM_LEDS) {
        gpio_pin_set(leds[led_index].port, leds[led_index].pin, 0); // 低电平点亮
    } else {
        LOG_ERR("Invalid LED index for turning on: %d\n", led_index);
    }
}

void force_led_off(int led_index) {
    if (led_index >= 0 && led_index < NUM_LEDS) {
        gpio_pin_set(leds[led_index].port, leds[led_index].pin, 1); // 高电平熄灭
    } else {
        LOG_ERR("Invalid LED index for turning off: %d\n", led_index);
    }
}

void reset_ADS1298() { 
    static int err;
    dev0 = device_get_binding(LEDEXT0);

    if (dev0 == NULL ){
        LOG_INF("Error: cannot find reset pin of SPI\n");
        return;
    }
    
    err = gpio_pin_configure(dev0, RESET, GPIO_OUTPUT_ACTIVE);
    if (err < 0 ){
        LOG_INF("Error: pin reset SPI can't be configured\n");
        return;
    }
    gpio_pin_set(dev0, RESET, 1);   
    k_usleep(10);  // wait 18 * TCLK = 18 * 514ns 
    gpio_pin_set(dev0, RESET, 0);
    k_msleep(500); 
    gpio_pin_set(dev0, RESET, 1);
    k_msleep(500);
}

void config_ADS1298() {
    // Ajouter les fonction pour configurer ADS1298
    ads1298_write_register(ADS129X_REG_CONFIG1, 0xC5); // HR mode, SPS: 0-32k, 1-16k, 2-8k, 3-4k, 4-2k, 5-1k, 6-500, 7-non_use
    ads1298_write_register(ADS129X_REG_CONFIG2, 0x00); // default register
	ads1298_write_register(ADS129X_REG_CONFIG3, 0xE0); // internal power ref = 4 V
    ads1298_write_register(ADS129X_REG_CONFIG4, 0x00); // default register

    ads1298_write_register(ADS129X_REG_CH1SET, TURN_OFF_CHANNEL);
    ads1298_write_register(ADS129X_REG_CH2SET, TURN_OFF_CHANNEL);
    ads1298_write_register(ADS129X_REG_CH3SET, TURN_OFF_CHANNEL);
    ads1298_write_register(ADS129X_REG_CH4SET, TURN_OFF_CHANNEL);

	ads1298_write_register(ADS129X_REG_CH5SET, TURN_OFF_CHANNEL);
    ads1298_write_register(ADS129X_REG_CH6SET, TURN_OFF_CHANNEL);
    ads1298_write_register(ADS129X_REG_CH7SET, GAIN_ONE);
    ads1298_write_register(ADS129X_REG_CH8SET, GAIN_ONE);
}

void config_system() {
	int err = 0;

	dk_leds_init();
	k_work_init(&brightness_work, brightness_work_handler);

	spi_init();
    shdn_init();
    set_shdn(true);

	k_sem_give(&ble_init_ok);
	k_timer_init(&mosfet_timer, mosfet_blink_callback, NULL);

	if (!device_is_ready(spi_ad8403)) {
        LOG_ERR("SPI device not ready\n");
        return;
    }
    LOG_INF("SPI device ready\n");

	for (int i = 0; i < NUM_LEDS; i++) {
        if (!device_is_ready(leds[i].port)) {
            LOG_ERR("LED %d device not ready\n", i);
            return;
        }
        gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_INACTIVE);
    }

	uart_init();

	if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED)) {
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err) {
			LOG_ERR("Failed to register authorization callbacks.\n");
			return;
		}
		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err) {
			LOG_ERR("Failed to register authorization info callbacks.\n");
			return;
		}
	}

	bt_enable(NULL);

	LOG_INF("Bluetooth initialized");

	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}

	LOG_INF("Advertising successfully started");
	dev1 = device_get_binding("GPIO_0");
	if (dev1 == NULL) {
		LOG_ERR("Error: cannot find one or more devices\n"); 
		return;
	}
	err = gpio_pin_configure(dev1, 26, GPIO_OUTPUT_ACTIVE | 0);
	if (err < 0) {
		LOG_ERR("Error: One or more leds can't confiigure\n");
	return;
	}

	// initialisation ADS
	LOG_INF("ADS_init : %s\n", ADS1298_init() ? "OK" : "FAIL");
	k_msleep(10);
	// eteindre puis rallumer l'ads
	LOG_INF("ADS1298_power_off\n");
	gpio_pin_set(dev1, 26, 1);
	k_msleep(1000);
	gpio_pin_set(dev1, 26, 0);
	LOG_INF("ADS Powered ON\n");
	k_msleep(1000);
	// Faire appel a la fonction reset_ADS1298() pour faire le reset.
	reset_ADS1298();
	LOG_INF("ADS1298_reset success\n");
	k_usleep(9); // 18 * 514ns
	err = ADS1298_send_wakeup();
	LOG_INF("HERE\n");
	if (err == true){LOG_INF("ADS1298_send_wakeup success\n");}
	else{LOG_INF("ADS1298_send_wakeup failed \n");}
	k_msleep(1000);

	// cette commande est necessaire avant de lire des registre datasheet ADS1298 page 64 
	err = ADS1298_send_stop_read_continuous();
	if (err == true) { LOG_INF("ADS1298_send_stop_read_continuous success\n"); }
	else{LOG_INF("ADS1298_send_stop_read_continuous failed \n");}
	k_msleep(500);

	// lire le registre ID:
	uint8_t regid = ads1298_read_ID();
	LOG_INF("ADS1298_ID %x\n", regid); // ID = 0x92 datasheet page 66
	k_msleep(1000);

	// Configurer ADS1298
	config_ADS1298();
	LOG_INF("ADS1298_config success\n");

	for (int i = 0; i < NUM_BUTTONS; i++) {
        if (!device_is_ready(buttons[i].spec.port)) {
            LOG_ERR("Error: button %d device %s is not ready\n", i, buttons[i].spec.port->name);
            return;
        }

        err = gpio_pin_configure_dt(&buttons[i].spec, GPIO_INPUT);
        if (err != 0) {
            LOG_ERR("Error %d: failed to configure button %d\n", err, i);
            return;
        }

        err = gpio_pin_interrupt_configure_dt(&buttons[i].spec, GPIO_INT_EDGE_TO_ACTIVE);
        if (err != 0) {
            LOG_ERR("Error %d: failed to configure interrupt on button %d\n", err, i);
            return;
        }

        gpio_init_callback(&buttons[i].callback, button_pressed, BIT(buttons[i].spec.pin));
        gpio_add_callback(buttons[i].spec.port, &buttons[i].callback);
        LOG_INF("Set up button %d\n", i);
    }

	current_led = -1;

	LOG_INF("ADS1298 initialized successfully\n");
    LOG_INF("current_led = %d\n", current_led);
    LOG_INF("current_brightness = %d\n", led_brightness_levels[current_led][current_brightness[current_led]]);

	turn_on_led(current_led);
	NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;
}

#define STACK_SIZE 2048
#define SAMPLE_SIZE 16
#define BATCH_SIZE  15
#define HEADER_SIZE 4
#define TOTAL_SIZE (HEADER_SIZE + SAMPLE_SIZE * BATCH_SIZE)

K_THREAD_STACK_DEFINE(data_collect_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(ble_send_stack, STACK_SIZE);

struct k_thread data_collect_thread;
struct k_thread ble_send_thread;
struct k_sem data_ready_sem;
struct k_mutex data_mutex;

uint8_t dummy_batch[TOTAL_SIZE];
uint8_t safe_copy[TOTAL_SIZE];  // 	for sent by BLE

void data_collect_thread_handler(void *arg1, void *arg2, void *arg3) {
    static int collect_counter = 0;
    uint32_t T0 = k_uptime_get_32();

    k_mutex_lock(&data_mutex, K_FOREVER);
    dummy_batch[0] = (T0 >> 24) & 0xFF;
    dummy_batch[1] = (T0 >> 16) & 0xFF;
    dummy_batch[2] = (T0 >> 8) & 0xFF;
    dummy_batch[3] = T0 & 0xFF;
    k_mutex_unlock(&data_mutex);

    while (1) {
        if (is_active) {
            k_msleep(1000);
            continue;
        }

        uint32_t Tnow = k_uptime_get_32();
        uint8_t delta = (uint8_t)(Tnow - T0);
        k_mutex_lock(&data_mutex, K_FOREVER);
        uint8_t *ptr = &dummy_batch[4 + collect_counter * 16];
        k_mutex_unlock(&data_mutex);

        ptr[0] = delta;

        for (int led_idx = 0; led_idx < NUM_LEDS; led_idx++) {
            force_led_on(led_idx);
            ADS1298_send_start();
            k_usleep(2);
            ADS1298_send_read_data();
            k_usleep(2);
            ADS1298_receive_data();

            if (led_idx < 3) {
                ptr[1 + led_idx * 3] = rx_buffer[21];
                ptr[2 + led_idx * 3] = rx_buffer[22];
                ptr[3 + led_idx * 3] = rx_buffer[23];
            } else {
                ptr[10] = rx_buffer[24];
                ptr[11] = rx_buffer[25];
                ptr[12] = rx_buffer[26];
            }

            force_led_off(led_idx);
        }

        ADS1298_send_start();
        k_usleep(2);
        ADS1298_send_read_data();
        k_usleep(2);
        ADS1298_receive_data();
        ptr[13] = rx_buffer[21];
        ptr[14] = rx_buffer[22];
        ptr[15] = rx_buffer[23];

        collect_counter++;

        if (collect_counter >= BATCH_SIZE) {
            collect_counter = 0;

            k_mutex_lock(&data_mutex, K_FOREVER);
            T0 = k_uptime_get_32();
            dummy_batch[0] = (T0 >> 24) & 0xFF;
            dummy_batch[1] = (T0 >> 16) & 0xFF;
            dummy_batch[2] = (T0 >> 8) & 0xFF;
            dummy_batch[3] = T0 & 0xFF;
            k_mutex_unlock(&data_mutex);

            k_sem_give(&data_ready_sem);
        }
    }
}

void ble_send_thread_handler(void *arg1, void *arg2, void *arg3) {
    int sent_ok = 0, sent_fail = 0;

    while (1) {
        k_sem_take(&data_ready_sem, K_FOREVER);

        if (current_conn) {
            k_mutex_lock(&data_mutex, K_FOREVER);
            memcpy(safe_copy, dummy_batch, TOTAL_SIZE);
            k_mutex_unlock(&data_mutex);

            int err = bt_nus_send(current_conn, safe_copy, sizeof(safe_copy));
            if (err) {
                sent_fail++;
                LOG_WRN("BLE send failed (%d) | Fail count: %d", err, sent_fail);
            } else {
                sent_ok++;
                LOG_INF("BLE sent OK | OK: %d, Fail: %d", sent_ok, sent_fail);
            }
        }
    }
}

int main(void) {
    int blink_status = 0;
    config_system();
    k_sem_init(&data_ready_sem, 0, 1);
    k_mutex_init(&data_mutex);

    k_thread_create(&data_collect_thread, data_collect_stack, STACK_SIZE,
        data_collect_thread_handler, NULL, NULL, NULL, 1, 0, K_NO_WAIT);

    k_thread_create(&ble_send_thread, ble_send_stack, STACK_SIZE,
        ble_send_thread_handler, NULL, NULL, NULL, 1, 0, K_NO_WAIT);

    while (1) {
        dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
        k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
    }
    return 0;
}