/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Peripheral Heart Rate over LE Coded PHY sample
 */
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>

// Short name to be used for tests of a connection
#define DEVICE_NAME	"nRF-HRS"
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

static const struct bt_data ad[] = {
#if defined(CONFIG_TEST_PERIPHERAL_CONNECTION)
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN)
#endif /* CONFIG_TEST_PERIPHERAL_CONNECTION */
};

static struct bt_le_adv_param m_adv_param = BT_LE_ADV_PARAM_INIT(
#if defined(CONFIG_TEST_PERIPHERAL_CONNECTION)
	BT_LE_ADV_OPT_CONN,
#else
	BT_LE_ADV_OPT_NONE,
#endif /* CONFIG_TEST_PERIPHERAL_CONNECTION */
	CONFIG_ADVERTISING_INTERVAL, CONFIG_ADVERTISING_INTERVAL, NULL);

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		printk("Connection failed, err 0x%02x %s\n", conn_err, bt_hci_err_to_str(conn_err));
		return;
	}

	printk("Connected to: %s\n", addr);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	int err;

	printk("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));

	err = bt_le_adv_start(&m_adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising restart failed to start (err %d)\n", err);
		return;
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

int main(void)
{
	int err;

	printk("Starting Bluetooth Peripheral Power Benchmark\n");

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	err = bt_le_adv_start(&m_adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return 0;
	}

	printk("Advertising started\n");

	for (;;) {
		k_sleep(K_FOREVER);
	}
}