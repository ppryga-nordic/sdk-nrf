/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Central Power Benchmark
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/scan.h>

#if defined(CONFIG_CONNECTION_PHY_1M)
#define PHY_CONFIG BT_GAP_LE_PHY_1M
#elif defined(CONFIG_CONNECTION_PHY_2M)
#define PHY_CONFIG BT_GAP_LE_PHY_2M
#else
#error "Unsupported PHY"
#endif /* CONFIG_CONNECTION_PHY_1M */

static struct bt_conn *default_conn;
static void change_phy(struct bt_conn *conn);

static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match, bool connectable)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN];
	struct bt_conn_le_create_param conn_create_params = BT_CONN_LE_CREATE_PARAM_INIT(
		BT_CONN_LE_OPT_NONE, BT_GAP_INIT_CONN_INT_MIN, BT_GAP_INIT_CONN_INT_MIN);
	struct bt_le_conn_param conn_params =
		BT_LE_CONN_PARAM_INIT(CONFIG_CONNECTION_INTERVAL, CONFIG_CONNECTION_INTERVAL, 0,
				      BT_GAP_MS_TO_CONN_TIMEOUT(4000));

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

	printk("Filters matched. Address: %s connectable: %s\n", addr, connectable ? "yes" : "no");

	err = bt_scan_stop();
	if (err) {
		printk("Stop LE scan failed (err %d)\n", err);
	}

	err = bt_conn_le_create(device_info->recv_info->addr, &conn_create_params, &conn_params,
				&default_conn);
	if (err) {
		printk("Create conn failed (err %d)\n", err);

		err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
		if (err) {
			printk("Scanning failed to start (err %d)\n", err);
			return;
		}
	}

	printk("Connection pending\n");
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL, NULL, NULL);

static void scan_init(void)
{
	int err;

	/* Use active scanning and disable duplicate filtering to handle any
	 * devices that might update their advertising data at runtime. */
	struct bt_le_scan_param scan_param = {.type = BT_LE_SCAN_TYPE_ACTIVE,
					      .interval = BT_GAP_SCAN_FAST_INTERVAL,
					      .window = BT_GAP_SCAN_FAST_WINDOW,
					      .options =
						      BT_LE_SCAN_OPT_CODED | BT_LE_SCAN_OPT_NO_1M};

	struct bt_scan_init_param scan_init = {
		.connect_if_match = 0, .scan_param = &scan_param, .conn_param = NULL};

	bt_scan_init(&scan_init);
	bt_scan_cb_register(&scan_cb);

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_NAME, CONFIG_PERIPHERAL_ADV_NAME);
	if (err) {
		printk("Scanning filters cannot be set (err %d)\n", err);
		return;
	}

	err = bt_scan_filter_enable(BT_SCAN_NAME_FILTER, false);
	if (err) {
		printk("Filters cannot be turned on (err %d)\n", err);
	}
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		printk("Failed to connect to %s, 0x%02x %s\n", addr, conn_err,
		       bt_hci_err_to_str(conn_err));

		bt_conn_unref(default_conn);
		default_conn = NULL;

		err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
		if (err) {
			printk("Scanning failed to start (err %d)\n", err);
		}

		return;
	}

	printk("Connected: %s\n", addr);

	change_phy(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s, reason 0x%02x %s\n", addr, reason, bt_hci_err_to_str(reason));

	if (default_conn != conn) {
		return;
	}

	bt_conn_unref(default_conn);
	default_conn = NULL;

	err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
	}
}

static void change_phy(struct bt_conn *conn)
{
	int err;
	const struct bt_conn_le_phy_param preferred_phy = {
		.options = BT_CONN_LE_PHY_OPT_NONE,
		.pref_rx_phy = PHY_CONFIG,
		.pref_tx_phy = PHY_CONFIG,
	};

	printk("Change PHY to: %d\n", PHY_CONFIG);
	err = bt_conn_le_phy_update(conn, &preferred_phy);
	if (err) {
		printk("bt_conn_le_phy_update() returned %d", err);
	}
}

void le_param_updated_cb(struct bt_conn *conn, uint16_t interval, uint16_t latency,
			 uint16_t timeout)
{
	printk("Connection parameters update: \n");
	printk("\t # interval: %d us\n", interval * 1250);
	printk("\t # latency: %d events\n", latency);
	printk("\t # supervision tiemout: %d us\n", timeout * 10);
}

void le_phy_updated_cb(struct bt_conn *conn, struct bt_conn_le_phy_info *param)
{
	printk("PHY updated: ");
	switch (param->tx_phy) {
	case BT_CONN_LE_TX_POWER_PHY_1M:
		printk("PHY: 1M\n");
		break;
	case BT_CONN_LE_TX_POWER_PHY_2M:
		printk("PHY: 2M\n");
		break;
	case BT_CONN_LE_TX_POWER_PHY_CODED_S8:
		printk("PHY: S8\n");
		break;
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_updated = le_param_updated_cb,
	.le_phy_updated = le_phy_updated_cb,
};

int main(void)
{
	int err;

	printk("Starting Bluetooth Central HR coded sample\n");

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	scan_init();

	err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return 0;
	}

	printk("Scanning successfully started\n");
	return 0;
}
