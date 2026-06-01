/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/**
 * @file pairing_mode.c
 * @brief Implementation of runtime SMP pairing method selection.
 *
 * Mode → mechanism mapping
 * ------------------------
 * | Mode (shell name)   | SMP pairing method      | auth_cb used   | OOB flags |
 * |---------------------|-------------------------|----------------|-----------|
 * | lesc_jw             | LE SC Just Works        | auth_cb_none   | off       |
 * | lesc_oob            | LE SC OOB               | auth_cb_oob    | SC on     |
 * | legacy_jw           | Legacy Just Works       | auth_cb_none   | off       |
 * | legacy_oob          | Legacy OOB              | auth_cb_oob    | legacy on |
 * | lesc_numeric        | LE SC Numeric Comparison| auth_cb_yesno  | off       |
 * | lesc_pk_display     | LE SC Passkey (display) | auth_cb_display| off       |
 * | lesc_pk_input       | LE SC Passkey (entry)   | auth_cb_input  | off       |
 *
 * pairing_accept() adds policy: e.g. lesc_jw rejects a peer that advertises OOB
 * or Legacy-only, so we do not fall back to a different method than selected.
 *
 * Pairing flow (typical)
 * ----------------------
 * 1. pairing_mode_set() / shell "pairing set" → register auth_cb, apply flags,
 *    refresh NFC NDEF (main.c).
 * 2. Peer connects; SMP Pairing Request received.
 * 3. pairing_accept() checks peer io_capability, oob_data_flag, auth_req.
 * 4. Host builds Pairing Response (IO from auth_cb, OOB from bt_le_oob_set_*).
 * 5. Stack picks method (Just Works / OOB / passkey / numeric) from both sides.
 * 6. For OOB/passkey/numeric, auth callbacks run; shell confirm/passkey completes.
 * 7. pairing_complete in main.c → pairing_mode_apply() restores configured flags.
 */

#include "pairing_mode.h"

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include <zephyr/bluetooth/conn.h>

/** AuthReq LE Secure Connections bit in Pairing Request/Response (Core Spec H). */
#define AUTH_SC_FLAG 0x08

/* Connection that is waiting for shell pairing confirm / passkey. */

static const struct pairing_mode_oob_ops *oob_ops;
static enum pairing_mode_id current_mode = PAIRING_MODE_LESC_OOB;
static struct bt_conn *pairing_conn;
static int (*nfc_refresh_fn)(void);

static void pairing_conn_set(struct bt_conn *conn)
{
	if (pairing_conn) {
		bt_conn_unref(pairing_conn);
		pairing_conn = NULL;
	}

	if (conn) {
		pairing_conn = bt_conn_ref(conn);
	}
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Pairing cancelled: %s\n", addr);
	pairing_conn_set(NULL);
}

static void auth_addr_print(struct bt_conn *conn, char *addr, size_t len)
{
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, len);
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char passkey_str[7];
	char addr[BT_ADDR_LE_STR_LEN];

	snprintk(passkey_str, sizeof(passkey_str), "%06u", passkey);
	auth_addr_print(conn, addr, sizeof(addr));
	printk("Passkey for %s: %s (use: pairing confirm)\n", addr, passkey_str);
	pairing_conn_set(conn);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char passkey_str[7];
	char addr[BT_ADDR_LE_STR_LEN];

	snprintk(passkey_str, sizeof(passkey_str), "%06u", passkey);
	auth_addr_print(conn, addr, sizeof(addr));
	printk("Confirm passkey for %s: %s (use: pairing confirm)\n", addr, passkey_str);
	pairing_conn_set(conn);
}

static void auth_passkey_entry(struct bt_conn *conn)
{
	printk("Enter passkey (use: pairing passkey <000000-999999>)\n");
	pairing_conn_set(conn);
}

static void auth_pairing_confirm(struct bt_conn *conn)
{
	printk("Confirm Just Works pairing (use: pairing confirm)\n");
	pairing_conn_set(conn);
}

/*
 * Invoked by the SMP stack when the negotiated method needs OOB data.
 * Only lesc_oob / legacy_oob modes register oob_data_request; other modes
 * cancel here if the peer somehow triggered OOB anyway.
 */
static void auth_oob_data_request(struct bt_conn *conn, struct bt_conn_oob_info *info)
{
	if (!oob_ops) {
		bt_conn_auth_cancel(conn);
		return;
	}

	switch (current_mode) {
	case PAIRING_MODE_LESC_OOB:
		if (info->type == BT_CONN_OOB_LE_SC && oob_ops->lesc_oob_set) {
			printk("LESC OOB data requested\n");
			oob_ops->lesc_oob_set(conn, info);
		} else {
			bt_conn_auth_cancel(conn);
		}
		break;
	case PAIRING_MODE_LEGACY_OOB:
		if (info->type == BT_CONN_OOB_LE_LEGACY && oob_ops->legacy_tk_set) {
			printk("Legacy TK value requested\n");
			oob_ops->legacy_tk_set(conn);
		} else {
			bt_conn_auth_cancel(conn);
		}
		break;
	default:
		bt_conn_auth_cancel(conn);
		break;
	}
}

/*
 * Called for each incoming Pairing Request before we send Pairing Response.
 * Filters peer features so the negotiated method stays within the selected mode.
 * Does not override IO capability (that comes from the registered auth_cb).
 */
static enum bt_security_err pairing_accept(struct bt_conn *conn,
					   const struct bt_conn_pairing_feat *const feat)
{
	bool peer_sc = feat->auth_req & AUTH_SC_FLAG;
	bool peer_oob = feat->oob_data_flag;

	ARG_UNUSED(conn);

	switch (current_mode) {
	case PAIRING_MODE_LESC_JUST_WORKS:
		if (!peer_sc) {
			return BT_SECURITY_ERR_AUTH_REQUIREMENT;
		}
		if (peer_oob) {
			return BT_SECURITY_ERR_AUTH_REQUIREMENT;
		}
		break;
	case PAIRING_MODE_LESC_OOB:
		if (!peer_sc) {
			return BT_SECURITY_ERR_AUTH_REQUIREMENT;
		}
		/*
		 * Note: bt_le_oob_set_sc_flag() here is too late — Zephyr already
		 * copied sc_oobd_present into Pairing Response before pairing_accept().
		 * SMP OOB flag must be correct via pairing_mode_apply() before pairing.
		 */
		printk("lesc_oob: peer OOB flag %u (have remote SC OOB: %u)\n", peer_oob,
		       (oob_ops && oob_ops->remote_sc_oob_ready &&
			oob_ops->remote_sc_oob_ready()) ?
			       1U :
			       0U);
		break;
	case PAIRING_MODE_LEGACY_JUST_WORKS:
		if (peer_sc) {
			return BT_SECURITY_ERR_AUTH_REQUIREMENT;
		}
		if (peer_oob) {
			return BT_SECURITY_ERR_AUTH_REQUIREMENT;
		}
		break;
	case PAIRING_MODE_LEGACY_OOB:
		if (peer_sc) {
			return BT_SECURITY_ERR_AUTH_REQUIREMENT;
		}
		/* Peer may set OOB in Pairing Request; allow legacy TK exchange. */
		if (peer_oob) {
			bt_le_oob_set_legacy_flag(true);
		}
		break;
	case PAIRING_MODE_LESC_NUMERIC_COMP:
	case PAIRING_MODE_LESC_PASSKEY_INPUT:
	case PAIRING_MODE_LESC_PASSKEY_DISPLAY:
		if (!peer_sc) {
			return BT_SECURITY_ERR_AUTH_REQUIREMENT;
		}
		if (peer_oob) {
			return BT_SECURITY_ERR_AUTH_REQUIREMENT;
		}
		break;
	default:
		break;
	}

	return BT_SECURITY_ERR_SUCCESS;
}

/*
 * Auth callback sets — each non-NULL handler changes our advertised IO capability.
 * See auth_cb_for_mode() for which set is registered per pairing_mode_id.
 */
static struct bt_conn_auth_cb auth_cb_none = {
	.cancel = auth_cancel,
	.pairing_accept = pairing_accept,
	/* No passkey/OOB → BT_SMP_IO_NO_INPUT_OUTPUT → Just Works (if no OOB flag). */
};

static struct bt_conn_auth_cb auth_cb_oob = {
	.cancel = auth_cancel,
	.oob_data_request = auth_oob_data_request,
	.pairing_accept = pairing_accept,
	/* oob_data_request → main.c supplies SC or legacy TK. */
};

static struct bt_conn_auth_cb auth_cb_yesno = {
	.cancel = auth_cancel,
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.pairing_accept = pairing_accept,
	/* Display + confirm → BT_SMP_IO_DISPLAY_YESNO → Numeric Comparison. */
};

static struct bt_conn_auth_cb auth_cb_display = {
	.cancel = auth_cancel,
	.passkey_display = auth_passkey_display,
	.pairing_confirm = auth_pairing_confirm,
	.pairing_accept = pairing_accept,
	/* Display only → peripheral shows passkey; central enters it. */
};

static struct bt_conn_auth_cb auth_cb_input = {
	.cancel = auth_cancel,
	.passkey_entry = auth_passkey_entry,
	.pairing_confirm = auth_pairing_confirm,
	.pairing_accept = pairing_accept,
	/* Keyboard only → peripheral enters passkey via shell. */
};

/* Zephyr allows only one global auth_cb; unregister before registering another. */
static int auth_cb_register(const struct bt_conn_auth_cb *cb)
{
	int err;

	err = bt_conn_auth_cb_register(NULL);
	if (err) {
		return err;
	}

	return bt_conn_auth_cb_register(cb);
}

/** Pick auth_cb_* so Zephyr advertises the IO caps for the target SMP method. */
static const struct bt_conn_auth_cb *auth_cb_for_mode(enum pairing_mode_id mode)
{
	switch (mode) {
	case PAIRING_MODE_LESC_JUST_WORKS:
	case PAIRING_MODE_LEGACY_JUST_WORKS:
		return &auth_cb_none;
	case PAIRING_MODE_LESC_OOB:
	case PAIRING_MODE_LEGACY_OOB:
		return &auth_cb_oob;
	case PAIRING_MODE_LESC_NUMERIC_COMP:
		return &auth_cb_yesno;
	case PAIRING_MODE_LESC_PASSKEY_DISPLAY:
		return &auth_cb_display;
	case PAIRING_MODE_LESC_PASSKEY_INPUT:
		return &auth_cb_input;
	default:
		return NULL;
	}
}

/*
 * Updates global SMP OOB-present flags read when building Pairing Response
 * (see sc_oobd_present / legacy_oobd_present in smp.c).
 */
void pairing_mode_apply(void)
{
	if (current_mode == PAIRING_MODE_LESC_OOB) {
		/*
		 * SMP OOB bit in Pairing Response: set only when we have the central's
		 * SC OOB (TNEP). If false while the phone read our NFC tag, the phone
		 * still uses our NDEF OOB; SMP uses LOCAL_ONLY (we supply local SC OOB).
		 */
		const bool remote_ready = oob_ops && oob_ops->remote_sc_oob_ready &&
					  oob_ops->remote_sc_oob_ready();

		bt_le_oob_set_sc_flag(remote_ready);
		bt_le_oob_set_legacy_flag(false);
	} else {
		bt_le_oob_set_sc_flag(false);
		bt_le_oob_set_legacy_flag(pairing_mode_nfc_includes_legacy_oob());
	}
}

bool pairing_mode_nfc_includes_sc_oob(void)
{
	return current_mode == PAIRING_MODE_LESC_OOB;
}

bool pairing_mode_nfc_includes_legacy_oob(void)
{
	return current_mode == PAIRING_MODE_LEGACY_OOB;
}

bool pairing_mode_accepts_remote_oob(void)
{
	return current_mode == PAIRING_MODE_LESC_OOB ||
	       current_mode == PAIRING_MODE_LEGACY_OOB;
}

void pairing_mode_fill_oob_rec(struct nfc_ndef_le_oob_rec_payload_desc *rec,
			       struct bt_le_oob_sc_data *le_sc_data,
			       uint8_t *tk_value)
{
	rec->le_sc_data = pairing_mode_nfc_includes_sc_oob() ? le_sc_data : NULL;
	rec->tk_value = pairing_mode_nfc_includes_legacy_oob() ? tk_value : NULL;
}

enum pairing_mode_id pairing_mode_get(void)
{
	return current_mode;
}

int pairing_mode_set(enum pairing_mode_id mode, int (*nfc_refresh)(void))
{
	const struct bt_conn_auth_cb *cb;
	int err;

	if (mode >= PAIRING_MODE_COUNT) {
		return -EINVAL;
	}

	cb = auth_cb_for_mode(mode);
	if (!cb) {
		return -EINVAL;
	}

	err = auth_cb_register(cb);
	if (err) {
		return err;
	}

	current_mode = mode;
	nfc_refresh_fn = nfc_refresh;
	pairing_mode_apply();

	if (nfc_refresh_fn) {
		err = nfc_refresh_fn();
		if (err) {
			printk("NFC NDEF refresh failed: %d\n", err);
		}
	}

	printk("Pairing mode: %s\n", pairing_mode_name(mode));

	return 0;
}

static const char *const mode_names[] = {
	[PAIRING_MODE_LESC_JUST_WORKS] = "lesc_jw",
	[PAIRING_MODE_LESC_OOB] = "lesc_oob",
	[PAIRING_MODE_LEGACY_JUST_WORKS] = "legacy_jw",
	[PAIRING_MODE_LEGACY_OOB] = "legacy_oob",
	[PAIRING_MODE_LESC_NUMERIC_COMP] = "lesc_numeric",
	[PAIRING_MODE_LESC_PASSKEY_INPUT] = "lesc_pk_input",
	[PAIRING_MODE_LESC_PASSKEY_DISPLAY] = "lesc_pk_display",
};

const char *pairing_mode_name(enum pairing_mode_id mode)
{
	if (mode >= PAIRING_MODE_COUNT) {
		return "unknown";
	}

	return mode_names[mode];
}

int pairing_mode_init(const struct pairing_mode_oob_ops *ops)
{
	if (!ops || !ops->lesc_oob_set || !ops->legacy_tk_set ||
	    !ops->remote_sc_oob_matches || !ops->remote_sc_oob_ready) {
		return -EINVAL;
	}

	oob_ops = ops;

	return pairing_mode_set(PAIRING_MODE_LESC_OOB, NULL);
}

static int pairing_mode_from_name(const char *name, enum pairing_mode_id *mode)
{
	for (enum pairing_mode_id i = 0; i < PAIRING_MODE_COUNT; i++) {
		if (!strcmp(name, mode_names[i])) {
			*mode = i;
			return 0;
		}
	}

	return -EINVAL;
}

#if defined(CONFIG_SHELL)
/* Shell: pairing list | show | set <mode> | confirm | passkey <pin> */

static int cmd_pairing_list(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	for (enum pairing_mode_id i = 0; i < PAIRING_MODE_COUNT; i++) {
		shell_print(sh, "  %s", mode_names[i]);
	}

	return 0;
}

static int cmd_pairing_show(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "%s", mode_names[current_mode]);
	return 0;
}

extern int pairing_mode_nfc_refresh(void);

static int cmd_pairing_set(const struct shell *sh, size_t argc, char **argv)
{
	enum pairing_mode_id mode;
	int err;

	if (argc != 2) {
		shell_error(sh, "Usage: pairing set <mode>");
		return -EINVAL;
	}

	err = pairing_mode_from_name(argv[1], &mode);
	if (err) {
		shell_error(sh, "Unknown mode '%s'. Use: pairing list", argv[1]);
		return err;
	}

	err = pairing_mode_set(mode, pairing_mode_nfc_refresh);
	if (err) {
		shell_error(sh, "Failed to set mode (%d)", err);
	}

	return err;
}

bool pairing_mode_confirm_pending(void)
{
	return pairing_conn != NULL;
}

int pairing_mode_user_confirm(void)
{
	int err;

	if (!pairing_conn) {
		return -ENOENT;
	}

	/* Numeric Comparison / passkey confirm path. */
	err = bt_conn_auth_passkey_confirm(pairing_conn);
	if (!err) {
		return 0;
	}

	/* Just Works confirmation path (if stack requested pairing_confirm). */
	return bt_conn_auth_pairing_confirm(pairing_conn);
}

static int cmd_pairing_confirm(const struct shell *sh, size_t argc, char **argv)
{
	int err;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	err = pairing_mode_user_confirm();
	if (err == -ENOENT) {
		shell_error(sh, "No pairing in progress");
	} else if (err) {
		shell_error(sh, "Confirm failed (%d)", err);
	}

	return err;
}

static int cmd_pairing_passkey(const struct shell *sh, size_t argc, char **argv)
{
	unsigned int passkey;
	int err;

	if (argc != 2) {
		shell_error(sh, "Usage: pairing passkey <000000-999999>");
		return -EINVAL;
	}

	if (!pairing_conn) {
		shell_error(sh, "No pairing in progress");
		return -ENOENT;
	}

	passkey = shell_strtoul(argv[1], 0, &err);
	if (err || passkey > 999999U) {
		shell_error(sh, "Passkey must be 0-999999");
		return -EINVAL;
	}

	err = bt_conn_auth_passkey_entry(pairing_conn, passkey);
	if (err) {
		shell_error(sh, "Passkey entry failed (%d)", err);
	}

	return err;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	pairing_cmds,
	SHELL_CMD(list, NULL, "List pairing modes", cmd_pairing_list),
	SHELL_CMD(show, NULL, "Show active pairing mode", cmd_pairing_show),
	SHELL_CMD(set, NULL, "Set pairing mode: pairing set <mode>", cmd_pairing_set),
	SHELL_CMD(confirm, NULL, "Confirm passkey or Just Works pairing", cmd_pairing_confirm),
	SHELL_CMD(passkey, NULL, "Enter passkey: pairing passkey <pin>", cmd_pairing_passkey),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(pairing, &pairing_cmds, "Bluetooth pairing mode control", NULL);

void pairing_mode_shell_init(void)
{
	/* Commands registered via SHELL_CMD_REGISTER. */
}

#else /* !CONFIG_SHELL */

void pairing_mode_shell_init(void)
{
}

#endif /* CONFIG_SHELL */
