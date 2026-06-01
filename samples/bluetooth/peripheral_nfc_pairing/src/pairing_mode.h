/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/**
 * @file pairing_mode.h
 * @brief Runtime selection of Bluetooth LE SMP pairing methods.
 *
 * A single pairing "mode" is kept until the application changes it (shell:
 * ``pairing set <mode>``). Each mode maps to one of the standard SMP pairing
 * methods defined in Bluetooth Core Specification Vol. 3, Part H (e.g. Just
 * Works, Passkey Entry, Numeric Comparison, OOB).
 *
 * The Zephyr host does not expose a direct "set pairing method" API. The final
 * method is negotiated from Pairing Request/Response fields (IO capability,
 * OOB data flag, AuthReq) on both peers. This module steers that negotiation
 * through three coordinated mechanisms:
 *
 * 1. **IO capability** — Which @ref bt_conn_auth_cb handlers are registered.
 *    Zephyr derives IO capability from non-NULL callbacks (see get_io_capa()
 *    in subsys/bluetooth/host/smp.c). Different auth_cb_* structures are
 *    swapped in pairing_mode_set() so the peripheral advertises the IO caps
 *    that match the desired method (e.g. DisplayYesNo → Numeric Comparison).
 *
 * 2. **SMP OOB flags** — bt_le_oob_set_sc_flag() / bt_le_oob_set_legacy_flag()
 *    control the OOB bit in our Pairing Response. If either peer sets OOB,
 *    the stack selects LE SC OOB or Legacy OOB instead of Just Works.
 *    pairing_mode_apply() sets these flags from the active mode.
 *
 * 3. **NFC Connection Handover** — pairing_mode_fill_oob_rec() and
 *    pairing_mode_nfc_includes_*() decide whether the static/TNEP NDEF LE OOB
 *    record includes le_sc_data and/or legacy TK. That tells the NFC poller
 *    what to expect before BLE connects. main.c calls pairing_mode_nfc_refresh()
 *    after a mode change to rebuild the tag message.
 *
 * During an active pairing, optional callbacks supply secrets when the stack
 * requests them (oob_data_request → main.c lesc_oob_data_set / legacy_tk_set),
 * or prompt via the shell (passkey_display / passkey_confirm / passkey_entry).
 * pairing_accept() rejects incompatible peer features for the selected mode.
 *
 * See pairing_mode.c for the mode → auth_cb / SMP / NFC mapping table.
 */

#ifndef PAIRING_MODE_H_
#define PAIRING_MODE_H_

#include <nfc/ndef/le_oob_rec.h>
#include <stdbool.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>

/**
 * Application-selected pairing policy.
 *
 * Names align with shell commands (pairing set <name>). Each value targets a
 * specific SMP pairing method when the peer supports a compatible feature set.
 */
enum pairing_mode_id {
	/**
	 * LE Secure Connections + Just Works.
	 * SMP: No OOB flags; IO = NoInputNoOutput (auth_cb_none).
	 * NFC: Address/name only, no OOB fields in NDEF.
	 */
	PAIRING_MODE_LESC_JUST_WORKS = 0,
	/**
	 * LE Secure Connections + OOB (typical NFC touch-to-pair).
	 * SMP: sc_oobd_present; oob_data_request supplies SC OOB data.
	 * NFC: le_sc_data in LE OOB record; TNEP may exchange remote OOB.
	 */
	PAIRING_MODE_LESC_OOB,
	/**
	 * Legacy pairing + Just Works (no LE SC).
	 * SMP: No OOB, no SC; pairing_accept rejects peer SC.
	 */
	PAIRING_MODE_LEGACY_JUST_WORKS,
	/**
	 * Legacy pairing + OOB (16-byte TK over NFC).
	 * SMP: legacy_oobd_present; legacy TK via oob_data_request.
	 * NFC: tk_value in LE OOB record.
	 */
	PAIRING_MODE_LEGACY_OOB,
	/**
	 * LE Secure Connections + Numeric Comparison.
	 * SMP: IO = DisplayYesNo (passkey_display + passkey_confirm).
	 * User confirms matching 6-digit value: pairing confirm.
	 */
	PAIRING_MODE_LESC_NUMERIC_COMP,
	/**
	 * LE Secure Connections + Passkey Entry (this device inputs PIN).
	 * SMP: IO = KeyboardOnly (passkey_entry).
	 * User enters PIN: pairing passkey <pin>.
	 */
	PAIRING_MODE_LESC_PASSKEY_INPUT,
	/**
	 * LE Secure Connections + Passkey Entry (this device displays PIN).
	 * SMP: IO = DisplayOnly (passkey_display).
	 */
	PAIRING_MODE_LESC_PASSKEY_DISPLAY,
	PAIRING_MODE_COUNT,
};

/**
 * OOB secret handlers implemented in main.c (NFC sample).
 *
 * Called from auth_oob_data_request() when the stack starts LE SC OOB or
 * Legacy OOB pairing and needs local/remote OOB material.
 */
struct pairing_mode_oob_ops {
	void (*lesc_oob_set)(struct bt_conn *conn, struct bt_conn_oob_info *oob_info);
	void (*legacy_tk_set)(struct bt_conn *conn);
	/**
	 * @brief True if LE SC OOB data for @p conn's peer was received (e.g. over TNEP).
	 */
	bool (*remote_sc_oob_matches)(struct bt_conn *conn);
	/** @brief True if any peer SC OOB is stored (controls SMP OOB flag before pairing). */
	bool (*remote_sc_oob_ready)(void);
};

/** Register OOB ops and select default mode (PAIRING_MODE_LESC_OOB). */
int pairing_mode_init(const struct pairing_mode_oob_ops *ops);

/** Return the mode last applied with pairing_mode_set(). */
enum pairing_mode_id pairing_mode_get(void);

/**
 * Select pairing mode (persists until the next pairing_mode_set()).
 *
 * Registers matching auth callbacks, updates SMP OOB flags, and optionally
 * rebuilds the NFC NDEF via @p nfc_refresh (use pairing_mode_nfc_refresh).
 */
int pairing_mode_set(enum pairing_mode_id mode, int (*nfc_refresh)(void));

/**
 * Set LE OOB NDEF fields per active mode.
 *
 * Called from tnep_initial_msg_encode() and carrier_prepare() in main.c.
 * Passing NULL for le_sc_data/tk_value omits that carrier from the NFC tag.
 */
void pairing_mode_fill_oob_rec(struct nfc_ndef_le_oob_rec_payload_desc *rec,
			       struct bt_le_oob_sc_data *le_sc_data,
			       uint8_t *tk_value);

/** Shell/log name for @p mode (e.g. "lesc_oob"). */
const char *pairing_mode_name(enum pairing_mode_id mode);

/** Regenerate OOB keys and rebuild initial TNEP NDEF (defined in main.c). */
int pairing_mode_nfc_refresh(void);

/**
 * Restore SMP OOB flags after pairing completes or fails.
 *
 * main.c calls this instead of clearing flags so the configured mode stays
 * in effect for the next connection.
 */
void pairing_mode_apply(void);

/** True when NDEF should contain le_sc_data (lesc_oob mode). */
bool pairing_mode_nfc_includes_sc_oob(void);

/** True when NDEF should contain tk_value (legacy_oob mode). */
bool pairing_mode_nfc_includes_legacy_oob(void);

/**
 * True when TNEP/static handover may set bt_le_oob_set_*_flag from peer NDEF.
 *
 * main.c oob_le_data_handle() ignores remote OOB unless this returns true.
 */
bool pairing_mode_accepts_remote_oob(void);

/** No-op unless CONFIG_SHELL (commands use SHELL_CMD_REGISTER). */
void pairing_mode_shell_init(void);

/**
 * @brief Confirm an in-progress pairing (passkey, numeric comparison, Just Works).
 *
 * Same as shell ``pairing confirm``. Intended for DK button handling in main.c.
 *
 * @retval 0 Success
 * @retval -ENOENT No pairing awaiting confirmation
 * @retval Negative Other error from the Bluetooth API
 */
int pairing_mode_user_confirm(void);

/** @brief True when a pairing is waiting for pairing_mode_user_confirm(). */
bool pairing_mode_confirm_pending(void);

#endif /* PAIRING_MODE_H_ */
