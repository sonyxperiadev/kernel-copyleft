// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "btfm_codec.h"
#include "btfm_codec_pkt.h"
#include "btfm_codec_btadv_interface.h"

void btfmcodec_initiate_hwep_shutdown(struct btfmcodec_char_device *btfmcodec_dev)
{
	wait_queue_head_t *rsp_wait_q =
		&btfmcodec_dev->rsp_wait_q[BTM_PKT_TYPE_HWEP_SHUTDOWN];
	int ret;
	uint8_t *status = &btfmcodec_dev->status[BTM_PKT_TYPE_HWEP_SHUTDOWN];

	*status = BTM_WAITING_RSP;
	BTFMCODEC_INFO("queuing work to shutdown");
	schedule_work(&btfmcodec_dev->wq_hwep_shutdown);
	BTFMCODEC_INFO("waiting here for shutdown");
	ret = wait_event_interruptible_timeout(*rsp_wait_q,
		*status != BTM_WAITING_RSP,
		msecs_to_jiffies(BTM_MASTER_CONFIG_RSP_TIMEOUT));

	/* Rethink of having a new packet to notify transport switch error */
	if (ret == 0) {
		BTFMCODEC_ERR("failed to recevie to complete hwep_shutdown");
		flush_work(&btfmcodec_dev->wq_hwep_shutdown);
	} else {
		if (*status == BTM_RSP_RECV) {
			BTFMCODEC_ERR("sucessfully closed hwep");
			return;
		} else if (*status == BTM_FAIL_RESP_RECV ||
			   *status == BTM_RSP_NOT_RECV_CLIENT_KILLED) {
			BTFMCODEC_ERR("Failed to close hwep");
			return;
		}
	}
}

void btfmcodec_move_to_next_state(struct btfmcodec_state_machine *state)
{
	mutex_lock(&state->state_machine_lock);
	if (state->current_state == BT_Connecting ||
	    state->current_state == BTADV_AUDIO_Connecting) {
		state->current_state += 1;
		BTFMCODEC_INFO("moving from %s to %s",
				coverttostring(state->current_state -1 ),
				coverttostring(state->current_state));
	} else {
		BTFMCODEC_ERR("State machine might have gone bad. reseting to default");
		state->current_state = IDLE;
	}

	state->prev_state = IDLE;
	mutex_unlock(&state->state_machine_lock);
}

void btfmcodec_revert_current_state(struct btfmcodec_state_machine *state)
{
	mutex_lock(&state->state_machine_lock);
	BTFMCODEC_INFO("reverting from %s to %s", coverttostring(state->current_state),
					coverttostring(state->prev_state));
	state->current_state = state->prev_state;
	state->prev_state = IDLE;
	mutex_unlock(&state->state_machine_lock);
}

void btfmcodec_set_current_state(struct btfmcodec_state_machine *state,
		btfmcodec_state current_state)
{
	mutex_lock(&state->state_machine_lock);
	BTFMCODEC_INFO("moving from %s to %s", coverttostring(state->current_state),
					coverttostring(current_state));
	state->prev_state = state->current_state;
	state->current_state = current_state;
	mutex_unlock(&state->state_machine_lock);
}

btfmcodec_state btfmcodec_get_current_transport(struct
					btfmcodec_state_machine *state)
{
	btfmcodec_state current_state;

	mutex_lock(&state->state_machine_lock);
	current_state = state->current_state;
	mutex_unlock(&state->state_machine_lock);
	return current_state;
}

int btfmcodec_frame_transport_switch_ind_pkt(struct btfmcodec_char_device *btfmcodec_dev,
					uint8_t active_transport,
					uint8_t status)
{
	struct btm_ctrl_pkt rsp;

	rsp.opcode = BTM_BTFMCODEC_TRANSPORT_SWITCH_FAILED_IND;
	rsp.len = BTM_PREPARE_AUDIO_BEARER_SWITCH_RSP_LEN;
	rsp.active_transport = active_transport;
	rsp.status = status;
	return btfmcodec_dev_enqueue_pkt(btfmcodec_dev, &rsp, (rsp.len +
					BTM_HEADER_LEN));
}

int btfmcodec_frame_prepare_bearer_rsp_pkt(struct btfmcodec_char_device *btfmcodec_dev,
					uint8_t active_transport,
					uint8_t status)
{
	struct btm_ctrl_pkt rsp;

	rsp.opcode = BTM_BTFMCODEC_PREPARE_AUDIO_BEARER_SWITCH_RSP;
	rsp.len =BTM_PREPARE_AUDIO_BEARER_SWITCH_RSP_LEN;
	rsp.active_transport = active_transport;
	rsp.status = status;
	return btfmcodec_dev_enqueue_pkt(btfmcodec_dev, &rsp, (rsp.len +
					BTM_HEADER_LEN));
}

int btfmcodec_wait_for_bearer_ind(struct btfmcodec_char_device *btfmcodec_dev)
{
	wait_queue_head_t *rsp_wait_q =
		&btfmcodec_dev->rsp_wait_q[BTM_PKT_TYPE_BEARER_SWITCH_IND];
	int ret;
	uint8_t *status = &btfmcodec_dev->status[BTM_PKT_TYPE_BEARER_SWITCH_IND];

	ret = wait_event_interruptible_timeout(*rsp_wait_q,
		*status != BTM_WAITING_RSP,
		msecs_to_jiffies(BTM_MASTER_CONFIG_RSP_TIMEOUT));

	if (ret == 0) {
		BTFMCODEC_ERR("failed to recevie BTM_BEARER_SWITCH_IND");
		ret = -MSG_INTERNAL_TIMEOUT;
	} else {
		if (*status == BTM_RSP_RECV) {
			ret = 0;
		} else if (*status == BTM_FAIL_RESP_RECV) {
			BTFMCODEC_ERR("Rx BTM_BEARER_SWITCH_IND with failure status");
			ret = -1;
		} else if (*status == BTM_RSP_NOT_RECV_CLIENT_KILLED) {
			BTFMCODEC_ERR("client killed so moving further");
			ret = -1;
		}
	}

	return ret;
}

int btfmcodec_initiate_hwep_configuration(struct btfmcodec_char_device *btfmcodec_dev)
{
	wait_queue_head_t *rsp_wait_q =
		&btfmcodec_dev->rsp_wait_q[BTM_PKT_TYPE_HWEP_CONFIG];
	uint8_t *status = &btfmcodec_dev->status[BTM_PKT_TYPE_HWEP_CONFIG];
	int ret;

	schedule_work(&btfmcodec_dev->wq_hwep_configure);

	*status = BTM_WAITING_RSP;
	ret = wait_event_interruptible_timeout(*rsp_wait_q,
		*status != BTM_WAITING_RSP,
		msecs_to_jiffies(BTM_MASTER_CONFIG_RSP_TIMEOUT));

	if (ret == 0) {
		BTFMCODEC_ERR("failed to recevie complete hwep configure");
		flush_work(&btfmcodec_dev->wq_hwep_configure);
		ret = -1;
	} else {
		if (*status == BTM_RSP_RECV) {
			ret = 0;
		} else if (*status == BTM_FAIL_RESP_RECV ||
			   *status == BTM_RSP_NOT_RECV_CLIENT_KILLED) {
			BTFMCODEC_ERR("Failed to close hwep moving back to previous state");
			ret = -1;
		}
	}

	return ret;
}

void btfmcodec_configure_hwep(struct btfmcodec_char_device *btfmcodec_dev)
{
	struct btfmcodec_data *btfmcodec = (struct btfmcodec_data *)btfmcodec_dev->btfmcodec;
	struct btfmcodec_state_machine *state = &btfmcodec->states;
	int ret;
	uint8_t status = MSG_SUCCESS;

	ret = btfmcodec_initiate_hwep_configuration(btfmcodec_dev);
	if (ret < 0) {
		status = MSG_FAILED_TO_CONFIGURE_HWEP;
		/* Move back to BTADV_AUDIO_Connected  from BT_Connecting */
		btfmcodec_revert_current_state(state);
	}

	ret = btfmcodec_frame_prepare_bearer_rsp_pkt(btfmcodec_dev,
		btfmcodec_get_current_transport(state), status);

	if (status != MSG_SUCCESS)
		return;

	if (ret < 0) {
		ret = btfmcodec_wait_for_bearer_ind(btfmcodec_dev);
		if (ret < 0) {
			/* Move back to BTADV_AUDIO_Connected for failure cases*/
			BTFMCODEC_ERR("moving back to previous state");
			btfmcodec_revert_current_state(state);
			/* close HWEP */
			btfmcodec_initiate_hwep_shutdown(btfmcodec_dev);
			if (ret == -MSG_INTERNAL_TIMEOUT) {
				btfmcodec_frame_transport_switch_ind_pkt(btfmcodec_dev, BT,
							MSG_INTERNAL_TIMEOUT);
			}
		} else {
			/* move from BT_Connecting to BT_Connected */
			btfmcodec_move_to_next_state(state);
		}
	} else {
		/* add code to handle packet errors */
	}
}

void btfmcodec_prepare_bearer(struct btfmcodec_char_device *btfmcodec_dev,
		enum transport_type new_transport)
{
	struct btfmcodec_data *btfmcodec = (struct btfmcodec_data *)btfmcodec_dev->btfmcodec;
	struct btfmcodec_state_machine *state = &btfmcodec->states;
	btfmcodec_state current_state;
	int ret = -1;

	if (new_transport > (ARRAY_SIZE(transport_type_text))) {
		btfmcodec_frame_prepare_bearer_rsp_pkt(btfmcodec_dev,
			(uint8_t) btfmcodec_get_current_transport(state),
			MSG_WRONG_TRANSPORT_TYPE);
		return;
	}

	BTFMCODEC_INFO("Rx to switch from transport type %s to %s",
		coverttostring(btfmcodec_get_current_transport(state)),
		transport_type_text[new_transport - 1]);

	current_state = btfmcodec_get_current_transport(state);
	if (new_transport == BT) {
		/* If BT is already active. send +ve ack to BTADV Audio Manager */
		if (current_state == BT_Connected ||
		    current_state == BT_Connecting) {
			btfmcodec_frame_prepare_bearer_rsp_pkt(btfmcodec_dev,
				(uint8_t)current_state, MSG_SUCCESS);
			return;
		} else if (current_state == BTADV_AUDIO_Connected ||
			   current_state == BTADV_AUDIO_Connecting||
			   current_state == IDLE) {
			if (btfmcodec_is_valid_cache_avb(btfmcodec)) {
				BTFMCODEC_INFO("detected BTADV audio Gaming usecase to BT usecase");
				btfmcodec_set_current_state(state, BT_Connecting);
				btfmcodec_configure_hwep(btfmcodec_dev);
			} else {
				if (current_state != IDLE)
					BTFMCODEC_INFO("detected BTADV Audio lossless to IDLE");
				BTFMCODEC_INFO("moving to IDLE as no config available");
				btfmcodec_set_current_state(state, IDLE);
				btfmcodec_frame_prepare_bearer_rsp_pkt(btfmcodec_dev,
					btfmcodec_get_current_transport(state),
					MSG_SUCCESS);
				/* No need wait for bearer switch indications as BTFMCODEC
				 * driver doesn't have configs to configure
				 */
			}
		}
	} else if(new_transport == BTADV) {
		/* If BTADV audio is already active. send +ve ack to BTADV audio Manager */
		if (current_state == BTADV_AUDIO_Connecting ||
		    current_state == BTADV_AUDIO_Connected) {
			btfmcodec_frame_prepare_bearer_rsp_pkt(btfmcodec_dev,
				(uint8_t)current_state, MSG_SUCCESS);
			return;
		} else {
			btfmcodec_set_current_state(state, BTADV_AUDIO_Connecting);
			if (btfmcodec_is_valid_cache_avb(btfmcodec)) {
				BTFMCODEC_INFO("detected BT to BTADV audio Gaming usecase");
			} else {
				BTFMCODEC_INFO("detected IDLE to BTADV audio lossless usecase");
			}

			ret = btfmcodec_frame_prepare_bearer_rsp_pkt(btfmcodec_dev,
						BTADV_AUDIO_Connecting, MSG_SUCCESS);
			if (ret < 0)
				return;

			/* Wait here to get Bearer switch indication */
			ret = btfmcodec_wait_for_bearer_ind(btfmcodec_dev);
			if (ret < 0) {
				BTFMCODEC_ERR("moving back to previous state");
				btfmcodec_revert_current_state(state);
				if (ret == -MSG_INTERNAL_TIMEOUT) {
					btfmcodec_frame_transport_switch_ind_pkt(
							btfmcodec_dev, BTADV,
							MSG_INTERNAL_TIMEOUT);
				}
			} else {
				btfmcodec_move_to_next_state(state);
			}
			if (ret < 0)
				return;

			if (btfmcodec_is_valid_cache_avb(btfmcodec)) {
				BTFMCODEC_INFO("Initiating BT port close...");
				btfmcodec_initiate_hwep_shutdown(btfmcodec_dev);
			}
		}
	} else if (new_transport == NONE) {
		/* Let ALSA handles the transport close  for BT */
		if (current_state != BT_Connecting && current_state != BT_Connected)
			btfmcodec_set_current_state(state, IDLE);
		btfmcodec_frame_prepare_bearer_rsp_pkt(btfmcodec_dev, (uint8_t)current_state,
							MSG_SUCCESS);
		return;
	}
}

void btfmcodec_wq_prepare_bearer(struct work_struct *work)
{
	struct btfmcodec_char_device *btfmcodec_dev = container_of(work,
						struct btfmcodec_char_device,
						wq_prepare_bearer);
	int idx = BTM_PKT_TYPE_PREPARE_REQ;
	BTFMCODEC_INFO("with new transport:%d", btfmcodec_dev->status[idx]);
	btfmcodec_prepare_bearer(btfmcodec_dev, btfmcodec_dev->status[idx]);
}
