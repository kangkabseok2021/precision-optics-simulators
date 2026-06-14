#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "physics_model.h"
#include "can_fd_frames.h"
#include "ecu_state_machine.h"

LOG_MODULE_REGISTER(ecu_main, LOG_LEVEL_INF);

CAN_MSGQ_DEFINE(cmd_msgq, 8);

static EcuStateMachine state_machine;

static void state_change_callback(const struct device *dev, enum can_state state,
				  struct can_bus_err_cnt err_cnt, void *user_data)
{
	EcuStateMachine *fsm = static_cast<EcuStateMachine*>(user_data);
	if (state == CAN_STATE_BUS_OFF) {
		LOG_ERR("CAN controller Bus-Off state detected!");
		fsm->set_bus_off(true);
	} else if (fsm->get_state() == EcuState::FAULT && state != CAN_STATE_BUS_OFF) {
		fsm->set_bus_off(false);
	}
}

int main(void)
{
	LOG_INF("Starting Zephyr CAN-FD ECU Node...");

	const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
	if (!device_is_ready(can_dev)) {
		LOG_ERR("CAN controller device not ready");
		return -1;
	}

	int err = can_set_mode(can_dev, CAN_MODE_FD);
	if (err != 0) {
		LOG_ERR("Failed to set CAN-FD mode: %d", err);
		return -1;
	}

	can_set_state_change_callback(can_dev, state_change_callback, &state_machine);

	struct can_filter filter = {
		.id = COMMAND_CAN_ID,
		.mask = CAN_STD_CID_MASK,
		.flags = 0
	};
	int filter_id = can_add_rx_filter_msgq(can_dev, &cmd_msgq, &filter);
	if (filter_id < 0) {
		LOG_ERR("Failed to register command filter: %d", filter_id);
		return -1;
	}

	uint16_t seq = 0;
	uint32_t start_time = k_uptime_get_32();

	LOG_INF("ECU initialized, starting main loop");

	while (1) {
		uint32_t loop_start = k_uptime_get_32();
		double dt = 1.0 / state_machine.get_telemetry_rate();

		struct can_frame cmd_frame;
		while (k_msgq_get(&cmd_msgq, &cmd_frame, K_NO_WAIT) == 0) {
			CommandFrame cmd;
			if (unpack_command(cmd_frame.data, cmd_frame.dlc, cmd)) {
				uint8_t status = state_machine.handle_command(cmd);
				LOG_INF("CMD ID 0x%02X executed with status %d", cmd.command_id, status);

				struct can_frame resp_frame;
				resp_frame.id = RESPONSE_CAN_ID;
				resp_frame.flags = 0;
				resp_frame.dlc = 8;
				ResponseFrame resp;
				pack_response(cmd.command_id, status, resp);
				memcpy(resp_frame.data, &resp, sizeof(ResponseFrame));

				int tx_err = can_send(can_dev, &resp_frame, K_MSEC(10), NULL, NULL);
				if (tx_err != 0) {
					LOG_WRN("Failed to send response frame: %d", tx_err);
				}
			}
		}

		state_machine.update(dt);

		if (state_machine.get_state() != EcuState::FAULT && state_machine.get_fault_bitmap() == 0) {
			enum can_state curr_state;
			struct can_bus_err_cnt err_cnt;
			can_get_state(can_dev, &curr_state, &err_cnt);
			if (curr_state == CAN_STATE_BUS_OFF) {
				LOG_INF("Triggering CAN bus recovery...");
				can_recover(can_dev, K_MSEC(100));
			}
		}

		struct can_frame tel_frame;
		tel_frame.id = TELEMETRY_CAN_ID;
		tel_frame.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;
		tel_frame.dlc = can_bytes_to_dlc(sizeof(TelemetryFrame));
		
		TelemetryFrame tel;
		uint32_t uptime_s = (k_uptime_get_32() - start_time) / 1000;
		pack_telemetry(state_machine.get_physics().get_state(), 
		               state_machine.get_fault_bitmap(), 
		               seq++, 
		               uptime_s, 
		               tel);
		memcpy(tel_frame.data, &tel, sizeof(TelemetryFrame));

		int tx_err = can_send(can_dev, &tel_frame, K_MSEC(10), NULL, NULL);
		if (tx_err != 0) {
			LOG_WRN("Failed to send telemetry frame: %d", tx_err);
		}

		uint32_t loop_duration = k_uptime_get_32() - loop_start;
		uint32_t period_ms = static_cast<uint32_t>(dt * 1000.0);
		if (loop_duration < period_ms) {
			k_msleep(period_ms - loop_duration);
		} else {
			k_yield();
		}
	}

	return 0;
}
