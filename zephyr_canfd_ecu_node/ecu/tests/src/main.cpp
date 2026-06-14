#include <zephyr/ztest.h>
#include "physics_model.h"
#include "can_fd_frames.h"
#include "ecu_state_machine.h"

ZTEST(ecu_tests, test_physics_model_step) {
	PhysicsModel model;
	const auto& s = model.get_state();
	
	zassert_equal(s.throttle, 0.0, "Expected initial throttle to be 0");
	zassert_equal(s.rpm, 800.0, "Expected initial RPM to be 800");
	
	model.set_throttle(50.0);
	model.step(1.0);
	zassert_true(s.rpm > 800.0, "RPM should increase after throttle input");
	zassert_true(s.rpm < 3400.0, "RPM should move towards target but not exceed it instantly");
}

ZTEST(ecu_tests, test_can_fd_frames_roundtrip) {
	PhysicsState state = {
		.throttle = 10.0,
		.rpm = 1500.0,
		.coolant_temp = 85.5,
		.oil_pressure = 420.0,
		.battery_voltage = 13.8,
		.fuel_level = 90.0,
		.vehicle_speed = 45.0,
		.ambient_temp = 20.0
	};
	TelemetryFrame frame;
	pack_telemetry(state, 1, 100, 10, frame);
	
	// Check fields are correctly serialized as big endian
	// 100 = 0x0064 -> big endian is [0x00, 0x64]
	uint8_t* bytes = reinterpret_cast<uint8_t*>(&frame.sequence_counter);
	zassert_equal(bytes[0], 0x00, "Sequence counter MSB mismatch");
	zassert_equal(bytes[1], 0x64, "Sequence counter LSB mismatch");

	// Uptime 10 = 0x0000000A
	uint8_t* uptime_bytes = reinterpret_cast<uint8_t*>(&frame.uptime);
	zassert_equal(uptime_bytes[3], 0x0A, "Uptime byte 3 mismatch");
}

ZTEST(ecu_tests, test_state_machine_transitions) {
	EcuStateMachine fsm;
	zassert_equal(fsm.get_state(), EcuState::INIT, "FSM should start in INIT state");
	
	fsm.update(0.1);
	zassert_equal(fsm.get_state(), EcuState::NORMAL, "FSM should transition to NORMAL after update");
	zassert_equal(fsm.get_telemetry_rate(), 50.0, "NORMAL telemetry rate should be 50Hz");
	
	// Force FAULT transition by injecting over-temperature physics
	fsm.get_physics().set_throttle(100.0);
	// Step physics repeatedly to heat up coolant_temp > 110 °C
	for (int i = 0; i < 50; ++i) {
		fsm.update(1.0);
	}
	zassert_equal(fsm.get_state(), EcuState::FAULT, "FSM should transition to FAULT state when temp > 110");
	zassert_true((fsm.get_fault_bitmap() & 0x01) != 0, "Over-temperature fault bit should be set");
}

ZTEST_SUITE(ecu_tests, NULL, NULL, NULL, NULL, NULL);
