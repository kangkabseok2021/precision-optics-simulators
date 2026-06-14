import pytest
import struct

try:
    import can
    CAN_SUPPORTED = True
except ImportError:
    CAN_SUPPORTED = False

def has_vcan0():
    if not CAN_SUPPORTED:
        return False
    try:
        bus = can.interface.Bus(channel='vcan0', interface='socketcan', fd=True)
        bus.shutdown()
        return True
    except Exception:
        return False

pytestmark = pytest.mark.skipif(not has_vcan0(), reason="vcan0 interface not available")

def test_telemetry_frame_received():
    bus = can.interface.Bus(channel='vcan0', interface='socketcan', fd=True)
    msg = bus.recv(timeout=1.5)
    
    assert msg is not None, "Failed to receive any frame on vcan0"
    assert msg.arbitration_id == 0x100, f"Expected telemetry ID 0x100, got 0x{msg.arbitration_id:x}"
    assert len(msg.data) == 48, f"Expected 48 byte payload, got {len(msg.data)}"
    
    rpm, coolant_temp = struct.unpack(">Hh", msg.data[0:4])
    assert rpm >= 800
    assert coolant_temp >= 20 * 256
    
    bus.shutdown()

def test_command_acknowledged():
    bus = can.interface.Bus(channel='vcan0', interface='socketcan', fd=True)
    
    # CMD 0x02: SET_THROTTLE, param = 50%
    cmd_data = bytearray([0x02, 50, 0, 0, 0, 0, 0, 0])
    msg = can.Message(arbitration_id=0x200, data=cmd_data, is_fd=False)
    bus.send(msg)
    
    resp = None
    for _ in range(15):
        m = bus.recv(timeout=0.1)
        if m and m.arbitration_id == 0x201:
            resp = m
            break
            
    assert resp is not None, "Failed to receive response command frame 0x201"
    assert resp.data[0] == 0x02
    assert resp.data[1] == 0x00 # status SUCCESS
    
    bus.shutdown()
