import pytest
import requests
import urllib3
import socket

urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

def is_port_open(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(0.5)
    try:
        s.connect(("localhost", port))
        s.close()
        return True
    except Exception:
        return False

pytestmark = pytest.mark.skipif(not is_port_open(8443), reason="Gateway HTTPS REST port 8443 not open")

def test_gateway_latest_telemetry():
    url = "https://localhost:8443/api/telemetry/latest"
    res = requests.get(url, verify=False, timeout=2.0)
    
    assert res.status_code == 200
    data = res.json()
    assert "rpm" in data
    assert "coolant_temp" in data
    assert "oil_pressure" in data
    assert "fault_bitmap" in data
    assert "sequence_counter" in data
    assert "uptime" in data

def test_gateway_telemetry_history():
    url = "https://localhost:8443/api/telemetry/history"
    res = requests.get(url, verify=False, timeout=2.0)
    
    assert res.status_code == 200
    data = res.json()
    assert isinstance(data, list)
    if len(data) > 0:
        assert "rpm" in data[0]
