from lab_assistant.hardware import protocol
from lab_assistant.hardware.protocol import Event, Telemetry


def test_parse_telemetry():
    tel = protocol.parse_line("TEL 1234 -12.5 300 450 4000 4100 1 90\r\n")
    assert tel == Telemetry(1234, -12.5, 300, 450, 4000, 4100, True, 90)


def test_parse_garbled_telemetry():
    assert protocol.parse_line("TEL 12 x 300") is None
    assert protocol.parse_line("TEL 1 2.0 3 4 5 6 1 zz") is None


def test_parse_events():
    assert protocol.parse_line("DONE TURN") == Event("DONE", ("TURN",))
    assert protocol.parse_line("ERR TURN_TIMEOUT") == Event("ERR", ("TURN_TIMEOUT",))
    assert protocol.parse_line("   ") is None


def test_encode():
    assert protocol.vel(150, -20.4, 0) == "VEL 150 -20 0.0"
    assert protocol.turn(-90) == "TURN -90.0"
    assert protocol.tilt(35.0) == "TILT 35"
