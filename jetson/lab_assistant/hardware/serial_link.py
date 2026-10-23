"""USB serial link to the Arduino Mega (as in Lab 4)."""
import logging
import threading
import time
from typing import Optional, Tuple

from . import protocol
from .protocol import Event, Telemetry

log = logging.getLogger(__name__)


class RobotLink:
    """Sends commands to the Mega; a reader thread keeps the latest telemetry.

    All methods return immediately. Callers poll `telemetry` and `turn_result()`.
    """

    def __init__(self, port: str, baud: int = 115200, ready_timeout_s: float = 3.0):
        self.port = port
        self.baud = baud
        self.ready_timeout_s = ready_timeout_s
        self.commanded: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # last VEL, for dead reckoning
        self._ser = None
        self._lock = threading.Lock()
        self._running = False
        self._ready = threading.Event()
        self._telemetry: Optional[Telemetry] = None
        self._telemetry_time = 0.0
        self._turn_state = "idle"  # idle | turning | done | failed

    # ---- lifecycle ----
    def open(self):
        import serial  # pyserial

        self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
        self._running = True
        threading.Thread(target=self._reader, name="serial-reader", daemon=True).start()
        # Opening the port resets the Mega; wait for its READY line.
        if not self._ready.wait(self.ready_timeout_s):
            log.warning("No READY from the Mega on %s, continuing anyway", self.port)

    def close(self):
        self._running = False
        if self._ser:
            self._ser.close()

    # ---- state ----
    @property
    def telemetry(self) -> Optional[Telemetry]:
        return self._telemetry

    def telemetry_age(self) -> float:
        return time.monotonic() - self._telemetry_time

    def turn_result(self) -> Optional[bool]:
        """None while turning, True when done, False if the Mega gave up."""
        if self._turn_state == "turning":
            return None
        return self._turn_state == "done"

    # ---- commands ----
    def vel(self, vx: float, vy: float, wz: float = 0.0):
        """Body-frame velocity: mm/s forward, mm/s left, deg/s CCW. Resend at least every 0.5 s."""
        self.commanded = (vx, vy, wz)
        self._send(protocol.vel(vx, vy, wz))

    def stop(self):
        self.commanded = (0.0, 0.0, 0.0)
        self._send(protocol.STOP)

    def turn(self, rel_deg: float):
        self.commanded = (0.0, 0.0, 0.0)
        self._turn_state = "turning"
        self._send(protocol.turn(rel_deg))

    def hold(self, gyro_heading_deg: float):
        self._send(protocol.hold(gyro_heading_deg))

    def tilt(self, deg: int):
        self._send(protocol.tilt(deg))

    def calibrate_gyro(self):
        self._send(protocol.CAL)

    def _send(self, line: str):
        if self._ser is None:
            raise RuntimeError("RobotLink.open() was not called")
        with self._lock:
            self._ser.write((line + "\n").encode("ascii"))

    # ---- reader thread ----
    def _reader(self):
        while self._running:
            try:
                raw = self._ser.readline()
            except Exception as e:  # unplugged cable etc.
                log.error("Serial read failed: %s", e)
                time.sleep(0.5)
                continue
            if not raw:
                continue
            msg = protocol.parse_line(raw.decode("ascii", errors="replace"))
            if isinstance(msg, Telemetry):
                self._telemetry = msg
                self._telemetry_time = time.monotonic()
            elif isinstance(msg, Event):
                self._handle_event(msg)

    def _handle_event(self, ev: Event):
        if ev.kind == "READY":
            self._ready.set()
        elif ev.kind == "DONE" and ev.args[:1] == ("TURN",):
            self._turn_state = "done"
        elif ev.kind == "ERR":
            if ev.args[:1] == ("TURN_TIMEOUT",):
                self._turn_state = "failed"
            log.warning("Mega: ERR %s", " ".join(ev.args))
