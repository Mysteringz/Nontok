"""Live localhost viewer for the HARDWARE_PROBE diagnostic's JSON lines.

Diagnostic only: motors stay inhibited by the firmware, and only the probe's
own no-motion commands are forwarded. Every line is appended to a JSONL log.
Usage: .venv-dashboard/bin/python tools/probe_dashboard.py [--bluetooth | --port /dev/cu.X]
The page header can also switch between USB and Bluetooth ports while running.
Both links update at 5 Hz. Bluetooth (9600 baud) sends a full sample once a second and a
short "F," line in between; the server merges them into ordinary samples for the page.
"""
import argparse
import collections
import datetime
import json
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import serial
from serial.tools import list_ports

ROOT = Path(__file__).resolve().parents[1]
COMMANDS = {"STATUS", "PROBE", "GYROINIT", "GYROZERO", "HEADING0", "STOP", "HELP"}
# Motion: one-motor identify spin, raw watchdogged drive (firmware caps |v| at 200)
# and the on-board 180-degree turn test.
MOTION = re.compile(r"SPIN [1-4] [+-]|(MOT|TURNTEST)( -?\d{1,3}){4}"
                    r"|WMAP( [1-4] -?1){4}|REF( \d{1,4}){5}|GO( [01]){3}( \d{1,3}){2}|TURN -?\d{1,3}( \d{1,3}){2}")
PAGE = ROOT / "tools" / "probe_dashboard.html"
# Competition calibration: 5 s of stationary readings per pose, appended one capture per line.
CAPTURES = ROOT / ".local-data" / "calibration" / "pose-captures.jsonl"
# Placement order for the tournament steps: P facing the back wall, N on the colour paper, then
# N turned exactly 90 deg each way and the boxes in their final parking pose (facing the side
# wall the robot turned toward). Turned checkpoints let step 03 finish on recorded readings.
POSES = {"p_point": "Checkpoint P (centre of semicircle)", "centre": "Checkpoint N (colour plate)",
         "turned_left": "N turned left (faces left wall)", "left_box": "Left parking box",
         "turned_right": "N turned right (faces right wall)", "right_box": "Right parking box"}
CAPTURE_SECONDS = 5
LIGHT_PINS = ("A0", "A7", "A2", "A6")  # Firmware adc[] order; physically left to right A0, A2, A7, A6.


BLUETOOTH_TAGS = ("DIVA", "FEETWASHER", "HC-0", "HC0", "DSD", "JDY", "BT0", "BT-")
SKIP_PORTS = ("Bluetooth-Incoming-Port", "debug-console")


def ports():
    """Serial ports the page may choose from, robot candidates first."""
    found = []
    for p in list_ports.comports():
        if not p.device.startswith("/dev/cu.") or any(skip in p.device for skip in SKIP_PORTS):
            continue
        kind = ("usb" if (p.vid, p.pid) == (0x1A86, 0x7523) else
                "bluetooth" if any(tag in p.device.upper() for tag in BLUETOOTH_TAGS) else "other")
        found.append({"device": p.device, "kind": kind})
    return sorted(found, key=lambda p: ("usb", "bluetooth", "other").index(p["kind"]))


def utc():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


# Ultrasonics in firmware index order; the F line carries all of them.
ULTRASONICS = ("left", "right", "front", "side_left", "side_right")


def fast_sample(text, full):
    """Expand a Bluetooth "F,..." line into a sample, other fields from the last full one."""
    parts = text.split(",")  # F, ms, adc x4, ultrasonic x n, gyro Z angle and rate, colour x4
    n = len(parts) - 12
    if not 1 <= n <= 8:
        return None
    try:
        ms, *adc = [int(v) for v in parts[1:6]]
        us = [int(v) for v in parts[6:6 + n]]
        yaw, rate = float(parts[6 + n]), float(parts[7 + n])
        rgbc = [int(v) for v in parts[8 + n:12 + n]]
    except ValueError:
        return None
    gyro = dict((full or {}).get("gyro") or {})
    gyro["angle_deg"] = (list(gyro.get("angle_deg") or [None, None]))[:2] + [yaw]
    gyro["rate_dps"] = (list(gyro.get("rate_dps") or [None, None]))[:2] + [rate]
    record = {k: v for k, v in (full or {}).items() if k in ("boot", "turntest", "step")}
    record.update({"type": "sample", "link": "bt", "fast": 1, "ms": ms, "adc": adc,
                   "ultrasonic_mm": us, "gyro": gyro, "color_rgbc": rgbc})
    return record


def channels(sample):
    """Every numeric reading in one sample, flat; None where missing (no echo = None)."""
    g = sample.get("gyro") or {}
    adc, us = sample.get("adc") or [None] * 4, sample.get("ultrasonic_mm") or [None, None]
    rgbc = sample.get("color_rgbc") or [None] * 4
    ch = {"ldr_" + pin: v for pin, v in zip(LIGHT_PINS, adc)}
    us = list(us) + [None] * (len(ULTRASONICS) - len(us))  # Older samples have fewer sensors.
    ch.update({"us_%s_mm" % name: v if v is not None and v > 0 else None for name, v in zip(ULTRASONICS, us)})
    for name, key in (("angle", "angle_deg"), ("rate", "rate_dps"), ("accel", "accel_raw")):
        ch.update({"%s_%s" % (name, axis): v for axis, v in zip("xyz", g.get(key) or [None] * 3)})
    ch.update({"color_" + c: v for c, v in zip(("red", "green", "blue", "clear"), rgbc)})
    ch["color_rg_ratio"] = rgbc[0] / rgbc[1] if rgbc[0] and rgbc[1] else None
    return ch


def stats(values):
    vals = [v for v in values if v is not None]
    if not vals:
        return {"n": 0}
    mean = sum(vals) / len(vals)
    sd = (sum((v - mean) ** 2 for v in vals) / len(vals)) ** 0.5
    return {"n": len(vals), "mean": round(mean, 3), "sd": round(sd, 3), "min": min(vals), "max": max(vals)}


def capture(link, pose, note):
    """Record CAPTURE_SECONDS of samples at a pose; returns the summary that was saved."""
    started, start_seq = utc(), link.seq
    time.sleep(CAPTURE_SECONDS)
    _, lines = link.since(start_seq)
    samples = [l for l in lines if l.get("type") == "sample"]
    per = [channels(s) for s in samples]
    summary = {name: stats([c[name] for c in per]) for name in (per[0] if per else {})}
    warnings = []
    if len(samples) < 5:
        warnings.append("only %d samples: link down or too slow" % len(samples))
    for side in ULTRASONICS:
        n = summary.get("us_%s_mm" % side, {}).get("n", 0)
        if samples and n < len(samples):
            warnings.append("%s ultrasonic: %d of %d samples had no echo" % (side, len(samples) - n, len(samples)))
        if summary.get("us_%s_mm" % side, {}).get("sd", 0) > 15:
            warnings.append("%s ultrasonic varies > 15 mm: something moved?" % side)
    rate = [abs(c["rate_z"]) for c in per if c.get("rate_z") is not None]
    if rate and max(rate) > 3:
        warnings.append("gyro saw rotation up to %.1f deg/s: the robot moved during the capture" % max(rate))
    if any(s.get("ms", 0) < samples[0].get("ms", 0) for s in samples):
        warnings.append("the Mega rebooted during the capture")
    record = {"pose": pose, "label": POSES[pose], "note": note, "started": started, "ended": utc(),
              "seconds": CAPTURE_SECONDS, "link": samples[-1].get("link", "usb") if samples else None,
              "light_pins_firmware_order": LIGHT_PINS, "light_positions_left_to_right": ["A0", "A2", "A7", "A6"],
              "samples_n": len(samples), "warnings": warnings, "stats": summary, "samples": samples}
    CAPTURES.parent.mkdir(parents=True, exist_ok=True)
    with open(CAPTURES, "a") as out:
        out.write(json.dumps(record) + "\n")
    link.add({"type": "marker", "text": "calibration capture: %s, %d samples%s" % (
        POSES[pose], len(samples), (", WARN: " + "; ".join(warnings)) if warnings else "")})
    return {k: v for k, v in record.items() if k != "samples"}


def saved_captures():
    if not CAPTURES.exists():
        return []
    rows = []
    for line in CAPTURES.read_text().splitlines():
        try:
            record = json.loads(line)
        except ValueError:
            continue
        record.pop("samples", None)
        rows.append(record)
    return rows


class Link:
    def __init__(self, port, log_path):
        self.port, self.log_path = port, log_path
        self.lock = threading.Lock()
        self.lines = collections.deque(maxlen=2400)  # (seq, record)
        self.seq = 0
        self.pending = collections.deque()
        self.status = "starting"
        self.generation = 0  # Bumped by every select(), so re-selecting a port reconnects it.
        threading.Thread(target=self.run, daemon=True).start()

    def add(self, record):
        record["received_at"] = utc()
        with self.lock:
            self.seq += 1
            self.lines.append((self.seq, record))
        with open(self.log_path, "a") as log:
            log.write(json.dumps(record) + "\n")

    def select(self, port):
        """Switch to another port, or None to disconnect; the reader thread follows."""
        self.pending.clear()
        self.port = port
        self.generation += 1
        self.status = "connecting " + port if port else "not connected"

    def run(self):
        while True:
            port, generation = self.port, self.generation
            if not port:
                self.status = "not connected"
                time.sleep(0.3)
                continue
            device = serial.Serial()
            device.port, device.baudrate, device.timeout = port, 115200, 0.1
            device.exclusive, device.dtr, device.rts = True, False, False
            device.write_timeout = 1  # A Bluetooth port without a link must not hang the thread.
            try:
                device.open()
                self.status = "port open " + port
                buffer = b""
                last_full = None  # Latest full sample, for merging F lines.
                next_ping = 0
                last_rx = time.time()
                while self.generation == generation:
                    # Bluetooth firmware only transmits within 5 s of a PING.
                    if time.time() >= next_ping:
                        device.write(b"PING\n")
                        next_ping = time.time() + 1.5
                    while self.pending:
                        device.write((self.pending.popleft() + "\n").encode("ascii"))
                    chunk = device.read(512)
                    if chunk:
                        last_rx = time.time()
                        self.status = "connected " + port
                    elif time.time() - last_rx > 20:
                        # macOS can open a Bluetooth port without forming the link: retry.
                        raise serial.SerialException("no data for 20 s (link not formed); reconnecting")
                    elif time.time() - last_rx > 3:
                        self.status = "port open, waiting for data " + port
                    buffer += chunk
                    while b"\n" in buffer:
                        raw, buffer = buffer.split(b"\n", 1)
                        text = raw.decode(errors="replace").strip()
                        if not text:
                            continue
                        if text.startswith("F,"):
                            record = fast_sample(text, last_full) or {"unparsed": text}
                        else:
                            try:
                                record = json.loads(text)
                            except ValueError:
                                record = {"unparsed": text}
                        if not isinstance(record, dict):
                            record = {"value": record}
                        if record.get("type") == "sample" and not record.get("fast"):
                            last_full = record
                        self.add(record)
                    buffer = buffer[-4096:]
            except (serial.SerialException, OSError) as error:
                if self.generation == generation:
                    self.status = "disconnected: %s" % error
            finally:
                if device.is_open:
                    try:
                        device.write(b"STOP\n")  # Leaving a link never leaves motors running.
                    except (serial.SerialException, OSError):
                        pass
                device.close()
            if self.generation == generation:
                time.sleep(1)

    def send(self, command):
        if command == "STOP":
            self.pending.clear()  # Nothing queued may run after a STOP.
            self.pending.appendleft(command)
        elif command.startswith("MOT "):
            # Keep only the newest drive command so a slow link never replays stale ones.
            for old in [c for c in self.pending if c.startswith("MOT ")]:
                self.pending.remove(old)
            self.pending.append(command)
        else:
            self.pending.append(command)

    def since(self, after):
        with self.lock:
            return self.seq, [{"seq": s, **r} for s, r in self.lines if s > after]


def handler(link):
    capture_lock = threading.Lock()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def send(self, body, mime="application/json"):
            raw = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(200)
            self.send_header("Content-Type", mime)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self):
            if self.path == "/":
                self.send(PAGE.read_bytes(), "text/html; charset=utf-8")
            elif self.path == "/api/ports":
                self.send({"ports": ports(), "selected": link.port})
            elif self.path == "/api/captures":
                self.send({"captures": saved_captures(), "file": str(CAPTURES.relative_to(ROOT)),
                           "poses": POSES, "seconds": CAPTURE_SECONDS})
            elif self.path.startswith("/api/lines"):
                try:
                    after = int(self.path.partition("after=")[2] or 0)
                except ValueError:
                    after = 0
                seq, lines = link.since(after)
                self.send({"seq": seq, "status": link.status, "lines": lines,
                           "log": str(link.log_path.relative_to(ROOT))})
            else:
                self.send_error(404)

        def do_POST(self):
            # Same-origin only, and only the probe's whitelisted no-motion commands.
            origin = self.headers.get("Origin")
            if origin and origin != "http://%s" % self.headers.get("Host"):
                return self.send_error(403)
            length = min(int(self.headers.get("Content-Length") or 0), 512)
            try:
                body = json.loads(self.rfile.read(length))
                command = body.get("command")
            except (ValueError, AttributeError):
                body, command = {}, None
            if self.path == "/api/connect":
                port = body.get("port")
                if port and port not in [p["device"] for p in ports()]:
                    return self.send_error(400)
                link.select(port or None)
                return self.send({"selected": link.port})
            if self.path == "/api/capture" and body.get("pose") in POSES:
                if not capture_lock.acquire(blocking=False):
                    return self.send_error(409, "a capture is already running")
                try:
                    note = body.get("note") if isinstance(body.get("note"), str) else ""
                    return self.send(capture(link, body["pose"], note[:200]))
                finally:
                    capture_lock.release()
            if self.path == "/api/mark" and isinstance(body.get("text"), str):
                # Operator label for the field log; never sent to the robot.
                link.add({"type": "marker", "text": body["text"][:200]})
                return self.send({"marked": True})
            valid = isinstance(command, str) and (command in COMMANDS or MOTION.fullmatch(command))
            if self.path != "/api/command" or not valid:
                return self.send_error(400)
            link.send(command)
            self.send({"sent": command})
    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", help="Serial device (default: first CH340 1A86:7523)")
    parser.add_argument("--bluetooth", action="store_true", help="Prefer the paired Bluetooth port")
    parser.add_argument("--http", type=int, default=8766)
    args = parser.parse_args()
    found = ports()
    # USB (CH340) first, then a paired Bluetooth serial port; the page can switch later.
    usb = next((p["device"] for p in found if p["kind"] == "usb"), None)
    bluetooth = next((p["device"] for p in found if p["kind"] == "bluetooth"), None)
    port = args.port or (bluetooth if args.bluetooth else usb or bluetooth)
    stamp = datetime.datetime.now().strftime("%Y-%m-%d/probe-live-%H%M%S.jsonl")
    log_path = ROOT / ".local-data" / "hardware-probe" / stamp
    log_path.parent.mkdir(parents=True, exist_ok=True)
    link = Link(port, log_path)
    server = ThreadingHTTPServer(("127.0.0.1", args.http), handler(link))
    print("Probe dashboard: http://127.0.0.1:%d  (serial %s, log %s)" % (args.http, port or "none: pick one on the page", log_path), flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
