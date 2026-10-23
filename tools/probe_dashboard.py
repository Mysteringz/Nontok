"""Live localhost viewer for the HARDWARE_PROBE diagnostic's JSON lines.

Diagnostic only: motors stay inhibited by the firmware, and only the probe's
own no-motion commands are forwarded. Every line is appended to a JSONL log.
Usage: .venv-dashboard/bin/python tools/probe_dashboard.py [--bluetooth | --port /dev/cu.X]
The page header can also switch between USB and Bluetooth ports while running.
Bluetooth samples arrive at 2 Hz (9600 baud); USB at 4 Hz.
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
# Motion: one-motor identify spin, and raw watchdogged drive (firmware caps |v| at 200).
MOTION = re.compile(r"SPIN [1-4] [+-]|MOT( -?\d{1,3}){4}")
PAGE = ROOT / "tools" / "probe_dashboard.html"


BLUETOOTH_TAGS = ("FEETWASHER", "HC-0", "HC0", "DSD", "JDY", "BT0", "BT-")
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
                        try:
                            record = json.loads(text)
                        except ValueError:
                            record = {"unparsed": text}
                        self.add(record if isinstance(record, dict) else {"value": record})
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
