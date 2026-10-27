"""Mac stand-in for the course Android controller (IDP_Controller.apk).

Drives the professor's Car_Volt_Feedback_24A sketch: one command letter per message over
Bluetooth (Serial3, 9600). The sketch stops by itself ~1.6 s after the last letter, so
the page repeats the held letter every 100 ms and sends Z on release.
Optionally also listens on USB, where the sketch echoes every letter it received.
Usage: .venv-dashboard/bin/python tools/car_controller.py [--bt /dev/cu.X] [--usb /dev/cu.Y]
"""
import argparse
import collections
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import serial
from serial.tools import list_ports

PAGE = Path(__file__).resolve().parent / "car_controller.html"
LETTERS = set("ABCDEFGHZzdbLM")  # Every letter the sketch's switch handles.
BLUETOOTH_TAGS = ("DIVA", "FEETWASHER", "HC-0", "HC0", "JDY", "BT0", "BT-")
SKIP_PORTS = ("Bluetooth-Incoming-Port", "debug-console")


def ports():
    found = []
    for p in list_ports.comports():
        if not p.device.startswith("/dev/cu.") or any(s in p.device for s in SKIP_PORTS):
            continue
        kind = ("usb" if "usbserial" in p.device or "wchusbserial" in p.device else
                "bluetooth" if any(t in p.device.upper() for t in BLUETOOTH_TAGS) else "other")
        found.append({"device": p.device, "kind": kind})
    return found


class Log:
    def __init__(self):
        self.lock = threading.Lock()
        self.lines = collections.deque(maxlen=1000)
        self.seq = 0

    def add(self, src, text):
        with self.lock:
            self.seq += 1
            self.lines.append({"seq": self.seq, "src": src, "text": text, "t": time.time()})

    def since(self, after):
        with self.lock:
            return self.seq, [l for l in self.lines if l["seq"] > after]


class Link:
    """One serial port with a reader thread; select() switches or closes it."""

    def __init__(self, name, baud, log):
        self.name, self.baud, self.log = name, baud, log
        self.port, self.generation, self.device = None, 0, None
        self.status, self.last_rx = "not connected", None
        threading.Thread(target=self.run, daemon=True).start()

    def select(self, port):
        self.port, self.last_rx = port, None
        self.generation += 1
        self.status = "connecting " + port if port else "not connected"

    def send(self, data):
        device = self.device
        if not device:
            return False
        try:
            device.write(data)
            return True
        except (serial.SerialException, OSError) as error:
            self.status = "write failed: %s" % error
            return False

    def run(self):
        while True:
            port, generation = self.port, self.generation
            if not port:
                time.sleep(0.2)
                continue
            device = serial.Serial()
            device.port, device.baudrate, device.timeout, device.write_timeout = port, self.baud, 0.1, 1
            device.dtr = device.rts = False
            try:
                device.open()
                self.device, self.status = device, "open " + port
                buffer = b""
                while self.generation == generation:
                    chunk = device.read(256)
                    if chunk:
                        self.last_rx, self.status = time.time(), "receiving " + port
                    buffer += chunk
                    while b"\n" in buffer:
                        raw, buffer = buffer.split(b"\n", 1)
                        text = raw.decode(errors="replace").strip()
                        if text:
                            self.log.add(self.name, text)
                    if len(buffer) > 64:  # Echoed letters arrive without newlines.
                        self.log.add(self.name, buffer.decode(errors="replace"))
                        buffer = b""
            except (serial.SerialException, OSError) as error:
                if self.generation == generation:
                    self.status = "disconnected: %s" % error
            finally:
                self.device = None
                if device.is_open:
                    try:
                        device.write(b"Z")  # Leaving a link never leaves motors running.
                    except (serial.SerialException, OSError):
                        pass
                    device.close()
            if self.generation == generation:
                time.sleep(1)


def handler(links, log):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, body, kind="application/json"):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(200)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == "/":
                return self.reply(PAGE.read_bytes(), "text/html; charset=utf-8")
            if self.path.startswith("/api/state"):
                after = int(self.path.partition("after=")[2] or 0)
                seq, lines = log.since(after)
                now = time.time()
                return self.reply({"seq": seq, "lines": lines, "ports": ports(), "links": {
                    name: {"port": l.port, "status": l.status,
                           "rx_age": None if l.last_rx is None else round(now - l.last_rx, 1)}
                    for name, l in links.items()}})
            self.send_error(404)

        def do_POST(self):
            body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
            if self.path == "/api/connect" and body.get("role") in links:
                links[body["role"]].select(body.get("port") or None)
                return self.reply({"ok": True})
            if self.path == "/api/send" and body.get("c") in LETTERS:
                return self.reply({"ok": links["bt"].send(body["c"].encode())})
            self.send_error(400)

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bt", help="Bluetooth serial port (default: first DIVA/Feetwasher/HC port)")
    parser.add_argument("--usb", help="USB port to monitor (default: first usbserial port)")
    parser.add_argument("--http", type=int, default=8767)
    args = parser.parse_args()
    found = ports()
    log = Log()
    links = {"bt": Link("bt", 9600, log), "usb": Link("usb", 115200, log)}
    links["bt"].select(args.bt or next((p["device"] for p in found if p["kind"] == "bluetooth"), None))
    links["usb"].select(args.usb or next((p["device"] for p in found if p["kind"] == "usb"), None))
    server = ThreadingHTTPServer(("127.0.0.1", args.http), handler(links, log))
    print("Car controller: http://127.0.0.1:%d  (bt %s, usb %s)" % (args.http, links["bt"].port, links["usb"].port), flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
