"""Store the tournament calibration on the robot (EEPROM), so a run needs no computer.

Sends the wheel map, the six checkpoint captures and the colour references through the probe
console (tools/start_console.sh, USB or Bluetooth), then CALSAVE, and checks the robot's CALSHOW
reply against what was sent. Start-facing side readings (no field wall there) are dropped, as in
tools/seek_steps.py. P is sent with no targets: the run no longer uses it.
Run: python3 tools/upload_calibration.py calibration/field-2026-10-30.json
"""
import json
import sys
import time
import urllib.request

BASE = "http://127.0.0.1:8766"
POSES = ("p_point", "centre", "turned_left", "left_box", "turned_right", "right_box")   # CAL 0..5
REF_KEYS = ("front", "side_left", "side_right", "left", "right")                        # REF order
DROP = {"turned_left": ("side_left",), "left_box": ("side_left",),
        "turned_right": ("side_right",), "right_box": ("side_right",),
        "p_point": REF_KEYS}   # P deleted (2 Nov): the run goes straight to N; its slot is sent empty
WMAP = "WMAP 1 1 2 -1 3 1 4 -1"   # FL FR RL RR (console Motors panel, 30 Oct)
GREEN_MILLI, RED_MILLI = 830, 1660


def get(path):
    return json.load(urllib.request.urlopen(BASE + path, timeout=5))


def send(command, ack, seq):
    """One command at a time (the Mega's input buffer is 64 bytes); returns the reply line."""
    req = urllib.request.Request(BASE + "/api/command", json.dumps({"command": command}).encode(),
                                 {"Content-Type": "application/json"})
    json.load(urllib.request.urlopen(req, timeout=5))
    t0 = time.time()
    while time.time() - t0 < 4:
        d = get("/api/lines?after=%d" % seq)
        for line in d["lines"]:
            seq = line["seq"]
            if "error" in line:
                sys.exit("%s refused: %s" % (command, line["error"]))
            if line.get("type") == ack:
                return line, seq
        time.sleep(0.05)
    sys.exit("no %s reply to %s (is the robot connected and running the probe firmware?)" % (ack, command))


def main():
    targets = json.load(open(sys.argv[1]))["targets"]
    seq = get("/api/lines?after=0")["seq"]
    _, seq = send(WMAP, "wmap", seq)
    sent = []
    for i, pose in enumerate(POSES):
        ref = dict(targets[pose]["ref"]) if pose in targets else {k: 9999 for k in REF_KEYS}
        for k in DROP.get(pose, ()):
            ref[k] = 9999
        values = [ref[k] for k in REF_KEYS]
        sent.append(values)
        _, seq = send("CAL %d %s" % (i, " ".join(str(v) for v in values)), "cal_set", seq)
        print("CAL %d %-13s %s" % (i, pose, values))
    _, seq = send("CALRG %d %d" % (GREEN_MILLI, RED_MILLI), "cal_rg", seq)
    shown, seq = send("CALSAVE", "cal", seq)
    ok = shown.get("valid") == 1 and shown.get("ref") == sent and shown.get("rg_milli") == [GREEN_MILLI, RED_MILLI]
    print("robot stored: valid %s, wheel map %s, colour %s" % (shown.get("valid"), shown.get("wmap"), shown.get("rg_milli")))
    if not ok:
        sys.exit("MISMATCH between what was sent and what the robot stored: %s" % shown)
    print("calibration stored on the robot and verified")


if __name__ == "__main__":
    main()
