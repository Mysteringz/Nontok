"""Stop-and-go controller for the tournament steps (prototype on the Mac).

Positions come from the ultrasonics: the robot stops, takes fresh readings, works out how far it
is from the recorded readings of the target checkpoint, makes one short timed move (PULSE) that
covers only part of the error, and repeats. Each move type learns its own mm-per-ms from what the
last move actually did, so battery and floor changes are absorbed.

Heading comes from the gyro, read only while the robot is stopped (QUIETGYRO): the firmware does
no I2C while a motor runs, because motor noise corrupted the bus; the chip buffers its samples
and they are drained after each move. Moves stay under 300 ms so that buffer cannot overflow.

  forward   front sensor vs the checkpoint's front reading
  sideways  the side sensor(s) the checkpoint had; at P the 45 deg pair. A strafe turns this
            robot, so each strafe direction learns how much opposite rotation to mix in
  heading   held after every move: back to the goal if it drifted more than 3 deg
  turn 90   gyro rotation to the held heading +/- 90, finer power under 12 deg to go
  park      straight creep to the box's front reading, side corrected, heading held

Run: python3 tools/seek_steps.py calibration/field-2026-10-30.json [first_step] [last_step] [left|right]
Steps: 1 centre at P, 2 straight to N, 3 colour turn, 4 park. Space/STOP in the console stops it.
"""
import json
import math
import statistics
import sys
import time
import urllib.request
from pathlib import Path

BASE = "http://127.0.0.1:8766"
ROOT = Path(__file__).resolve().parents[1]
WMAP = "WMAP 1 1 2 -1 3 1 4 -1"   # FL FR RL RR, from the console's Motors panel (30 Oct)
GREEN, RED = 0.83, 1.66           # colour R/G references (console defaults, confirmed 30 Oct)
KEYS = ("front", "side_left", "side_right", "left", "right")   # REF order
IDX = {"left": 0, "right": 1, "front": 2, "side_left": 3, "side_right": 4}   # ultrasonic_mm order
# Side readings that face back toward the start see no field wall: never used as targets.
DROP = {"turned_left": ("side_left",), "left_box": ("side_left",),
        "turned_right": ("side_right",), "right_box": ("side_right",)}

PWM = {"x": 85, "y": 100, "w": 100}          # power per move type (strafing and turning need more)
ROT_PWM, ROT_FINE_PWM = 120, 80               # rotation power; under 12 deg to go the gentler one
HOLD_TOL = 4                                  # deg: the gentlest rotation pulse still turns ~7 deg
TOL = {"x": 20, "y": 20, "beam": 20}          # mm: close enough
FRACTION = 0.6                                # cover this share of the error per move: undershoot
DEAD_MS = 60                                  # soft start: the first ~60 ms barely moves the robot
T_MIN, T_MAX = 120, 300                       # ms per move: under ~110 ms the soft start never reaches moving power;
                                              # a 600 ms drive veered 200 mm off line (30 Oct); the gyro
                                              # FIFO holds ~0.43 s, so a move must stay under ~300 ms
SETTLE_S = 0.35                               # wait after a move before measuring
SCANS = 3                                     # fresh ultrasonic scans per measurement (median)
SIDE_WINDOW = 250                             # a side reading this far from its target is another surface
PROBE_MS = 120                                # heading probe rotation each way
SPREAD = 40                                   # mm: a sensor varying more across one measurement is not trusted
BEAM_WINDOW = 150                             # a 45 deg beam this far off its target is on another surface


def get(path):
    return json.load(urllib.request.urlopen(BASE + path, timeout=5))


def post(path, body):
    req = urllib.request.Request(BASE + path, json.dumps(body).encode(), {"Content-Type": "application/json"})
    return json.load(urllib.request.urlopen(req, timeout=5))


class Abort(Exception):
    pass


class Link:
    def __init__(self, log_path):
        self.seq = get("/api/lines?after=0")["seq"]
        self.last_rx = time.time()
        self.log = open(log_path, "a")

    def record(self, **event):
        event["t"] = round(time.time(), 3)
        self.log.write(json.dumps(event) + "\n")
        self.log.flush()

    def poll(self):
        d = get("/api/lines?after=%d" % self.seq)
        self.seq = d["seq"]
        for line in d["lines"]:
            if line.get("type") == "sample":
                self.last_rx = time.time()
            elif "error" in line:
                raise Abort("robot error: " + line["error"])
            elif line.get("type") == "stop":
                raise Abort("STOP received (console or operator)")
            elif line.get("type") == "marker" and "REBOOT" in line.get("text", ""):
                raise Abort(line["text"])
        if time.time() - self.last_rx > 2.5:
            raise Abort("no data from the robot for 2.5 s")
        return d["lines"]

    def send(self, command, ack=None, timeout=3.0):
        post("/api/command", {"command": command})
        if not ack:
            return None
        t0 = time.time()
        while time.time() - t0 < timeout:   # one command at a time: the Mega's input buffer is 64 bytes
            for line in self.poll():
                if line.get("type") == ack:
                    return line
            time.sleep(0.03)
        raise Abort("no %s after %s" % (ack, command))

    def yaw(self):
        """CCW-positive heading from the last measured sample (QUIETGYRO: drained while stopped);
        None if the gyro is not zeroed. After lost frames (FIFO overflow, bus error) the angle
        keeps integrating but carries an unknown offset: lost() tells the caller to re-base."""
        g = getattr(self, "last_gyro", {})
        if not g.get("bias_valid") or "angle_deg" not in g:
            return None
        return -g["angle_deg"][2] * 1.009   # chip Z points down; 4 x 90 deg read 357.4 (27 Oct)

    def lost(self):
        """Count of gyro data losses so far (overflows + restarts + resets)."""
        g = getattr(self, "last_gyro", {})
        return g.get("overflows", 0) + g.get("gyro_restarts", 0) + g.get("gyro_resets", 0)

    def stop(self):
        for _ in range(3):
            try:
                post("/api/command", {"command": "STOP"})
            except Exception:
                pass

    def measure(self):
        """Median of SCANS fresh samples per sensor (-1 = fewer than two echoes); colour ratio."""
        rows, colour = [], []
        self.poll()        # drop samples queued before now: some were taken while still moving
        skip = 1           # and the next one, whose ping sweep may have started before the stop
        t0 = time.time()
        while len(rows) < SCANS:
            if time.time() - t0 > 4:
                raise Abort("no fresh samples")
            for line in self.poll():
                if line.get("type") == "sample" and not line.get("fast"):
                    if skip:
                        skip -= 1
                        continue
                    rows.append(line["ultrasonic_mm"])
                    self.last_gyro = line.get("gyro") or {}
                    c = line.get("color_rgbc") or [0, 0]
                    if c[0] and c[1]:
                        colour.append(c[0] / c[1])
            time.sleep(0.05)
        out = []
        for i in range(5):
            vals = [r[i] for r in rows if r[i] > 0]
            ok = len(vals) >= 2 and max(vals) - min(vals) <= SPREAD
            out.append(statistics.median(vals) if ok else -1)
        return out, (statistics.median(colour) if colour else None)

    def pulse(self, x, y, w, pwm, ms):
        ms = int(max(T_MIN, min(T_MAX, ms)))
        self.record(kind="pulse", x=x, y=y, w=w, pwm=pwm, ms=ms)
        post("/api/command", {"command": "PULSE %d %d %d %d %d" % (x, y, w, pwm, ms)})
        t0 = time.time()
        while time.time() - t0 < ms / 1000 + 2.0:
            if any(l.get("type") == "pulse_done" for l in self.poll()):
                time.sleep(SETTLE_S)
                return ms
            time.sleep(0.02)
        raise Abort("pulse did not finish")


class Seeker:
    def __init__(self, link):
        self.link = link
        self.k = {"x": 0.35, "y": 0.25}   # mm of error removed per ms of move (learned)
        self.slope = None                 # heading: mm per ms of CCW rotation, per beam (learned)
        self.goal = None                  # gyro heading to hold (deg, CCW +); set by the first hold
        self.goal_lost = 0                # gyro data-loss count when the goal was set
        self.wcomp = {}                   # rotation mixed into a move (axis, sign) to cancel its spin
        self.rate = {}                    # deg per ms beyond the dead time, per rotation power

    def refs(self, target, cal):
        ref = dict(cal[target]["ref"])
        for k in DROP.get(target, ()):
            ref[k] = 9999
        return {k: v for k, v in ref.items() if v != 9999}

    def errors(self, ref, us, creep, beam_window=BEAM_WINDOW):
        """ex (+ = drive forward), ey (+ = strafe left), beam errors; None = not measurable."""
        def r(k):
            v = us[IDX[k]]
            return v if v > 0 else None
        ex = r("front") - ref["front"] if "front" in ref and r("front") else None
        def side(k):
            v = r(k)
            return v if k in ref and v and abs(v - ref[k]) < SIDE_WINDOW else None
        ls, rs = side("side_left"), side("side_right")
        ey, sideways_from_45 = None, False
        if "side_left" in ref and "side_right" in ref:
            if ls and rs:
                ey = ((ls - rs) - (ref["side_left"] - ref["side_right"])) / 2
        elif "side_right" in ref and rs:
            ey = ref["side_right"] - rs
        elif "side_left" in ref and ls:
            ey = ls - ref["side_left"]
        if ey is None and creep and "left" in ref and "right" in ref and r("left") and r("right"):
            ey, sideways_from_45 = ((r("left") - r("right")) - (ref["left"] - ref["right"])) * 0.354, True
        beams = {}
        if not sideways_from_45:
            for k in ("left", "right"):
                if k in ref and ref[k] >= 200 and r(k) and abs(ref[k] - r(k)) <= beam_window:
                    beams[k] = ref[k] - r(k)
        return ex, ey, beams

    def translate(self, axis, err, max_ms=T_MAX):
        """One move along x or y toward the target, then a fresh measurement. A strafe turns this
        robot (the rear wheels push harder): each strafe direction learns how much opposite
        rotation to mix in from the gyro change it caused. Returns (ms, readings)."""
        ms = min(max_ms, DEAD_MS + FRACTION * abs(err) / self.k[axis])
        if axis == "y":
            ms = min(ms, 200)   # a 300 ms strafe also rotated the robot (30 Oct)
        sign = 1 if err > 0 else -1
        before = self.link.yaw()
        w = self.wcomp.get((axis, sign), 0)
        ms = self.link.pulse(sign * 100 if axis == "x" else 0, sign * 100 if axis == "y" else 0, w, PWM[axis], ms)
        us, _ = self.link.measure()
        after = self.link.yaw()
        if before is not None and after is not None:   # forward moves drift ~5 deg CW, strafes CCW
            spun = after - before
            gain = 2 if axis == "y" else 1.5
            self.wcomp[(axis, sign)] = int(max(-40, min(40, w - gain * spun)))
            self.link.record(kind="move_spin", axis=axis, sign=sign, spun=spun, wcomp=self.wcomp[(axis, sign)])
        return ms, us

    def hold(self, us):
        """Keep the gyro heading: back to the goal if a move turned the robot more than 3 deg."""
        yaw = self.link.yaw()
        if yaw is None:
            return us
        if self.goal is None or self.link.lost() != self.goal_lost:
            if self.goal is not None:
                print("    gyro lost data: holding the current heading from here", flush=True)
            self.goal, self.goal_lost = yaw, self.link.lost()
        if abs(self.goal - yaw) <= HOLD_TOL:
            return us
        print("    heading %.1f, holding %.1f" % (yaw, self.goal), flush=True)
        self.rotate_to(self.goal)
        return self.link.measure()[0]

    def learn(self, axis, before, after, ms):
        if before is None or after is None:
            return
        moved = abs(before) - abs(after) if before * after > 0 else abs(before) + abs(after)
        if moved < -25:
            if axis == "x":
                raise Abort("%s move went the wrong way (%.0f -> %.0f mm)" % (axis, before, after))
            return   # sideways from the jumpy 45 deg beams: a bad reading, not a reason to stop
        if moved > 5:
            k = moved / max(10, ms - DEAD_MS)
            new = 0.5 * self.k[axis] + 0.5 * k
            self.k[axis] = max(0.05, min(1.5, max(self.k[axis] / 1.5, min(self.k[axis] * 1.5, new))))
        else:
            self.k[axis] = max(0.05, self.k[axis] * 0.6)   # did not move: make the next move longer

    def probe_heading(self, ref, us0, creep, beam_window):
        """Find a rotation unit that measurably turns the robot: CCW pulses from 130 ms, 30 ms
        longer each time a pulse moves no beam by 15 mm (static friction makes rotation very
        non-linear). Returns {beam: mm per unit of CCW}, the unit in ms and the new readings."""
        unit = 130
        while unit <= 280:
            self.link.pulse(0, 0, 100, PWM["w"], unit)
            us1, _ = self.link.measure()
            gain = {}
            for k in ("left", "right"):
                a, b = us0[IDX[k]], us1[IDX[k]]
                if k in ref and a > 0 and b > 0 and abs(b - a) >= 15:
                    gain[k] = b - a
            self.link.record(kind="probe", unit=unit, us0=us0, us1=us1, gain=gain)
            print("    heading probe %d ms CCW: beam change %s" % (unit, gain or "none"), flush=True)
            if gain:
                return gain, unit, us1
            us0, unit = us1, unit + 30
        return {}, unit, us0

    def seek(self, name, target, cal, creep=False, max_moves=40, tol=20, heading=True, beam_tol=30,
             beam_window=BEAM_WINDOW):
        ref = self.refs(target, cal)
        print("%s -> %s targets %s" % (name, target, ref), flush=True)
        self.link.record(kind="seek_start", name=name, target=target, ref=ref)
        self.slope = None
        us, _ = self.link.measure()
        lost, last_axis, jumps, prev_front, moved_x = 0, "y", 0, None, 0
        for move in range(max_moves):
            ex, ey, beams = self.errors(ref, us, creep, beam_window)
            if not heading:
                beams = {}
            self.link.record(kind="state", move=move, us=us, ex=ex, ey=ey, beams=beams, k=self.k)
            fmt = lambda v: "-" if v is None else "%+.0f" % v
            print("  #%02d us %s  forward %s  sideways %s  beams %s" % (
                move, [int(v) for v in us], fmt(ex), fmt(ey), {k: round(v) for k, v in beams.items()}), flush=True)
            jumped = ex is not None and prev_front and abs(us[IDX["front"]] - prev_front) > 150 + moved_x
            if jumped and ey is not None and abs(ey) > tol:
                # The front beam slid off a narrow surface (the colour box edge, 30 Oct) after a
                # sideways move: do not drive on it; finish the sideways correction first.
                print("    front jumped %d -> %d without a forward move: sideways first" % (prev_front, us[IDX["front"]]), flush=True)
                _, us = self.translate("y", ey)
                us = self.hold(us)
                continue
            if jumped:
                jumps += 1
                if jumps <= 2:   # implausible jump without a forward/back move: measure again
                    us, _ = self.link.measure()
                    continue
            jumps = 0
            prev_front, moved_x = (us[IDX["front"]] if us[IDX["front"]] > 0 else prev_front), 0
            if ex is None:
                lost += 1
                if lost > 2:
                    raise Abort("front reading lost")
                us, _ = self.link.measure()
                continue
            x_ok = abs(ex) <= tol
            y_ok = ey is None and not any(k in ref for k in ("side_left", "side_right")) and not creep or \
                (ey is not None and abs(ey) <= tol)
            b_ok = all(abs(e) <= beam_tol for e in beams.values())
            if x_ok and y_ok and b_ok:
                print("  reached %s after %d moves" % (target, move), flush=True)
                self.link.record(kind="seek_end", target=target, result="reached", moves=move, us=us)
                return us
            # Largest remaining problem first; heading only once roughly in place.
            both_off = not x_ok and ey is not None and not y_ok
            if both_off and abs(ex) < 400:
                last_axis = "x"   # close in: line up sideways before driving on (keep the front beam on the box)
            if not x_ok and (y_ok or ey is None or (both_off and last_axis == "y")):
                last_axis = "x"
                ms, us2 = self.translate("x", ex)
                moved_x = 1000
                self.learn("x", ex, self.errors(ref, us2, creep, beam_window)[0], ms)
                us2 = self.hold(us2)
            elif ey is not None and not y_ok:
                last_axis = "y"
                ms, us2 = self.translate("y", ey)
                self.learn("y", ey, self.errors(ref, us2, creep, beam_window)[1], ms)
                us2 = self.hold(us2)
            elif beams and not b_ok:
                # Heading in whole rotation units (see probe_heading), one at a time with a fresh
                # measurement after each, so an overshoot is corrected by the next unit back.
                if not self.slope:
                    self.slope, self.unit, us = self.probe_heading(ref, us, creep, beam_window)
                    moved_x = 1000
                    if not self.slope:
                        print("    beams do not respond to rotation: heading left as is", flush=True)
                        self.link.record(kind="seek_end", target=target, result="reached_no_heading", moves=move, us=us)
                        return us
                    continue
                cur = self.errors(ref, us, creep, beam_window)[2]
                units = [cur[k] / self.slope[k] for k in cur if k in self.slope]
                if not units:
                    self.slope = None
                    continue
                n = sum(units) / len(units)   # rotation units still wanted, + = CCW
                gain = min(abs(self.slope[k]) for k in self.slope)
                if all(abs(cur[k]) <= max(beam_tol, gain / 2) for k in cur):
                    print("  reached %s (heading within half a rotation unit) after %d moves" % (target, move), flush=True)
                    self.link.record(kind="seek_end", target=target, result="reached_unit", moves=move, us=us)
                    return us
                self.link.pulse(0, 0, 100 if n > 0 else -100, PWM["w"], self.unit)
                print("    rotate one unit (%d ms) %s, %.1f units wanted" % (self.unit, "ccw" if n > 0 else "cw", n), flush=True)
                us2, _ = self.link.measure()
                us, moved_x = us2, 1000
                continue
            elif not x_ok:
                ms, us2 = self.translate("x", ex)
                moved_x = 1000
                self.learn("x", ex, self.errors(ref, us2, creep, beam_window)[0], ms)
                us2 = self.hold(us2)
            else:
                # Only y unmeasurable (side wall out of range): good enough on x and heading.
                print("  reached %s (sideways not measurable here) after %d moves" % (target, move), flush=True)
                self.link.record(kind="seek_end", target=target, result="reached_no_sideways", moves=move, us=us)
                return us
            us = us2
        raise Abort("%s: not reached in %d moves" % (target, max_moves))

    def park(self, target, cal, max_moves=40):
        """Final approach: creep straight forward to the box's front reading (short moves, half the
        remaining distance each), never forward once within 10 mm of it; strafe only if the
        back-wall side drifts more than 30 mm. No heading moves."""
        ref = self.refs(target, cal)
        side = next((k for k in ("side_right", "side_left") if k in ref), None)
        print("04 -> %s (straight creep) front %d, %s %s" % (target, ref["front"], side, ref.get(side)), flush=True)
        self.link.record(kind="park_start", target=target, ref=ref)
        us, _ = self.link.measure()
        prev, squared = None, False
        for move in range(max_moves):
            front = us[IDX["front"]]
            s_now = us[IDX[side]] if side else -1
            ey = None
            if side and s_now > 0 and abs(s_now - ref[side]) < SIDE_WINDOW:
                ey = ref[side] - s_now if side == "side_right" else s_now - ref[side]
            print("  #%02d us %s  front %s  sideways %s" % (move, [int(v) for v in us], front,
                                                          "-" if ey is None else "%+.0f" % ey), flush=True)
            self.link.record(kind="park", move=move, us=us, ey=ey)
            if front <= 0 or (prev and abs(front - prev) > 150):   # lost or implausible echo: look again
                prev, (us, _) = None, self.link.measure()
                continue
            prev = front
            if ey is not None and abs(ey) > 30:
                _, us = self.translate("y", ey)
            elif front > ref["front"] + 15:   # the shortest forward move still travels ~20 mm
                dist = front - ref["front"]
                _, us = self.translate("x", dist / FRACTION * 0.5, max_ms=200)
            else:
                yaw = self.link.yaw()
                if yaw is not None and self.goal is not None and abs(self.goal - yaw) > HOLD_TOL:
                    us = self.hold(us)
                    continue
                if not squared:
                    squared = True
                    us = self.square_to_beams(ref, us)
                    continue
                print("  parked: front %d (target %d), heading %s" % (front, ref["front"],
                      "-" if yaw is None or self.goal is None else "%+.1f deg off" % (yaw - self.goal)), flush=True)
                self.link.record(kind="park_end", us=us, yaw=yaw, goal=self.goal)
                return us
            us = self.hold(us)
        raise Abort("park: not finished in %d moves" % max_moves)

    def square_to_beams(self, ref, us, tries=3):
        """Absolute heading from the box capture (taken square): one small gyro-measured rotation
        gives each 45 deg beam's mm per degree; rotate by (target - reading) / slope, measured
        on the gyro. Corrects placement skew and gyro error that the held heading carries."""
        beams = [k for k in ("left", "right") if k in ref and ref[k] >= 200]
        if not beams:
            return us
        for attempt in range(tries):
            y0 = self.link.yaw()
            if y0 is None:
                return us
            us1, y1 = self.rotate_pulse(+1, y0)
            if y1 is None:
                return us1
            slope = {k: (us1[IDX[k]] - us[IDX[k]]) / (y1 - y0) for k in beams
                     if us[IDX[k]] > 0 and us1[IDX[k]] > 0 and abs(us1[IDX[k]] - us[IDX[k]]) >= 10}
            slope = {k: v for k, v in slope.items() if abs(v) >= 4}   # 2.4 mm/deg gave a wrong 9 deg fix (right box)
            fixes = [(ref[k] - us1[IDX[k]]) / slope[k] for k in slope]
            self.link.record(kind="square_beams", attempt=attempt, us0=us, us1=us1, dyaw=y1 - y0, slope=slope, fixes=fixes)
            if not fixes:
                print("    box beams do not respond to rotation: heading kept", flush=True)
                return us1
            fix = max(-12, min(12, sum(fixes) / len(fixes)))
            print("    box beams: %s mm/deg, heading fix %+.1f deg" % ({k: round(v, 1) for k, v in slope.items()}, fix), flush=True)
            self.goal, self.goal_lost = y1 + fix, self.link.lost()
            if abs(fix) <= 1.5:
                return us1
            self.rotate_to(self.goal, tol=1.5, strict=False)
            us, _ = self.link.measure()
        return us

    def rotate_pulse(self, direction, y0):
        """The gentlest rotation that measurably turns the robot (>= 1.5 deg), for probing:
        starts at the shortest move and lengthens by 30 ms while it does not turn."""
        ms = T_MIN
        while ms <= 240:
            self.link.pulse(0, 0, 100 * direction, ROT_FINE_PWM, ms)
            us, _ = self.link.measure()
            y1 = self.link.yaw()
            if y1 is None or abs(y1 - y0) >= 1.5:
                return us, y1
            ms += 30
        return us, None

    def rotate_to(self, goal, tol=HOLD_TOL, strict=True):
        """Gyro rotation, read only while stopped (QUIETGYRO). Each pulse aims at 80 % of the
        remaining angle from the rate measured at that power; under 12 deg it uses less power
        for finer steps. A pulse that barely turns makes the next one longer; an overshoot is
        corrected by the next pulse back."""
        start_ms, best, stalls = {ROT_PWM: 200, ROT_FINE_PWM: T_MIN}, None, 0   # untried rate: big ~30 deg, fine shortest
        lost0 = self.link.lost()
        for move in range(25 if strict else 6):
            yaw = self.link.yaw()
            if yaw is None or self.link.lost() != lost0:
                raise Abort("gyro not zeroed, or lost data during a rotation")
            err = goal - yaw
            self.link.record(kind="rotate", move=move, yaw=yaw, err=err)
            if abs(err) <= tol:
                return yaw
            if not strict:   # best effort: stop once pulses no longer get closer (pulse too coarse)
                stalls = stalls + 1 if best is not None and abs(err) >= best - 0.5 else 0
                best = abs(err) if best is None else min(best, abs(err))
                if stalls >= 2:
                    return yaw
            pwm = ROT_PWM if abs(err) > 12 else ROT_FINE_PWM
            ms = DEAD_MS + 0.8 * abs(err) / self.rate[pwm] if self.rate.get(pwm) else start_ms[pwm]
            ms = self.link.pulse(0, 0, 100 if err > 0 else -100, pwm, ms)
            self.link.measure()
            after = self.link.yaw()
            if after is None or self.link.lost() != lost0:
                raise Abort("gyro lost data during a rotation")
            turned = abs(after - yaw)
            print("    rotate %s %d ms @%d: %.1f -> %.1f (goal %.1f)" % ("ccw" if err > 0 else "cw", ms, pwm, yaw, after, goal), flush=True)
            if turned < 1.0:
                self.rate.pop(pwm, None)
                start_ms[pwm] = min(T_MAX, ms + 40)   # static friction: push longer
            else:
                r = turned / max(10, ms - DEAD_MS)
                self.rate[pwm] = r if pwm not in self.rate else 0.5 * self.rate[pwm] + 0.5 * r
        if not strict:
            return self.link.yaw()
        raise Abort("rotation: not within %.0f deg after 25 pulses" % tol)

    def square_to_wall(self, span=50, plateau=40):
        """Start: the robot faces the main wall at an unknown angle. Sweep the heading from
        -span to +span (gyro-measured, small steps) recording the front distance; a flat wall
        reads least when faced square, and the beam cone makes that a flat-bottomed dip, so
        the square heading is the middle of the readings within `plateau` mm of the least.
        Rotate there and hold it."""
        self.link.measure()
        y0 = self.link.yaw()
        if y0 is None:
            raise Abort("gyro not zeroed: keep the robot still")
        print("01 square to the wall: sweep %+d..%+d deg around %.1f" % (-span, span, y0), flush=True)
        self.rotate_to(y0 - span, tol=4)
        scan = []
        for step in range(30):
            us, _ = self.link.measure()
            yaw = self.link.yaw()
            if yaw is None:
                raise Abort("gyro lost data during the sweep")
            scan.append((yaw, us[IDX["front"]], us[IDX["left"]], us[IDX["right"]]))
            print("  sweep %+6.1f deg  front %5d  45L %5d  45R %5d" % (yaw - y0, *[int(v) for v in scan[-1][1:]]), flush=True)
            self.link.record(kind="sweep", yaw=yaw, us=us)
            if yaw >= y0 + span:
                break
            self.rotate_pulse(+1, yaw)
        # Flat stretches: consecutive front readings within `plateau` mm of each other (one
        # missing echo allowed). Other surfaces (short side walls, the colour box) make their
        # own stretches; the main wall is the one nearest the direction the robot was placed.
        runs, cur = [], []
        for yaw, f, _, _ in scan:
            if f <= 0:
                continue
            if cur and abs(f - cur[-1][1]) > plateau:
                runs.append(cur)
                cur = []
            cur.append((yaw, f))
        runs.append(cur)
        runs = [r for r in runs if len(r) >= 3]
        if not runs:
            raise Abort("sweep: no flat stretch of front readings (wall not seen)")
        mid = lambda r: (r[0][0] + r[-1][0]) / 2
        wall = min(runs, key=lambda r: abs(mid(r) - y0))
        normal, dist = mid(wall), statistics.median(f for _, f in wall)
        edge = wall[0][0] <= scan[0][0] + 2 or wall[-1][0] >= scan[-1][0] - 2
        print("  main wall %d mm, flat from %+.1f to %+.1f; square heading %+.1f from the start%s" % (
            dist, wall[0][0] - y0, wall[-1][0] - y0, normal - y0, "  WARNING: at the sweep edge" if edge else ""), flush=True)
        self.link.record(kind="wall_normal", normal=normal, dist=dist, runs=runs, edge=edge)
        self.goal, self.goal_lost = normal, self.link.lost()
        self.rotate_to(normal, tol=3, strict=False)
        return self.link.measure()[0]

    def find_p(self, cal, steps_each_way=14, good=70):
        """After square_to_wall: onto P's line (front = P's front), then strafe along it
        comparing the 45 deg beams with P's. Keep going while the match improves, reverse after
        it worsens twice; at most `steps_each_way` strafes on each side (stay in the sector)."""
        ref = self.refs("p_point", cal)
        match = lambda u: sum(abs(u[IDX[k]] - ref[k]) if u[IDX[k]] > 0 else 400 for k in ("left", "right"))
        us, _ = self.link.measure()
        for _ in range(8):   # forward/back onto P's line
            ex = us[IDX["front"]] - ref["front"] if us[IDX["front"]] > 0 else None
            if ex is None or abs(ex) <= 30:
                break
            _, us = self.translate("x", ex)
            us = self.hold(us)
        guess = ((us[IDX["left"]] - us[IDX["right"]]) - (ref["left"] - ref["right"])) if us[IDX["left"]] > 0 and us[IDX["right"]] > 0 else 1
        direction = 1 if guess > 0 else -1          # + = strafe left (creep formula sign)
        best, worse, steps = match(us), 0, {1: 0, -1: 0}
        print("01 find P along its line: front %d (P %d), beams %s vs P %d/%d, match %d; first try %s" % (
            us[IDX["front"]], ref["front"], [int(us[IDX["left"]]), int(us[IDX["right"]])], ref["left"], ref["right"],
            best, "left" if direction > 0 else "right"), flush=True)
        for i in range(2 * steps_each_way + 4):
            if best <= good:
                break
            if steps[direction] >= steps_each_way:
                direction = -direction
                if steps[direction] >= steps_each_way:
                    break
            _, us = self.translate("y", direction * 80)
            steps[direction] += 1
            us = self.hold(us)
            if us[IDX["front"]] > 0 and abs(us[IDX["front"]] - ref["front"]) > 40:
                _, us = self.translate("x", us[IDX["front"]] - ref["front"])
            m = match(us)
            print("  strafe %s: beams %s match %d (best %d)" % ("left" if direction > 0 else "right",
                  [int(us[IDX["left"]]), int(us[IDX["right"]])], m, best), flush=True)
            self.link.record(kind="find_p", direction=direction, us=us, match=m, best=best)
            if m < best - 10:
                best, worse = m, 0
            elif m > best + 30:   # clearly worse (the beams are jumpy): two of these and reverse
                worse += 1
                if worse >= 2:
                    direction, worse = -direction, 0
        print("  P search done: match %d" % best, flush=True)
        return us

    def turn_to(self, target, cal, ccw):
        """The colour turn: 90 deg from the held heading (or the current one) on the gyro."""
        self.link.measure()
        start = self.goal if self.goal is not None and self.link.lost() == self.goal_lost else self.link.yaw()
        if start is None:
            raise Abort("gyro not valid for the turn (not zeroed or lost frames)")
        self.goal, self.goal_lost = start + (90 if ccw else -90), self.link.lost()
        print("turn %s on the gyro: %.1f -> %.1f deg" % ("left" if ccw else "right", start, self.goal), flush=True)
        yaw = self.rotate_to(self.goal)
        print("  turned: %.1f deg" % (yaw - start), flush=True)


def colour_name(ratio):
    split = math.sqrt(GREEN * RED)
    conf = min(1, abs(math.log(ratio / split)) / (math.log(RED / GREEN) / 2))
    return ("RED" if ratio > split else "GREEN"), conf


def main():
    cal = json.load(open(sys.argv[1]))["targets"]
    first = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    last = int(sys.argv[3]) if len(sys.argv) > 3 else 4
    log_dir = ROOT / ".local-data" / "seek-runs"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / time.strftime("seek-%Y%m%d-%H%M%S.jsonl")
    link = Link(log_path)
    seeker = Seeker(link)
    post("/api/mark", {"text": "seek_steps %d-%d started (ultrasonic only)" % (first, last)})
    try:
        link.send("QUIETGYRO", ack="quietgyro")
        link.send(WMAP, ack="wmap")
        link.measure()
        seeker.goal, seeker.goal_lost = link.yaw(), link.lost()   # placed square: hold this heading
        if first <= 1 <= last:
            seeker.square_to_wall()
            seeker.find_p(cal)
            seeker.seek("01", "p_point", cal, creep=True, tol=30, heading=False)
        if first <= 2 <= last:
            seeker.seek("02", "centre", cal, tol=30, heading=False)
        turn = None
        if first <= 3 <= last:
            _, ratio = link.measure()
            name, conf = colour_name(ratio)
            print("03 colour %s (R/G %.3f, confidence %.0f%%)" % (name, ratio, 100 * conf), flush=True)
            link.record(kind="colour", name=name, ratio=ratio, confidence=conf)
            if conf < 0.5:
                raise Abort("colour unclear")
            turn = "left" if name == "GREEN" else "right"
            seeker.turn_to("turned_" + turn, cal, ccw=turn == "left")
            seeker.seek("03", "turned_" + turn, cal, tol=40, heading=False)
        if first <= 4 <= last:
            if turn is None:
                turn = sys.argv[4] if len(sys.argv) > 4 else sys.exit("step 4 alone needs left|right")
            seeker.park(turn + "_box", cal)
        print("DONE", flush=True)
        post("/api/mark", {"text": "seek_steps finished"})
    except (Abort, KeyboardInterrupt) as e:
        link.stop()
        post("/api/mark", {"text": "seek_steps STOPPED: %s" % e})
        print("STOPPED:", e)
        sys.exit(1)
    finally:
        print("log:", log_path)


if __name__ == "__main__":
    main()
