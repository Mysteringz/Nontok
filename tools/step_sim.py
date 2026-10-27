"""Arena simulation of the hardware probe's tournament step controller.

Mirrors stepService()/goUpdate()/probeNext() in firmware/hardware_probe_mega/src/main.cpp:
same constants, control law, 5 ms control, 10 Hz ranges, braking and the +/-3 deg heading
probe. Keep the two in step when tuning. The arena, sensor mounts, motor stall PWM, speeds
and the deliberately wrong gyro scale are assumptions, not measurements.
Run: python3 tools/step_sim.py
"""
import math

W, BACK, MAXR = 750.0, 2000.0, 3000.0
SIDE_FROM = {-W: 1000.0, W: 700.0}   # like the field at P: the right wall reaches it, the left does not
MOUNTS = [(150, 95, 45), (150, -95, -45), (150, 0, 0), (0, 115, 90), (0, -115, -90)]  # fwd, left, deg
GYRO_SCALE_ERROR = 0.97              # simulated gyro reads 3 % short: worse than the real 0.9 %
NO_REF = 9999


def ray(px, py, ang):
    dx, dy = -math.sin(ang), math.cos(ang)
    best = MAXR + 1
    if dy > 1e-9:
        t = (BACK - py) / dy
        if abs(px + dx * t) <= W:
            best = min(best, t)
    for wx in (-W, W):
        if abs(dx) > 1e-9:
            t = (wx - px) / dx
            if t > 0 and SIDE_FROM[wx] <= py + dy * t <= BACK:
                best = min(best, t)
    return best


def sense(x, y, psi):
    """Ranges in firmware order: left 45, right 45, front, left side, right side (-1 = no echo)."""
    out = []
    for f, l, a in MOUNTS:
        sx = x - f * math.sin(psi) - l * math.cos(psi)
        sy = y + f * math.cos(psi) - l * math.sin(psi)
        d = min(ray(sx, sy, psi + math.radians(a + k)) for k in (-7.5, -3.75, 0, 3.75, 7.5))  # 15 deg cone
        out.append(int(d) if 20 <= d <= MAXR else -1)
    return out


def capture(pose):
    """What the console sends as REF: every reading the capture had, NO_REF where it had none."""
    return [v if v > 0 else NO_REF for v in sense(*pose)]


def clamp(v, a, b):
    return max(a, min(b, v))


class Robot:
    def __init__(self, x, y, psi):
        self.x, self.y, self.psi, self.rate, self.t = x, y, psi, 0.0, 0.0

    def yaw(self):  # what the gyro reports, scale error included
        return math.degrees(self.psi) * GYRO_SCALE_ERROR

    def step(self, ux, uy, uw, maxpwm, minpwm, dt=0.005):
        wheel = [ux - uy - uw, ux + uy + uw, ux + uy - uw, ux - uy + uw]
        peak = max(1, *map(abs, wheel))
        pwm = [0 if abs(u / peak) < 0.03 else math.copysign(minpwm + (maxpwm - minpwm) * abs(u / peak), u) for u in wheel]
        FL, FR, RL, RR = [0 if abs(p) < 45 else p / 255 for p in pwm]   # motors stall under PWM 45
        vx, vy = (FL + FR + RL + RR) / 4 * 400, (-FL + FR + RL - RR) / 4 * 400
        w = math.degrees((-FL + FR - RL + RR) / 4 * math.radians(250))
        self.rate += (w - self.rate) * min(1, dt / 0.05)                  # 50 ms rotational lag
        self.x += (-vx * math.sin(self.psi) - vy * math.cos(self.psi)) * dt
        self.y += (vx * math.cos(self.psi) - vy * math.sin(self.psi)) * dt
        self.psi += math.radians(self.rate) * dt
        self.t += dt

    def pose(self):
        return dict(x=round(self.x), y=round(self.y), true_yaw=round(math.degrees(self.psi), 2))


def go(r, ref, target, creep, probe, maxpwm=120, minpwm=55):
    """One GO step; returns (reached, final gyro yaw, probe fixes)."""
    start, last_us, settled_at = r.t, -1, None
    ex = ey = 9999.0
    ux = uy = 0.0
    phase, probes, fixes, reads = "pos", 0, [], 0
    sums = {}
    probe_base = target
    while r.t - start < 40:
        fresh = int(r.t / 0.1) != last_us
        if fresh:
            last_us = int(r.t / 0.1)
            l45, r45, fr, ls, rs = us = sense(r.x, r.y, r.psi)
            okx = ref[2] != NO_REF and fr > 0
            ex = fr - ref[2] if okx else 9999
            oky = True
            if ref[3] != NO_REF and ref[4] != NO_REF:
                oky = ls > 0 and rs > 0
                ey = ((ls - rs) - (ref[3] - ref[4])) / 2 if oky else 9999
            elif ref[4] != NO_REF:
                oky = rs > 0
                ey = ref[4] - rs if oky else 9999
            elif ref[3] != NO_REF:
                oky = ls > 0
                ey = ls - ref[3] if oky else 9999
            else:
                oky = False
            if not oky and creep and ref[0] != NO_REF and ref[1] != NO_REF and l45 > 0 and r45 > 0:
                ey, oky = ((l45 - r45) - (ref[0] - ref[1])) * 0.354, True
            if not oky:
                ey = 9999
            ux = (clamp(ex / 250, -1, 1) if abs(ex) > 12 else 0) if okx else (0.3 if creep else 0)
            uy = clamp(ey / 150, -1, 1) if oky and abs(ey) > 8 else 0
            if fr > 0 and ux > 0 and (fr < 30 or (okx and fr < ref[2] - 20)):
                ux = 0
        e = target - r.yaw()
        coast = e * r.rate > 0 and abs(e) < abs(r.rate) * 0.06
        still = abs(e) < 0.6 and abs(r.rate) < 2
        uw = 0 if still or coast else clamp(e / 25, -1, 1)
        if phase in ("plus", "minus", "zero"):
            r.step(0, 0, uw, maxpwm, minpwm)
            if not still:
                settled_at = None
                continue
            if settled_at is None:
                settled_at, reads = r.t, 0
                continue
            if r.t - settled_at < 0.2 or not fresh:
                continue
            if reads == 0:
                sums[phase] = [[0, 0], [0, 0]]
            for b in (0, 1):
                if us[b] > 0:
                    sums[phase][b][0] += us[b]
                    sums[phase][b][1] += 1
            reads += 1
            if reads < 2:
                continue
            settled_at = None
            if phase == "plus":
                target, phase = probe_base - 3, "minus"
            elif phase == "minus":
                target, phase = probe_base, "zero"
            else:
                fix, wsum = 0.0, 0
                for b in (0, 1):
                    n = [sums[p][b][1] for p in ("plus", "minus", "zero")]
                    if ref[b] != NO_REF and ref[b] >= 200 and all(n):
                        mean = {p: sums[p][b][0] / sums[p][b][1] for p in ("plus", "minus", "zero")}
                        slope = (mean["plus"] - mean["minus"]) / 6
                        if abs(slope) >= 3:
                            fix += (ref[b] - mean["zero"]) / slope
                            wsum += 1
                fix = clamp(fix / wsum, -8, 8) if wsum else 0
                probes += 1
                fixes.append(round(fix, 2))
                target = probe_base + fix
                phase = "pos" if abs(fix) > 2 and probes < 2 else "final"
            continue
        r.step(ux, uy, 0 if abs(e) < 0.8 or coast else clamp(e / 25, -1, 1), maxpwm, minpwm)
        lateral = ref[3] != NO_REF or ref[4] != NO_REF
        pos_ok = ux == 0 and uy == 0 and abs(ex) < 15 and (not lateral or abs(ey) < 10)
        if pos_ok and abs(e) < 1.5:
            if settled_at is None:
                settled_at = r.t
            elif r.t - settled_at >= 0.6:
                settled_at = None
                if phase == "pos" and probe and any(ref[b] != NO_REF and ref[b] >= 200 for b in (0, 1)):
                    probe_base, target, phase = target, target + 3, "plus"
                else:
                    return True, r.yaw(), fixes
        else:
            settled_at = None
    return False, r.yaw(), fixes


def turn(r, target, maxpwm=120, minpwm=55):
    start, settled_at = r.t, None
    while r.t - start < 10:
        e = target - r.yaw()
        coast = e * r.rate > 0 and abs(e) < abs(r.rate) * 0.06
        still = abs(e) < 0.6 and abs(r.rate) < 2
        r.step(0, 0, 0 if still or coast else clamp(e / 25, -1, 1), maxpwm, minpwm)
        if still:
            settled_at = settled_at if settled_at is not None else r.t
            if r.t - settled_at >= 0.4:
                return True
        else:
            settled_at = None
    return False


P, N = (0, 900, 0.0), (0, 1650, 0.0)
TURNED = {90: (0, 1650, math.radians(90)), -90: (0, 1650, math.radians(-90))}
BOX = {90: (-480, 1780, math.radians(90)), -90: (480, 1780, math.radians(-90))}

if __name__ == "__main__":
    print("readings at P:", sense(*P), " N:", sense(*N), " turned L:", sense(*TURNED[90]), " box L:", sense(*BOX[90]))
    for start in [(150, 700, 8.0), (-200, 600, -12.0), (100, 1000, 5.0), (0, 500, 0.0)]:
        print(f"start x={start[0]} y={start[1]} yaw={start[2]} deg (gyro scale error {100 - 100 * GYRO_SCALE_ERROR:.0f} %)")
        for deg in (90, -90):
            r = Robot(start[0], start[1], math.radians(start[2]))
            ok1, ref_yaw, f1 = go(r, capture(P), r.yaw(), creep=True, probe=True)
            p1 = r.pose()
            ok2, ref_yaw, f2 = go(r, capture(N), ref_yaw, creep=False, probe=True)
            ok3 = turn(r, ref_yaw + deg)
            t_gyro = r.pose()["true_yaw"]
            ok3b, ref_yaw, f3 = go(r, capture(TURNED[deg]), r.yaw(), creep=False, probe=True)
            t_aligned = r.pose()["true_yaw"]
            ok4, _, f4 = go(r, capture(BOX[deg]), ref_yaw, creep=False, probe=True)
            print(f"  {'left ' if deg > 0 else 'right'}: 01 {ok1} {p1} fix {f1} | 02 {ok2} fix {f2} | "
                  f"03 gyro turn ends {t_gyro} deg, aligned {t_aligned} deg (fix {f3}) | 04 {ok4} {r.pose()} t={r.t:.0f}s")
