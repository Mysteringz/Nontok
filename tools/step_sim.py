"""Arena simulation of the hardware probe's tournament step controller.

Mirrors goUpdate()/stepService() in firmware/hardware_probe_mega/src/main.cpp: same constants,
control law, 5 ms control, 10 Hz ranges, braking. Keep the two in step when tuning. The arena,
sensor mounts, motor stall PWM and speeds are assumptions, not measurements.
Run: python3 tools/step_sim.py
"""
import math
W, BACK, MAXR = 750.0, 2000.0, 3000.0           # side walls x=+/-750, back wall y=2000
MOUNTS = [(150, 95, 45), (150, -95, -45), (150, 0, 0), (0, 115, 90), (0, -115, -90)]  # fwd, left, angle
def ray(px, py, ang):
    dx, dy = -math.sin(ang), math.cos(ang); best = MAXR + 1
    if dy > 1e-9: t = (BACK - py) / dy; x = px + dx * t; best = min(best, t) if abs(x) <= W else best
    for wx in (-W, W):
        if abs(dx) > 1e-9:
            t = (wx - px) / dx
            if t > 0 and 0 <= py + dy * t <= BACK: best = min(best, t)
    return best
def sense(x, y, psi):
    out = []
    for f, l, a in MOUNTS:
        sx = x + f * -math.sin(psi) + l * -math.cos(psi); sy = y + f * math.cos(psi) + l * -math.sin(psi)
        d = min(ray(sx, sy, psi + math.radians(a + k)) for k in (-7.5, -3.75, 0, 3.75, 7.5))  # 15 deg cone
        out.append(int(d) if 20 <= d <= MAXR else -1)
    return out
def refs(pose, mode):
    l45, r45, fr, ls, rs = sense(*pose)
    lat = (ls - rs) if mode <= 2 else (rs if mode == 3 else ls)
    return dict(front=fr if fr > 0 else 0, lat=lat, h45=l45 - r45, mean45=(l45 + r45) // 2)
def clamp(v, a, b): return max(a, min(b, v))
def run(start, mode, ref, maxpwm=120, minpwm=55, turn=None, square=None, verbose=False):
    x, y, psi = start; t = 0.0; dt = 0.005; rate = 0.0
    yaw = lambda: math.degrees(psi)
    if turn is not None: target = (square if square is not None else yaw()) + turn
    elif mode == 1: target = yaw()
    elif mode == 2: target = square if square is not None else yaw()
    else: target = (square + (90 if mode == 3 else -90)) if square is not None else yaw()
    ex = ey = 9999.0; eh = 0.0; ux = uy = 0.0; settled = None; last_us = -1
    while t < (10 if turn is not None else 30):
        if turn is None and int(t / 0.1) != last_us:                           # 10 Hz ranges
            last_us = int(t / 0.1); l45, r45, fr, ls, rs = sense(x, y, psi); okx = oky = True
            if ref['front'] > 0 and fr > 0: ex = fr - ref['front']
            elif ref['mean45'] > 0 and l45 > 0 and r45 > 0: ex = (l45 + r45) / 2 - ref['mean45']
            else: okx = False; ex = 9999
            if mode <= 2 and ls > 0 and rs > 0: ey = ((ls - rs) - ref['lat']) / 2
            elif mode == 3 and rs > 0: ey = -(rs - ref['lat'])
            elif mode == 4 and ls > 0: ey = ls - ref['lat']
            else: oky = False; ey = 9999
            eh = (l45 - r45) - ref['h45'] if l45 > 0 and r45 > 0 else 0
            if mode == 1 and l45 > 0 and r45 > 0 and oky and abs(ey) < 40 and abs(eh) > 8 and (fr <= 0 or fr > (ls + rs) / 2):
                target += clamp(0.02 * eh, -1, 1)
            ux = (clamp(ex / 250, -1, 1) if abs(ex) > 12 else 0) if okx else (0.3 if mode == 1 else 0)
            uy = clamp(ey / 150, -1, 1) if oky and abs(ey) > 8 else 0
            if 0 < fr < 70 and ux > 0: ux = 0
        e = target - yaw(); coast = e * rate > 0 and abs(e) < abs(rate) * 0.06   # brake: coast the last 60 ms
        if turn is not None: uw = 0 if abs(e) < 0.6 or coast else clamp(e / 25, -1, 1); ux = uy = 0
        else: uw = 0 if abs(e) < 0.8 or coast else clamp(e / 25, -1, 1)
        wheel = [ux - uy - uw, ux + uy + uw, ux + uy - uw, ux - uy + uw]; peak = max(1, *map(abs, wheel))
        pwm = [0 if abs(u / peak) < 0.03 else math.copysign(minpwm + (maxpwm - minpwm) * abs(u / peak), u) for u in wheel]
        spd = [0 if abs(p) < 45 else p / 255 for p in pwm]                       # motors stall under PWM 45
        FL, FR, RL, RR = spd
        vx, vy, w = (FL + FR + RL + RR) / 4 * 400, (-FL + FR + RL - RR) / 4 * 400, (-FL + FR - RL + RR) / 4 * math.radians(250)
        wv = math.degrees(w); rate += (wv - rate) * min(1, dt / 0.05)   # inertia: 50 ms lag
        x += (vx * -math.sin(psi) + vy * -math.cos(psi)) * dt; y += (vx * math.cos(psi) + vy * -math.sin(psi)) * dt; psi += math.radians(rate) * dt
        if turn is not None: ok = abs(target - yaw()) < 0.6 and abs(rate) < 2; hold = 0.4
        else: ok = ux == 0 and uy == 0 and abs(target - yaw()) < 1.5 and abs(ex) < 15 and abs(ey) < 10 and (mode != 1 or abs(eh) < 20); hold = 0.6
        settled = (settled if settled is not None else t) if ok else None
        if settled is not None and t - settled >= hold: return dict(ok=True, t=round(t, 1), x=round(x), y=round(y), yaw=round(yaw(), 2))
        t += dt
    return dict(ok=False, t=round(t, 1), x=round(x), y=round(y), yaw=round(yaw(), 2), ex=ex, ey=ey, eh=eh)

P, N = (0, 900, 0.0), (0, 1650, 0.0)
BOX_L, BOX_R = (-480, 1780, math.radians(90)), (480, 1780, math.radians(-90))
print("sensor view at P:", sense(*P), " at N:", sense(*N), " box L:", sense(*BOX_L))
for start in [(150, 700, math.radians(8)), (-200, 600, math.radians(-12)), (100, 1000, math.radians(5)), (0, 500, 0)]:
    r1 = run(start, 1, refs(P, 1)); sq = r1['yaw']
    st = (r1['x'], r1['y'], math.radians(r1['yaw']))
    r2 = run(st, 2, refs(N, 2), square=sq)
    st2 = (r2['x'], r2['y'], math.radians(r2['yaw']))
    rl = run(st2, 0, None, turn=90, square=sq); rr = run(st2, 0, None, turn=-90, square=sq)
    p3 = run((rl['x'], rl['y'], math.radians(rl['yaw'])), 3, refs(BOX_L, 3), square=sq)
    p4 = run((rr['x'], rr['y'], math.radians(rr['yaw'])), 4, refs(BOX_R, 4), square=sq)
    print(f"start x={start[0]:5.0f} y={start[1]:5.0f} yaw={math.degrees(start[2]):5.1f}")
    print("  01 P  :", r1); print("  02 N  :", r2); print("  03 L/R:", rl, rr); print("  04 box:", p3, p4)
