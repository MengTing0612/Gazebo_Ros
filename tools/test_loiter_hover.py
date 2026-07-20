#!/usr/bin/env python3
"""LOITER hover test: arm → STABILIZE takeoff → LOITER + neutral RC.

Uses same arming pattern as flight_bridge/mavlink_adapter.py.
Key: In LOITER, RC=[1500,1500,1500,1500] means "pilot neutral" → APM holds position+alt.
"""
import sys, time, math
from pymavlink import mavutil


URL = sys.argv[1] if len(sys.argv) > 1 else "tcp:127.0.0.1:5760"
TAKEOFF_THR = 1750
TARGET_ALT = 3.0
LOITER_S = 25


def send_rc(mav, r, p, t, y):
    mav.mav.rc_channels_override_send(mav.target_system, mav.target_component,
                                       r, p, t, y, 0, 0, 0, 0)


def wait_armed(mav, timeout=8):
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if msg and (msg.base_mode & 0x80):  # MAV_MODE_FLAG_SAFETY_ARMED
            return True
        send_rc(mav, 1500, 1500, 1000, 2000)  # keep rudder gesture
        time.sleep(0.05)
    return False


def get_alt(mav):
    msg = mav.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
    if msg and msg.relative_alt is not None:
        return msg.relative_alt / 1000.0
    return None


def main():
    print(f"→ Connecting {URL}")
    mav = mavutil.mavlink_connection(URL, robust_parsing=True)
    mav.wait_heartbeat()
    print(f"  OK sys={mav.target_system} comp={mav.target_component}")

    # ── Arm ────────────────────────────────────────────────────────
    print("→ Arming (MAV_CMD + rudder gesture)...")
    send_rc(mav, 1500, 1500, 1000, 1500)
    time.sleep(0.1)
    mav.mav.command_long_send(mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1.0, 0, 0, 0, 0, 0, 0)
    if not wait_armed(mav):
        print("  ARM FAILED"); return
    print("  Armed")

    # ── STABILIZE + takeoff ────────────────────────────────────────
    print(f"→ STABILIZE takeoff, target={TARGET_ALT}m")
    mav.set_mode(0)
    time.sleep(0.3)
    t0 = time.time()
    while True:
        send_rc(mav, 1500, 1500, TAKEOFF_THR, 1500)
        alt = get_alt(mav)
        elapsed = time.time() - t0
        if alt and elapsed - int(elapsed) < 0.06:
            print(f"  t={elapsed:4.1f}s  alt={alt:.2f}m")
        if alt and alt >= TARGET_ALT:
            print(f"  → reached {alt:.2f}m, switching LOITER"); break
        if elapsed > 20:
            print("  → timeout, switching anyway"); break
        time.sleep(0.05)

    # ── LOITER + neutral RC ────────────────────────────────────────
    print(f"→ LOITER mode, neutral RC [1500,1500,1500,1500] for {LOITER_S}s")
    mav.set_mode(5)
    time.sleep(0.5)

    # Verify
    hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
    print(f"  mode={hb.custom_mode}" if hb else "  no heartbeat")

    positions = []
    t0 = time.time()
    while time.time() - t0 < LOITER_S:
        send_rc(mav, 1500, 1500, 1500, 1500)  # ← NEUTRAL: APM does everything
        alt = get_alt(mav)
        gps = mav.recv_match(type="GPS_RAW_INT", blocking=False)
        elapsed = time.time() - t0
        if alt is not None and elapsed - int(elapsed) < 0.06:
            lat = gps.lat / 1e7 if gps and gps.lat else 0
            lon = gps.lon / 1e7 if gps and gps.lon else 0
            positions.append((elapsed, alt, lat, lon))
            print(f"  t={elapsed:5.1f}s  alt={alt:.2f}m  lat={lat:.6f}  lon={lon:.6f}")
        time.sleep(0.05)

    # ── Summary ────────────────────────────────────────────────────
    if positions:
        alts = [p[1] for p in positions]
        print(f"\n→ LOITER summary ({len(positions)} samples):")
        print(f"  alt: min={min(alts):.2f}  max={max(alts):.2f}  mean={sum(alts)/len(alts):.2f}")
        if len(positions) > 5:
            lats = [p[2] for p in positions]
            lons = [p[3] for p in positions]
            lat0, lon0 = positions[0][2], positions[0][3]
            drifts = [math.sqrt((la-lat0)**2 + (lo-lon0)**2) * 111320 for la, lo in zip(lats, lons)]
            print(f"  drift: max={max(drifts):.1f}m  final={drifts[-1]:.1f}m")

    # ── Land ───────────────────────────────────────────────────────
    print("→ Landing: STABILIZE, throttle down")
    mav.set_mode(0)
    for _ in range(100):
        send_rc(mav, 1500, 1500, 1300, 1500)
        time.sleep(0.05)
    send_rc(mav, 1500, 1500, 1000, 1500)
    time.sleep(0.5)
    mav.arducopter_disarm()
    print("Done.")


if __name__ == "__main__":
    main()
