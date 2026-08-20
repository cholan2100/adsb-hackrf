import os
import tempfile

import numpy as np

import adsb_hackrf as A


def hx(m):
    return int(m, 16)


def check(cond, name):
    if not cond:
        raise AssertionError("FAIL: " + name)
    print("ok:", name)


# --- CRC ---
check(A.crc_ok(hx("8D406B902015A678D4D220AA4BDA"), 112), "crc ok good long")
check(A.crc_ok(hx("8D406B902015A678D4D220AA4BDB"), 112) == False, "crc bad long")
check(A.crc_ok(hx("8D485020994409940838175B284F"), 112), "crc ok velocity")
check(A.crc_ok(hx("8D40058B58C901375147EFD09357"), 112), "crc ok pos even")
check(A.crc_ok(hx("8D40058B58C904A87F402D3B8C59"), 112), "crc ok pos odd")

# --- DF / ICAO / TC ---
check(A.df_of(hx("8D406B902015A678D4D220AA4BDA"), 112) == 17, "df17")
check(A.typecode(hx("8D406B902015A678D4D220AA4BDA")) == 4, "tc4")
check(A.callsign(hx("8D406B902015A678D4D220AA4BDA")) == "EZY85MH", "callsign")
check(A.category(hx("8D406B902015A678D4D220AA4BDA")) == 0, "category")

# --- altitude ---
check(A.airborne_altitude(hx("8D40058B58C901375147EFD09357")) == 39000, "alt 39000")

# --- velocity ---
v = A.airborne_velocity(hx("8D485020994409940838175B284F"))
check(v[0] == 159 and abs(v[1] - 182.88) < 0.1 and v[2] == -832 and v[3] == "GS", "velocity vgs %r" % (v,))
v2 = A.airborne_velocity(hx("8DA05F219B06B6AF189400CBC33F"))
check(v2[0] == 375 and abs(v2[1] - 243.98) < 0.1 and v2[2] == -2304 and v2[3] == "TAS", "velocity tas %r" % (v2,))
sv = A.surface_state(hx("8FC8200A3AB8F5F893096B000000"))
check(sv[0] == 19 and abs(sv[1] - 42.2) < 0.1, "surface vel %r" % (sv,))

# --- airborne position even+odd ---
even = hx("8D40058B58C901375147EFD09357")
odd = hx("8D40058B58C904A87F402D3B8C59")
p00 = A.airborne_position(even, odd, even_newer=False)
p01 = A.airborne_position(even, odd, even_newer=True)
check(abs(p00[0] - 49.81755) < 0.002 and abs(p00[1] - 6.08442) < 0.002, "air pos odd-new %r" % (p00,))
check(49.0 < p01[0] < 50.0 and 5.5 < p01[1] < 6.5, "air pos even-new plausible %r" % (p01,))

# --- surface position even+odd with ref ---
se = hx("8CC8200A3AC8F009BCDEF2000000")
so = hx("8FC8200A3AB8F5F893096B000000")
sp = A.surface_position(se, so, -43.496, 172.558, even_newer=False)
check(abs(sp[0] - -43.48564) < 0.002 and abs(sp[1] - 172.53942) < 0.002, "surface pos %r" % (sp,))

# --- squawk (DF5/21) decode sanity ---
print("note: squawk not unit-tested against an external vector")

# ================= end-to-end synthetic demod =================
def make_valid(msg):
    data = msg >> 24
    rem = A.crc24_rem(data.to_bytes(11, "big") + b"\0\0\0")
    return (msg & ~0xFFFFFF) | rem


def make_signal(hexmsg):
    rng = np.random.default_rng(0)
    msg = int(hexmsg, 16)
    bits = format(msg, "0112b")
    n_slots = 16 + 112 * 2
    slot_vals = np.zeros(n_slots, dtype=np.float64)
    for s in range(16):
        if s in (0, 2, 7, 9):
            slot_vals[s] = 1.0
    for k, b in enumerate(bits):
        if b == "1":
            slot_vals[16 + 2 * k] = 1.0
        else:
            slot_vals[16 + 2 * k + 1] = 1.0
    samples = np.repeat(slot_vals, 4)
    noise = rng.uniform(0.0, 0.12, samples.size)
    samples = (samples + noise) * 300.0
    slots = samples.reshape(-1, 4).sum(axis=1)
    return slots, msg


for raw in (
    "8D406B902015A678D4D220AA4BDA",
    "8D485020994409940838175B284F",
    "8D40058B58C901375147EFD09357",
    "8FC8200A3AB8F5F893096B000000",
):
    hxmsg = make_valid(int(raw, 16))
    slots, msg = make_signal("%028X" % hxmsg)
    frames = A.detect_frames(slots)
    check(len(frames) == 1, "synthetic one frame %s" % raw)
    fm = frames[0][0]
    check(fm == msg, "synthetic bits reproduce %s" % raw)
    check(A.crc_ok(fm, frames[0][1]), "synthetic crc %s" % raw)

# two frames back to back (120us apart)
slots1, msg1 = make_signal("8D406B902015A678D4D220AA4BDA")
slots2, msg2 = make_signal("8D40058B58C901375147EFD09357")
gap = np.zeros(240, dtype=np.float64)
both = np.concatenate([slots1, gap, slots2])
frames = A.detect_frames(both)
check(len(frames) == 2, "synthetic two frames")

# ============ raw-file save/replay (offline --from-raw path) ============
def make_raw(hexmsg, sps=4, seed=0, amp=300.0, noise=0.12):
    rng = np.random.default_rng(seed)
    msg = int(hexmsg, 16)
    bits = format(msg, "0112b")
    n = 16 + 112 * 2
    sv = np.zeros(n)
    for s in range(16):
        if s in (0, 2, 7, 9):
            sv[s] = 1.0
    for k, b in enumerate(bits):
        if b == "1":
            sv[16 + 2 * k] = 1.0
        else:
            sv[16 + 2 * k + 1] = 1.0
    samples = np.repeat(sv, sps)
    i = (samples + rng.uniform(0, noise, samples.size)) * amp
    q = rng.uniform(-noise, noise, samples.size) * amp
    out = np.empty(samples.size * 2, dtype=np.int16)
    out[0::2] = np.clip(i, -32767, 32767).astype(np.int16)
    out[1::2] = np.clip(q, -32767, 32767).astype(np.int16)
    return out, msg


def replay_check(rate, sps):
    r1, m1 = make_raw("8D406B902015A678D4D220AA4BDA", sps=sps, seed=10)
    r2, m2 = make_raw("8D40058B58C901375147EFD09357", sps=sps, seed=11)
    lead = np.zeros(4096 * 8, dtype=np.int16)
    gap = np.zeros(2000 * 8, dtype=np.int16)
    trail = np.zeros(4096 * 8, dtype=np.int16)
    raw_stream = np.concatenate([lead, r1, gap, r2, trail])
    path = os.path.join(tempfile.gettempdir(), "replay_test.c16")
    raw_stream.tofile(path)

    class NS:
        pass

    ns = NS()
    ns.from_raw = path
    ns.lat = None
    ns.lon = None
    ns.rate = rate
    rec = A.Receiver(ns)
    rec.start()
    data = np.fromfile(rec._rawsrc, dtype="<i2")
    outs = rec.feed(data)
    check(len(outs) == 2, "raw-file replay @%.0fMHz finds 2 frames (got %d)" % (rate / 1e6, len(outs)))
    check(sorted(o[0] for o in outs) == sorted(["%028X" % m1, "%028X" % m2]),
          "raw-file replay messages @%.0fMHz" % (rate / 1e6))
    check(rec.decoder.n_good == 2, "raw-file replay crc_ok @%.0fMHz" % (rate / 1e6))
    rec.stop()
    os.remove(path)


replay_check(8e6, 4)
replay_check(2e6, 1)

# --- test adsb_sdr ---
import adsb_sdr as S
check(S.crc_ok(hx("8D406B902015A678D4D220AA4BDA"), 112), "adsb_sdr: crc ok good long")
check(S.df_of(hx("8D406B902015A678D4D220AA4BDA"), 112) == 17, "adsb_sdr: df17")

def replay_check_sdr(rate, sps):
    r1, m1 = make_raw("8D406B902015A678D4D220AA4BDA", sps=sps, seed=10)
    r2, m2 = make_raw("8D40058B58C901375147EFD09357", sps=sps, seed=11)
    lead = np.zeros(4096 * 8, dtype=np.int16)
    gap = np.zeros(2000 * 8, dtype=np.int16)
    trail = np.zeros(4096 * 8, dtype=np.int16)
    raw_stream = np.concatenate([lead, r1, gap, r2, trail])
    path = os.path.join(tempfile.gettempdir(), "replay_test_sdr.c16")
    raw_stream.tofile(path)

    class NS:
        pass

    ns = NS()
    ns.device = "hackrf"
    ns.device_args = ""
    ns.from_raw = path
    ns.lat = None
    ns.lon = None
    ns.rate = rate
    rec = S.Receiver(ns)
    rec.start()
    data = np.fromfile(rec._rawsrc, dtype="<i2")
    outs = rec.feed(data)
    check(len(outs) == 2, "adsb_sdr: raw replay @%.0fMHz finds 2 frames" % (rate / 1e6))
    check(sorted(o[0] for o in outs) == sorted(["%028X" % m1, "%028X" % m2]),
          "adsb_sdr: raw replay messages @%.0fMHz" % (rate / 1e6))
    check(rec.decoder.n_good == 2, "adsb_sdr: raw replay crc_ok @%.0fMHz" % (rate / 1e6))
    rec.stop()
    os.remove(path)

replay_check_sdr(2e6, 1)

print("ALL TESTS PASSED")