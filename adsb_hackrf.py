#!/usr/bin/env python
import argparse
import math
import os
import sys
import time
from collections import deque

import numpy as np
import SoapySDR
from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_CS16

SAMPLE_RATE = 2_000_000
RAW_HDR = b"ADSBRAW1"
SLOTS_PER_FRAME = 240
CHAR_TABLE = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ#####_###############0123456789######"


# --------------------------------------------------------- CRC and bit tools
def crc24_rem(mbytes):
    G = [0xFF, 0xFA, 0x04, 0x80]
    work = list(mbytes)
    n = len(work)
    for ibyte in range(n - 3):
        for ibit in range(8):
            if work[ibyte] & (0x80 >> ibit):
                work[ibyte] ^= G[0] >> ibit
                work[ibyte + 1] ^= 0xFF & ((G[0] << (8 - ibit)) | (G[1] >> ibit))
                work[ibyte + 2] ^= 0xFF & ((G[1] << (8 - ibit)) | (G[2] >> ibit))
                work[ibyte + 3] ^= 0xFF & ((G[2] << (8 - ibit)) | (G[3] >> ibit))
    return (work[-3] << 16) | (work[-2] << 8) | work[-1]


def crc_ok(msg, nbits):
    return crc24_rem(msg.to_bytes(nbits // 8, "big")) == 0


def crc_fix(msg, nbits):
    nb = nbits // 8
    if crc24_rem(msg.to_bytes(nb, "big")) == 0:
        return msg, 0
    for i in range(nbits):
        m = msg ^ (1 << i)
        if crc24_rem(m.to_bytes(nb, "big")) == 0:
            return m, 1
    for i in range(max(0, nbits - 26), nbits):
        for j in range(i + 1, nbits):
            m = msg ^ (1 << i) ^ (1 << j)
            if crc24_rem(m.to_bytes(nb, "big")) == 0:
                return m, 2
    return None


def bitstr(msg, nbits):
    return format(msg, "0%db" % nbits)


def df_of(msg, nbits):
    return (msg >> (nbits - 5)) & 0x1F


def typecode(msg):
    df = (msg >> (112 - 5)) & 0x1F
    if df not in (17, 18):
        return None
    return (msg >> (112 - 37)) & 0x1F


def hexstr(msg, nbits):
    return "%0*X" % (nbits // 4, msg)


# -------------------------------------------------------- message field codes
def gray2int(num):
    num ^= num >> 8
    num ^= num >> 4
    num ^= num >> 2
    num ^= num >> 1
    return num


def decode_altcode(binstr):
    Mbit = binstr[6]
    Qbit = binstr[8]
    if int(binstr, 2) == 0:
        return None
    if Mbit == "0":
        if Qbit == "1":
            vbin = binstr[:6] + binstr[7] + binstr[9:]
            return int(vbin, 2) * 25 - 1000
        gc = (
            binstr[10] + binstr[12] + binstr[1] + binstr[3] + binstr[5]
            + binstr[7] + binstr[9] + binstr[11] + binstr[0] + binstr[2] + binstr[4]
        )
        n500 = gray2int(int(gc[:8], 2))
        n100 = gray2int(int(gc[8:], 2))
        if n100 in (0, 5, 6):
            return None
        if n100 == 7:
            n100 = 5
        if n500 % 2:
            n100 = 6 - n100
        return (n500 * 500 + n100 * 100) - 1300
    return int(binstr[:6] + binstr[7:], 2) * 3.28084


def airborne_altitude(msg):
    mb = bitstr(msg, 112)[32:]
    altbin = mb[8:20]
    return decode_altcode(altbin[:6] + "0" + altbin[6:])


def callsign(msg):
    csbin = bitstr(msg, 112)[40:88]
    cs = "".join(CHAR_TABLE[int(csbin[i * 6:i * 6 + 6], 2)] for i in range(8))
    return cs.replace("#", "").rstrip("_")


def category(msg):
    return int(bitstr(msg, 112)[37:40], 2)


def airborne_velocity(msg):
    mb = bitstr(msg, 112)[32:]
    subtype = int(mb[5:8], 2)
    if subtype not in (1, 2, 3, 4):
        return None
    if int(mb[14:24], 2) == 0 or int(mb[25:35], 2) == 0:
        return None
    if subtype in (1, 2):
        mult = 4 if subtype == 2 else 1
        v_ew = (-1 if mb[13] == "1" else 1) * (int(mb[14:24], 2) - 1) * mult
        v_ns = (-1 if mb[24] == "1" else 1) * (int(mb[25:35], 2) - 1) * mult
        spd = int(math.sqrt(v_ew * v_ew + v_ns * v_ns))
        trk = math.degrees(math.atan2(v_ew, v_ns))
        trk = trk if trk >= 0 else trk + 360
        spd_type = "GS"
    else:
        trk = int(mb[14:24], 2) / 1024 * 360.0
        spd = int(mb[25:35], 2)
        spd = None if spd == 0 else (spd - 1) * (4 if subtype == 4 else 1)
        spd_type = "TAS" if mb[24] == "1" else "IAS"
    vr_sign = -1 if mb[36] == "1" else 1
    vr = int(mb[37:46], 2)
    vs = None if vr == 0 else int(vr_sign * (vr - 1) * 64)
    return spd, trk, vs, spd_type


def surface_state(msg):
    mb = bitstr(msg, 112)[32:]
    trk = int(mb[13:20], 2) * 360 / 128 if mb[12] == "1" else None
    mov = int(mb[5:12], 2)
    if mov == 0 or mov > 124:
        spd = None
    elif mov == 1:
        spd = 0.0
    elif mov == 124:
        spd = 175.0
    else:
        mov_lb = [2, 9, 13, 39, 94, 109, 124]
        kts_lb = [0.125, 1, 2, 15, 70, 100, 175]
        step = [0.125, 0.25, 0.5, 1, 2, 5]
        i = next(k for k, v in enumerate(mov_lb) if v > mov)
        spd = kts_lb[i - 1] + (mov - mov_lb[i - 1]) * step[i - 1]
    return spd, trk


def squawk(msg):
    mbin = bitstr(msg, 56)
    b = mbin[19:32]
    C1, A1, C2, A2, C4, A4, X, B1, D1, B2, D2, B4, D4 = b
    return "%d%d%d%d" % (
        int(A4 + A2 + A1, 2),
        int(B4 + B2 + B1, 2),
        int(C4 + C2 + C1, 2),
        int(D4 + D2 + D1, 2),
    )


# ------------------------------------------------------------ CPR (position)
def cprNL(lat):
    if abs(lat) < 1e-6:
        return 59
    if abs(lat) > 87:
        return 1
    if abs(lat) == 87:
        return 2
    nz = 15
    a = 1 - math.cos(math.pi / (2 * nz))
    b = math.cos(math.radians(abs(lat))) ** 2
    return math.floor(2 * math.pi / math.acos(1 - a / b))


def cpr_fields(msg):
    mb = bitstr(msg, 112)[32:]
    oe = int(mb[21])
    lat = int(mb[22:39], 2) / 131072
    lon = int(mb[39:56], 2) / 131072
    return oe, lat, lon


def airborne_position(e0, o0, even_newer):
    _, lat_e, lon_e = cpr_fields(e0)
    _, lat_o, lon_o = cpr_fields(o0)
    j = math.floor(59 * lat_e - 60 * lat_o + 0.5)
    lat_e_f = (360 / 60) * (j % 60 + lat_e)
    lat_o_f = (360 / 59) * (j % 59 + lat_o)
    if lat_e_f >= 270:
        lat_e_f -= 360
    if lat_o_f >= 270:
        lat_o_f -= 360
    if cprNL(lat_e_f) != cprNL(lat_o_f):
        return None
    if even_newer:
        lat = lat_e_f
        nl = cprNL(lat)
        ni = max(nl, 1)
        m = math.floor(lon_e * (ni - 1) - lon_o * ni + 0.5)
        lon = (360 / ni) * (m % ni + lon_e)
    else:
        lat = lat_o_f
        nl = cprNL(lat)
        ni = max(nl - 1, 1)
        m = math.floor(lon_e * (ni - 1) - lon_o * ni + 0.5)
        lon = (360 / ni) * (m % ni + lon_o)
    if lon > 180:
        lon -= 360
    return lat, lon


def surface_position(e0, o0, lat_ref, lon_ref, even_newer):
    _, lat_e, lon_e = cpr_fields(e0)
    _, lat_o, lon_o = cpr_fields(o0)
    j = math.floor(59 * lat_e - 60 * lat_o + 0.5)
    lat_e_n = (90 / 60) * (j % 60 + lat_e)
    lat_o_n = (90 / 59) * (j % 59 + lat_o)
    lat_e_f = lat_e_n if lat_ref > 0 else lat_e_n - 90
    lat_o_f = lat_o_n if lat_ref > 0 else lat_o_n - 90
    if cprNL(lat_e_f) != cprNL(lat_o_f):
        return None
    if even_newer:
        lat = lat_e_f
        nl = cprNL(lat_e_f)
        ni = max(nl, 1)
        m = math.floor(lon_e * (ni - 1) - lon_o * ni + 0.5)
        lon0 = (90 / ni) * (m % ni + lon_e)
    else:
        lat = lat_o_f
        nl = cprNL(lat_o_f)
        ni = max(nl - 1, 1)
        m = math.floor(lon_e * (ni - 1) - lon_o * ni + 0.5)
        lon0 = (90 / ni) * (m % ni + lon_o)
    lons = [(lon0 + 90 * k + 180) % 360 - 180 for k in range(4)]
    lon = min(lons, key=lambda x: abs(lon_ref - x))
    return lat, lon


# ------------------------------------------------ preamble detect + bit extract
def detect_frames(slots):
    frames = []
    M = len(slots)
    if M < SLOTS_PER_FRAME:
        return frames
    cs = np.empty(M + 1, dtype=np.float64)
    cs[0] = 0
    np.cumsum(slots, out=cs[1:])
    total16 = cs[16:] - cs[:-16]
    p_range = np.arange(0, M - 15, dtype=np.int64)
    Eh = slots[p_range] + slots[p_range + 2] + slots[p_range + 7] + slots[p_range + 9]
    El = total16[p_range] - Eh
    thr = np.percentile(slots, 30) * 3.0
    pmin = np.minimum.reduce(
        [slots[p_range], slots[p_range + 2], slots[p_range + 7], slots[p_range + 9]]
    )
    cand = (Eh > 4 * thr) & (pmin > thr) & (Eh >= 3.0 * El)
    last = -1
    for i in np.nonzero(cand)[0]:
        p = int(p_range[i])
        if p < last:
            continue
        if p + SLOTS_PER_FRAME > M:
            break
        frame = extract_bits(slots, p)
        dfh = df_of(frame, 112)
        if dfh in (0, 4, 5, 11, 20, 21):
            frame = frame >> (112 - 56)
            frames.append((frame, 56, p))
        else:
            frames.append((frame, 112, p))
        last = p + SLOTS_PER_FRAME
    return frames


def extract_bits(slots, p):
    reg = 0
    for k in range(112):
        reg = (reg << 1) | (1 if slots[p + 16 + 2 * k] > slots[p + 17 + 2 * k] else 0)
    return reg


# --------------------------------------------------------------- decoder core
class Decoder:
    def __init__(self, lat_ref=None, lon_ref=None):
        self.lat_ref = lat_ref
        self.lon_ref = lon_ref
        self.pos = {}
        self.recent = deque(maxlen=30)
        self.n_total = 0
        self.n_good = 0

    def process(self, slots):
        out = []
        for msg, nbits, p in detect_frames(slots):
            self.n_total += 1
            fixed = crc_fix(msg, nbits)
            if not fixed:
                continue
            msg, errs = fixed
            self.n_good += 1
            d = self.render(msg, nbits)
            if d:
                out.append((d, errs))
        return out

    def render(self, msg, nbits):
        df = df_of(msg, nbits)
        icao = None
        if nbits == 112:
            icao = (msg >> (112 - 32)) & 0xFFFFFF
            self.recent.append(icao)
        fields = ["DF%d" % df]
        if icao is not None:
            fields.append("%06X" % icao)
        if nbits == 56:
            if df == 11:
                icao = (msg >> 24) & 0xFFFFFF
                self.recent.append(icao)
                fields = ["DF%d" % df, "%06X" % icao, "CA=%d" % ((msg >> 21) & 7)]
                return fields, hexstr(msg, nbits)
            if df in (4, 5, 20, 21):
                mbytes = msg.to_bytes(7, "big")
                rem = crc24_rem(list(mbytes[:4]) + [0, 0, 0])
                ap = (mbytes[4] << 16) | (mbytes[5] << 8) | mbytes[6]
                guess = rem ^ ap
                if any(guess == x for x in self.recent):
                    fields.append("%06X" % guess)
                if df in (5, 21):
                    fields.append("SQK=%s" % squawk(msg))
                elif df in (0, 4, 20):
                    ac = (msg >> 24) & 0x1FFF
                    alt = decode_altcode(format(ac, "013b"))
                    if alt is not None:
                        fields.append("ALT=%dft" % alt)
            return fields, hexstr(msg, nbits)

        tc = typecode(msg)
        fields.append("TC%d" % tc)
        if 1 <= tc <= 4:
            cs = callsign(msg)
            if cs:
                fields.append("CALL=%s" % cs)
            fields.append("CAT=%d" % category(msg))
        elif 5 <= tc <= 8:
            spd, trk = surface_state(msg)
            if spd is not None:
                fields.append("GS=%.2fkt" % spd)
            if trk is not None:
                fields.append("TRK=%.1f" % trk)
            pos = self.pair_cpr(icao, msg, "s")
            if pos:
                fields.append("POS=%.5f,%.5f" % pos)
        elif (9 <= tc <= 18) or (20 <= tc <= 22):
            alt = airborne_altitude(msg)
            if alt is not None:
                fields.append("ALT=%dft" % alt)
            pos = self.pair_cpr(icao, msg, "a")
            if pos:
                fields.append("POS=%.5f,%.5f" % pos)
        elif tc == 19:
            v = airborne_velocity(msg)
            if v:
                spd, trk, vs, st = v
                if spd is not None:
                    fields.append("SPD=%d%s" % (spd, st))
                if trk is not None:
                    fields.append("TRK=%.1f" % trk)
                if vs is not None:
                    fields.append("VR=%dftm" % vs)
        return fields, hexstr(msg, nbits)

    def pair_cpr(self, icao, msg, kind):
        st = self.pos.setdefault(icao, {"a": {}, "s": {}})[kind]
        oe, _, _ = cpr_fields(msg)
        st[oe] = (msg, time.monotonic())
        if 0 not in st or 1 not in st:
            return None
        e = st[0]
        o = st[1]
        even_newer = e[1] > o[1]
        if kind == "a":
            pos = airborne_position(e[0], o[0], even_newer)
        else:
            if self.lat_ref is None:
                return None
            pos = surface_position(e[0], o[0], self.lat_ref, self.lon_ref, even_newer)
        if pos:
            st.clear()
        return pos


# ------------------------------------------------------------------ receiver
class Receiver:
    def __init__(self, args):
        self.args = args
        if args.from_raw:
            self.sdr = None
        else:
            devs = SoapySDR.Device.enumerate("driver=hackrf")
            if not devs:
                raise RuntimeError("ERROR: No HackRF device found")
            self.sdr = SoapySDR.Device(devs[0])
        self.decoder = Decoder(lat_ref=args.lat, lon_ref=args.lon)
        self._chunks = 0
        self.t0 = time.monotonic()
        self._tail = np.zeros(0, dtype=np.float32)
        self._rawsrc = None
        self._rawfile = None
        self._stats_t = None
        self.sps = self._sps(args.rate)

    @staticmethod
    def _sps(rate):
        return max(1, int(round(rate * 0.5e-6)))

    def start(self):
        if self.sdr is None:
            self._rawsrc = open(self.args.from_raw, "rb")
            hdr = self._rawsrc.read(16)
            if hdr[:8] == RAW_HDR:
                rate = int.from_bytes(hdr[8:12], "little")
                self.args.rate = float(rate)
                self.sps = self._sps(rate)
            else:
                self._rawsrc.seek(0)
            return
        self.sdr.setFrequency(SOAPY_SDR_RX, 0, self.args.freq * 1e6)
        self.sdr.setSampleRate(SOAPY_SDR_RX, 0, float(self.args.rate))
        for el, val in (("LNA", self.args.lna), ("AMP", self.args.amp), ("VGA", self.args.vga)):
            if val is not None:
                self.sdr.setGain(SOAPY_SDR_RX, 0, el, val)
        self.stream = self.sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [0])
        self.sdr.activateStream(self.stream)
        self.read_chunk(2 ** 18)
        if self.args.save_raw:
            os.makedirs(self.args.save_raw, exist_ok=True)
            fn = os.path.join(self.args.save_raw, time.strftime("iq_%Y%m%d_%H%M%S.c16"))
            self._rawfile = open(fn, "wb")
            self._rawfile.write(
                RAW_HDR + int(round(self.args.rate)).to_bytes(4, "little") + b"\x00" * 4
            )

    def stop(self):
        if self._rawfile:
            self._rawfile.close()
        if self._rawsrc:
            self._rawsrc.close()
        if self.sdr is not None:
            self.sdr.deactivateStream(self.stream)
            self.sdr.closeStream(self.stream)

    def read_chunk(self, n):
        buf = [np.zeros(n * 2, dtype="int16")]
        got = 0
        while got < n:
            res = self.sdr.readStream(self.stream, buf, n - got, timeoutUs=3000000)
            if res.ret < 0:
                raise RuntimeError("readStream error %d" % res.ret)
            if res.ret == 0:
                continue
            got += int(res.ret)
        return buf[0]

    def feed(self, samples):
        blk = 4096
        sps = self.sps
        n = samples.size
        if n < 2:
            return []
        nblocks = n // (2 * blk)
        used = nblocks * 2 * blk
        if nblocks:
            c = samples[:used].astype(np.float32).reshape(nblocks, blk, 2)
            c -= c.mean(axis=1, keepdims=True)
            mag = np.sqrt(c[..., 0] ** 2 + c[..., 1] ** 2).ravel()
            slots = mag.reshape(nblocks * blk // sps, sps).sum(axis=1)
        else:
            slots = np.zeros(0, dtype=np.float32)
        rem = samples[used:]
        if rem.size:
            cr = rem.astype(np.float32).reshape(-1, 2)
            cr -= cr.mean(axis=0)
            rm = np.sqrt(cr[:, 0] ** 2 + cr[:, 1] ** 2)
            rk = rm.size // sps
            if rk:
                rm = rm[:rk * sps].reshape(rk, sps).sum(axis=1)
            slots = np.concatenate([slots, rm])
        if self._tail.size:
            slots = np.concatenate([self._tail, slots])
        self._last_slots = slots
        out = []
        for (fields, rawhex), errs in self.decoder.process(slots):
            out.append((rawhex, fields, errs))
        self._tail = slots[-256:]
        return out

    def emit(self, rawhex, fields, errs):
        note = "" if errs == 0 else " [fixed%d]" % errs
        print("*%s;%s" % (rawhex, note), "  ".join(fields))
        sys.stdout.flush()

    def _stats(self):
        if not self.args.stats:
            return
        now = time.monotonic()
        if self._stats_t is not None and now - self._stats_t < 5.0:
            return
        self._stats_t = now
        med = float(np.median(self._last_slots))
        act = float((self._last_slots > 6 * med).mean())
        print(
            "[stats %.0fs] raw=%d crc_ok=%d pulse_duty=%4.2f%% floor=%.0f"
            % (now - self.t0, self.decoder.n_total, self.decoder.n_good,
               100 * act, med),
            file=sys.stderr,
        )

    def run(self):
        while True:
            if self.sdr is None:
                raw = np.fromfile(self._rawsrc, dtype="<i2", count=2 ** 19)
                if raw.size == 0:
                    break
            else:
                raw = self.read_chunk(2 ** 18)
                if self.args.save_raw:
                    self._rawfile.write(raw.tobytes())
            for out in self.feed(raw):
                self.emit(*out)
            self._stats()
            if self.sdr is not None and self.args.seconds \
                    and time.monotonic() - self.t_start >= self.args.seconds:
                break


def main():
    ap = argparse.ArgumentParser(
        description="ADS-B (1090 MHz Mode-S) receiver for HackRF via SoapySDR"
    )
    ap.add_argument("--freq", type=float, default=1090, help="RX frequency in MHz")
    ap.add_argument("--rate", type=float, default=SAMPLE_RATE,
                    help="sample rate (Hz, default 2 MHz)")
    ap.add_argument("--lna", type=float, default=40, help="LNA gain dB (0-40)")
    ap.add_argument("--vga", type=float, default=40, help="VGA gain dB (0-62)")
    ap.add_argument("--amp", type=float, default=0, help="AMP (RF) gain dB (0-14)")
    ap.add_argument("--lat", type=float, default=None, help="receiver latitude (surface CPR)")
    ap.add_argument("--lon", type=float, default=None, help="receiver longitude (surface CPR)")
    ap.add_argument("--seconds", type=float, default=None, help="exit after N seconds")
    ap.add_argument("--stats", action="store_true", help="print periodic signal stats to stderr")
    ap.add_argument("--save-raw", default=None, metavar="DIR",
                    help="record raw CS16 IQ (interleaved int16 I/Q, little-endian, --rate Hz) to DIR as iq_<ts>.c16")
    ap.add_argument("--from-raw", default=None, metavar="FILE",
                    help="decode a saved raw IQ file (--save-raw output) instead of the live SDR")
    args = ap.parse_args()

    rec = Receiver(args)
    rec.start()
    if args.from_raw:
        print("Replaying raw file %s ..." % args.from_raw, file=sys.stderr)
    else:
        print(
            "Listening on %.0f MHz (fs=%.0f Hz, LNA=%.0f VGA=%.0f AMP=%.0f dB). Ctrl-C to stop."
            % (args.freq, args.rate, args.lna, args.vga, args.amp),
            file=sys.stderr,
        )
    t_start = time.monotonic()
    rec.t_start = t_start
    try:
        rec.run()
    except KeyboardInterrupt:
        pass
    finally:
        rec.stop()
    print(
        "Decoded %d/%d frames (CRC-valid/total)." % (rec.decoder.n_good, rec.decoder.n_total),
        file=sys.stderr,
    )


if __name__ == "__main__":
    main()