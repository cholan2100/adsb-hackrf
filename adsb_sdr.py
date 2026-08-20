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
    mbytes = msg.to_bytes(nb, "big")
    if crc24_rem(mbytes) == 0:
        return msg, 0
    df = (msg >> (nbits - 5)) & 0x1F
    # Only attempt 1-bit error correction for standard DF17/DF18 ADS-B frames
    if df in (17, 18):
        for i in range(nbits):
            m = msg ^ (1 << i)
            if crc24_rem(m.to_bytes(nb, "big")) == 0:
                return m, 1
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
def detect_frames(slots, threshold_ratio=2.0):
    frames = []
    M = len(slots)
    if M < SLOTS_PER_FRAME:
        return frames

    sub = slots[::8] if M > 4000 else slots
    f_noise = float(np.percentile(sub, 30))
    thr = f_noise * threshold_ratio

    p_cands = np.nonzero(slots[:-16] > thr)[0]
    if len(p_cands) == 0:
        return frames

    last = -1
    for p in p_cands:
        p = int(p)
        if p < last:
            continue
        if p + SLOTS_PER_FRAME > M:
            break

        s0, s1, s2, s3 = slots[p], slots[p + 1], slots[p + 2], slots[p + 3]
        s6, s7, s8, s9, s10 = (
            slots[p + 6],
            slots[p + 7],
            slots[p + 8],
            slots[p + 9],
            slots[p + 10],
        )

        if s0 > s1 and s2 > s1 and s2 > s3 and s7 > s6 and s7 > s8 and s9 > s8 and s9 > s10:
            Eh = s0 + s2 + s7 + s9
            E_low = (
                s1
                + s3
                + slots[p + 4]
                + slots[p + 5]
                + s6
                + s8
                + s10
                + slots[p + 11]
                + slots[p + 12]
                + slots[p + 13]
                + slots[p + 14]
                + slots[p + 15]
            )
            if Eh >= 0.70 * E_low:
                frame = extract_bits(slots, p)
                dfh = df_of(frame, 112)
                nbits = 56 if dfh in (0, 4, 5, 11, 20, 21) else 112
                f = (frame >> (112 - 56)) if nbits == 56 else frame
                frames.append((f, nbits, p))
                last = p + SLOTS_PER_FRAME
    return frames


def extract_bits(slots, p):
    reg = 0
    for k in range(112):
        reg = (reg << 1) | (1 if slots[p + 16 + 2 * k] > slots[p + 17 + 2 * k] else 0)
    return reg


# --------------------------------------------------------------- decoder core
class Decoder:
    def __init__(self, lat_ref=None, lon_ref=None, threshold_ratio=1.75):
        self.lat_ref = lat_ref
        self.lon_ref = lon_ref
        self.threshold_ratio = threshold_ratio
        self.pos = {}
        self.recent = deque(maxlen=30)
        self.n_total = 0
        self.n_good = 0

    def process(self, slots):
        out = []
        for msg, nbits, p in detect_frames(slots, threshold_ratio=self.threshold_ratio):
            self.n_total += 1
            fixed = crc_fix(msg, nbits)
            if not fixed:
                # Try adjacent sample phase shifts (p+1, p-1) to recover sub-sample timing jitter
                for shift in (1, -1):
                    ps = p + shift
                    if 0 <= ps and ps + SLOTS_PER_FRAME <= len(slots):
                        m = extract_bits(slots, ps)
                        dfh = df_of(m, 112)
                        nb = 56 if dfh in (0, 4, 5, 11, 20, 21) else 112
                        m = (m >> (112 - 56)) if nb == 56 else m
                        fixed = crc_fix(m, nb)
                        if fixed:
                            msg, nbits = m, nb
                            break
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
        if tc is not None:
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
        elif df in (20, 21):
            if df == 21:
                fields.append("SQK=%s" % squawk(msg >> 56))
            elif df == 20:
                ac = (msg >> 80) & 0x1FFF
                alt = decode_altcode(format(ac, "013b"))
                if alt is not None:
                    fields.append("ALT=%dft" % alt)
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
        self.driver_name = "raw"
        self.device_info = {}
        if getattr(args, "from_raw", None):
            self.sdr = None
        else:
            self.sdr, self.driver_name, self.device_info = self._find_device(args)
        self.decoder = Decoder(
            lat_ref=getattr(args, "lat", None),
            lon_ref=getattr(args, "lon", None),
            threshold_ratio=getattr(args, "threshold", 2.0),
        )
        self._chunks = 0
        self.t0 = time.monotonic()
        self._tail = np.zeros(0, dtype=np.float32)
        self._rawsrc = None
        self._rawfile = None
        self._stats_t = None
        self.sps = self._sps(getattr(args, "rate", SAMPLE_RATE))

    @staticmethod
    def _find_device(args):
        dev_req = (getattr(args, "device", None) or "hackrf").lower()
        dev_args = getattr(args, "device_args", "") or ""

        def search_driver(driver_key):
            query = f"driver={driver_key}"
            if dev_args:
                query = f"{query},{dev_args}"
            results = SoapySDR.Device.enumerate(query)
            if results:
                return results[0], driver_key
            return None, None

        selected = None
        driver_found = None

        if dev_req == "hackrf":
            dev, drv = search_driver("hackrf")
            if dev is None:
                raise RuntimeError(
                    "ERROR: No HackRF device found. Ensure HackRF is connected and USB driver is installed."
                )
            selected, driver_found = dev, drv
        elif dev_req in ("rtlsdr", "rtl"):
            dev, drv = search_driver("rtlsdr")
            if dev is None:
                raise RuntimeError(
                    "ERROR: No RTL-SDR device found. Ensure RTL-SDR dongle is connected and driver is installed."
                )
            selected, driver_found = dev, drv
        elif dev_req == "auto":
            # Check HackRF first (default preference), then RTL-SDR, then generic
            for drv_name in ("hackrf", "rtlsdr"):
                dev, drv = search_driver(drv_name)
                if dev is not None:
                    selected, driver_found = dev, drv
                    break
            if selected is None:
                all_devs = SoapySDR.Device.enumerate(dev_args)
                if all_devs:
                    selected = all_devs[0]
                    driver_found = dict(selected).get("driver", "generic")
                else:
                    raise RuntimeError("ERROR: No SDR device found (probed HackRF, RTL-SDR, generic).")
        else:
            query = f"driver={dev_req}" if "=" not in dev_req else dev_req
            if dev_args:
                query = f"{query},{dev_args}"
            results = SoapySDR.Device.enumerate(query)
            if not results:
                raise RuntimeError(f"ERROR: No device found matching '{query}'")
            selected = results[0]
            driver_found = dev_req

        info_dict = dict(selected)
        sdr = SoapySDR.Device(selected)
        return sdr, driver_found, info_dict

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

        # Configure Frequency & Sample Rate
        self.sdr.setFrequency(SOAPY_SDR_RX, 0, self.args.freq * 1e6)
        self.sdr.setSampleRate(SOAPY_SDR_RX, 0, float(self.args.rate))

        # Configure Frequency Correction (PPM)
        if getattr(self.args, "ppm", None):
            try:
                self.sdr.setFrequencyCorrection(SOAPY_SDR_RX, 0, float(self.args.ppm))
            except Exception as e:
                print(f"[warning] Frequency correction (PPM) not supported: {e}", file=sys.stderr)

        # Configure Bias-Tee if requested
        if getattr(self.args, "biastee", False):
            try:
                self.sdr.writeSetting("biastee", "true")
            except Exception as e:
                print(f"[warning] Bias-Tee could not be enabled: {e}", file=sys.stderr)

        is_hackrf = "hackrf" in self.driver_name.lower() or "hackrf" in str(self.device_info).lower()
        is_rtlsdr = "rtl" in self.driver_name.lower() or "rtl" in str(self.device_info).lower()

        if is_hackrf:
            for el, val in (("LNA", getattr(self.args, "lna", None)),
                            ("AMP", getattr(self.args, "amp", None)),
                            ("VGA", getattr(self.args, "vga", None))):
                if val is not None:
                    try:
                        self.sdr.setGain(SOAPY_SDR_RX, 0, el, float(val))
                    except Exception as e:
                        print(f"[warning] Failed to set HackRF gain {el}={val}: {e}", file=sys.stderr)
        elif is_rtlsdr:
            # Configure RTL-SDR Hardware Offset Tuning
            if getattr(self.args, "offset_tune", True):
                try:
                    self.sdr.writeSetting("offset_tune", "true")
                except Exception:
                    pass

            # Configure AGC or Manual Gain
            if getattr(self.args, "agc", False):
                try:
                    self.sdr.setGainMode(SOAPY_SDR_RX, 0, True)
                except Exception as e:
                    print(f"[warning] Failed to enable Tuner AGC: {e}", file=sys.stderr)
                try:
                    self.sdr.writeSetting("digital_agc", "true")
                except Exception as e:
                    print(f"[warning] Failed to enable Digital AGC: {e}", file=sys.stderr)
            else:
                try:
                    self.sdr.setGainMode(SOAPY_SDR_RX, 0, False)
                except Exception:
                    pass
                try:
                    self.sdr.writeSetting("digital_agc", "false")
                except Exception:
                    pass
                if getattr(self.args, "gain", None) is not None:
                    try:
                        self.sdr.setGain(SOAPY_SDR_RX, 0, float(self.args.gain))
                    except Exception as e:
                        print(f"[warning] Failed to set RTL-SDR gain={self.args.gain}: {e}", file=sys.stderr)
        else:
            # Generic SDR gain handling
            if getattr(self.args, "agc", False):
                try:
                    self.sdr.setGainMode(SOAPY_SDR_RX, 0, True)
                except Exception:
                    pass
            elif getattr(self.args, "gain", None) is not None:
                try:
                    self.sdr.setGain(SOAPY_SDR_RX, 0, float(self.args.gain))
                except Exception:
                    pass

        stream_args = {}
        if is_rtlsdr:
            stream_args = {
                "bufflen": "65536",
                "buffers": "64",
                "asyncBuffs": "64",
            }

        self.stream = self.sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [0], stream_args)
        self.sdr.activateStream(self.stream)
        self.read_chunk(131072)
        if getattr(self.args, "save_raw", None):
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
            try:
                self.sdr.deactivateStream(self.stream)
            except Exception:
                pass
            try:
                self.sdr.closeStream(self.stream)
            except Exception:
                pass

    def read_chunk(self, n):
        out = np.empty(n * 2, dtype=np.int16)
        got = 0
        while got < n:
            sub = [out[got * 2 :]]
            res = self.sdr.readStream(self.stream, sub, n - got, timeoutUs=3000000)
            if res.ret < 0:
                if res.ret in (-4, -1):  # SOAPY_SDR_OVERFLOW or TIMEOUT, continue
                    continue
                raise RuntimeError("readStream error %d" % res.ret)
            if res.ret == 0:
                continue
            got += int(res.ret)
        return out

    def feed(self, samples):
        n = samples.size
        if n < 2:
            return []

        I = samples[0::2].astype(np.float32)
        Q = samples[1::2].astype(np.float32)

        # Software DC Blocker: remove DC baseline offset from I and Q
        if getattr(self.args, "dc_block", True):
            I -= I.mean()
            Q -= Q.mean()

        mag = np.sqrt(I * I + Q * Q)

        rate = float(getattr(self.args, "rate", SAMPLE_RATE))
        if abs(rate - 2_000_000.0) < 1.0:
            slots = mag
        elif self.sps > 1 and (rate % 2_000_000 == 0):
            sps = self.sps
            n_slots = mag.size // sps
            slots = mag[: n_slots * sps].reshape(n_slots, sps).sum(axis=1)
        else:
            # High-precision linear interpolation to 2.0 MSPS (e.g. 2.4 MSPS -> 2.0 MSPS)
            n_out = int(round(len(mag) * (2_000_000.0 / rate)))
            x = np.linspace(0, len(mag) - 1, n_out, dtype=np.float32)
            slots = np.interp(x, np.arange(len(mag), dtype=np.float32), mag)

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
        if not getattr(self.args, "stats", False):
            return
        now = time.monotonic()
        if self._stats_t is not None and now - self._stats_t < 5.0:
            return
        self._stats_t = now
        med = float(np.median(self._last_slots))
        act = float((self._last_slots > 2.5 * med).mean()) if med > 0 else 0.0
        print(
            "[stats %.0fs] raw=%d crc_ok=%d pulse_duty=%4.2f%% floor=%.0f"
            % (now - self.t0, self.decoder.n_total, self.decoder.n_good,
               100 * act, med),
            file=sys.stderr,
        )

    def run(self):
        if self.sdr is None:
            while True:
                raw = np.fromfile(self._rawsrc, dtype="<i2", count=2 ** 19)
                if raw.size == 0:
                    break
                for out in self.feed(raw):
                    self.emit(*out)
                self._stats()
            return

        import queue
        import threading

        q = queue.Queue(maxsize=32)
        stop_evt = threading.Event()

        def reader():
            while not stop_evt.is_set():
                try:
                    chunk = self.read_chunk(131072)
                    q.put(chunk, timeout=0.5)
                except Exception:
                    if stop_evt.is_set():
                        break

        th = threading.Thread(target=reader, daemon=True)
        th.start()

        try:
            while True:
                try:
                    raw = q.get(timeout=0.5)
                except queue.Empty:
                    if stop_evt.is_set():
                        break
                    continue

                if getattr(self.args, "save_raw", None):
                    self._rawfile.write(raw.tobytes())

                for out in self.feed(raw):
                    self.emit(*out)
                self._stats()

                if getattr(self.args, "seconds", None) and time.monotonic() - self.t_start >= self.args.seconds:
                    break
        finally:
            stop_evt.set()


def main():
    ap = argparse.ArgumentParser(
        description="ADS-B (1090 MHz Mode-S) receiver for HackRF & RTL-SDR via SoapySDR"
    )
    ap.add_argument(
        "--device", "-d",
        default="hackrf",
        choices=["hackrf", "rtlsdr", "auto"],
        help="SDR hardware device (hackrf, rtlsdr, or auto; default: hackrf)",
    )
    ap.add_argument("--device-args", default="", help="additional SoapySDR device query arguments")
    ap.add_argument("--freq", type=float, default=1090, help="RX frequency in MHz (default: 1090)")
    ap.add_argument("--rate", type=float, default=None,
                    help="sample rate in Hz (default: 2.4 MHz for RTL-SDR, 2.0 MHz for HackRF)")

    # HackRF Gain settings
    ap.add_argument("--lna", type=float, default=40, help="HackRF LNA gain dB (0-40, default: 40)")
    ap.add_argument("--vga", type=float, default=40, help="HackRF VGA gain dB (0-62, default: 40)")
    ap.add_argument("--amp", type=float, default=0, help="HackRF AMP (RF preamp) gain dB (0-14, default: 0)")

    # RTL-SDR Gain, AGC & Tuner settings
    ap.add_argument("--gain", "-g", type=float, default=40, help="RTL-SDR / general tuner gain in dB (default: 40)")
    ap.add_argument("--agc", action="store_true", default=None,
                    help="enable Dual AGC — Tuner AGC + RTL2832 Digital AGC (default for RTL-SDR)")
    ap.add_argument("--no-agc", action="store_false", dest="agc",
                    help="disable AGC, use manual --gain instead")
    ap.add_argument("--ppm", type=float, default=0, help="frequency correction in PPM (for RTL-SDR crystal offset)")
    ap.add_argument("--biastee", action="store_true", help="enable Bias-Tee (power active antenna/LNA)")
    ap.add_argument("--no-offset-tune", action="store_false", dest="offset_tune", default=True,
                    help="disable RTL-SDR hardware offset tuning mode")
    ap.add_argument("--no-dc-block", action="store_false", dest="dc_block", default=True,
                    help="disable software DC block filter")

    # Decode and Output options
    ap.add_argument("--threshold", "-t", type=float, default=2.0,
                    help="preamble detection threshold ratio relative to noise floor (default: 2.0 = ~8 dB)")
    ap.add_argument("--lat", type=float, default=None, help="receiver latitude (for surface CPR)")
    ap.add_argument("--lon", type=float, default=None, help="receiver longitude (for surface CPR)")
    ap.add_argument("--seconds", type=float, default=None, help="exit after N seconds")
    ap.add_argument("--stats", action="store_true", help="print periodic signal stats to stderr")
    ap.add_argument("--save-raw", default=None, metavar="DIR",
                    help="record raw CS16 IQ (interleaved int16 I/Q, little-endian, --rate Hz) to DIR as iq_<ts>.c16")
    ap.add_argument("--from-raw", default=None, metavar="FILE",
                    help="decode a saved raw IQ file (--save-raw output) instead of live SDR")
    args = ap.parse_args()

    user_rate = args.rate  # None means user didn't specify

    # Temporary default rate for Receiver init (refined after device detection)
    if args.rate is None:
        args.rate = 2_000_000.0

    try:
        rec = Receiver(args)
    except RuntimeError as e:
        print(f"{e}", file=sys.stderr)
        sys.exit(1)

    # Apply RTL-SDR-specific defaults after device detection
    is_rtlsdr = "rtl" in rec.driver_name.lower()
    if is_rtlsdr:
        if user_rate is None:
            args.rate = 2_400_000.0
            rec.sps = rec._sps(int(args.rate))
        if args.agc is None:
            args.agc = True
    else:
        if args.agc is None:
            args.agc = False

    try:
        rec.start()
    except RuntimeError as e:
        print(f"{e}", file=sys.stderr)
        sys.exit(1)

    if args.from_raw:
        print("Replaying raw file %s ..." % args.from_raw, file=sys.stderr)
    else:
        drv = rec.driver_name.upper()
        if "HACKRF" in drv:
            gain_info = f"LNA={args.lna:.0f} VGA={args.vga:.0f} AMP={args.amp:.0f} dB"
        elif "RTL" in drv:
            gain_info = "Dual AGC=ON" if args.agc else f"Gain={args.gain:.1f} dB (PPM={args.ppm:+.0f})"
        else:
            gain_info = f"Gain={args.gain} dB"

        dc_str = "DC-Block: ON" if getattr(args, "dc_block", True) else "DC-Block: OFF"
        print(
            f"Listening with [{drv}] on {args.freq:.0f} MHz (fs={args.rate:.0f} Hz, {gain_info}, {dc_str}). Ctrl-C to stop.",
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
