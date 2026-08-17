#!/usr/bin/env python
import os
import re
import sys
import threading
import time

HEX6 = re.compile(r"^[0-9A-F]{6}$")


class AC:
    __slots__ = ("icao", "call", "cat", "alt", "spd", "trk", "vr", "pos",
                 "last", "first", "msgs")

    def __init__(self, icao):
        self.icao = icao
        self.call = ""
        self.cat = None
        self.alt = None
        self.spd = None
        self.trk = None
        self.vr = None
        self.pos = None
        self.last = time.monotonic()
        self.first = self.last
        self.msgs = 1


def parse_line(line, acs, t):
    try:
        s = line.decode("utf-8", "replace").strip()
    except Exception:
        return
    if not s.startswith("*"):
        return
    semi = s.find(";")
    if semi < 0:
        return
    tokens = s[semi + 1:].split()
    icao = None
    for tok in tokens:
        if HEX6.match(tok) and not tok.startswith(("DF", "TC", "CA", "SQK")):
            icao = tok
            break
    if not icao:
        return
    ac = acs.get(icao)
    if ac is None:
        ac = AC(icao)
        acs[icao] = ac
    ac.last = t
    ac.msgs += 1
    for tok in tokens:
        if tok.startswith("CALL="):
            ac.call = tok[5:]
        elif tok.startswith("CAT="):
            ac.cat = tok[4:]
        elif tok.startswith("ALT="):
            ac.alt = tok[4:]
        elif tok.startswith("SPD="):
            ac.spd = tok[4:]
        elif tok.startswith("GS="):
            ac.spd = tok[3:]
        elif tok.startswith("TRK="):
            ac.trk = tok[4:]
        elif tok.startswith("VR="):
            ac.vr = tok[3:]
        elif tok.startswith("POS="):
            ac.pos = tok[4:]


def fmt_age(secs):
    secs = max(0, int(secs))
    if secs < 90:
        return "%d s" % secs
    m, s = divmod(secs, 60)
    return "%d m %d s" % (m, s)


def render(acs, t):
    rows = sorted(acs.values(), key=lambda a: a.last, reverse=True)
    out = ["\x1b[2J\x1b[H"]
    out.append(" ADS-B 1090 MHz   aircraft: %d   frames: %d"
               % (len(rows), sum(a.msgs for a in rows)))
    out.append("")
    out.append("%-7s %-8s %-9s %-9s %-8s %-6s %-6s %-17s %-5s"
               % ("ID", "CALL", "AGE", "ALT", "SPD", "TRK", "VR", "POS", "MSG"))
    out.append("-" * 74)
    for a in rows:
        if t - a.last > 600:
            continue
        out.append("%-7s %-8s %-9s %-9s %-8s %-6s %-6s %-17s %-5d"
                   % (a.icao, a.call or "-", fmt_age(t - a.last),
                      a.alt or "-", a.spd or "-", a.trk or "-", a.vr or "-",
                      a.pos or "-", a.msgs))
    out.append("")
    out.append("Ctrl-C to quit")
    return ("\r\n".join(out) + "\r\n").encode("utf-8", "replace")


def main():
    acs = {}
    stop = threading.Event()

    def reader():
        buf = b""
        try:
            while True:
                chunk = os.read(0, 65536)
                if not chunk:
                    break
                buf += chunk
                lines = buf.split(b"\n")
                buf = lines.pop()
                for ln in lines:
                    parse_line(ln, acs, time.monotonic())
        except OSError:
            pass
        finally:
            stop.set()

    threading.Thread(target=reader, daemon=True).start()
    try:
        while not stop.is_set():
            sys.stdout.buffer.write(render(acs, time.monotonic()))
            sys.stdout.flush()
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.buffer.write(b"\r\n")
        sys.stdout.flush()


if __name__ == "__main__":
    main()