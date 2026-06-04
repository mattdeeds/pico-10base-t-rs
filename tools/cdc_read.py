#!/usr/bin/env python3
"""Read the device's USB-CDC telemetry for DUR seconds and print it.

The firmware emits 1 Hz status lines over USB CDC ([R2b] heartbeat, [Rx] decode
stats, and on the router build [Cyw43]/[Wan]/[Fwd]/[Nat]/[Perf]/[Sink]). The host
must assert DTR to receive them (gotcha: without DTR the CDC stays silent).

Auto-detects the Pico CDC by USB product string ("Pico-10BASE-T"), so it survives
the /dev/ttyACM* number changing across resets. Override with CDC_PATH=/dev/ttyACMx.

Usage:  cdc_read.py [seconds]      (default 9)
Example: python3 tools/cdc_read.py 12 | grep '\\[Perf\\]'
"""
import os, time, fcntl, struct, sys, glob

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 9.0

def find_pico():
    if os.environ.get("CDC_PATH"):
        return os.environ["CDC_PATH"]
    for dev in sorted(glob.glob("/dev/ttyACM*")):
        n = os.path.basename(dev)
        try:
            with open("/sys/class/tty/%s/device/../product" % n) as f:
                if "Pico-10BASE-T" in f.read():
                    return dev
        except OSError:
            continue
    return None

deadline = time.time() + 15
fd = None
while time.time() < deadline:
    path = find_pico()
    if path:
        try:
            fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
            break
        except (FileNotFoundError, OSError):
            pass
    time.sleep(0.2)
if fd is None:
    print("ERROR: no Pico CDC found (product string 'Pico-10BASE-T')")
    sys.exit(1)

fcntl.ioctl(fd, 0x5416, struct.pack("I", 0x002))  # TIOCMBIS, TIOCM_DTR
end = time.time() + DUR
buf = b""
while time.time() < end:
    try:
        d = os.read(fd, 4096)
        if d:
            buf += d
    except BlockingIOError:
        time.sleep(0.03)
os.close(fd)
sys.stdout.write(buf.decode("ascii", "replace"))
