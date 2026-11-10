#!/usr/bin/env python3
"""Save a run's log from the EV's USB serial port.

Usage:
    python3 tools/capture_run.py [port] [--out DIR] [--wait SECONDS]
    python3 tools/capture_run.py --watch        # leave running all session

Plug the Pico in (it does not reset), then press START on the car at the result
screen: the firmware prints every run it has stored (the last 5, oldest first).
Each run printed is saved as
    <out>/<time>-r<N>.log   everything printed, as is
    <out>/<time>-r<N>.csv   the odometry + steering record ([TEL] BEGIN/END CSV)
    <out>/<time>-r<N>.html  the path and time plots (tools/plot_run.py)
<time> is when it was saved, N the run's number since the car was powered on.
A run already saved (same run number and power-on time) is skipped, so pressing
START again, or after every run, never makes duplicates.

Without --watch it stops after one run. With --watch it keeps going: start it
once, then for every run just plug in and press START. It waits for the Pico to
appear, survives unplugging, and saves each dump as it arrives.
The port defaults to the first /dev/cu.usbmodem*; --out defaults to run_logs/
(git-ignored). The Pico only sends while a program has the port open, so this
(or a serial monitor) must be running when START is pressed.

Standard library only: no pyserial needed.
"""
import argparse
import glob
import os
import re
import sys
import termios
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plot_run   # noqa: E402  (same folder)

END_MARKERS = ("START = print the stored runs",   # last line of every report
               "START = print this again")       # (older firmware)
RUN_ID = re.compile(r"\[SM\] run #(\d+) \(power-on \+([\d.]+) s\)")


def run_id(lines):
    """(number, 'number@power-on s') of the run in these lines, or (None, None)."""
    for line in lines:
        m = RUN_ID.search(line)
        if m:
            return m.group(1), f"{m.group(1)}@{m.group(2)}"
    return None, None


def saved_ids(out):
    """Ids of the runs already saved in `out`, so a re-print isn't saved twice."""
    ids = {}
    for path in glob.glob(os.path.join(out, "*.log")):
        try:
            with open(path, errors="replace") as f:
                _, rid = run_id(f)
        except OSError:
            continue
        if rid:
            ids[rid] = path
    return ids


def open_port(path):
    fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0                                   # iflag: raw
    attrs[1] = 0                                   # oflag
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0                                   # lflag: no echo, no canonical
    attrs[4] = attrs[5] = termios.B115200          # ignored by USB CDC, set anyway
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def find_port(given):
    return given or next(iter(sorted(glob.glob("/dev/cu.usbmodem*"))), None)


def save(out, log, csv_rows, seen):
    num, rid = run_id(log)
    if rid and rid in seen:
        print(f"\n>>> Run #{num} already saved: {seen[rid]}")
        return
    stamp = time.strftime("%Y%m%d-%H%M%S") + (f"-r{num}" if num else "")
    log_path = os.path.join(out, stamp + ".log")
    with open(log_path, "w") as f:
        f.write("\n".join(log) + "\n")
    print(f"\n>>> Saved {log_path}")
    if rid:
        seen[rid] = log_path
    if csv_rows:
        csv_path = os.path.join(out, stamp + ".csv")
        with open(csv_path, "w") as f:
            f.write("\n".join(csv_rows) + "\n")
        print(f">>> Saved {csv_path} ({len(csv_rows) - 1} samples)")
        try:
            print(f">>> Plot  {plot_run.write_plot(csv_path)}")
        except Exception as e:          # a plot must never lose a capture
            print(f">>> (no plot: {e})")
    else:
        print(">>> No [TEL] record in it.")


def capture(port, out, deadline, watch):
    """Read one port until a run has been saved (or, with watch, until unplugged).
    Returns the number of runs saved."""
    fd = open_port(port)
    print(f"Listening on {port}. Press START on the car at the result screen.")
    log, csv_rows, in_csv, buf, saved = [], [], False, b"", 0
    seen = saved_ids(out)
    try:
        while deadline is None or time.time() < deadline:
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                chunk = b""
            except OSError:            # unplugged
                print("(Pico unplugged)")
                break
            if not chunk:
                if not os.path.exists(port):
                    print("(Pico unplugged)")
                    break
                time.sleep(0.02)
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode(errors="replace").rstrip("\r")
                print(line)
                log.append(line)
                if "[TEL] BEGIN CSV" in line:
                    in_csv, csv_rows = True, []
                elif "[TEL] END CSV" in line:
                    in_csv = False
                elif in_csv:
                    csv_rows.append(line)
                elif any(m in line for m in END_MARKERS) and (csv_rows or len(log) > 3):
                    save(out, log, csv_rows, seen)
                    saved += 1
                    log, csv_rows = [], []
                    if not watch:
                        return saved
    finally:
        os.close(fd)
    return saved


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", help="serial port (default: first /dev/cu.usbmodem*)")
    ap.add_argument("--out", default="run_logs", help="output directory (default: run_logs)")
    ap.add_argument("--wait", type=float, default=180.0, help="give up after this many seconds (not with --watch)")
    ap.add_argument("--watch", action="store_true", help="keep running: save every run printed, across plug/unplug")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    if not args.watch:
        port = find_port(args.port)
        if not port:
            sys.exit("No /dev/cu.usbmodem* found: is the Pico plugged in?")
        if capture(port, args.out, time.time() + args.wait, watch=False) == 0:
            sys.exit("No run received. Was START pressed at the result screen?")
        return

    print("Watching for the Pico. Plug it in and press START after each run. Ctrl-C to quit.")
    try:
        while True:
            port = find_port(args.port)
            if port and os.path.exists(port):
                try:
                    capture(port, args.out, None, watch=True)
                except OSError as e:
                    print(f"(port error: {e}; retrying)")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
