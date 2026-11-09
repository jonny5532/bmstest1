#!/usr/bin/env python3
"""Send CLI command(s) to the BMS over USB serial and print the response.

Usage: bms_cmd.py [-t SECONDS] [-p PORT] CMD [CMD ...]

For each command: drains any stale buffered output, sends the command, then
prints from the firmware's "Received command: '<cmd>'" echo up to the final
"OK" / "ERR <reason>" line (log lines from elsewhere may be interleaved).
The tty settings are restored on exit so a plain `cat` (ser.sh) still works.

Exit codes: 0 ok, 1 command returned ERR, 2 port in use by another process,
3 command not acknowledged, 4 no OK/ERR within SECONDS.
"""
import argparse
import glob
import os
import re
import sys
import termios
import time

import serial

ACK_TIMEOUT = 2.0

# Log lines are "<millis>ms <text>"; a command ends with an OK or ERR line
RESULT = re.compile(rb"^\d+ms (OK|ERR .*)$", re.M)


def find_port():
    ports = glob.glob("/dev/serial/by-id/usb-S3XY-BMS_Controller_*")
    if len(ports) != 1:
        sys.exit(f"expected one BMS controller port, found {ports}")
    return ports[0]


def port_users(port):
    """PIDs (other than ours) holding the tty open, e.g. a ser.sh/watch.sh cat."""
    real = os.path.realpath(port)
    pids = []
    for pid in filter(str.isdigit, os.listdir("/proc")):
        if int(pid) == os.getpid():
            continue
        try:
            fds = os.listdir(f"/proc/{pid}/fd")
        except OSError:
            continue
        for fd in fds:
            try:
                if os.readlink(f"/proc/{pid}/fd/{fd}") == real:
                    pids.append(int(pid))
                    break
            except OSError:
                pass
    return pids


def open_port(port):
    """Open the port exclusively. Returns (serial, saved termios); exits 2 if busy."""
    users = port_users(port)
    if users:
        sys.stderr.write(f"{port} is open by pid(s) {users}; stop ser.sh/watch.sh first\n")
        sys.exit(2)
    # A tty opens with ECHO on, which would send the BMS's own output straight
    # back into its CLI. Turn echo off first thing, and leave it off on close.
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        saved = termios.tcgetattr(fd)
        saved[3] &= ~(termios.ECHO | termios.ECHOE | termios.ECHOK | termios.ECHONL | termios.ECHOCTL)
        termios.tcsetattr(fd, termios.TCSANOW, saved)
    finally:
        os.close(fd)
    return serial.Serial(port, 115200, timeout=0.1, exclusive=True), saved


def close_port(s, saved):
    """Restore canonical, no-echo tty settings so a plain `cat` still works."""
    termios.tcsetattr(s.fileno(), termios.TCSANOW, saved)
    s.close()


def drain(s, quiet=0.3, limit=3.0):
    """Discard backlog until the line has been idle for `quiet` seconds."""
    start = last = time.monotonic()
    while time.monotonic() - last < quiet and time.monotonic() - start < limit:
        if s.read(4096):
            last = time.monotonic()


def exchange(s, cmd, timeout):
    """Run one command. Returns (code, output): code 0 on OK, else the exit
    code to use; output runs from the ack line to the OK/ERR line inclusive
    (or whatever arrived, on failure)."""
    drain(s)
    s.write(cmd.encode() + b"\n")
    s.flush()

    ack = f"Received command: '{cmd.strip()}'".encode()
    buf = b""
    start = time.monotonic()
    while ack not in buf:
        if time.monotonic() - start > ACK_TIMEOUT:
            return 3, buf.decode(errors="replace")
        buf += s.read(4096)

    i = buf.index(ack)
    out = buf[buf.rfind(b"\n", 0, i) + 1:]
    end = time.monotonic() + timeout
    while True:
        m = RESULT.search(out)
        if m:
            return (0 if m.group(1) == b"OK" else 1), out[:m.end()].decode(errors="replace") + "\n"
        if time.monotonic() > end:
            return 4, out.decode(errors="replace")
        out += s.read(4096)


def run(s, cmd, timeout):
    """Run one command and print its output; returns the exit code to use."""
    code, out = exchange(s, cmd, timeout)
    sys.stdout.write(out)
    sys.stdout.flush()
    if code == 3:
        sys.stderr.write(f"no ack for {cmd!r}\n")
    elif code == 4:
        sys.stderr.write(f"no OK/ERR for {cmd!r} within {timeout}s\n")
    return code


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", default=None)
    ap.add_argument("-t", "--timeout", type=float, default=10.0,
                    help="max seconds to wait for OK/ERR after each ack")
    ap.add_argument("cmds", nargs="+")
    args = ap.parse_args()

    s, saved = open_port(args.port or find_port())
    try:
        for cmd in args.cmds:
            rc = run(s, cmd, args.timeout)
            if rc:
                sys.exit(rc)
    finally:
        close_port(s, saved)


if __name__ == "__main__":
    main()
