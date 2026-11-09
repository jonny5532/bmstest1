#!/usr/bin/env python3
"""Interactive BMS CLI, run on the Pi next to bms_cmd.py:

    ssh -t rpi@10.131.131.50 python3 cli.py [--quiet]

Shows the live log in colour with a prompt underneath. Tab completes command
and parameter names, Up/Down recalls history (~/.bms_cli_history).
Ctrl-D exits, Ctrl-C clears the current line.

Local commands (not sent to the BMS):
    /log    toggle the background log (command replies are always shown)
"""
import argparse
import os
import re
import readline
import sys
import threading

from bms_cmd import close_port, drain, exchange, find_port, open_port

HISTORY = os.path.expanduser("~/.bms_cli_history")
TS = re.compile(r"^(\d+ms )(.*)$")

COLOR = sys.stdout.isatty()


def c(code, text):
    return f"\033[{code}m{text}\033[0m" if COLOR else text


def colorize(line):
    m = TS.match(line)
    ts, text = (m.group(1), m.group(2)) if m else ("", line)
    if text == "OK":
        text = c("1;32", text)
    elif text.startswith("ERR "):
        text = c("1;31", text)
    elif text.startswith("Received command:"):
        text = c("36", text)
    elif re.search(r"event|warn|error|fail|fault|overran", text, re.I):
        text = c("33", text)
    elif text.startswith("["):
        text = c("35", text)
    return c("2", ts) + text


class Cli:
    def __init__(self, s, show_log):
        self.s = s
        self.show_log = show_log
        self.in_reply = False
        self.stop = threading.Event()
        self.out_lock = threading.Lock()
        # What we redraw ourselves, and the same for input(): \001/\002 tell
        # readline the escape codes take no space
        self.prompt_drawn = c("1;34", "bms> ")
        self.prompt = re.sub(r"(\033\[[0-9;]*m)", "\001\\1\002", self.prompt_drawn)
        self.commands = []
        self.params = []

    def load_names(self):
        """Ask the BMS for its command and parameter names, for completion."""
        for cmd, dest in (("help", self.commands), ("params", self.params)):
            code, out = exchange(self.s, cmd, 5.0)
            if code:
                continue
            for line in out.splitlines()[1:-1]:
                m = TS.match(line)
                if m and m.group(2).split():
                    dest.append(m.group(2).split()[0])

    def complete(self, text, state):
        words = readline.get_line_buffer()[:readline.get_begidx()].split()
        if not words:
            options = self.commands
        elif words[0] in ("get", "set") and len(words) == 1:
            options = self.params
        else:
            options = []
        matches = [o + " " for o in options if o.startswith(text)]
        return matches[state] if state < len(matches) else None

    def show(self, line):
        """Print a line above the prompt, then redraw the prompt and input."""
        with self.out_lock:
            sys.stdout.write("\r\033[K" + line + "\n" + self.prompt_drawn + readline.get_line_buffer())
            sys.stdout.flush()

    def reader(self):
        buf = b""
        while not self.stop.is_set():
            try:
                data = self.s.read(4096)
            except Exception as e:  # e.g. the BMS rebooted and the port vanished
                self.show(c("1;31", f"serial read failed: {e}; Ctrl-D to exit"))
                return
            if not data:
                continue
            buf += data
            *lines, buf = buf.split(b"\n")
            for raw in lines:
                line = raw.decode(errors="replace").rstrip("\r")
                text = TS.match(line).group(2) if TS.match(line) else line
                if text.startswith("Received command:"):
                    self.in_reply = True
                if self.show_log or self.in_reply:
                    self.show(colorize(line))
                if text == "OK" or text.startswith("ERR "):
                    self.in_reply = False

    def loop(self):
        t = threading.Thread(target=self.reader, daemon=True)
        t.start()
        while True:
            try:
                line = input(self.prompt).strip()
            except EOFError:
                sys.stdout.write("\n")
                break
            except KeyboardInterrupt:
                sys.stdout.write("\n")
                continue
            if not line:
                continue
            if line == "/log":
                self.show_log = not self.show_log
                with self.out_lock:
                    print(c("2", f"background log {'on' if self.show_log else 'off'}"))
                continue
            self.s.write(line.encode() + b"\n")
        self.stop.set()
        t.join()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", default=None)
    ap.add_argument("-q", "--quiet", action="store_true", help="start with the background log off")
    args = ap.parse_args()

    s, saved = open_port(args.port or find_port())
    try:
        drain(s)  # skip the backlog buffered while nobody was reading
        cli = Cli(s, show_log=not args.quiet)
        cli.load_names()

        try:
            readline.read_history_file(HISTORY)
        except OSError:
            pass
        readline.set_completer(cli.complete)
        readline.set_completer_delims(" ")
        readline.parse_and_bind("tab: complete")

        print(c("2", "BMS CLI: 'help' lists commands, /log toggles the log, Ctrl-D exits"))
        cli.loop()
    finally:
        readline.set_history_length(1000)
        try:
            readline.write_history_file(HISTORY)
        except OSError:
            pass
        close_port(s, saved)


if __name__ == "__main__":
    main()
