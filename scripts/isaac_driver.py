#!/usr/bin/env python3
"""
Drive Isaac Sim's GUI from outside via python-xlib (X11 only).

Usage:
    python3 scripts/isaac_driver.py windows           # list windows
    python3 scripts/isaac_driver.py focus-isaac       # raise + focus main window
    python3 scripts/isaac_driver.py keys "Alt+w"      # send a key chord
    python3 scripts/isaac_driver.py type "hello"      # type text via clipboard
    python3 scripts/isaac_driver.py paste FILE        # paste file content + Ctrl-Enter
"""
from __future__ import annotations

import argparse
import os
import re
import sys
import time
from pathlib import Path
from typing import Optional

from Xlib import X, XK, display
from Xlib.ext import xtest
from Xlib.protocol import event


def open_display():
    return display.Display()


# ---------------------------------------------------------------------------
# Window discovery
# ---------------------------------------------------------------------------
def list_windows(d):
    root = d.screen().root
    NET_CLIENT_LIST = d.intern_atom("_NET_CLIENT_LIST")
    NET_WM_NAME = d.intern_atom("_NET_WM_NAME")
    WM_NAME = d.intern_atom("WM_NAME")
    UTF8 = d.intern_atom("UTF8_STRING")
    r = root.get_full_property(NET_CLIENT_LIST, 0)
    out = []
    if not r:
        return out
    for wid in list(r.value):
        w = d.create_resource_object("window", wid)
        name = None
        for atom, target in ((NET_WM_NAME, UTF8), (WM_NAME, 0)):
            try:
                p = w.get_full_property(atom, target)
                if p and p.value:
                    v = p.value
                    if isinstance(v, bytes):
                        v = v.decode("utf-8", "replace")
                    name = v
                    break
            except Exception:
                pass
        out.append((wid, name or ""))
    return out


def find_window(d, pattern: str):
    rx = re.compile(pattern, re.IGNORECASE)
    for wid, name in list_windows(d):
        if rx.search(name):
            return wid, name
    return None, None


def find_isaac(d):
    return find_window(d, r"Isaac Sim Full")


# ---------------------------------------------------------------------------
# Window focusing
# ---------------------------------------------------------------------------
def focus_window(d, wid: int):
    w = d.create_resource_object("window", wid)
    NET_ACTIVE = d.intern_atom("_NET_ACTIVE_WINDOW")
    root = d.screen().root
    # Standard ICCCM request to activate the window.
    ev = event.ClientMessage(
        window=w, client_type=NET_ACTIVE,
        data=(32, [2, X.CurrentTime, 0, 0, 0]),
    )
    root.send_event(ev, event_mask=X.SubstructureRedirectMask | X.SubstructureNotifyMask)
    w.configure(stack_mode=X.Above)
    w.set_input_focus(X.RevertToParent, X.CurrentTime)
    d.sync()


# ---------------------------------------------------------------------------
# Key chord sending via XTest
# ---------------------------------------------------------------------------
MODS = {
    "ctrl":  XK.XK_Control_L,
    "control": XK.XK_Control_L,
    "shift": XK.XK_Shift_L,
    "alt":   XK.XK_Alt_L,
    "super": XK.XK_Super_L,
    "meta":  XK.XK_Alt_L,
}

SPECIAL = {
    "enter": XK.XK_Return,
    "return": XK.XK_Return,
    "tab": XK.XK_Tab,
    "esc": XK.XK_Escape,
    "escape": XK.XK_Escape,
    "space": XK.XK_space,
    "backspace": XK.XK_BackSpace,
    "delete": XK.XK_Delete,
    "left": XK.XK_Left,
    "right": XK.XK_Right,
    "up": XK.XK_Up,
    "down": XK.XK_Down,
    "home": XK.XK_Home,
    "end": XK.XK_End,
    "f1": XK.XK_F1, "f2": XK.XK_F2, "f3": XK.XK_F3, "f4": XK.XK_F4,
    "f5": XK.XK_F5, "f6": XK.XK_F6, "f7": XK.XK_F7, "f8": XK.XK_F8,
    "f9": XK.XK_F9, "f10": XK.XK_F10, "f11": XK.XK_F11, "f12": XK.XK_F12,
}


def parse_chord(chord: str) -> tuple[list[int], int]:
    """Returns (mod_keysyms, key_keysym)."""
    parts = [p.strip() for p in chord.split("+") if p.strip()]
    mods = []
    key = None
    for p in parts[:-1]:
        ks = MODS.get(p.lower())
        if ks is None:
            raise ValueError(f"unknown modifier {p}")
        mods.append(ks)
    last = parts[-1]
    if last.lower() in SPECIAL:
        key = SPECIAL[last.lower()]
    elif len(last) == 1:
        key = XK.string_to_keysym(last)
    else:
        key = XK.string_to_keysym(last) or 0
        if not key:
            raise ValueError(f"unknown key {last!r}")
    return mods, key


def send_chord(d, chord: str, hold_ms: int = 30):
    mods, key = parse_chord(chord)
    mod_codes = [d.keysym_to_keycode(m) for m in mods]
    key_code = d.keysym_to_keycode(key)
    if not key_code:
        raise ValueError(f"no keycode for {chord}")
    for mc in mod_codes:
        xtest.fake_input(d, X.KeyPress, mc)
    xtest.fake_input(d, X.KeyPress, key_code)
    d.sync()
    time.sleep(hold_ms / 1000.0)
    xtest.fake_input(d, X.KeyRelease, key_code)
    for mc in reversed(mod_codes):
        xtest.fake_input(d, X.KeyRelease, mc)
    d.sync()
    time.sleep(0.05)


# ---------------------------------------------------------------------------
# X clipboard set (CLIPBOARD selection)
# ---------------------------------------------------------------------------
class ClipboardOwner:
    """Hold X11 CLIPBOARD selection and serve requests until released.

    The X clipboard protocol requires whoever 'set' the clipboard to remain
    alive and answer selection requests. We park a window owning CLIPBOARD
    and a loop that serves SelectionRequest events. Caller is expected to
    daemonize / fork this loop while it dispatches.
    """

    def __init__(self, content: bytes):
        self.d = open_display()
        self.root = self.d.screen().root
        self.win = self.root.create_window(
            0, 0, 1, 1, 0, X.CopyFromParent,
            event_mask=X.PropertyChangeMask,
        )
        self.content = content
        self.CLIPBOARD = self.d.intern_atom("CLIPBOARD")
        self.TARGETS = self.d.intern_atom("TARGETS")
        self.UTF8 = self.d.intern_atom("UTF8_STRING")
        self.STRING = self.d.intern_atom("STRING")
        self.TEXT = self.d.intern_atom("TEXT")
        self.win.set_selection_owner(self.CLIPBOARD, X.CurrentTime)
        self.d.sync()

    def serve(self, deadline_s: float):
        end = time.time() + deadline_s
        while time.time() < end:
            self.d.pending_events()
            try:
                ev = self.d.next_event()
            except Exception:
                continue
            if ev.type == X.SelectionRequest:
                self._respond(ev)
            elif ev.type == X.SelectionClear:
                # someone else took the clipboard; we're done
                return

    def _respond(self, ev):
        req = ev
        target = req.target
        prop = req.property if req.property else target
        if target in (self.UTF8, self.STRING, self.TEXT):
            req.requestor.change_property(prop, target, 8, self.content)
            ok = True
        elif target == self.TARGETS:
            data = [self.TARGETS, self.UTF8, self.STRING, self.TEXT]
            req.requestor.change_property(prop, self.d.intern_atom("ATOM"), 32, data)
            ok = True
        else:
            prop = X.NONE
            ok = False
        reply = event.SelectionNotify(
            time=req.time, requestor=req.requestor, selection=req.selection,
            target=req.target, property=prop,
        )
        req.requestor.send_event(reply, event_mask=0)
        self.d.sync()


def set_clipboard_and_serve(content: bytes, serve_for_s: float = 30.0):
    """Fork a background process that owns CLIPBOARD for ``serve_for_s``."""
    pid = os.fork()
    if pid == 0:
        # Child: own + serve.
        try:
            owner = ClipboardOwner(content)
            owner.serve(serve_for_s)
        except Exception as exc:
            sys.stderr.write(f"clipboard-owner error: {exc}\n")
        os._exit(0)
    # Parent: return the child PID so caller can wait().
    time.sleep(0.2)  # give child a moment to take ownership
    return pid


# ---------------------------------------------------------------------------
# Type text via clipboard (set + paste). Reliable and fast.
# ---------------------------------------------------------------------------
def paste_text(d, text: str):
    pid = set_clipboard_and_serve(text.encode("utf-8"))
    time.sleep(0.3)
    send_chord(d, "Ctrl+v")
    time.sleep(0.2)
    return pid


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("windows")
    sub.add_parser("focus-isaac")
    s = sub.add_parser("focus"); s.add_argument("pattern")
    s = sub.add_parser("keys");  s.add_argument("chord")
    s = sub.add_parser("type");  s.add_argument("text")
    s = sub.add_parser("paste-file"); s.add_argument("path"); s.add_argument("--run", action="store_true")
    s = sub.add_parser("click")
    s.add_argument("x"); s.add_argument("y"); s.add_argument("--button", default="1")
    s = sub.add_parser("shot")
    s.add_argument("--x", type=int, default=0); s.add_argument("--y", type=int, default=0)
    s.add_argument("--w", type=int, default=0); s.add_argument("--h", type=int, default=0)
    s.add_argument("--out", default="/tmp/shot.png")
    args = ap.parse_args()

    d = open_display()
    if args.cmd == "windows":
        for wid, name in list_windows(d):
            print(f"  0x{wid:08x}  {name}")
        return

    if args.cmd == "focus-isaac":
        wid, name = find_isaac(d)
        if not wid:
            print("no Isaac window found"); sys.exit(1)
        focus_window(d, wid)
        time.sleep(0.3)
        print(f"focused 0x{wid:08x}  {name}")
        return

    if args.cmd == "focus":
        wid, name = find_window(d, args.pattern)
        if not wid:
            print(f"no window matching {args.pattern!r}"); sys.exit(1)
        focus_window(d, wid)
        time.sleep(0.3)
        print(f"focused 0x{wid:08x}  {name}")
        return

    if args.cmd == "keys":
        send_chord(d, args.chord)
        print(f"sent {args.chord}")
        return

    if args.cmd == "click":
        x, y = int(args.x), int(args.y)
        button = int(args.button)
        # Warp pointer to (x, y) on the screen, then synthesize a press/release.
        d.screen().root.warp_pointer(x, y)
        d.sync(); time.sleep(0.05)
        xtest.fake_input(d, X.ButtonPress, button); d.sync()
        time.sleep(0.05)
        xtest.fake_input(d, X.ButtonRelease, button); d.sync()
        print(f"clicked button {button} at ({x},{y})")
        return

    if args.cmd == "shot":
        from PIL import Image
        root = d.screen().root
        g = root.get_geometry()
        x0, y0 = args.x, args.y
        w  = args.w if args.w > 0 else (g.width  - x0)
        h  = args.h if args.h > 0 else (g.height - y0)
        chunks = []; row = 0
        while row < h:
            ch = min(200, h - row)
            img = root.get_image(x0, y0 + row, w, ch, X.ZPixmap, 0xffffffff)
            chunks.append(img.data); row += ch
        raw = b"".join(chunks)
        pil = Image.frombytes("RGB", (w, h), raw, "raw", "BGRX")
        pil.save(args.out, "PNG", optimize=True)
        print(f"saved {args.out} ({w}x{h})")
        return

    if args.cmd == "type":
        pid = paste_text(d, args.text)
        print(f"pasted {len(args.text)} chars (clipboard pid={pid})")
        return

    if args.cmd == "paste-file":
        path = Path(args.path).expanduser().resolve()
        content = path.read_text()
        wid, name = find_isaac(d)
        if not wid:
            print("no Isaac window found"); sys.exit(1)
        focus_window(d, wid)
        time.sleep(0.4)
        # Select all and replace
        send_chord(d, "Ctrl+a"); time.sleep(0.1)
        send_chord(d, "Delete"); time.sleep(0.1)
        pid = paste_text(d, content)
        print(f"pasted {len(content)} chars into Isaac (clipboard pid={pid})")
        if args.run:
            time.sleep(0.5)
            send_chord(d, "Ctrl+Return")
            print("sent Ctrl+Enter (run)")
        return


if __name__ == "__main__":
    main()
