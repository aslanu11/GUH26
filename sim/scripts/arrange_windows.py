"""Place the Isaac Sim window on the left half of :0 and the KitMesh page on the right.

Uses only libX11 through ctypes (no xdotool on this box): an EWMH
_NET_MOVERESIZE_WINDOW request to the window manager, after clearing any
maximized state. Run: python3 scripts/arrange_windows.py
"""

from __future__ import annotations

import ctypes
import ctypes.util
import os
import re
import subprocess

os.environ.setdefault("DISPLAY", ":0")
os.environ.setdefault("XAUTHORITY", "/run/user/1000/gdm/Xauthority")
TOP_BAR = 32

x11 = ctypes.cdll.LoadLibrary(ctypes.util.find_library("X11"))
x11.XOpenDisplay.restype = ctypes.c_void_p
x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
x11.XInternAtom.restype = ctypes.c_ulong
x11.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
x11.XDefaultRootWindow.restype = ctypes.c_ulong
x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
x11.XDisplayWidth.argtypes = [ctypes.c_void_p, ctypes.c_int]
x11.XDisplayHeight.argtypes = [ctypes.c_void_p, ctypes.c_int]


class ClientMessage(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_int),
        ("serial", ctypes.c_ulong),
        ("send_event", ctypes.c_int),
        ("display", ctypes.c_void_p),
        ("window", ctypes.c_ulong),
        ("message_type", ctypes.c_ulong),
        ("format", ctypes.c_int),
        ("data", ctypes.c_long * 5),
    ]


class XEvent(ctypes.Union):
    _fields_ = [("xclient", ClientMessage), ("pad", ctypes.c_long * 24)]


x11.XSendEvent.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_long, ctypes.POINTER(XEvent)]
x11.XFlush.argtypes = [ctypes.c_void_p]

display = x11.XOpenDisplay(None)
if not display:
    raise SystemExit("Cannot open the X display")
root = x11.XDefaultRootWindow(display)
width, height = x11.XDisplayWidth(display, 0), x11.XDisplayHeight(display, 0)
MASK = (1 << 20) | (1 << 19)  # SubstructureRedirect | SubstructureNotify


def send(window: int, atom: str, values) -> None:
    event = XEvent()
    event.xclient.type = 33  # ClientMessage
    event.xclient.send_event = 1
    event.xclient.display = display
    event.xclient.window = window
    event.xclient.message_type = x11.XInternAtom(display, atom.encode(), 0)
    event.xclient.format = 32
    for i, value in enumerate(values):
        event.xclient.data[i] = value
    x11.XSendEvent(display, root, 0, MASK, ctypes.byref(event))


def find(title_pattern: str) -> int | None:
    tree = subprocess.run(["xwininfo", "-root", "-tree"], capture_output=True, text=True).stdout
    for line in tree.splitlines():
        match = re.match(r'\s*(0x[0-9a-f]+) "([^"]*)": \("(?!mutter-x11-frames)', line)
        if match and re.search(title_pattern, match.group(2)):
            return int(match.group(1), 16)
    return None


def place(window: int, x: int, y: int, w: int, h: int) -> None:
    vert = x11.XInternAtom(display, b"_NET_WM_STATE_MAXIMIZED_VERT", 0)
    horz = x11.XInternAtom(display, b"_NET_WM_STATE_MAXIMIZED_HORZ", 0)
    send(window, "_NET_WM_STATE", [0, vert, horz, 2, 0])  # remove maximized
    flags = 10 | (1 << 8) | (1 << 9) | (1 << 10) | (1 << 11) | (2 << 12)  # static gravity, x y w h, from pager
    send(window, "_NET_MOVERESIZE_WINDOW", [flags, x, y, w, h])
    x11.XFlush(display)


# GNOME's usable area excludes the top bar and the dock.
area = subprocess.run(["xprop", "-root", "_NET_WORKAREA"], capture_output=True, text=True).stdout
numbers = [int(n) for n in re.findall(r"\d+", area.split("=", 1)[-1])[:4]] if "=" in area else []
left, top, usable_w, usable_h = numbers if len(numbers) == 4 else (0, TOP_BAR, width, height - TOP_BAR)
import sys

if "--isaac-full" in sys.argv:
    # Browser viewing: Isaac fills the work area and is raised, so the page's
    # live feed (a capture of that screen area) shows only Isaac.
    window = find("^Isaac Sim 5\\.1\\.0$")
    if window is None:
        raise SystemExit("Isaac Sim window not found")
    place(window, left, top, usable_w, usable_h)
    send(window, "_NET_ACTIVE_WINDOW", [2, 0, 0, 0, 0])
    x11.XFlush(display)
    print(f"Isaac fills the work area: x={left} y={top} w={usable_w} h={usable_h}")
else:
    half = usable_w // 2
    for pattern, x in (("^Isaac Sim 5\\.1\\.0$", left), ("KitMesh Packing Cell", left + half)):
        window = find(pattern)
        if window is None:
            print(f"not found: {pattern}")
            continue
        place(window, x, top, half, usable_h)
        print(f"placed {pattern} at x={x} y={top} w={half} h={usable_h}")
