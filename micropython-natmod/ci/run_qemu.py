#!/usr/bin/env python3
"""Run tiny_bclibc test suite on MicroPython QEMU.

Usage:
  python3 ci/run_qemu.py <firmware.elf> <natmod-dir> [--machine MACHINE] [--qemu-extra ARGS]

<natmod-dir> must contain:
  tiny_bclibc.mpy       — built for the target architecture
  tiny_bclibc_types.py  — Python companion module
  test_bclibc.py        — test suite

Examples:
  # Cortex-M3 (armv7m)
  python3 ci/run_qemu.py firmware.elf micropython-natmod/

  # Cortex-M0 / nRF51 (armv6m)
  python3 ci/run_qemu.py firmware.elf micropython-natmod/ \\
    --machine microbit \\
    --qemu-extra "-global nrf51-soc.flash-size=1048576 -global nrf51-soc.sram-size=262144"
"""

import sys
import os
import argparse

# pyboard.py lives in MicroPython's tools/ directory.
# MPY_DIR env var (set in CI) takes precedence over the local default.
_HERE = os.path.dirname(os.path.abspath(__file__))
_MPY_ROOT = os.environ.get("MPY_DIR") or os.path.join(_HERE, "..", "..", "micropython")
sys.path.insert(0, os.path.join(_MPY_ROOT, "tools"))

from pyboard import Pyboard  # noqa: E402


def read_file(path: str) -> bytes:
    with open(path, "rb") as f:
        return f.read()


def inject_mpy(mpy_data: bytes) -> bytes:
    """Mount tiny_bclibc.mpy from a RAM buffer and import it as tiny_bclibc."""
    buf_repr = repr(mpy_data).encode()
    return (
        b"import sys, io, vfs\n"
        b"__buf = " + buf_repr + b"\n"
        b"class _F(io.IOBase):\n"
        b"  def __init__(self): self.off=0\n"
        b"  def ioctl(self,r,a): return 0 if r==4 else -1\n"
        b"  def readinto(self,b):\n"
        b"    b[:]=memoryview(__buf)[self.off:self.off+len(b)]\n"
        b"    self.off+=len(b); return len(b)\n"
        b"class _FS:\n"
        b"  def mount(self,r,m): pass\n"
        b"  def chdir(self,p): pass\n"
        b"  def stat(self,p):\n"
        b"    if p=='/__injected.mpy': return tuple(0 for _ in range(10))\n"
        b"    raise OSError(-2)\n"
        b"  def open(self,p,m): return _F()\n"
        b"vfs.mount(_FS(),'/__remote')\n"
        b"sys.path.insert(0,'/__remote')\n"
        b"sys.modules['tiny_bclibc']=__import__('__injected')\n"
    )


def inject_types(types_src: bytes) -> bytes:
    """Exec tiny_bclibc_types.py and register it in sys.modules."""
    public = b"Shot,Wind,Config,Request,DRAG_G1,DRAG_G7,DRAG_CUSTOM"
    return (
        types_src + b"\n"
        b"import sys as _s\n"
        b"class _M: pass\n"
        b"_m=_M(); _m.__name__='tiny_bclibc_types'\n"
        b"for _k in (" + b",".join(b"'" + n + b"'" for n in public.split(b",")) + b",):\n"
        b"  setattr(_m,_k,globals()[_k])\n"
        b"_s.modules['tiny_bclibc_types']=_m\n"
    )


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("firmware", help="Path to firmware.elf")
    ap.add_argument(
        "natmod_dir",
        help="Directory with tiny_bclibc.mpy / tiny_bclibc_types.py / test_bclibc.py",
    )
    ap.add_argument(
        "--machine",
        default="mps2-an385",
        help="QEMU -machine value (default: mps2-an385)",
    )
    ap.add_argument("--qemu-extra", default="", help="Extra QEMU arguments inserted before -serial")
    args = ap.parse_args()

    mpy_data = read_file(os.path.join(args.natmod_dir, "tiny_bclibc.mpy"))
    types_src = read_file(os.path.join(args.natmod_dir, "tiny_bclibc_types.py"))
    test_src = read_file(os.path.join(args.natmod_dir, "test_bclibc.py"))

    extra = f" {args.qemu_extra}" if args.qemu_extra else ""
    qemu_cmd = (
        f"qemu-system-arm "
        f"-machine {args.machine} "
        f"-nographic "
        f"-monitor null "
        f"-semihosting"
        f"{extra} "
        f"-serial pty "
        f"-kernel {args.firmware}"
    )

    print(f"[QEMU] Starting: {qemu_cmd}", flush=True)
    pyb = Pyboard(f"execpty:{qemu_cmd}")
    pyb.enter_raw_repl()

    print("[QEMU] Injecting tiny_bclibc.mpy ...", flush=True)
    pyb.exec_(inject_mpy(mpy_data), timeout=30)

    print("[QEMU] Injecting tiny_bclibc_types ...", flush=True)
    pyb.exec_(inject_types(types_src), timeout=15)

    print("[QEMU] Running test_bclibc.py ...", flush=True)
    output = pyb.exec_(test_src, timeout=120)

    pyb.exit_raw_repl()
    pyb.close()

    text = output.decode("utf-8", errors="replace")
    print(text)

    if "FAIL" in text:
        print("[QEMU] RESULT: FAILED", file=sys.stderr)
        sys.exit(1)

    print("[QEMU] RESULT: ALL PASSED")


if __name__ == "__main__":
    main()
