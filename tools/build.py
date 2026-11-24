#!/usr/bin/env python3
"""
skyforge-fc build driver (no make required).

    python tools/build.py test       # build + run all host unit tests
    python tools/build.py sim        # build simulator shared lib + CLI
    python tools/build.py firmware   # cross-compile STM32F405 .elf/.bin
    python tools/build.py all        # everything

Compiler selection:
  host     : $CC if set, else `python -m ziglang cc` if the ziglang package
             is importable, else `cc`.
  firmware : `python -m ziglang cc -target thumb-freestanding-eabihf`
             (use the Makefile for arm-none-eabi-gcc).
"""
import glob
import os
import shlex
import struct
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BUILD = os.path.join(ROOT, "build")
IS_WIN = sys.platform.startswith("win")
EXE = ".exe" if IS_WIN else ""
SHLIB = ".dll" if IS_WIN else (".dylib" if sys.platform == "darwin" else ".so")


def have_zig():
    try:
        import ziglang  # noqa: F401
        return True
    except ImportError:
        return False


def zig_cc():
    # private zig cache inside build/ so parallel builds of other projects
    # sharing the per-user global cache cannot interfere
    cache = os.path.join(BUILD, "zig-cache")
    os.environ.setdefault("ZIG_GLOBAL_CACHE_DIR", cache)
    os.environ.setdefault("ZIG_LOCAL_CACHE_DIR", cache)
    return [sys.executable, "-m", "ziglang", "cc"]


def host_cc():
    if os.environ.get("CC"):
        return shlex.split(os.environ["CC"])
    if have_zig():
        return zig_cc()
    return ["cc"]


def run(cmd, **kw):
    """Run a tool from the repo root; print a short form of the command."""
    shown = [c for c in cmd if not c.endswith(".c")]
    nsrc = len(cmd) - len(shown)
    print("  " + " ".join(shown) + (f"  <{nsrc} sources>" if nsrc else ""), flush=True)
    r = subprocess.run(cmd, cwd=ROOT, **kw)
    if r.returncode != 0:
        sys.exit(f"build step failed (exit {r.returncode})")


def rel(*p):
    """Repo-relative path with forward slashes (commands run with cwd=ROOT)."""
    return "/".join(p)


def srcs(*pattern):
    return sorted(os.path.relpath(f, ROOT).replace(os.sep, "/")
                  for f in glob.glob(os.path.join(ROOT, *pattern)))


CORE = srcs("src", "core", "*.c")
DRIVERS = srcs("src", "drivers", "*.c")
SIM = [rel("sim", "quad_sim.c")]
HOST_FLAGS = ["-std=c11", "-O2", "-g", "-Wall", "-Wextra", "-Wshadow", "-Werror",
              "-Wno-unused-function", "-DFC_HOST"]


def build_tests():
    out = os.path.join(BUILD, "host")
    os.makedirs(out, exist_ok=True)
    tests = srcs("tests", "test_*.c")
    exes = []
    for t in tests:
        name = os.path.splitext(os.path.basename(t))[0]
        exe = os.path.join(out, name + EXE)
        run(host_cc() + HOST_FLAGS + ["-o", exe, t] + CORE + DRIVERS + SIM + ["-lm"])
        exes.append(exe)
    return exes


def cmd_test():
    exes = build_tests()
    failed = []
    for exe in exes:
        print(f"\n=== {os.path.basename(exe)} ===", flush=True)
        r = subprocess.run([exe])
        if r.returncode != 0:
            failed.append(os.path.basename(exe))
    print()
    if failed:
        print("FAILED: " + ", ".join(failed))
        sys.exit(1)
    print(f"ALL {len(exes)} TEST BINARIES PASSED")


def cmd_sim():
    out = os.path.join(BUILD, "host")
    os.makedirs(out, exist_ok=True)
    lib = os.path.join(out, "skyforge_sim" + SHLIB)
    flags = [f for f in HOST_FLAGS if f != "-g"]
    run(host_cc() + flags + ["-shared", "-fPIC", "-o", lib] + SIM + CORE + ["-lm"])
    exe = os.path.join(out, "sim_main" + EXE)
    run(host_cc() + flags + ["-o", exe, rel("sim", "sim_main.c")] + SIM + CORE + ["-lm"])
    return lib, exe


FW_SRC = (CORE + DRIVERS + srcs("src", "platform", "stm32f4", "*.c") +
          srcs("src", "boards", "f405_generic", "*.c") + [rel("src", "app", "main.c")])


def elf_sizes(path):
    """Tiny ELF32 section-header reader: returns (text+rodata+data, data+bss)."""
    with open(path, "rb") as f:
        d = f.read()
    shoff = struct.unpack_from("<I", d, 0x20)[0]
    shentsize, shnum, shstrndx = struct.unpack_from("<HHH", d, 0x2E)
    secs = []
    for i in range(shnum):
        secs.append(struct.unpack_from("<IIIIIIIIII", d, shoff + i * shentsize))
    strtab_off = secs[shstrndx][4]

    def name(s):
        o = strtab_off + s[0]
        return d[o:d.index(b"\0", o)].decode()

    flash = ram = 0
    for s in secs:
        n, typ, flags, size = name(s), s[1], s[2], s[5]
        if not flags & 0x2:  # SHF_ALLOC
            continue
        if typ == 8:  # NOBITS (.bss, stack)
            ram += size
        elif flags & 0x1 and n.startswith(".data"):
            flash += size
            ram += size
        else:
            flash += size
    return flash, ram


def check_vectors(binf):
    """Sanity-check the image: initial SP at top of SRAM, reset/IRQ vectors
    inside flash with the Thumb bit set."""
    with open(os.path.join(ROOT, binf), "rb") as f:
        words = struct.unpack("<98I", f.read(4 * 98))
    sp, reset = words[0], words[1]
    ok = sp == 0x20020000
    used = [w for w in words[1:] if w]
    ok &= all(0x08000000 <= (w & ~1) < 0x080E0000 and (w & 1) for w in used)
    print(f"  vector table: SP={sp:#x} Reset_Handler={reset:#x} "
          f"({len(used)} vectors) -> {'OK' if ok else 'BAD'}")
    if not ok:
        sys.exit("vector table check failed")


def cmd_firmware():
    if not have_zig():
        sys.exit("firmware build needs `pip install ziglang` (or use the Makefile with arm-none-eabi-gcc)")
    out = os.path.join(BUILD, "fw")
    os.makedirs(out, exist_ok=True)
    elf = os.path.join(out, "skyforge-f405.elf")
    binf = os.path.join(out, "skyforge-f405.bin")
    # zig's plain "cortex_m4" CPU model has no FPU; +vfp4d16sp = FPv4-SP-D16
    # (the Cortex-M4F single-precision FPU), eabihf = hard-float calling conv.
    flags = ["-target", "thumb-freestanding-eabihf", "-mcpu=cortex_m4+vfp4d16sp",
             "-std=c11", "-O2", "-g", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function",
             "-ffreestanding", "-fno-builtin", "-fno-math-errno", "-ffunction-sections",
             "-fdata-sections", "-fno-unwind-tables", "-fno-asynchronous-unwind-tables",
             "-nostdlib", "-DSTM32F405xx", "-DBOARD_F405_GENERIC",
             "-I" + rel("src"), "-I" + rel("src", "boards", "f405_generic"),
             "-Wl,--gc-sections", "-Wl,--entry=Reset_Handler",
             "-Wl,-T," + rel("ld", "stm32f405.ld")]
    run(zig_cc() + flags + ["-o", elf] + FW_SRC)
    run(zig_cc()[:3] + ["objcopy", "-O", "binary", elf, binf])
    check_vectors(binf)
    flash, ram = elf_sizes(elf)
    print(f"\n  {os.path.relpath(elf, ROOT)}: flash {flash} B ({100*flash/917504:.1f}% of 896 KiB app area), "
          f"static RAM {ram} B ({100*ram/131072:.1f}% of 128 KiB)")
    print(f"  {os.path.relpath(binf, ROOT)}: {os.path.getsize(binf)} B")


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "all"
    if cmd == "test":
        cmd_test()
    elif cmd == "sim":
        cmd_sim()
    elif cmd == "firmware":
        cmd_firmware()
    elif cmd == "all":
        cmd_test()
        cmd_sim()
        cmd_firmware()
    elif cmd == "clean":
        import shutil
        shutil.rmtree(BUILD, ignore_errors=True)
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main()
