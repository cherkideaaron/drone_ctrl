import os, platform, shutil, subprocess, sys, struct

HERE    = os.path.dirname(os.path.abspath(__file__))
SOURCES = [os.path.join(HERE, "core", "controller.c"),
           os.path.join(HERE, "dll",  "dll_wrapper.c")]
OUT_DIR = os.path.join(HERE, "build")
IS_WIN  = platform.system() == "Windows"
OUT     = os.path.join(OUT_DIR, "controller.dll" if IS_WIN else "libcontroller.so")
PYTHON_BITS = struct.calcsize("P") * 8


def run_cmd(cmd):
    print(">", " ".join(cmd))
    return subprocess.call(cmd) == 0


def gcc_machine(path):
    try:
        r = subprocess.run([path, "-dumpmachine"], capture_output=True, text=True, timeout=5)
        return r.stdout.strip() if r.returncode == 0 else ""
    except Exception:
        return ""


def build():
    os.makedirs(OUT_DIR, exist_ok=True)
    warn = ["-O2", "-Wall", "-Wextra", "-std=c99", "-pedantic"]

    gcc_candidates = ["x86_64-w64-mingw32-gcc", "gcc"]
    for name in gcc_candidates:
        gcc = shutil.which(name)
        if not gcc:
            continue
        machine = gcc_machine(gcc)
        if PYTHON_BITS == 64 and "x86_64" not in machine:
            print(f"  Skipping {gcc!r} (target={machine!r}) - need 64-bit to match Python")
            continue
        print(f"  Using {gcc!r} (target={machine!r})")
        fPIC = [] if IS_WIN else ["-fPIC"]
        if run_cmd([gcc] + warn + ["-shared"] + fPIC + ["-o", OUT] + SOURCES + ["-lm"]):
            return True

    cl = shutil.which("cl")
    if cl:
        cmd = [cl, "/nologo", "/O2", "/W3", "/LD"] + SOURCES + \
              [f"/Fe:{OUT}", f"/Fo:{OUT_DIR}\\"]
        if run_cmd(cmd):
            return True

    try:
        import ziglang
        target = ["-target", "x86_64-windows-gnu"] if IS_WIN else []
        cmd = [sys.executable, "-m", "ziglang", "cc", "-shared", "-O2"] + target + \
              ["-o", OUT] + SOURCES
        if run_cmd(cmd):
            return True
    except ImportError:
        pass

    return False


if __name__ == "__main__":
    print(f"Python is {PYTHON_BITS}-bit; building matching DLL...")
    if build():
        print(f"\nOK  ->  {OUT}")
    else:
        print("\nNo working 64-bit C compiler found.")
        print("Easiest fix:  pip install ziglang   then run   python build.py   again")
        print("Or install MSYS2 and add C:\\msys64\\mingw64\\bin to your PATH.")
        sys.exit(1)