import ctypes, os, platform

MODE_NAMES = ["IDLE", "LIFT", "HOVER", "DESCEND", "LANDED"]

_HERE = os.path.dirname(os.path.abspath(__file__))
_LIB  = os.path.join(_HERE, "..", "build",
                     "controller.dll" if platform.system() == "Windows"
                     else "libcontroller.so")


def _load():
    path = os.path.abspath(_LIB)
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"{path} not found.\nRun:  python build.py"
        )
    lib = ctypes.CDLL(path)
    f   = ctypes.c_float
    fp  = ctypes.POINTER(f)

    lib.ctrl_dll_create.restype     = ctypes.c_void_p
    lib.ctrl_dll_destroy.argtypes   = [ctypes.c_void_p]
    lib.ctrl_dll_configure.argtypes = [ctypes.c_void_p, f, f, f, f]
    lib.ctrl_dll_start.argtypes     = [ctypes.c_void_p]
    lib.ctrl_dll_step.argtypes      = [ctypes.c_void_p, f, f, f, fp, fp]
    lib.ctrl_dll_step.restype       = ctypes.c_int
    lib.ctrl_dll_mixer.argtypes     = [f, f, f, f, fp]
    lib.ctrl_dll_mode_name.argtypes = [ctypes.c_int]
    lib.ctrl_dll_mode_name.restype  = ctypes.c_char_p
    return lib


def mode_name_from_dll(mode_int):
    return _load().ctrl_dll_mode_name(mode_int).decode()


def mixer(thrust, roll=0.0, pitch=0.0, yaw=0.0):
    out = (ctypes.c_float * 4)()
    _load().ctrl_dll_mixer(thrust, roll, pitch, yaw, out)
    return list(out)


class CController:
    def __init__(self, hover_height=2.0, climb_speed=1.0,
                 hover_time=5.0, descend_speed=0.7):
        self._lib    = _load()
        self._h      = self._lib.ctrl_dll_create()
        self._motors = (ctypes.c_float * 4)()
        self._status = (ctypes.c_float * 4)()
        self.configure(hover_height, climb_speed, hover_time, descend_speed)

    def configure(self, hover_height, climb_speed, hover_time, descend_speed):
        self._lib.ctrl_dll_configure(self._h,
                                     hover_height, climb_speed,
                                     hover_time, descend_speed)

    def start(self):
        self._lib.ctrl_dll_start(self._h)

    def step(self, height, climb_rate, dt):
        mode = self._lib.ctrl_dll_step(self._h, height, climb_rate, dt,
                                       self._motors, self._status)
        return mode, list(self._motors), {
            "sp_height":  self._status[0],
            "sp_climb":   self._status[1],
            "thrust":     self._status[2],
            "mode_time":  self._status[3],
        }

    def __del__(self):
        try:
            self._lib.ctrl_dll_destroy(self._h)
        except Exception:
            pass