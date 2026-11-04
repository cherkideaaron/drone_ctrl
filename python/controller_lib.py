import ctypes, os, platform

MODE_NAMES = ["IDLE", "LIFT", "HOVER", "OUTBOUND", "FAR_HOLD", "RETURN", "DESCEND", "LANDED"]
IDLE, LIFT, HOVER, OUTBOUND, FAR_HOLD, RETURN, DESCEND, LANDED = range(8)

_HERE = os.path.dirname(os.path.abspath(__file__))
_LIB = os.path.join(_HERE, "..", "build",
                    "controller.dll" if platform.system() == "Windows" else "libcontroller.so")


def _load():
    path = os.path.abspath(_LIB)
    if not os.path.exists(path):
        raise FileNotFoundError(f"{path} not found - run:  python build.py")
    lib = ctypes.CDLL(path)
    f = ctypes.c_float
    fp = ctypes.POINTER(f)
    lib.ctrl_dll_create.restype = ctypes.c_void_p
    lib.ctrl_dll_destroy.argtypes = [ctypes.c_void_p]
    lib.ctrl_dll_configure.argtypes = [ctypes.c_void_p, fp]
    lib.ctrl_dll_start.argtypes = [ctypes.c_void_p]
    lib.ctrl_dll_step.argtypes = [ctypes.c_void_p, fp, f, fp, fp]
    lib.ctrl_dll_step.restype = ctypes.c_int
    lib.ctrl_dll_mixer.argtypes = [f, f, f, f, fp]
    return lib


def mixer(thrust, roll=0.0, pitch=0.0, yaw=0.0):
    out = (ctypes.c_float * 4)()
    _load().ctrl_dll_mixer(thrust, roll, pitch, yaw, out)
    return list(out)


class CController:
    def __init__(self, hover_height=2.0, climb_speed=1.0, hover_time=5.0, descend_speed=0.7,
                 diag_horiz=2.0, diag_angle=45.0, diag_speed=1.0, far_hold=1.5):
        self._lib = _load()
        self._h = self._lib.ctrl_dll_create()
        self._sens = (ctypes.c_float * 10)()
        self._motors = (ctypes.c_float * 4)()
        self._status = (ctypes.c_float * 8)()
        self.configure(hover_height, climb_speed, hover_time, descend_speed,
                       diag_horiz, diag_angle, diag_speed, far_hold)

    def configure(self, hover_height=2.0, climb_speed=1.0, hover_time=5.0, descend_speed=0.7,
                  diag_horiz=2.0, diag_angle=45.0, diag_speed=1.0, far_hold=1.5):
        cfg = (ctypes.c_float * 8)(hover_height, climb_speed, hover_time, descend_speed,
                                   diag_horiz, diag_angle, diag_speed, far_hold)
        self._lib.ctrl_dll_configure(self._h, cfg)

    def start(self):
        self._lib.ctrl_dll_start(self._h)

    def step(self, sensors, dt):
        for i, v in enumerate(sensors):
            self._sens[i] = v
        mode = self._lib.ctrl_dll_step(self._h, self._sens, dt, self._motors, self._status)
        s = self._status
        return mode, list(self._motors), {
            "sp": (s[0], s[1], s[2]), "roll_des": s[3], "pitch_des": s[4],
            "thrust": s[5], "mode_time": s[6]}

    def __del__(self):
        try:
            self._lib.ctrl_dll_destroy(self._h)
        except Exception:
            pass
