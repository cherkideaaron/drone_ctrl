from vpython import (canvas, vector, box, cylinder, ring, sphere, color, rate, button,
                     wtext, compound, curve, label)
import math

ARM = 0.30
ROTOR_R = 0.17


ROTOR_POS = [(+ARM, +ARM), (-ARM, -ARM), (+ARM, -ARM), (-ARM, +ARM)]
ROTOR_DIR = [+1, +1, -1, -1]
SPIN_MAX = 60.0
MODE_COLOR = {"OUTBOUND": (1.0, 0.55, 0.1), "RETURN": (0.2, 0.85, 1.0)}


def _cross(a, b):
    return vector(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x)


class QuadView:
    def __init__(self, diag_horiz=2.0, top_height=4.0):
        self.start_requested = False
        self.reset_requested = False
        self.gust_requested = False
        self._frame = 0
        self._ang = [0.0] * 4
        self._last_trail_mode = None

        self.scene = canvas(title="Quadcopter - own C controller (Step 2: balance + 45 deg out-and-back)",
                            width=1100, height=640, background=vector(0.09, 0.10, 0.13))
        span = max(diag_horiz, 2.0)
        self.scene.center = vector(span * 0.5, top_height * 0.5, 0)
        self.scene.camera.pos = vector(span * 0.5 + 1.5, top_height * 0.5 + 0.8, 7.5 + span * 0.4)
        self.scene.camera.axis = self.scene.center - self.scene.camera.pos
        self.scene.up = vector(0, 1, 0)

        self._build_world(top_height, span)
        self._build_drone()

        self.scene.append_to_caption("\n")
        button(text="  Start  ", bind=self._on_start)
        button(text="  Reset  ", bind=self._on_reset)
        button(text="  Gust (kick the drone)  ", bind=self._on_gust)
        self.scene.append_to_caption("\n\n")
        self.hud = wtext(text="")
        self.scene.append_to_caption("\n")
        self._new_trail(color.white)


    def _build_world(self, top, span):
        half = int(span + 5)
        box(pos=vector(0, -0.02, 0), size=vector(2 * half, 0.04, 2 * half), color=vector(0.20, 0.22, 0.25))
        for i in range(-half, half + 1):
            curve(vector(i, 0.001, -half), vector(i, 0.001, half), color=vector(0.30, 0.32, 0.36), radius=0.004)
            curve(vector(-half, 0.001, i), vector(half, 0.001, i), color=vector(0.30, 0.32, 0.36), radius=0.004)
        cylinder(pos=vector(0, 0, 0), axis=vector(0, 0.01, 0), radius=0.55, color=vector(0.85, 0.75, 0.15))
        cylinder(pos=vector(0, 0.011, 0), axis=vector(0, 0.002, 0), radius=0.45, color=vector(0.20, 0.22, 0.25))

        rtop = max(3.0, top + 1.0)
        cylinder(pos=vector(-1.6, 0, -1.6), axis=vector(0, rtop, 0), radius=0.012, color=color.white)
        for k in range(int(rtop / 0.5) + 1):
            y = k * 0.5
            cylinder(pos=vector(-1.6, y, -1.6), axis=vector(0.12, 0, 0), radius=0.006, color=color.white)
            if k % 2 == 0:
                label(pos=vector(-1.85, y, -1.6), text=f"{y:.0f} m", box=False,
                      height=11, color=color.white, opacity=0)
        self.shadow = cylinder(pos=vector(0, 0.015, 0), axis=vector(0, 0.002, 0),
                               radius=0.25, color=color.black, opacity=0.5)
        self.setpoint_ring = ring(pos=vector(0, 0.0, 0), axis=vector(0, 1, 0), radius=0.5,
                                  thickness=0.012, color=color.green)
        self.setpoint_dot = sphere(pos=vector(-1.6, 0, -1.6), radius=0.045, color=color.green)


    def _build_drone(self):

        parts = [box(pos=vector(0, 0, 0), size=vector(0.26, 0.07, 0.26), color=vector(0.15, 0.17, 0.20)),
                 box(pos=vector(0, 0.045, 0), size=vector(0.12, 0.03, 0.12), color=vector(0.9, 0.35, 0.1))]
        for ang in (45, -45):
            a = box(pos=vector(0, 0, 0), size=vector(2 * ARM * 1.414 + 0.06, 0.03, 0.05),
                    color=vector(0.25, 0.27, 0.30))
            a.rotate(angle=math.radians(ang), axis=vector(0, 1, 0), origin=vector(0, 0, 0))
            parts.append(a)
        for (fx, rz) in ROTOR_POS:
            parts.append(cylinder(pos=vector(fx, 0.0, rz), axis=vector(0, 0.06, 0), radius=0.035,
                                  color=vector(0.1, 0.1, 0.12)))
        parts.append(box(pos=vector(0.17, 0.0, 0), size=vector(0.06, 0.04, 0.06), color=color.red))
        self.body = compound(parts)
        self._body_len = self.body.axis.mag

        self.blades = []
        for i in range(4):
            pair = []
            for _ in range(2):
                pair.append(box(pos=vector(0, 0.2, 0), size=vector(2 * ROTOR_R, 0.008, 0.03),
                                color=color.white if ROTOR_DIR[i] > 0 else vector(0.7, 0.85, 1.0)))
            self.blades.append(pair)
        self.discs = [cylinder(pos=vector(0, 0.2, 0), axis=vector(0, 0.004, 0), radius=ROTOR_R,
                               color=color.white, opacity=0.0) for _ in range(4)]


    def _new_trail(self, col):
        self.trail = curve(color=col, radius=0.012)
        self._trail_n = 0

    def clear_trail(self):
        self.trail.clear()
        self._new_trail(color.white)
        self._last_trail_mode = None


    def _on_start(self, b):
        self.start_requested = True

    def _on_reset(self, b):
        self.reset_requested = True

    def _on_gust(self, b):
        self.gust_requested = True

    def wait_frame(self, fps):
        rate(fps)


    def update(self, phys, motors, sp, mode_name, mode, thrust, mode_time):

        pos = vector(phys.x, phys.z, -phys.y)


        cp, sp_ = math.cos(phys.pitch), math.sin(phys.pitch)
        cr, sr = math.cos(phys.roll), math.sin(phys.roll)
        fwd = vector(cp, sp_, 0)
        up0 = vector(-sp_, cp, 0)
        right = _cross(fwd, up0)
        up = up0 * cr + right * sr
        right = _cross(fwd, up)

        self.body.pos = pos + up * 0.0 + vector(0, 0.12, 0)
        self.body.axis = fwd * self._body_len
        self.body.up = up

        dt_frame = 1.0 / 60.0
        for i, (fx, rz) in enumerate(ROTOR_POS):
            centre = self.body.pos + fwd * fx + right * rz + up * 0.08
            self._ang[i] += ROTOR_DIR[i] * SPIN_MAX * motors[i] * dt_frame
            for k, b in enumerate(self.blades[i]):
                a = self._ang[i] + k * math.pi / 2
                axis = fwd * math.cos(a) + _cross(up, fwd) * math.sin(a)
                b.pos = centre
                b.axis = axis * (2 * ROTOR_R)
                b.up = up
            self.discs[i].pos = centre
            self.discs[i].axis = up * 0.004
            self.discs[i].opacity = 0.0 if motors[i] < 0.05 else min(0.22, 0.05 + 0.25 * motors[i])

        self.shadow.pos = vector(phys.x, 0.015, -phys.y)
        self.shadow.radius = 0.22 + 0.025 * phys.z
        self.shadow.opacity = max(0.12, 0.55 / (1.0 + 0.35 * phys.z))

        self.setpoint_ring.pos = vector(sp[0], sp[2], -sp[1])
        self.setpoint_dot.pos = vector(-1.6, sp[2], -1.6)


        if mode_name != self._last_trail_mode:
            if mode_name in MODE_COLOR or self._last_trail_mode in MODE_COLOR:
                c = MODE_COLOR.get(mode_name, (1, 1, 1))
                self._new_trail(vector(*c))
            self._last_trail_mode = mode_name
        if mode > 0 and self._frame % 3 == 0 and self._trail_n < 4000:
            self.trail.append(pos)
            self._trail_n += 1

        self._frame += 1
        if self._frame % 6 == 0:
            speed = math.sqrt(phys.vx ** 2 + phys.vy ** 2 + phys.vz ** 2)
            self.hud.text = (
                f"<b>MODE: {mode_name}</b> &nbsp;&nbsp; mode time {mode_time:5.1f} s<br>"
                f"forward {phys.x:5.2f} m &nbsp; height {phys.z:5.2f} m &nbsp; speed {speed:4.2f} m/s &nbsp; "
                f"collective thrust {thrust*100:4.0f} %<br>"
                f"tilt: pitch {math.degrees(phys.pitch):+5.1f}&deg; &nbsp; roll {math.degrees(phys.roll):+5.1f}&deg; "
                f"&nbsp;&nbsp; motors &nbsp; M0 {motors[0]*100:3.0f} % &nbsp; M1 {motors[1]*100:3.0f} % "
                f"&nbsp; M2 {motors[2]*100:3.0f} % &nbsp; M3 {motors[3]*100:3.0f} %")
