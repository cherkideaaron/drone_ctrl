from vpython import (canvas, vector, box, cylinder, ring, sphere, color, rate, button,
                     wtext, compound, curve, label)
import math

ARM     = 0.30
ROTOR_R = 0.17
ROTOR_POS = [(+ARM, +ARM), (-ARM, -ARM), (+ARM, -ARM), (-ARM, +ARM)]
ROTOR_DIR = [+1, +1, -1, -1]
SPIN_MAX  = 60.0


class QuadView:
    def __init__(self, hover_height_hint=2.0):
        self.start_requested = False
        self.reset_requested = False
        self._frame = 0

        self.scene = canvas(title="Quadcopter - own C controller (Step 1: lift / hover / descend)",
                            width=1100, height=640, background=vector(0.09, 0.10, 0.13))
        self.scene.center = vector(0, max(1.2, hover_height_hint * 0.55), 0)
        self.scene.camera.pos = vector(4.6, hover_height_hint * 0.75 + 1.0, 5.6)
        self.scene.camera.axis = self.scene.center - self.scene.camera.pos
        self.scene.up = vector(0, 1, 0)

        self._build_world(hover_height_hint)
        self._build_drone()

        self.scene.append_to_caption("\n")
        button(text="  Start  ", bind=self._on_start)
        button(text="  Reset  ", bind=self._on_reset)
        self.scene.append_to_caption("\n\n")
        self.hud = wtext(text="")
        self.scene.append_to_caption("\n")

        self._spin = [0.0] * 4
        self.update(0.0, 0.0, [0, 0, 0, 0], 0.0, "IDLE", 0.0, 0.0)

    def _build_world(self, hh):
        box(pos=vector(0, -0.02, 0), size=vector(10, 0.04, 10), color=vector(0.20, 0.22, 0.25))
        for i in range(-5, 6):
            curve(vector(i, 0.001, -5), vector(i, 0.001, 5), color=vector(0.30, 0.32, 0.36), radius=0.004)
            curve(vector(-5, 0.001, i), vector(5, 0.001, i), color=vector(0.30, 0.32, 0.36), radius=0.004)
        cylinder(pos=vector(0, 0, 0), axis=vector(0, 0.01, 0), radius=0.55,
                 color=vector(0.85, 0.75, 0.15))
        cylinder(pos=vector(0, 0.011, 0), axis=vector(0, 0.002, 0), radius=0.45,
                 color=vector(0.20, 0.22, 0.25))
        top = max(3.0, hh + 1.0)
        cylinder(pos=vector(-1.6, 0, -1.6), axis=vector(0, top, 0), radius=0.012, color=color.white)
        n = int(top / 0.5)
        for k in range(n + 1):
            y = k * 0.5
            cylinder(pos=vector(-1.6, y, -1.6), axis=vector(0.12, 0, 0), radius=0.006, color=color.white)
            if k % 2 == 0:
                label(pos=vector(-1.8, y, -1.6), text=f"{y:.0f} m", box=False,
                      height=11, color=color.white, opacity=0)
        self.shadow = cylinder(pos=vector(0, 0.015, 0), axis=vector(0, 0.002, 0),
                               radius=0.25, color=color.black, opacity=0.5)
        self.altline = cylinder(pos=vector(0, 0, 0), axis=vector(0, 0.01, 0), radius=0.005,
                                color=vector(0.4, 0.8, 1.0), opacity=0.6)
        self.setpoint_ring = ring(pos=vector(0, 0.0, 0), axis=vector(0, 1, 0), radius=0.55,
                                  thickness=0.012, color=color.green)
        self.setpoint_dot = sphere(pos=vector(-1.6, 0, -1.6), radius=0.045, color=color.green)

    def _build_drone(self):
        parts = [box(pos=vector(0, 0, 0), size=vector(0.26, 0.07, 0.26), color=vector(0.15, 0.17, 0.20)),
                 box(pos=vector(0, 0.045, 0.06), size=vector(0.12, 0.03, 0.10), color=vector(0.9, 0.35, 0.1))]
        for ang in (45, -45):
            a = box(pos=vector(0, 0, 0), size=vector(2 * ARM * 1.414 + 0.06, 0.03, 0.05),
                    color=vector(0.25, 0.27, 0.30))
            a.rotate(angle=math.radians(ang), axis=vector(0, 1, 0), origin=vector(0, 0, 0))
            parts.append(a)
        for (x, z) in ROTOR_POS:
            parts.append(cylinder(pos=vector(x, 0.0, z), axis=vector(0, 0.06, 0), radius=0.035,
                                  color=vector(0.1, 0.1, 0.12)))
        parts.append(box(pos=vector(0, 0.0, 0.17), size=vector(0.06, 0.04, 0.06), color=color.red))
        self.body = compound(parts)
        self.body_origin = vector(0, 0.12, 0)
        self.body.pos    = self.body_origin

        self.blades, self.discs = [], []
        for i, (x, z) in enumerate(ROTOR_POS):
            pair = []
            for extra in (0, 90):
                b = box(pos=vector(x, 0.2, z), size=vector(2 * ROTOR_R, 0.008, 0.03),
                        color=color.white if ROTOR_DIR[i] > 0 else vector(0.7, 0.85, 1.0))
                b.rotate(angle=math.radians(extra), axis=vector(0, 1, 0))
                pair.append(b)
            self.blades.append(pair)
            self.discs.append(cylinder(pos=vector(x, 0.2, z), axis=vector(0, 0.004, 0),
                                       radius=ROTOR_R, color=color.white, opacity=0.0))

    def _on_start(self, b): self.start_requested = True
    def _on_reset(self, b): self.reset_requested = True
    def wait_frame(self, fps): rate(fps)
    def set_mode_text(self, text): pass

    def update(self, height, vz, motors, sp_height, mode, thrust, mode_time):
        base = vector(0, height, 0)
        self.body.pos = self.body_origin + base

        dt_frame = 1.0 / 60.0
        for i, (x, z) in enumerate(ROTOR_POS):
            spin   = ROTOR_DIR[i] * SPIN_MAX * motors[i] * dt_frame
            centre = vector(x, 0.2 + height, z)
            for b in self.blades[i]:
                b.pos = centre
                b.rotate(angle=spin, axis=vector(0, 1, 0), origin=centre)
            self.discs[i].pos     = centre
            self.discs[i].opacity = 0.0 if motors[i] < 0.05 else min(0.22, 0.05 + 0.25 * motors[i])

        self.shadow.radius  = 0.22 + 0.025 * height
        self.shadow.opacity = max(0.12, 0.55 / (1.0 + 0.35 * height))
        self.altline.axis   = vector(0, max(height, 0.01), 0)
        self.setpoint_ring.pos = vector(0, sp_height, 0)
        self.setpoint_dot.pos  = vector(-1.6, sp_height, -1.6)

        self._frame += 1
        if self._frame % 6 == 0:
            self.hud.text = (
                f"<b>MODE: {mode}</b> &nbsp;&nbsp; mode time {mode_time:5.1f} s<br>"
                f"height {height:5.2f} m &nbsp;&nbsp; setpoint {sp_height:5.2f} m &nbsp;&nbsp; "
                f"climb rate {vz:+5.2f} m/s &nbsp;&nbsp; collective thrust {thrust*100:4.0f} %<br>"
                f"motors &nbsp; M0 {motors[0]*100:3.0f} % &nbsp; M1 {motors[1]*100:3.0f} % &nbsp; "
                f"M2 {motors[2]*100:3.0f} % &nbsp; M3 {motors[3]*100:3.0f} %")