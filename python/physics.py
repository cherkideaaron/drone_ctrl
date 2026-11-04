import math

G = 9.81


class QuadPhysics:
    def __init__(self, mass=1.5, thrust_to_weight=2.0, motor_tau=0.05, drag=0.10,
                 arm=0.20, inertia=0.03, ang_damp=0.2):
        self.mass = mass
        self.max_thrust_total = thrust_to_weight * mass * G
        self.motor_tau = motor_tau
        self.drag = drag
        self.arm = arm
        self.inertia = inertia
        self.ang_damp = ang_damp
        self.reset()

    def reset(self):
        self.x = self.y = self.z = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.roll = self.pitch = 0.0
        self.p = self.q = 0.0
        self.motor_actual = [0.0] * 4
        self.impact_speed = None
        self._was_airborne = False
        self.ext_torque = [0.0, 0.0]


    def kick(self, roll_rate=0.0, pitch_rate=0.0):
        self.p += roll_rate
        self.q += pitch_rate

    def tilt_deg(self):
        return math.degrees(max(abs(self.roll), abs(self.pitch)))


    def step(self, motors, dt):
        k = min(dt / self.motor_tau, 1.0)
        for i in range(4):
            self.motor_actual[i] += (motors[i] - self.motor_actual[i]) * k
        m = self.motor_actual
        tm = self.max_thrust_total / 4.0
        thrust = tm * sum(m)
        tau_roll = self.arm * tm * ((m[1] + m[2]) - (m[0] + m[3])) + self.ext_torque[0]
        tau_pitch = self.arm * tm * ((m[0] + m[2]) - (m[1] + m[3])) + self.ext_torque[1]


        cr, sr = math.cos(self.roll), math.sin(self.roll)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        nx, ny, nz = -sp * cr, -sr, cp * cr
        ax = (thrust * nx - self.drag * self.vx) / self.mass
        ay = (thrust * ny - self.drag * self.vy) / self.mass
        az = (thrust * nz - self.drag * self.vz) / self.mass - G

        on_ground = self.z <= 0.0
        if on_ground and az <= 0.0 and self.vz <= 0.0:

            self.vx = self.vy = self.vz = 0.0
            self.roll = self.pitch = self.p = self.q = 0.0
            self.z = 0.0
            return

        self.vx += ax * dt
        self.vy += ay * dt
        self.vz += az * dt
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt

        self.p += (tau_roll / self.inertia - self.ang_damp * self.p) * dt
        self.q += (tau_pitch / self.inertia - self.ang_damp * self.q) * dt
        self.roll += self.p * dt
        self.pitch += self.q * dt

        if self.z > 0.02:
            self._was_airborne = True
        if self.z <= 0.0:
            if self._was_airborne and self.vz < 0.0:
                self.impact_speed = -self.vz
                self._was_airborne = False
            self.z = 0.0
            self.vx = self.vy = self.vz = 0.0
            self.roll = self.pitch = self.p = self.q = 0.0

    def sensors(self):
        return [self.x, self.y, self.z, self.vx, self.vy, self.vz,
                self.roll, self.pitch, self.p, self.q]
