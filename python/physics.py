G = 9.81


class QuadPhysics:
    def __init__(self, mass=1.5, thrust_to_weight=2.0, motor_tau=0.05, drag=0.10):
        self.mass = mass
        self.max_thrust_total = thrust_to_weight * mass * G
        self.motor_tau = motor_tau
        self.drag = drag
        self.reset()

    def reset(self):
        self.height = 0.0
        self.vz = 0.0
        self.motor_actual = [0.0] * 4
        self.impact_speed = None
        self._was_airborne = False

    def step(self, motors, dt):
        k = min(dt / self.motor_tau, 1.0)
        for i in range(4):
            self.motor_actual[i] += (motors[i] - self.motor_actual[i]) * k

        thrust = sum(self.motor_actual) / 4.0 * self.max_thrust_total
        force = thrust - self.mass * G - self.drag * self.vz
        acc = force / self.mass

        on_ground = self.height <= 0.0
        if on_ground and acc <= 0.0 and self.vz <= 0.0:
            self.vz = 0.0
            self.height = 0.0
            return

        self.vz += acc * dt
        self.height += self.vz * dt
        if self.height > 0.02:
            self._was_airborne = True
        if self.height <= 0.0:
            if self._was_airborne and self.vz < 0.0:
                self.impact_speed = -self.vz
                self._was_airborne = False
            self.height = 0.0
            self.vz = 0.0

    def sensors(self):
        return self.height, self.vz