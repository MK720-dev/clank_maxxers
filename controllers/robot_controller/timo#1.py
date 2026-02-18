import math
from controller import Robot

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def deadband(x, db):
    if abs(x) < db:
        return 0.0
    return x


class EPuckController:
    def __init__(self, robot):
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())

        self.left_motor = robot.getDevice("left wheel motor")
        self.right_motor = robot.getDevice("right wheel motor")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        self.camera = robot.getDevice("camera")
        self.camera.enable(self.timestep)

        self.ps = []
        for i in range(8):
            s = robot.getDevice(f"ps{i}")
            s.enable(self.timestep)
            self.ps.append(s)

        self.max_speed = 6.28

        self.state = "SEEK_GREEN"
        self.state_enter_time = self.robot.getTime()

        # Filters
        self.alpha = 0.25
        self.err_f = 0.0
        self.green_frac_f = 0.0

        # Speeds and gains
        self.scan_speed = 1.3
        self.turn_speed = 1.7
        self.forward_speed = 3.2
        self.kp = 1.6
        self.err_deadband = 0.04

        # When green is centered enough, move forward
        self.center_tol = 0.18

        # Pass detection
        self.passed_green = False
        self.pass_hold_time = 0.35

        # IR safety
        self.front_raw_turn = 120.0
        self.front_raw_stop = 160.0
        self.front_raw_crit = 260.0

        # Debug
        self.last_debug_time = 0.0
        self.debug_period = 0.2

    def now(self):
        return self.robot.getTime()

    def time_in_state(self):
        return self.now() - self.state_enter_time

    def enter_state(self, s):
        self.state = s
        self.state_enter_time = self.now()

    def debug_print(self, msg):
        t = self.now()
        if t - self.last_debug_time >= self.debug_period:
            self.last_debug_time = t
            print(msg)

    def set_wheels(self, left, right):
        left = clamp(left, -self.max_speed, self.max_speed)
        right = clamp(right, -self.max_speed, self.max_speed)
        self.left_motor.setVelocity(left)
        self.right_motor.setVelocity(right)
        return left, right

    def turn_in_place(self, speed, direction):
        s = abs(speed)
        if direction == "left":
            return self.set_wheels(-s, s)
        return self.set_wheels(s, -s)

    def arc_turn(self, base_speed, turn_strength):
        turn_strength = clamp(turn_strength, -1.0, 1.0)
        left = base_speed * (1.0 - turn_strength)
        right = base_speed * (1.0 + turn_strength)
        return self.set_wheels(left, right)

    def stop(self):
        return self.set_wheels(0.0, 0.0)

    def read_camera_frame(self):
        try:
            import numpy as np
        except Exception:
            return None

        w = self.camera.getWidth()
        h = self.camera.getHeight()
        img = self.camera.getImage()
        if img is None:
            return None

        arr = np.frombuffer(img, dtype=np.uint8).reshape((h, w, 4))
        bgr = arr[:, :, :3]
        rgb = bgr[:, :, ::-1].copy()
        return rgb

    def read_ir_raw_front(self):
        raw = [float(s.getValue()) for s in self.ps]
        return max(raw[0], raw[7])

    def find_any_green_centroid(self, frame):
        """
        Very permissive:
        If any green-ish pixels exist, compute centroid across whole frame.

        Returns found, error, green_frac
        """
        if frame is None:
            return False, 0.0, 0.0

        try:
            import numpy as np
        except Exception:
            return False, 0.0, 0.0

        H, W = frame.shape[0], frame.shape[1]
        img = frame.astype(np.int16)

        r = img[:, :, 0]
        g = img[:, :, 1]
        b = img[:, :, 2]

        # More permissive green mask
        mask = (g > 45) & ((g - r) > 10) & ((g - b) > 10)

        count = int(mask.sum())
        frac = count / float(H * W)

        # "Any amount" but avoid single pixel noise
        if count < 25:
            return False, 0.0, frac

        ys, xs = mask.nonzero()
        cx = float(xs.mean())

        error = (cx - (W / 2.0)) / (W / 2.0)
        error = clamp(error, -1.0, 1.0)

        return True, error, frac

    def step(self):
        frame = self.read_camera_frame()
        raw_front = self.read_ir_raw_front()

        found, err, green_frac = self.find_any_green_centroid(frame)

        # Filter
        self.err_f = (1.0 - self.alpha) * self.err_f + self.alpha * err
        self.green_frac_f = (1.0 - self.alpha) * self.green_frac_f + self.alpha * green_frac

        e = deadband(self.err_f, self.err_deadband)

        # IR safety overrides
        if raw_front > self.front_raw_crit:
            self.enter_state("RECOVER")

        left_cmd = 0.0
        right_cmd = 0.0

        if self.state == "SEEK_GREEN":
            if found:
                self.enter_state("ALIGN_TO_GREEN")
            else:
                # Scan until any green appears
                direction = "right" if math.sin(0.7 * self.now()) > 0 else "left"
                left_cmd, right_cmd = self.turn_in_place(self.scan_speed, direction)

        elif self.state == "ALIGN_TO_GREEN":
            if not found and self.green_frac_f < 0.0005:
                # Lost it, go back to seek
                self.enter_state("SEEK_GREEN")
                left_cmd, right_cmd = self.stop()
            else:
                # Rotate toward green strongly first
                turn = clamp(self.kp * e, -1.0, 1.0)

                # If green is far off center, rotate in place
                if abs(self.err_f) > self.center_tol:
                    direction = "right" if self.err_f > 0 else "left"
                    left_cmd, right_cmd = self.turn_in_place(self.turn_speed, direction)
                else:
                    # Green centered enough, move forward while keeping it centered
                    if raw_front > self.front_raw_stop:
                        # If close to wall, do not move forward
                        left_cmd, right_cmd = self.turn_in_place(1.6, "left")
                    else:
                        left_cmd, right_cmd = self.arc_turn(self.forward_speed, turn)

                    # If we saw a decent amount of green and then it drops, assume we crossed it
                    if self.green_frac_f > 0.02:
                        self.passed_green = True

                    if self.passed_green and self.green_frac_f < 0.003:
                        self.enter_state("POST_PASS")
                        self.passed_green = False

        elif self.state == "POST_PASS":
            # Drive forward a little to fully clear the strip, then reset
            t = self.time_in_state()
            if t < self.pass_hold_time:
                left_cmd, right_cmd = self.set_wheels(3.0, 3.0)
            else:
                self.enter_state("SEEK_GREEN")

        elif self.state == "RECOVER":
            t = self.time_in_state()
            if t < 0.45:
                left_cmd, right_cmd = self.set_wheels(-2.4, -2.4)
            elif t < 1.15:
                left_cmd, right_cmd = self.turn_in_place(2.2, "left")
            else:
                self.enter_state("SEEK_GREEN")

        else:
            self.enter_state("SEEK_GREEN")
            left_cmd, right_cmd = self.stop()

        self.debug_print(
            f"state={self.state:12s} found={found} greenFrac={self.green_frac_f:.4f} e={self.err_f:+.2f} rawF={raw_front:.1f} wheels=({left_cmd:+.2f},{right_cmd:+.2f})"
        )

    def run(self):
        while self.robot.step(self.timestep) != -1:
            self.step()


def main():
    robot = Robot()
    ctrl = EPuckController(robot)
    ctrl.run()

if __name__ == "__main__":
    main()
