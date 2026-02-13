from controller import Robot
import numpy as np
import cv2


class Controller:
    """
    Minimal gate workflow (camera-only):

    1) SCANNING
       Rotate in a 3-phase sweep (left 60°, right 120°, left 60°) and segment colors.

    2) SETTLING
       Once green is detected, keep rotating in the *same direction* for N frames while
       collecting (area, cx, cy). This avoids locking on a single noisy frame and ensures
       heading_steps includes the extra rotation that happens during settling.

    3) DRIVE_TO_STANDOFF
       Drive straight forward until green area reaches AREA_NEAR (proxy for "close enough").

    4) ROTATE_TO_FORWARD
       Rotate back using heading_steps so the robot returns to its initial forward orientation.

    5) CROSS_GATE
       Drive forward until we confirm the gate has been crossed:
         - must have "seen the gate strongly" (ENTER_TH) at least once,
         - must move for MIN_STEPS,
         - then green must be "gone" (EXIT_TH) for LOST_K consecutive frames.

    6) Reset and repeat.
    """

    def __init__(self, robot: Robot):
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())

        # --- Devices ---
        self.left_motor = robot.getDevice("left wheel motor")
        self.right_motor = robot.getDevice("right wheel motor")
        self.camera = robot.getDevice("camera")

        # Motors: velocity control
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # Camera
        self.camera.enable(self.timestep)

        # --- Motion parameters ---
        self.max_velocity = self.left_motor.getMaxVelocity()
        self.scan_speed = 0.05 * self.max_velocity     # slow rotation for scanning
        self.fwd_speed = 0.80 * self.max_velocity      # straight drive speed

        # --- Scan sweep parameters (steps -> degrees; calibrate steps_for_60 once) ---
        self.steps_for_60 = 40
        self.steps_for_120 = 2 * self.steps_for_60

        # --- Adaptive detection for far gates ---
        self.DETECT_TH_BASE = 0.035      # your original threshold
        self.DETECT_TH = self.DETECT_TH_BASE

        self.FALLBACK_MIN = 0.010        # ignore tiny noise (tune)
        self.TOP_N = 10                  # use mean of top N g_area values
        self.TH_SCALE = 0.85             # new DETECT_TH = mean(topN) * TH_SCALE
        self.scan_area_buf = []          # stores g_area during one sweep

        self.AREA_NEAR = 0.60       # stand-off proxy (stop driving when reached)
        self.LOSE_TH = 0.012        # if green falls below this in DRIVE, we consider it lost

        # --- Cross thresholds (robust crossing) ---
        self.ENTER_TH = 0.90        # must see strong green at least once while crossing
        self.EXIT_TH = 0.01         # "green gone"
        self.LOST_K = 10            # consecutive frames of "gone"
        self.MIN_STEPS = 60         # minimum forward steps before declaring crossed

        # --- State machine ---
        self.state = "SCANNING"
        self.data = None

        # --- Scan phase state ---
        self.scan_phase = 1         # 1: left 60, 2: right 120, 3: left 60, 4: done
        self.scan_steps = 0

        # --- Heading bookkeeping (step-based) ---
        # heading_steps counts how many scan/settle rotation steps we have applied from the initial pose.
        # +1 means "we rotated left one step"; -1 means "rotated right one step".
        self.heading_steps = 0
        self.last_rot_dir = 0       # +1 left, -1 right (tracks scan rotation direction)
        self.settle_rot_dir = 0     # frozen rotation direction during settling

        # --- Settling (collect several centroid samples while continuing rotation) ---
        self.settling = False
        self.settle_buf = []
        self.settle_steps = 0
        self.settle_duration = 45

        # --- Cross bookkeeping ---
        self.cross_steps = 0
        self.cross_seen_gate = False
        self.cross_lost_count = 0

        # --- Debug (optional windows) ---
        self.debug_viz = True
        self.frame_count = 0

    # -------------------------
    # Low-level helpers
    # -------------------------
    def move(self, left_speed: float, right_speed: float) -> None:
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)

    def capture_frame(self) -> np.ndarray:
        """
        Webots camera returns BGRA bytes. We reshape to (H, W, 4) and keep BGR channels.
        """
        buf = self.camera.getImage()
        img = np.frombuffer(buf, dtype=np.uint8).reshape(
            (self.camera.getHeight(), self.camera.getWidth(), 4)
        )
        return img[:, :, 0:3]  # BGR

    def clean_mask(self, mask: np.ndarray) -> np.ndarray:
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def find_centroid(self, mask: np.ndarray):
        """
        Returns (cx, cy) or None if mask is empty.
        """
        m = cv2.moments(mask)
        if m["m00"] == 0:
            return None
        cx = int(m["m10"] / m["m00"])
        cy = int(m["m01"] / m["m00"])
        return (cx, cy)

    # -------------------------
    # Segmentation (HSV)
    # -------------------------
    def red_mask(self, bgr: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        # red wraps hue boundary
        lower1, upper1 = (0, 120, 90), (10, 255, 255)
        lower2, upper2 = (170, 120, 90), (179, 255, 255)
        m1 = cv2.inRange(hsv, lower1, upper1)
        m2 = cv2.inRange(hsv, lower2, upper2)
        return cv2.bitwise_or(m1, m2)

    def green_mask(self, bgr: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        lower, upper = (40, 20, 20), (95, 255, 255)
        return cv2.inRange(hsv, lower, upper)

    # -------------------------
    # Phase 1-2: Scan + Settle
    # -------------------------
    def scan(self):
        """
        SCANNING:
          - Run the 3-phase sweep and look for green.
          - If green area > DETECT_TH, enter SETTLING.

        SETTLING:
          - Keep rotating in the same direction that was active when settling started.
          - Collect (g_area, cx, cy) for settle_duration frames.
          - Return LOCKED with average centroid + average area.
        """
        frame = self.capture_frame()

        g_mask = self.clean_mask(self.green_mask(frame))
        r_mask = self.clean_mask(self.red_mask(frame))

        g_area = np.count_nonzero(g_mask) / g_mask.size
        r_area = np.count_nonzero(r_mask) / r_mask.size
        centroid = self.find_centroid(g_mask)

        # Debug visualization (optional)
        if self.debug_viz:
            try:
                cv2.imshow("camera", frame)
                cv2.imshow("green_mask", g_mask)
                cv2.imshow("red_mask", r_mask)
                cv2.waitKey(1)
            except Exception:
                pass

        self.frame_count += 1
        if self.frame_count % 5 == 0:
            print(f"[scan] phase={self.scan_phase} step={self.scan_steps} g_area={g_area:.4f} r_area={r_area:.4f}")

        # --- Trigger settling ---
        if (not self.settling) and (g_area > self.DETECT_TH):
            print("[scan] green detected -> SETTLING")
            self.settling = True
            self.settle_buf = []
            self.settle_steps = 0
            # Freeze direction so settling uses same rotation direction as scan
            self.settle_rot_dir = self.last_rot_dir

        # --- Settling phase ---
        if self.settling:
            # Keep rotating exactly like the scan was rotating when settling started
            # AND account for that rotation in heading_steps.
            if self.settle_rot_dir == +1:
                self.move(-self.scan_speed, self.scan_speed)  # rotate left
                self.heading_steps += 1
            elif self.settle_rot_dir == -1:
                self.move(self.scan_speed, -self.scan_speed)  # rotate right
                self.heading_steps -= 1
            else:
                self.move(0.0, 0.0)

            if centroid is not None:
                cx, cy = centroid
                self.settle_buf.append((g_area, cx, cy))

            self.settle_steps += 1

            # After collecting enough frames, lock the target centroid
            if self.settle_steps >= self.settle_duration:
                self.move(0.0, 0.0)
                self.settling = False

                areas = np.array([p[0] for p in self.settle_buf], dtype=np.float32)
                xs = np.array([p[1] for p in self.settle_buf], dtype=np.float32)
                ys = np.array([p[2] for p in self.settle_buf], dtype=np.float32)

                locked = (float(np.mean(areas)), float(np.mean(xs)), float(np.mean(ys)))
                print(f"[settle] LOCKED area={locked[0]:.4f} cx={locked[1]:.1f} cy={locked[2]:.1f}")
                return ("LOCKED", locked)

            return ("SETTLING", None)

        # --- Scan motion (3-phase sweep) ---
        # Phase 1: rotate LEFT 60°
        if self.scan_phase == 1:
            self.last_rot_dir = +1
            self.move(-self.scan_speed, self.scan_speed)
            self.scan_steps += 1
            self.heading_steps += 1
            if self.scan_steps >= self.steps_for_60:
                self.scan_phase = 2
                self.scan_steps = 0

        # Phase 2: rotate RIGHT 120°
        elif self.scan_phase == 2:
            self.last_rot_dir = -1
            self.move(self.scan_speed, -self.scan_speed)
            self.scan_steps += 1
            self.heading_steps -= 1
            if self.scan_steps >= self.steps_for_120:
                self.scan_phase = 3
                self.scan_steps = 0

        # Phase 3: rotate LEFT 60° back toward start direction
        elif self.scan_phase == 3:
            self.last_rot_dir = +1
            self.move(-self.scan_speed, self.scan_speed)
            self.scan_steps += 1
            self.heading_steps += 1
            if self.scan_steps >= self.steps_for_60:
                self.scan_phase = 4
                self.scan_steps = 0

        # Phase 4: scan complete (no lock)
        elif self.scan_phase == 4:
            self.move(0.0, 0.0)
            
            if (not self.settling) and (len(self.scan_area_buf) > 0):
                areas = np.array(self.scan_area_buf, dtype=np.float32)
                top_n = min(self.TOP_N, len(areas))
                top_vals = np.sort(areas)[-top_n:]
                fallback_score = np.mean(top_vals)

                print(f"[scan] sweep done: fallback_score(mean top{top_n})={fallback_score:.4f}, prev DETECT_TH={self.DETECT_TH:.4f}")

                if fallback_score >= self.FALLBACK_MIN:
                    # Lower/adjust detection threshold for the next sweep (but don't exceed base)
                    new_th = min(self.DETECT_TH_BASE, fallback_score * self.TH_SCALE)
                    self.DETECT_TH = max(self.FALLBACK_MIN, new_th)
                    print(f"[scan] updated DETECT_TH -> {self.DETECT_TH:.4f}")
                else:
                    # Not enough evidence: reset to base (or keep previous—your choice)
                    self.DETECT_TH = self.DETECT_TH_BASE
                    print(f"[scan] no meaningful green evidence -> reset DETECT_TH -> {self.DETECT_TH:.4f}")

                self.scan_area_buf = []
                self.scan_phase = 1
                self.scan_steps = 0

        self.scan_area_buf.append(g_area)
        return ("SCANNING", {"g_area": g_area, "r_area": r_area})

    # -------------------------
    # Phase 3: Drive to stand-off
    # -------------------------
    def drive_to_standoff(self) -> bool:
        """
        Drive forward (no steering here) until green occupies AREA_NEAR of the view.
        This is a camera-only proxy for "we are at the desired stand-off distance".
        """
        frame = self.capture_frame()
        g_mask = self.clean_mask(self.green_mask(frame))
        g_area = np.count_nonzero(g_mask) / g_mask.size
        centroid = self.find_centroid(g_mask)

        # If we lose the gate, stop and move on (caller decides what to do)
        if centroid is None or g_area < self.LOSE_TH:
            self.move(0.0, 0.0)
            print("[drive] lost gate -> stop")
            return True

        self.move(self.fwd_speed, self.fwd_speed)

        if g_area >= self.AREA_NEAR:
            self.move(0.0, 0.0)
            print(f"[drive] stand-off reached: g_area={g_area:.3f}")
            return True

        return False

    # -------------------------
    # Phase 4: Rotate back to forward
    # -------------------------
    def rotate_to_forward(self) -> bool:
        """
        Undo rotation until heading_steps == 0.
        Uses the SAME scan_speed magnitude and decrements heading_steps one step per timestep.
        """
        if self.heading_steps == 0:
            self.move(0.0, 0.0)
            return True

        if self.heading_steps > 0:
            # currently left of forward -> rotate RIGHT to reduce heading_steps
            self.move(self.scan_speed, -self.scan_speed)
            self.heading_steps -= 1
        else:
            # currently right of forward -> rotate LEFT
            self.move(-self.scan_speed, self.scan_speed)
            self.heading_steps += 1

        return False

    # -------------------------
    # Phase 5: Cross gate robustly
    # -------------------------
    def cross_gate(self) -> bool:
        """
        Cross confirmation:
          - Drive forward continuously.
          - Must "enter" gate view strongly at least once (ENTER_TH).
          - Must move for at least MIN_STEPS.
          - Then green must drop below EXIT_TH for LOST_K consecutive frames.
        """
        frame = self.capture_frame()
        g_mask = self.clean_mask(self.green_mask(frame))
        g_area = np.count_nonzero(g_mask) / g_mask.size

        # Drive forward
        self.move(self.fwd_speed, self.fwd_speed)
        self.cross_steps += 1

        # Mark that we actually reached the gate region
        if g_area > self.ENTER_TH:
            self.cross_seen_gate = True

        # After we've seen the gate, count "lost" frames
        if self.cross_seen_gate and g_area < self.EXIT_TH:
            self.cross_lost_count += 1
        else:
            self.cross_lost_count = 0

        # Only declare success if we moved enough AND gate disappeared consistently
        if (self.cross_steps >= self.MIN_STEPS
                and self.cross_seen_gate
                and self.cross_lost_count >= self.LOST_K):
            self.move(0.0, 0.0)
            print(f"[cross] crossed gate: steps={self.cross_steps}, lost_count={self.cross_lost_count}")
            self.cross_steps = 0
            self.cross_seen_gate = False
            self.cross_lost_count = 0
            return True

        print(f"[cross] step={self.cross_steps} g_area={g_area:.3f} seen={self.cross_seen_gate} lost={self.cross_lost_count}")

        return False

    # -------------------------
    # Main loop
    # -------------------------
    def run(self):
        while self.robot.step(self.timestep) != -1:

            if self.state == "SCANNING":
                self.state, self.data = self.scan()

            elif self.state == "SETTLING":
                self.state, self.data = self.scan()

            elif self.state == "LOCKED":
                # LOCKED means we have a stable green centroid estimate and we can proceed.
                print("[state] LOCKED -> DRIVE_TO_STANDOFF")
                self.state = "DRIVE_TO_STANDOFF"

            elif self.state == "DRIVE_TO_STANDOFF":
                # Drive until the green gate fills enough of the view.
                if self.drive_to_standoff():
                    print("[state] DRIVE_TO_STANDOFF -> ROTATE_TO_FORWARD")
                    self.state = "ROTATE_TO_FORWARD"

            elif self.state == "ROTATE_TO_FORWARD":
                # Return to the initial "forward" heading using heading_steps.
                if self.rotate_to_forward():
                    print("[state] ROTATE_TO_FORWARD -> CROSS_GATE")
                    self.state = "CROSS_GATE"

            elif self.state == "CROSS_GATE":
                # Drive through gate until it disappears for long enough.
                if self.cross_gate():
                    print("[state] CROSS_GATE -> reset and SCANNING")

                    # Reset scan sweep and heading bookkeeping for next gate
                    self.scan_phase = 1
                    self.scan_steps = 0
                    self.heading_steps = 0
                    self.cross_steps = 0
                    self.cross_seen_gate = False
                    self.cross_lost_count = 0
                    self.last_rot_dir = 0
                    self.settle_rot_dir = 0
                    self.settling = False
                    self.settle_buf = []
                    self.settle_steps = 0
                    self.scan_area_buf = []


                    self.state = "SCANNING"
                    self.DETECT_TH = self.DETECT_TH_BASE

RobotController = Controller(Robot())
RobotController.run()
