from controller import Robot
import numpy as np 
import cv2

class Controller:
    def __init__(self, robot):
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())
        self.k = 0.002 # Proportional gain for turning, adjust as needed
        self.state = "SCANNING"   # SEARCH → ALIGN → CROSS
        self.cooldown_steps = 0  # Add cooldown counter
        self.cooldown_duration = 100  # ~2 seconds at 50ms timestep, adjust as needed
        
        # Settling mechanism variables for stable centroid/crossing detection
        self.settling = False
        self.settle_buf = []
        self.settle_steps = 0
        self.settle_duration = 12     # frames to collect
        self.settle_min_hits = 7      # required valid centroid frames
        self.settle_std_th = 18       # px std threshold
        self.lose_k = 4     

        # Initialize devices here (e.g., motors, sensors)
        self.left_motor = robot.getDevice("left wheel motor")
        self.right_motor = robot.getDevice("right wheel motor")
        self.camera = robot.getDevice("camera")
        self.frame_count =  0

        # Enable devices
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        self.camera.enable(self.timestep)

        # Get max velocity for later use
        self.max_velocity = self.left_motor.getMaxVelocity()
        self.base_speed = 0.5 * self.max_velocity
        self.scan_speed = 0.05 * self.max_velocity

        self.scan_phase = 1         # 0 = forward sweep, 1 = return sweep
        self.scan_steps =  0
        self.steps_for_60 = 30 # calibrate this
        self.steps_for_120 = 2 * self.steps_for_60
        self.scanning_done = False

    def move(self, left_speed, right_speed):
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)

    def capture_frame(self):
        buffer_image = self.camera.getImage()
        img = np.frombuffer(buffer_image, dtype=np.uint8).reshape((self.camera.getHeight(), self.camera.getWidth(), 4)) 
        return img[:,:,0:3]

    def red_mask(self, img):
        # img must match your capture_frame format:
        # - if capture_frame returns BGR -> use BGR2HSV
        # - if capture_frame returns RGB -> use RGB2HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Red wraps around hue: [0..10] and [170..179]
        lower1 = (0, 120,  90)
        upper1 = (10, 255, 255)

        lower2 = (170, 120,  90)
        upper2 = (179, 255, 255)

        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)

        return cv2.bitwise_or(mask1, mask2)


    def green_mask(self, img):
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_green = (40, 20, 20)   # Tighter saturation & value
        upper_green = (95, 255, 255)  # Narrower hue range
        mask = cv2.inRange(hsv_img, lower_green, upper_green)
        return mask

    def clean_mask(self, mask):
        kernel = np.ones((3, 3), np.uint8)
        cleaned_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        cleaned_mask = cv2.morphologyEx(cleaned_mask, cv2.MORPH_CLOSE, kernel)
        return cleaned_mask

    def find_centroid(self, mask):
        moments = cv2.moments(mask)
        print(f"m00: {moments['m00']}, mask sum: {np.sum(mask)}")  # DEBUG
        if moments["m00"] == 0:
            return None
        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
        print(f"Centroid found at: ({cx}, {cy})")
        return (cx, cy)

    def EMA(self, current, previous, alpha=0.5):
        if previous is None:
            return current
        return alpha * current + (1 - alpha) * previous

    def mask_area_ratio(self, mask):
        return np.sum(mask > 0) / mask.size

    def detect_colors(self, frame):
        green = self.clean_mask(self.green_mask(frame))
        red   = self.clean_mask(self.red_mask(frame))

        cv2.imshow("Green Mask", green)  # DEBUG
        cv2.imshow("Red Mask", red)      # DEBUG

        g_area = self.mask_area_ratio(green)
        r_area = self.mask_area_ratio(red)

        g_cent = self.find_centroid(green)
        r_cent = self.find_centroid(red)

        return {
            "green": {"mask": green, "area": g_area, "centroid": g_cent},
            "red":   {"mask": red,   "area": r_area, "centroid": r_cent},
        }

    def pick_target_color(self, det, detect_th=0.01, margin=1.3):
        g, r = det["green"], det["red"]

        g_ok = g["area"] > detect_th and g["centroid"] is not None
        r_ok = r["area"] > detect_th and r["centroid"] is not None

        if g_ok and (not r_ok or g["area"] > margin * r["area"]):
            return "green"
        if r_ok and (not g_ok or r["area"] > margin * g["area"]):
            return "red"
        return None

    def largest_blob_centroid(self, mask, min_area_px=200):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < min_area_px:
            return None

        M = cv2.moments(c)
        if M["m00"] == 0:
            return None
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return (cx, cy)

    def scan(self):

        # ---- capture + segment ----
        frame = self.capture_frame()

        g_mask = self.clean_mask(self.green_mask(frame))
        r_mask = self.clean_mask(self.red_mask(frame))

        g_area = np.count_nonzero(g_mask) / g_mask.size
        r_area = np.count_nonzero(r_mask) / r_mask.size

        DETECT_TH = 0.01

        # ---- DEBUG VIS ----
        # NOTE: imshow requires waitKey to refresh
        try:
            cv2.imshow("camera", frame)
            cv2.imshow("green_mask", g_mask)
            cv2.imshow("red_mask", r_mask)
            cv2.waitKey(1)
        except Exception as e:
            # If windows don't show, prints still help
            pass

        # Print occasionally so it doesn't spam too hard
        if not hasattr(self, "frame_count"):
            self.frame_count = 0
        self.frame_count += 1
        if self.frame_count % 10 == 0:
            print(f"[scan] phase={self.scan_phase} step={self.scan_steps} g_area={g_area:.4f} r_area={r_area:.4f}")

        if g_area > DETECT_TH: 
            print("[scan] green detected!")
            self.settling = True
            centroid = self.find_centroid(g_mask)
            self.settle_buf.append((g_area, centroid))
            self.settle_steps += 1

            if len(self.settle_steps) > self.settle_duration:
                self.settling = False
                self.settle_steps = 0

            if len(self.settle_buf) > self.settle_min_hits:
                self.move(0.0, 0.0)

        # ---- scan motion (phases) ----
        if self.scan_phase == 1:
            # rotate LEFT 90 -> 150 (60°)
            self.move(-self.scan_speed, self.scan_speed)
            self.scan_steps += 1
            if self.scan_steps >= self.steps_for_60:
                self.scan_phase = 2
                self.scan_steps = 0

        elif self.scan_phase == 2:
            # rotate RIGHT 150 -> 30 (120°)
            self.move(self.scan_speed, -self.scan_speed)
            self.scan_steps += 1
            if self.scan_steps >= self.steps_for_120:
                self.scan_phase = 3
                self.scan_steps = 0

        elif self.scan_phase == 3:
            # optional return 30 -> 90 (60°)
            self.move(-self.scan_speed, self.scan_speed)
            self.scan_steps += 1
            if self.scan_steps >= self.steps_for_60:
                self.scan_phase = 4
                self.scan_steps = 0

        elif self.scan_phase == 4:
            self.move(0.0, 0.0)
            return ("DONE", {"g_area": g_area, "r_area": r_area})

        return ("SCANNING", {"g_area": g_area, "r_area": r_area})


    def run(self):
        while self.robot.step(self.timestep) != -1:
            state, data = self.scan()

            if state == "DONE":
                print("[DONE] scan complete — restarting")
                self.scan_phase = 1
                self.scan_steps = 0
            

RobotController = Controller(Robot())
RobotController.run()