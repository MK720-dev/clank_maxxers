
from controller import Robot, Camera
import colorsys

# -----------------------------
# Robot Setup
# -----------------------------
robot = Robot()
timestep = 64

# -----------------------------
# Motors
# -----------------------------
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

FORWARD_SPEED = 6.28
TURN_SPEED = 0.4 * FORWARD_SPEED
SCAN_SPEED = 0.3 * FORWARD_SPEED

# -----------------------------
# Camera
# -----------------------------
camera = robot.getDevice('camera')
camera.enable(timestep)

# -----------------------------
# STATES
# -----------------------------
SCAN = 0
TRACK = 1
STOP = 2
FORWARD = 3

state = SCAN

# -----------------------------
# STRICT GREEN DETECTION
# -----------------------------
def classify_hsv(h, s, v):
    if 0.27 < h < 0.40 and s > 0.5 and v > 0.3:
        return "GREEN"
    return "NONE"

# -----------------------------
#            Green Coverage
# -----------------------------
def green_coverage_ratio():
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()

    step = 3
    green_pixels = 0
    total_pixels = 0

    for y in range(0, height, step):
        for x in range(0, width, step):
            r = camera.imageGetRed(image, width, x, y)
            g = camera.imageGetGreen(image, width, x, y)
            b = camera.imageGetBlue(image, width, x, y)

            r, g, b = r/255.0, g/255.0, b/255.0
            h, s, v = colorsys.rgb_to_hsv(r, g, b)

            if classify_hsv(h, s, v) == "GREEN":
                green_pixels += 1

            total_pixels += 1

    if total_pixels == 0:
        return 0

    return green_pixels / total_pixels

# -----------------------------
# Detect Green Position
# -----------------------------
def detect_green_position():
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()

    # Scan almost full vertical area
    y0 = int(height * 0.05)
    y1 = int(height * 0.75)

    left_x0, left_x1     = int(width * 0.05), int(width * 0.35)
    center_x0, center_x1 = int(width * 0.35), int(width * 0.65)
    right_x0, right_x1   = int(width * 0.65), int(width * 0.95)

    step = 2  

    def region_green(x0, x1):
        green_count = 0
        total = 0
        for y in range(y0, y1, step):
            for x in range(x0, x1, step):
                r = camera.imageGetRed(image, width, x, y)
                g = camera.imageGetGreen(image, width, x, y)
                b = camera.imageGetBlue(image, width, x, y)

                r, g, b = r/255.0, g/255.0, b/255.0
                h, s, v = colorsys.rgb_to_hsv(r, g, b)

                if classify_hsv(h, s, v) == "GREEN":
                    green_count += 1
                total += 1

        if total == 0:
            return 0
        return green_count / total

    left_ratio   = region_green(left_x0, left_x1)
    center_ratio = region_green(center_x0, center_x1)
    right_ratio  = region_green(right_x0, right_x1)

    min_detect_threshold = 0.01

    max_ratio = max(left_ratio, center_ratio, right_ratio)

    if max_ratio < min_detect_threshold:
        return None

    if max_ratio == center_ratio:
        return "CENTER"
    elif max_ratio == left_ratio:
        return "LEFT"
    else:
        return "RIGHT"

# -----------------------------
# Stop Conditions
# -----------------------------
GREEN_STOP_THRESHOLD = 0.92
STOP_CONFIRMATION = 8
green_stop_counter = 0

# -----------------------------
# Forward State Timing
# -----------------------------
FORWARD_DURATION_MS = 2500
forward_counter = 0

# -----------------------------
# Main Loop
# -----------------------------
while robot.step(timestep) != -1:

    green_ratio = green_coverage_ratio()
    green_pos = detect_green_position()

    # =============================
    # STATE: SCAN
    # =============================
    if state == SCAN:

        left_motor.setVelocity(-SCAN_SPEED)
        right_motor.setVelocity(SCAN_SPEED)

        if green_pos is not None:
            print("Green detected. Im Switching to TRACK.")
            state = TRACK

    # =============================
    # STATE: TRACK
    # =============================
    elif state == TRACK:

        # Stop condition
        if green_ratio > GREEN_STOP_THRESHOLD:
            green_stop_counter += 1
            if green_stop_counter >= STOP_CONFIRMATION:
                state = STOP
                continue
        else:
            green_stop_counter = 0

        # Steering logic
        if green_pos == "LEFT":
            left_motor.setVelocity(0.3 * FORWARD_SPEED)
            right_motor.setVelocity(0.6 * FORWARD_SPEED)

        elif green_pos == "RIGHT":
            left_motor.setVelocity(0.6 * FORWARD_SPEED)
            right_motor.setVelocity(0.3 * FORWARD_SPEED)

        elif green_pos == "CENTER":
            left_motor.setVelocity(0.6 * FORWARD_SPEED)
            right_motor.setVelocity(0.6 * FORWARD_SPEED)

        else:
            left_motor.setVelocity(-TURN_SPEED)
            right_motor.setVelocity(TURN_SPEED)

    # =============================
    # STATE: STOP
    # =============================
    elif state == STOP:

        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        print("FULL GREEN WALL REACHED — MOVING FORWARD")

        green_stop_counter = 0
        forward_counter = 0

        state = FORWARD

    # =============================
    # STATE: FORWARD
    # =============================
    elif state == FORWARD:

        left_motor.setVelocity(0.6 * FORWARD_SPEED)
        right_motor.setVelocity(0.6 * FORWARD_SPEED)

        forward_counter += timestep

        if forward_counter >= FORWARD_DURATION_MS:
            print("Forward complete — returning to SCAN")
            state = SCAN
