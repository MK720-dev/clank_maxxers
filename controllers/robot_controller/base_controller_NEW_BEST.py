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
TURN_SPEED = 0.6 * FORWARD_SPEED
SCAN_SPEED = 0.5 * FORWARD_SPEED

# -----------------------------
# Camera
# -----------------------------
camera = robot.getDevice('camera')
camera.enable(timestep)

# -----------------------------
# Run Timer
# -----------------------------
run_start_time = robot.getTime()
wall_counter = 0

def elapsed_run_time():
    return robot.getTime() - run_start_time

# -----------------------------
# STATES
# -----------------------------
SCAN = 0
TRACK = 1
STOP = 2
FORWARD = 3

state = SCAN

# -----------------------------
# Phased SCAN (bounded sweep + recenter)
# NOTE: "steps" are controller iterations, not degrees.
# Tune steps_for_60 to visually match ~60° in your world.
# -----------------------------
steps_for_60 = 8
steps_for_120 = 2 * steps_for_60
scan_phase = 1
scan_steps = 0
scan_balance = 0
recentering = False
recenter_steps = 0

# -----------------------------
# STRICT GREEN DETECTION
# -----------------------------
def classify_hsv(h, s, v):
    if 0.27 < h < 0.40 and s > 0.5 and v > 0.3:
        return "GREEN"
    return "NONE"


# -----------------------------
# Green Coverage
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

            r, g, b = r / 255.0, g / 255.0, b / 255.0
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

    left_x0, left_x1 = int(width * 0.05), int(width * 0.35)
    center_x0, center_x1 = int(width * 0.35), int(width * 0.65)
    right_x0, right_x1 = int(width * 0.65), int(width * 0.95)

    step = 2

    def region_green(x0, x1):
        green_count = 0
        total = 0
        for y in range(y0, y1, step):
            for x in range(x0, x1, step):
                r = camera.imageGetRed(image, width, x, y)
                g = camera.imageGetGreen(image, width, x, y)
                b = camera.imageGetBlue(image, width, x, y)

                r, g, b = r / 255.0, g / 255.0, b / 255.0
                h, s, v = colorsys.rgb_to_hsv(r, g, b)

                if classify_hsv(h, s, v) == "GREEN":
                    green_count += 1
                total += 1

        if total == 0:
            return 0
        return green_count / total

    left_ratio = region_green(left_x0, left_x1)
    center_ratio = region_green(center_x0, center_x1)
    right_ratio = region_green(right_x0, right_x1)

    min_detect_threshold = 0.03

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
# Helper: in-place turn
# -----------------------------
def set_turn(direction, speed):
    # direction +1 = CCW, -1 = CW
    left_motor.setVelocity(-direction * speed)
    right_motor.setVelocity(direction * speed)


# -----------------------------
# Main Loop
# -----------------------------
while robot.step(timestep) != -1:
    green_ratio = green_coverage_ratio()
    green_pos = detect_green_position()

    # =============================
    # STATE: SCAN (PHASED SWEEP)
    # =============================
    if state == SCAN:
        if green_pos is not None:
            print("Green detected. Switching to TRACK.")
            state = TRACK
            # Reset scan machine for the next time we re-enter SCAN
            scan_phase = 1
            scan_steps = 0
            scan_balance = 0
            recentering = False
            recenter_steps = 0
        else:
            if recentering:
                # Return to original forward heading estimate after full sweep
                if recenter_steps > 0:
                    recenter_dir = -1 if scan_balance > 0 else 1
                    set_turn(recenter_dir, SCAN_SPEED)
                    recenter_steps -= 1
                else:
                    recentering = False
                    scan_phase = 1
                    scan_steps = 0
                    scan_balance = 0
            else:
                # 3-phase bounded sweep: +60, -120, +60
                if scan_phase == 1:
                    set_turn(1, SCAN_SPEED)
                    scan_steps += 1
                    scan_balance += 1
                    if scan_steps >= steps_for_60:
                        scan_phase = 2
                        scan_steps = 0

                elif scan_phase == 2:
                    set_turn(-1, SCAN_SPEED)
                    scan_steps += 1
                    scan_balance -= 1
                    if scan_steps >= steps_for_120:
                        scan_phase = 3
                        scan_steps = 0

                else:  # scan_phase == 3
                    set_turn(1, SCAN_SPEED)
                    scan_steps += 1
                    scan_balance += 1
                    if scan_steps >= steps_for_60:
                        # Sweep finished: recenter back to forward if needed
                        scan_phase = 1
                        scan_steps = 0
                        recenter_steps = abs(scan_balance)
                        recentering = recenter_steps > 0


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
            # If green is temporarily lost during TRACK, keep original recovery turn.
            set_turn(1, TURN_SPEED)

    # =============================
    # STATE: STOP
    # =============================
    elif state == STOP:

        left_motor.setVelocity(0)
        right_motor.setVelocity(0)

        wall_counter += 1
        print(
            f"FULL GREEN WALL REACHED (#{wall_counter}) — elapsed: {elapsed_run_time():.2f}s — MOVING FORWARD"
        )

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