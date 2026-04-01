# =============================================================================
# defend_magenta_goal.py
# Controller for a robot DEFENDING THE MAGENTA GOAL (right side of field)
#
# Field orientation (top-down view):
#   CYAN goal  <---  [-1.1]  ...field...  [+1.1]  ---> MAGENTA goal
#
# This robot:
#   - Starts at approximately (+0.62, 0), facing -x (toward cyan/opponent)
#   - Own goal:      MAGENTA  at x ≈ +1.15  (BEHIND the robot)
#   - Opponent goal: CYAN     at x ≈ -1.15  (AHEAD of the robot)
#   - Camera faces forward (-x) → sees opponent goal and ball when facing right
#   - ds_back sensor points toward own magenta goal
#
# Strategy:
#   ATTACK  → ball visible: steer toward ball centroid, full speed ahead
#   SCAN    → ball just lost: spin in place to re-acquire ball
#   RETREAT → ball missing too long: back up toward own goal
#   GOALIE  → at own goal: slow spin scan, ready to react to ball
#
# NOTE: This file is the mirror counterpart of defend_cyan_goal.py.
#   Both robots start facing their opponent, so camera-based ball tracking
#   and ds_back-based goal detection work identically — only the goal colors
#   being detected differ.
# =============================================================================

from controller import Robot, Camera, Lidar
import math

TIME_STEP = 64
MAX_SPEED = 10.0

# ─────────────────────── ROBOT INIT ───────────────────────
robot = Robot()

camera = robot.getDevice('camera')
camera.enable(TIME_STEP)
cam_width  = camera.getWidth()
cam_height = camera.getHeight()

lidar = robot.getDevice('LDS-01')
lidar.enable(TIME_STEP)

# Wheel order: [0]=front_left, [1]=front_right, [2]=back_left, [3]=back_right
wheels = []
for name in ['front_left_wheel', 'front_right_wheel',
             'back_left_wheel',  'back_right_wheel']:
    w = robot.getDevice(name)
    w.setPosition(float('inf'))
    w.setVelocity(0.0)
    wheels.append(w)

distance_sensors = {}
for name in ['ds_front_left', 'ds_front_right', 'ds_back', 'ds_left', 'ds_right']:
    ds = robot.getDevice(name)
    ds.enable(TIME_STEP)
    distance_sensors[name] = ds

# ─────────────────────── HELPERS ───────────────────────
def set_speed(left, right):
    """Drive left and right wheel pairs together."""
    wheels[0].setVelocity(left)   # front left
    wheels[2].setVelocity(left)   # back  left
    wheels[1].setVelocity(right)  # front right
    wheels[3].setVelocity(right)  # back  right

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def get_ds(name):
    return distance_sensors[name].getValue()

# ─────────────────────── COLOR DETECTION ───────────────────────
def is_yellow(r, g, b):
    """Detect the ball (yellow sphere)."""
    if r > 140 and g > 140 and b < 100:
        return True
    if r > 160 and g > 160 and b < 130 and r > b + 50 and g > b + 50:
        return True
    if r > 100 and g > 100 and b < 60 and r > b + 60 and g > b + 60:
        return True
    return False

def is_cyan(r, g, b):
    """Detect the CYAN goal walls."""
    return g > 150 and b > 150 and r < 80

def is_magenta(r, g, b):
    """Detect the MAGENTA goal walls."""
    return r > 150 and b > 150 and g < 80

# Role assignments for THIS controller (defending magenta)
def is_own_goal(r, g, b):
    """Own goal is MAGENTA — seeing this means we're facing the wrong way."""
    return is_magenta(r, g, b)

def is_opponent_goal(r, g, b):
    """Opponent goal is CYAN — camera sees this when facing correctly."""
    return is_cyan(r, g, b)

# ─────────────────────── TUNING CONSTANTS ───────────────────────
WALL_FRONT_THRESH = 0.15   # (m) front sensor threshold → stop and escape
WALL_SIDE_THRESH  = 0.11   # (m) side sensor threshold → gentle lateral nudge
GOAL_BACK_THRESH  = 0.30   # (m) ds_back reading → robot has reached own goal area

RETREAT_SPEED = MAX_SPEED * 0.45
SCAN_SPEED    = MAX_SPEED * 0.42
GOALIE_SPEED  = MAX_SPEED * 0.30

BALL_PX_MIN    = 4    # min yellow pixels to confirm ball is visible
OWN_GOAL_PX    = 20   # min own-goal pixels to confirm we're facing it

SCAN_DURATION   = 28  # steps (~1.8 s) spinning before we give up and retreat
GOALIE_DURATION = 50  # steps (~3.2 s) in goalie mode before another scan attempt

# ─────────────────────── STATE MACHINE ───────────────────────
# States: 'ATTACK' | 'SCAN' | 'RETREAT' | 'GOALIE'
state        = 'ATTACK'
scan_timer   = 0
goalie_timer = 0

# ─────────────────────── MAIN LOOP ───────────────────────
while robot.step(TIME_STEP) != -1:

    # ── 1. READ DISTANCE SENSORS ──
    fl   = get_ds('ds_front_left')
    fr   = get_ds('ds_front_right')
    ds_b = get_ds('ds_back')
    ds_l = get_ds('ds_left')
    ds_r = get_ds('ds_right')

    # ── 2. SCAN CAMERA FOR COLORS ──
    image = camera.getImage()

    ball_x_sum     = 0
    ball_count     = 0
    own_goal_count = 0

    for py in range(cam_height):
        for px in range(cam_width):
            r = Camera.imageGetRed(image,   cam_width, px, py)
            g = Camera.imageGetGreen(image, cam_width, px, py)
            b = Camera.imageGetBlue(image,  cam_width, px, py)

            if is_yellow(r, g, b):
                ball_x_sum += px
                ball_count += 1
            elif is_own_goal(r, g, b):
                own_goal_count += 1
            # (opponent goal color not needed for decisions, but could log here)

    ball_visible     = ball_count     >= BALL_PX_MIN
    own_goal_visible = own_goal_count >= OWN_GOAL_PX

    # ── 3. STATE TRANSITIONS ──
    if ball_visible:
        # Ball in view → always switch to attack immediately
        state        = 'ATTACK'
        scan_timer   = 0
        goalie_timer = 0
    else:
        if state == 'ATTACK':
            # Just lost the ball → begin search spin
            state      = 'SCAN'
            scan_timer = 0

        elif state == 'SCAN':
            scan_timer += 1
            if scan_timer > SCAN_DURATION:
                # Couldn't find ball by spinning → retreat to own goal
                state = 'RETREAT'

        elif state == 'RETREAT':
            # Check if we've reached the goal area: back sensor close,
            # OR we can see our own goal in camera (approached from angle)
            if ds_b < GOAL_BACK_THRESH or own_goal_visible:
                state        = 'GOALIE'
                goalie_timer = 0

        elif state == 'GOALIE':
            goalie_timer += 1
            if goalie_timer > GOALIE_DURATION:
                # Haven't seen ball for a while → try a fresh spin search
                state      = 'SCAN'
                scan_timer = 0

    # ── 4. FRONT OBSTACLE OVERRIDE ──
    # This fires before any state behavior to prevent wall collisions.
    if fl < WALL_FRONT_THRESH or fr < WALL_FRONT_THRESH:
        # Obstacle dead ahead — back up and rotate toward the more open side
        if ds_l > ds_r:
            # More space on left: pivot left (counterclockwise from above)
            set_speed(-MAX_SPEED * 0.45, MAX_SPEED * 0.20)
        else:
            # More space on right: pivot right (clockwise from above)
            set_speed(MAX_SPEED * 0.20, -MAX_SPEED * 0.45)
        continue   # skip normal state logic this step

    # ── 5. STATE BEHAVIORS ──
    left_speed  = 0.0
    right_speed = 0.0

    if state == 'ATTACK':
        # ── ATTACK: chase ball at full speed ──
        # Compute steering from ball centroid in camera image.
        # Both robots face their opponent, so camera steering logic is identical:
        # error > 0 → ball is right of center → steer right (left wheel faster).
        # error < 0 → ball is left of center  → steer left  (right wheel faster).
        ball_cx    = ball_x_sum / ball_count
        error      = (ball_cx - cam_width / 2.0) / (cam_width / 2.0)  # [-1, +1]
        steer      = error * MAX_SPEED * 0.85
        left_speed  = clamp(MAX_SPEED + steer, MAX_SPEED * 0.10, MAX_SPEED)
        right_speed = clamp(MAX_SPEED - steer, MAX_SPEED * 0.10, MAX_SPEED)

        # Gentle side-wall nudge to avoid getting glued to the boundary
        if ds_l < WALL_SIDE_THRESH:
            right_speed = clamp(right_speed + MAX_SPEED * 0.20, 0, MAX_SPEED)
        if ds_r < WALL_SIDE_THRESH:
            left_speed  = clamp(left_speed  + MAX_SPEED * 0.20, 0, MAX_SPEED)

    elif state == 'SCAN':
        # ── SCAN: spin clockwise (from above) to sweep for ball ──
        # At ~0.42 * MAX_SPEED the robot completes ~360° in ≈ SCAN_DURATION steps.
        left_speed  =  SCAN_SPEED
        right_speed = -SCAN_SPEED

    elif state == 'RETREAT':
        # ── RETREAT: back up toward magenta goal (which is behind this robot) ──
        # ds_back faces toward +x (own magenta goal) because this robot faces -x.
        left_speed  = -RETREAT_SPEED
        right_speed = -RETREAT_SPEED

        # Lateral correction while reversing: push away from side walls
        # so we arrive centered in front of our own goal.
        if ds_l < WALL_SIDE_THRESH:
            # Too close to left wall → steer right (reduce backward speed on left)
            left_speed = clamp(left_speed + MAX_SPEED * 0.25, -RETREAT_SPEED, 0)
        if ds_r < WALL_SIDE_THRESH:
            # Too close to right wall → steer left (reduce backward speed on right)
            right_speed = clamp(right_speed + MAX_SPEED * 0.25, -RETREAT_SPEED, 0)

    elif state == 'GOALIE':
        # ── GOALIE: slow spin scan while parked in front of own goal ──
        # Spin clockwise so we sweep through the field looking for ball.
        # The ATTACK transition fires the moment the ball appears in camera.
        left_speed  =  GOALIE_SPEED
        right_speed = -GOALIE_SPEED

    set_speed(left_speed, right_speed)
