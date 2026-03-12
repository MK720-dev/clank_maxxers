from controller import Robot
import math
from enum import Enum, auto
import json

# ----------------------------
# Tunables
# ----------------------------
DUMP_FILE = "maze_dump.json"
TIME_STEP = 16
WHEEL_RADIUS_M = 0.0205
AXLE_LENGTH_M = 0.052

PROBE_DIST_M = 0.08
WALL_SPIKE_THRESH = 95.0

FWD_SPEED = 4
TURN_SPEED = 0.5

TILE_SIZE_M = 0.25

LEFT_MOTOR_NAME = "left wheel motor"
RIGHT_MOTOR_NAME = "right wheel motor"
LEFT_ENC_NAME = "left wheel sensor"
RIGHT_ENC_NAME = "right wheel sensor"
PS_NAMES = [f"ps{i}" for i in range(8)]

# Old fixed-step turn params are kept, but turn is now gyro-based.
TURN_90_STEPS = 140

MICRO_CORRECT_STEPS = 5
MICRO_CORRECT_SPEED = 0.05  # much lower than TURN_SPEED

FORWARD_CORRECT_STEPS = 5
FORWARD_CORRECT_SPEED = 0.25

# Gyro micro-calibration (ABS 90 reference)
YAW_EPS_DEG = 4.0          # only correct if off by more than this
YAW_MAX_MICRO_STEPS = 60   # safety cap for correction loop
YAW_SLOW_TURN = 0.25       # slow turn speed for micro-correction (rad/s motor vel)

# ----------------------------
# Direction maps (absolute)
# ----------------------------
DIRS = ["N", "E", "S", "W"]
DX = {"N": 0, "E": 1, "S": 0,  "W": -1}
DY = {"N": 1, "E": 0, "S": -1, "W": 0}
LEFT_OF  = {"N": "W", "E": "N", "S": "E", "W": "S"}
RIGHT_OF = {"N": "E", "E": "S", "S": "W", "W": "N"}
BACK_OF  = {"N": "S", "E": "W", "S": "N", "W": "E"}
TURN_ORDER = {"N": 0, "E": 1, "S": 2, "W": 3}

# Map heading to an absolute yaw target (multiples of 90 degrees).
# We do not care about true world yaw; we care about consistent 0/90/180/270 in our grid frame.
HEADING_TO_YAW = {
    "E": 0.0,
    "N": math.pi / 2.0,
    "W": math.pi,
    "S": -math.pi / 2.0,
}

# ----------------------------
# State Machine
# ----------------------------
class State(Enum):
    PROBING         = auto()
    DECIDE_NEXT     = auto()
    MOVE_ONE_TILE   = auto()
    RETURN_TO_START = auto()
    IDLE            = auto()
    BACKTRACK       = auto()

# ----------------------------
# JSON helpers
# ----------------------------
def cell_to_key(cell):
    return f"{cell[0]},{cell[1]}"

def log_event(ctx, tag, extra=None):
    if extra is None:
        extra = {}
    ctx["events"].append({
        "t":       float(ctx["robot"].getTime()),
        "tag":     tag,
        "state":   ctx["state"].name,
        "tile":    list(ctx["tile"]),
        "heading": ctx["heading"],
        "extra":   extra,
    })

def save_dump(ctx, filename=DUMP_FILE):
    payload = {
        "meta":          {"time": float(ctx["robot"].getTime())},
        "start_tile":    list(ctx["start_tile"]),
        "tile":          list(ctx["tile"]),
        "heading":       ctx["heading"],
        "visited_tiles": [list(t) for t in sorted(ctx["visited_tiles"])],
        "opens_map":     {cell_to_key(k): sorted(v)
                          for k, v in ctx["opens_map"].items()},
        "visited_exits": [[list(t), d]
                          for t, d in sorted(ctx["visited_exits"],
                          key=lambda x: (x[0], x[1]))],
        "parent":        {cell_to_key(k): (list(v) if v else None)
                          for k, v in ctx["parent"].items()},
        "events":        ctx["events"],
    }
    with open(filename, "w") as f:
        json.dump(payload, f, indent=2)

# ----------------------------
# Gyro yaw tracking + absolute 90 reference
# ----------------------------
def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def settle(ctx, steps=3):
    for _ in range(steps):
        if ctx["robot"].step(ctx["timestep"]) == -1:
            break

def detect_yaw_axis(ctx):
    # Spin briefly and pick gyro axis with strongest response.
    ctx["left_motor"].setVelocity(TURN_SPEED)
    ctx["right_motor"].setVelocity(-TURN_SPEED)
    settle(ctx, 2)

    sums = [0.0, 0.0, 0.0]
    for _ in range(8):
        if ctx["robot"].step(ctx["timestep"]) == -1:
            break
        g = ctx["gyro"].getValues()
        sums[0] += abs(g[0])
        sums[1] += abs(g[1])
        sums[2] += abs(g[2])

    ctx["left_motor"].setVelocity(0.0)
    ctx["right_motor"].setVelocity(0.0)
    settle(ctx, 2)

    axis = max(range(3), key=lambda i: sums[i])
    return axis

def estimate_gyro_bias(ctx, axis, samples=20):
    # Must be stationary.
    ctx["left_motor"].setVelocity(0.0)
    ctx["right_motor"].setVelocity(0.0)
    settle(ctx, 2)

    acc = 0.0
    n = 0
    for _ in range(samples):
        if ctx["robot"].step(ctx["timestep"]) == -1:
            break
        acc += ctx["gyro"].getValues()[axis]
        n += 1
    return acc / max(1, n)

def update_yaw(ctx):
    # Integrate yaw using fixed dt each step.
    dt = ctx["timestep"] / 1000.0
    g = ctx["gyro"].getValues()
    omega = g[ctx["yaw_axis"]] - ctx["yaw_bias"]
    ctx["yaw"] = wrap_pi(ctx["yaw"] + omega * dt)

def set_heading_and_target(ctx, heading):
    ctx["heading"] = heading
    ctx["yaw_target"] = HEADING_TO_YAW[heading]

def micro_correct_to_target(ctx, eps_deg=YAW_EPS_DEG, slow_turn=YAW_SLOW_TURN, max_steps=YAW_MAX_MICRO_STEPS):
    # Correct only if error is significant.
    eps = math.radians(eps_deg)

    for _ in range(max_steps):
        if ctx["robot"].step(ctx["timestep"]) == -1:
            break
        update_yaw(ctx)

        err = wrap_pi(ctx["yaw_target"] - ctx["yaw"])
        if abs(err) <= eps:
            break

        # Reduce error by turning in the appropriate direction.
        if err > 0:
            # need to increase yaw
            ctx["left_motor"].setVelocity(-slow_turn)
            ctx["right_motor"].setVelocity(slow_turn)
        else:
            ctx["left_motor"].setVelocity(slow_turn)
            ctx["right_motor"].setVelocity(-slow_turn)

    ctx["left_motor"].setVelocity(0.0)
    ctx["right_motor"].setVelocity(0.0)
    settle(ctx, 1)

# ----------------------------
# Low-level motion
# ----------------------------
def drive_distance(robot, left_motor, right_motor,
                   left_enc, right_enc, timestep, dist_m, speed):
    target    = abs(dist_m)
    traveled  = 0.0
    prev_l    = left_enc.getValue()
    prev_r    = right_enc.getValue()
    direction = 1.0 if dist_m >= 0.0 else -1.0
    left_motor.setVelocity(direction * speed)
    right_motor.setVelocity(direction * speed)
    while robot.step(timestep) != -1:
        l, r   = left_enc.getValue(), right_enc.getValue()
        dl, dr = (l - prev_l) * WHEEL_RADIUS_M, (r - prev_r) * WHEEL_RADIUS_M
        prev_l, prev_r = l, r
        traveled += abs(0.5 * (dl + dr))
        if traveled >= target:
            break
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

def turn_90(robot, left_motor, right_motor,
            left_enc, right_enc, timestep, turn_dir, speed):
    # REPLACED: gyro-based turn to absolute 90 (relative direction).
    # We keep signature unchanged so behavior functions remain unchanged.
    # This function relies on the "ctx" yaw micro-correction indirectly via turn_to_dir(),
    # but can also be used standalone. Here we do a fixed-time coarse spin then stop.
    lv, rv = (-speed, speed) if turn_dir == "left" else (speed, -speed)
    left_motor.setVelocity(lv)
    right_motor.setVelocity(rv)
    # coarse spin for a short duration; micro-correction happens in turn_to_dir()
    for _ in range(TURN_90_STEPS):
        robot.step(timestep)
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

def turn_180(robot, left_motor, right_motor,
             left_enc, right_enc, timestep, speed):
    # Kept for compatibility; not used by your behavior functions.
    target = math.pi * 1.14
    turned = 0.0
    prev_l = left_enc.getValue()
    prev_r = right_enc.getValue()
    left_motor.setVelocity(speed)
    right_motor.setVelocity(-speed)
    while robot.step(timestep) != -1:
        l, r   = left_enc.getValue(), right_enc.getValue()
        dl, dr = (l - prev_l) * WHEEL_RADIUS_M, (r - prev_r) * WHEEL_RADIUS_M
        prev_l, prev_r = l, r
        turned += abs((dr - dl) / AXLE_LENGTH_M)
        if turned >= target:
            break
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    for _ in range(10):
        robot.step(timestep)

def turn_to_dir(robot, ctx, target_dir):
    """
    IMPORTANT: We do NOT change how behavior functions call turn_to_dir.
    We augment turning with an absolute yaw reference:
      - do your existing 90-degree stepping logic to get close
      - snap yaw to the exact 90-degree target using micro_correct_to_target
    """
    while ctx["heading"] != target_dir:
        cur_idx = DIRS.index(ctx["heading"])
        tgt_idx = DIRS.index(target_dir)
        diff    = (tgt_idx - cur_idx) % 4
        if diff == 1:
            turn_90(robot, ctx["left_motor"], ctx["right_motor"],
                    ctx["left_enc"], ctx["right_enc"], ctx["timestep"], "right", TURN_SPEED)
            set_heading_and_target(ctx, RIGHT_OF[ctx["heading"]])
        elif diff == 3:
            turn_90(robot, ctx["left_motor"], ctx["right_motor"],
                    ctx["left_enc"], ctx["right_enc"], ctx["timestep"], "left", TURN_SPEED)
            set_heading_and_target(ctx, LEFT_OF[ctx["heading"]])
        else:  # diff == 2
            for _ in range(2):
                turn_90(robot, ctx["left_motor"], ctx["right_motor"],
                        ctx["left_enc"], ctx["right_enc"], ctx["timestep"], "right", TURN_SPEED)
                set_heading_and_target(ctx, RIGHT_OF[ctx["heading"]])
            # After two rights, heading already updated via two steps.

    # Snap to exact absolute 90 target for this heading if drifted.
    micro_correct_to_target(ctx)

# ----------------------------
# Probing
# ----------------------------
def probe_all_dirs(robot, ctx):
    open_exits = set()
    for d in DIRS:
        is_wall, _ = probe_dir(robot, ctx, d)
        if not is_wall:
            open_exits.add(d)
    return open_exits

def probe_dir(robot, ctx, abs_dir):
    original_heading = ctx["heading"]

    turn_to_dir(robot, ctx, abs_dir)
    drive_distance(robot, ctx["left_motor"], ctx["right_motor"],
                   ctx["left_enc"], ctx["right_enc"], ctx["timestep"],
                   PROBE_DIST_M, FWD_SPEED)

    ps_vals = [s.getValue() for s in ctx["ps"]]
    reading = max(ps_vals[0], ps_vals[7])
    is_wall = reading >= WALL_SPIKE_THRESH

    drive_distance(robot, ctx["left_motor"], ctx["right_motor"],
                   ctx["left_enc"], ctx["right_enc"], ctx["timestep"],
                   -PROBE_DIST_M, FWD_SPEED)
    turn_to_dir(robot, ctx, original_heading)

    return is_wall, reading

def probe_open_exits(robot, ctx):
    open_exits = set()
    readings = {}
    original_heading = ctx["heading"]

    probing_dirs = [RIGHT_OF[original_heading], LEFT_OF[original_heading], original_heading]

    for d in probing_dirs:
        is_wall, reading = probe_dir(robot, ctx, d)
        readings[d] = round(reading, 1)
        if not is_wall:
            open_exits.add(d)

    return open_exits, readings

# ----------------------------
# Heading recalibration (your existing utilities)
# ----------------------------
def find_heading_offset(known_opens, observed_opens):
    rotation_map = {
        0:   {"N":"N", "E":"E", "S":"S", "W":"W"},  # 0
        1:   {"N":"E", "E":"S", "S":"W", "W":"N"},  # 90 CW
        2:   {"N":"S", "E":"W", "S":"N", "W":"E"},  # 180
        3:   {"N":"W", "E":"N", "S":"E", "W":"S"},  # 90 CCW
    }
    for steps, rmap in rotation_map.items():
        rotated = {rmap[d] for d in known_opens}
        if rotated == observed_opens:
            return steps
    return None

def recalibrate_heading(robot, ctx):
    cur = ctx["tile"]
    if cur not in ctx["opens_map"]:
        return

    known_opens = ctx["opens_map"][cur]
    observed_opens = probe_all_dirs(robot, ctx)

    offset = find_heading_offset(known_opens, observed_opens)
    if offset is None:
        print("RECALIBRATE | ambiguous pattern, skipping")
        return
    if offset == 0:
        print("RECALIBRATE | heading confirmed correct")
        return

    cur_idx = DIRS.index(ctx["heading"])
    true_heading = DIRS[(cur_idx + offset) % 4]
    print(f"RECALIBRATE | was {ctx['heading']}, correcting to {true_heading}")
    set_heading_and_target(ctx, true_heading)
    micro_correct_to_target(ctx)

def micro_correct(ctx, direction):
    # Kept EXACT signature and behavior, but now also snaps to target afterwards.
    sign = -1.0 if direction == "left" else 1.0
    ctx["left_motor"].setVelocity(sign * TURN_SPEED)
    ctx["right_motor"].setVelocity(-sign * MICRO_CORRECT_SPEED)
    for _ in range(MICRO_CORRECT_STEPS):
        if ctx["robot"].step(ctx["timestep"]) == -1:
            break
        update_yaw(ctx)
    ctx["left_motor"].setVelocity(0.0)
    ctx["right_motor"].setVelocity(0.0)
    # After any micro turn, snap back to the current heading's absolute 90 target.
    micro_correct_to_target(ctx)

def forward_correct(ctx):
    ctx["left_motor"].setVelocity(FORWARD_CORRECT_SPEED)
    ctx["right_motor"].setVelocity(FORWARD_CORRECT_SPEED)
    for _ in range(FORWARD_CORRECT_STEPS):
        if ctx["robot"].step(ctx["timestep"]) == -1:
            break
        update_yaw(ctx)
    ctx["left_motor"].setVelocity(0.0)
    ctx["right_motor"].setVelocity(0.0)
    # Forward motion can induce yaw drift; snap if needed.
    micro_correct_to_target(ctx)

# -------------------------
# Behavior Functions
# NOTE: As requested, these are left as-is (no logic changes).
# -------------------------
def behavior_probing(ctx):
    cur = ctx["tile"]

    if cur in ctx["opens_map"]:
        print("PROBING | tile", cur, "already known, skipping")
        log_event(ctx, "PROBING_SKIP", extra={"reason": "already known", "opens": sorted(ctx["opens_map"][cur])})
        ctx["state"] = State.DECIDE_NEXT
        return

    robot      = ctx["robot"]
    open_exits = set()
    readings   = {}

    original_heading = ctx["heading"]
    probing_dirs = [RIGHT_OF[original_heading], LEFT_OF[original_heading], original_heading]

    for d in probing_dirs:
        is_wall, reading = probe_dir(robot, ctx, d)
        readings[d] = round(reading, 1)
        if not is_wall:
            open_exits.add(d)

    ctx["opens_map"][cur] = open_exits

    print("PROBING | tile", cur, "heading", ctx["heading"],
            "| opens:", sorted(open_exits), "| readings:", readings)
    log_event(ctx, "PROBING", extra={"opens": sorted(open_exits), "readings": readings})
    ctx["state"] = State.DECIDE_NEXT

def behavior_decide_next(ctx):

    cur = ctx["tile"]

    if cur not in ctx["visited_tiles"]:
        ctx["visited_tiles"].add(cur)

    print("DECIDE_NEXT (placeholder) | tile", cur, "| opens:", sorted(ctx["opens_map"][cur]))
    print("visited_tiles has cur?", cur in ctx["visited_tiles"])
    print("DECIDE_NEXT | visited_exits:", sorted(ctx["visited_exits"]))
    print("DECIDE_NEXT | stack:", ctx["stack"])

    open_dirs = ctx["opens_map"][cur]

    ctx["chosen_dir"] = None
    priority_dirs = [d for d in DIRS if d in open_dirs and d != BACK_OF[ctx["heading"]]]
    for d in priority_dirs:
        nxt = (cur[0] + DX[d], cur[1] + DY[d])
        if (cur, d) not in ctx["visited_exits"]:
            ctx["chosen_dir"] = d
            ctx["visited_exits"].add((cur, d))
            break

    if ctx["chosen_dir"] is None:
        print("DECIDE_NEXT | no unvisited exits, need to backtrack")
        ctx["state"] = State.BACKTRACK
    else:
        print("DECIDE_NEXT | chosen dir:", ctx["chosen_dir"])
        ctx["state"] = State.MOVE_ONE_TILE

def behavior_move_one_tile(ctx):

    cur = ctx["tile"]
    chosen = ctx["chosen_dir"]
    turn_to_dir(ctx["robot"], ctx, chosen)
    nxt = (cur[0] + DX[chosen], cur[1] + DY[chosen])
    ctx["visited_exits"].add((nxt, BACK_OF[chosen]))
    ctx["stack"].append(nxt)
    ctx["tile"] = nxt
    ctx["heading"] = chosen
    ctx["yaw_target"] = HEADING_TO_YAW[ctx["heading"]]  # keep yaw target aligned
    print("MOVE_ONE_TILE (placeholder) | going from", cur, "->", nxt, "| heading", ctx["heading"])
    log_event(ctx, "MOVE_ONE_TILE", extra={"from": list(cur), "to": list(nxt), "heading": ctx["heading"]})
    drive_distance(ctx["robot"], ctx["left_motor"], ctx["right_motor"],ctx["left_enc"], ctx["right_enc"], ctx["timestep"], TILE_SIZE_M, FWD_SPEED)
    forward_correct(ctx)
    ctx["state"] = State.PROBING

def behavior_backtrack(ctx):
    if not ctx["stack"] or ctx["stack"][-1] == ctx["start_tile"]:
        ctx["state"] = State.RETURN_TO_START
        return
    cur = ctx["stack"].pop()  # current tile
    if not ctx["stack"]:
        # popped back to start, go to decide_next to check for remaining exits
        ctx["tile"] = ctx["start_tile"]
        drive_distance(ctx["robot"], ctx["left_motor"], ctx["right_motor"],
                       ctx["left_enc"], ctx["right_enc"], ctx["timestep"], TILE_SIZE_M, FWD_SPEED)
        ctx["state"] = State.DECIDE_NEXT  # skip probing, start is already known
        return
    prev = ctx["stack"][-1]   # peek, don't pop - we'll pop next time we backtrack

    dx = prev[0] - cur[0]
    dy = prev[1] - cur[1]
    chosen = next(d for d in DIRS if DX[d] == dx and DY[d] == dy)

    cur_idx = DIRS.index(ctx["heading"])
    tgt_idx = DIRS.index(chosen)
    diff = (tgt_idx - cur_idx) % 4
    turn_dir = "right" if diff <= 2 else "left"

    turn_to_dir(ctx["robot"], ctx, chosen)
    ctx["heading"] = chosen
    ctx["yaw_target"] = HEADING_TO_YAW[ctx["heading"]]
    micro_correct(ctx, turn_dir)

    ctx["heading"] = chosen
    ctx["yaw_target"] = HEADING_TO_YAW[ctx["heading"]]
    drive_distance(ctx["robot"], ctx["left_motor"], ctx["right_motor"],
                   ctx["left_enc"], ctx["right_enc"], ctx["timestep"], TILE_SIZE_M, FWD_SPEED)
    ctx["tile"] = prev
    print("BACKTRACK | going back from", cur, "->", prev)
    log_event(ctx, "BACKTRACK", extra={"from": list(cur), "to": list(prev)})
    ctx["state"] = State.PROBING

def behavior_return_to_start(ctx):
    print("RETURN_TO_START (placeholder)")
    ctx["state"] = State.IDLE
    return

def behavior_idle(ctx):
    ctx["left_motor"].setVelocity(0.0)
    ctx["right_motor"].setVelocity(0.0)
    return

# ----------------------------
# Main
# ----------------------------
def main():
    robot    = Robot()
    timestep = int(robot.getBasicTimeStep())

    left_motor  = robot.getDevice(LEFT_MOTOR_NAME)
    right_motor = robot.getDevice(RIGHT_MOTOR_NAME)
    left_motor.setPosition(float("inf"))
    right_motor.setPosition(float("inf"))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    left_enc  = robot.getDevice(LEFT_ENC_NAME)
    right_enc = robot.getDevice(RIGHT_ENC_NAME)
    left_enc.enable(timestep)
    right_enc.enable(timestep)

    ps = []
    for n in PS_NAMES:
        s = robot.getDevice(n)
        s.enable(timestep)
        ps.append(s)

    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)

    ctx = {
        # Robot devices
        "robot":       robot,
        "timestep":    timestep,
        "left_motor":  left_motor,
        "right_motor": right_motor,
        "left_enc":    left_enc,
        "right_enc":   right_enc,
        "ps":          ps,
        "gyro":        gyro,

        # Pose
        "start_tile":  (0, 0),
        "tile":        (0, 0),
        "heading":     "E",

        # Gyro yaw state (absolute 90 reference)
        "yaw":         0.0,
        "yaw_axis":    2,     # will be overwritten by detect_yaw_axis()
        "yaw_bias":    0.0,   # will be overwritten by estimate_gyro_bias()
        "yaw_target":  HEADING_TO_YAW["E"],

        # Maze knowledge
        "opens_map":     {},
        "stack":         [(0,0)],
        "visited_exits": set(),
        "visited_tiles": set(),
        "parent":        {(0, 0): None},

        # Behavior
        "state":      State.PROBING,
        "chosen_dir": None,
        "prev_dir": None,
        "events":     [],
    }

    for _ in range(10):
        if robot.step(timestep) == -1:
            return

    # Detect yaw axis + bias once, at startup.
    ctx["yaw_axis"] = detect_yaw_axis(ctx)
    ctx["yaw_bias"] = estimate_gyro_bias(ctx, ctx["yaw_axis"], samples=25)
    # Initialize yaw target to current heading.
    ctx["yaw_target"] = HEADING_TO_YAW[ctx["heading"]]
    # Take a few steps to initialize yaw integration.
    settle(ctx, 2)

    print("=== Maze Navigator ===")
    print("GYRO | yaw_axis:", ctx["yaw_axis"], "| yaw_bias:", round(ctx["yaw_bias"], 6))

    cycle       = 0
    last_dump_t = robot.getTime()

    try:
        while robot.step(timestep) != -1:
            cycle += 1
            if cycle % 15 != 0:
                continue

            st = ctx["state"]
            if   st == State.PROBING:         behavior_probing(ctx)
            elif st == State.DECIDE_NEXT:     behavior_decide_next(ctx)
            elif st == State.MOVE_ONE_TILE:   behavior_move_one_tile(ctx)
            elif st == State.RETURN_TO_START: behavior_return_to_start(ctx)
            elif st == State.BACKTRACK:       behavior_backtrack(ctx)
            else:                             behavior_idle(ctx)

            t = robot.getTime()
            if t - last_dump_t >= 30.0:
                save_dump(ctx)
                last_dump_t = t

    except KeyboardInterrupt:
        pass

    save_dump(ctx)
    print("Saved", DUMP_FILE, "| events:", len(ctx["events"]))

if __name__ == "__main__":
    main()