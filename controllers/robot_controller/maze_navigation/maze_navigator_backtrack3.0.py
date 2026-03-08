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
FWD_SPEED = 3.0
TURN_SPEED = 3.0
TILE_SIZE_M = 0.25
LEFT_MOTOR_NAME = "left wheel motor"
RIGHT_MOTOR_NAME = "right wheel motor"
LEFT_ENC_NAME = "left wheel sensor"
RIGHT_ENC_NAME = "right wheel sensor"
PS_NAMES = [f"ps{i}" for i in range(8)]

# ----------------------------
# Direction maps (absolute)
# ----------------------------
DIRS = ["N", "E", "S", "W"]
DX = {"N": 0, "E": 1, "S": 0,  "W": -1}
DY = {"N": 1, "E": 0, "S": -1, "W": 0}
LEFT_OF  = {"N": "W", "E": "N", "S": "E", "W": "S"}
RIGHT_OF = {"N": "E", "E": "S", "S": "W", "W": "N"}
BACK_OF  = {"N": "S", "E": "W", "S": "N", "W": "E"}

# ----------------------------
# State Machine
# ----------------------------
class State(Enum):
    PROBING         = auto()
    DECIDE_NEXT     = auto()
    MOVE_ONE_TILE   = auto()
    RETURN_TO_START = auto()
    IDLE            = auto()

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
    target = (math.pi / 2.0) * 1.08
    turned = 0.0
    prev_l = left_enc.getValue()
    prev_r = right_enc.getValue()
    lv, rv = (-speed, speed) if turn_dir == "left" else (speed, -speed)
    left_motor.setVelocity(lv)
    right_motor.setVelocity(rv)
    while robot.step(timestep) != -1:
        l, r   = left_enc.getValue(), right_enc.getValue()
        dl, dr = (l - prev_l) * WHEEL_RADIUS_M, (r - prev_r) * WHEEL_RADIUS_M
        prev_l, prev_r = l, r
        turned += abs((dr - dl) / AXLE_LENGTH_M)
        if turned >= target:
            break
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

def turn_180(robot, left_motor, right_motor,
             left_enc, right_enc, timestep, speed):
    target = math.pi * 1.106
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

def turn_to_dir(robot, ctx, target_dir):
    while ctx["heading"] != target_dir:
        cur_idx = DIRS.index(ctx["heading"])
        tgt_idx = DIRS.index(target_dir)
        diff    = (tgt_idx - cur_idx) % 4
        if diff == 1:
            turn_90(robot, ctx["left_motor"], ctx["right_motor"],
                    ctx["left_enc"], ctx["right_enc"], ctx["timestep"], "right", TURN_SPEED)
            ctx["heading"] = RIGHT_OF[ctx["heading"]]
        elif diff == 3:
            turn_90(robot, ctx["left_motor"], ctx["right_motor"],
                    ctx["left_enc"], ctx["right_enc"], ctx["timestep"], "left", TURN_SPEED)
            ctx["heading"] = LEFT_OF[ctx["heading"]]
        else:  # diff == 2
            turn_180(robot, ctx["left_motor"], ctx["right_motor"],
                     ctx["left_enc"], ctx["right_enc"], ctx["timestep"], TURN_SPEED)
            ctx["heading"] = BACK_OF[ctx["heading"]]

# ----------------------------
# Probing
# ----------------------------
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


def behavior_probing(ctx):
    cur = ctx["tile"]

    if cur in ctx["opens_map"]:
        print("PROBING | tile", cur, "already known, skipping")
        ctx["state"] = State.DECIDE_NEXT
        return

    robot      = ctx["robot"]
    open_exits = set()
    readings   = {}

    for d in DIRS:
        is_wall, reading = probe_dir(robot, ctx, d)
        readings[d] = round(reading, 1)
        if not is_wall:
            open_exits.add(d)

    # Safety: direction we arrived from is always open
    if cur != ctx["start_tile"]:
        open_exits.add(BACK_OF[ctx["heading"]])

    ctx["opens_map"][cur] = open_exits

    print("PROBING | tile", cur, "heading", ctx["heading"],
          "| opens:", sorted(open_exits), "| readings:", readings)
    log_event(ctx, "PROBING", extra={"opens": sorted(open_exits), "readings": readings})

    ctx["state"] = State.DECIDE_NEXT

# ----------------------------
# Placeholders
# ----------------------------
def behavior_decide_next(ctx):
    print("DECIDE_NEXT (placeholder) | tile", ctx["tile"])
    ctx["state"] = State.IDLE

def behavior_move_one_tile(ctx):
    print("MOVE_ONE_TILE (placeholder) | tile", ctx["tile"])
    ctx["state"] = State.IDLE

def behavior_return_to_start(ctx):
    print("RETURN_TO_START (placeholder)")
    ctx["state"] = State.IDLE

def behavior_idle(ctx):
    ctx["left_motor"].setVelocity(0.0)
    ctx["right_motor"].setVelocity(0.0)

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

    ctx = {
        # Robot devices
        "robot":       robot,
        "timestep":    timestep,
        "left_motor":  left_motor,
        "right_motor": right_motor,
        "left_enc":    left_enc,
        "right_enc":   right_enc,
        "ps":          ps,
        # Pose
        "start_tile":  (0, 0),
        "tile":        (0, 0),
        "heading":     "E",
        # Maze knowledge
        "opens_map":     {},
        "visited_tiles": set(),
        "visited_exits": set(),
        "parent":        {(0, 0): None},
        # Behavior
        "state":      State.PROBING,
        "chosen_dir": None,
        "events":     [],
    }

    for _ in range(10):
        if robot.step(timestep) == -1:
            return

    print("=== Maze Navigator ===")

    cycle       = 0
    last_dump_t = robot.getTime()

    while robot.step(timestep) != -1:
        cycle += 1
        if cycle % 15 != 0:
            continue

        st = ctx["state"]
        if   st == State.PROBING:         behavior_probing(ctx)
        elif st == State.DECIDE_NEXT:     behavior_decide_next(ctx)
        elif st == State.MOVE_ONE_TILE:   behavior_move_one_tile(ctx)
        elif st == State.RETURN_TO_START: behavior_return_to_start(ctx)
        else:                             behavior_idle(ctx)

        t = robot.getTime()
        if t - last_dump_t >= 30.0:
            save_dump(ctx)
            last_dump_t = t

    save_dump(ctx)
    print("Saved", DUMP_FILE, "| events:", len(ctx["events"]))

if __name__ == "__main__":
    main()