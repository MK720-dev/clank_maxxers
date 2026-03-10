# -*- coding: utf-8 -*-
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

# -----------------------------
# JSON helpers
# -----------------------------
def cell_to_key(cell):
    return f"{cell[0]},{cell[1]}"

def set_to_sorted_list(s):
    return sorted(list(s))

def log_event(ctx, tag, extra=None):
    if extra is None:
        extra = {}
    ctx["events"].append({
        "t": float(ctx["robot"].getTime()),
        "tag": tag,
        "state": ctx["state"].name if "state" in ctx else str(ctx.get("state")),
        "tile": [ctx["tile"][0], ctx["tile"][1]] if "tile" in ctx else None,
        "heading": ctx.get("heading"),
        "chosen_dir": ctx.get("chosen_dir"),
        "extra": extra,
    })

def save_dump(ctx, filename=DUMP_FILE):
    opens_map_json = {}
    for cell, opens in ctx.get("opens_map", {}).items():
        opens_map_json[cell_to_key(cell)] = set_to_sorted_list(opens)

    visited_tiles_json = [[c[0], c[1]] for c in sorted(ctx.get("visited_tiles", set()))]
    visited_exits_json = [[[c[0], c[1]], d] for (c, d) in sorted(ctx.get("visited_exits", set()), key=lambda x: (x[0][0], x[0][1], x[1]))]

    payload = {
        "meta": {"time": float(ctx["robot"].getTime())},
        "start_tile": list(ctx.get("start_tile", (0, 0))),
        "start_heading": ctx.get("start_heading"),
        "tile": list(ctx.get("tile", (0, 0))),
        "heading": ctx.get("heading"),
        "visited_tiles": visited_tiles_json,
        "visited_exits": visited_exits_json,
        "opens_map": opens_map_json,
        "stack": [list(t) for t in ctx.get("stack", [])],
        "events": ctx.get("events", []),
    }

    with open(filename, "w") as f:
        json.dump(payload, f, indent=2)

# ----------------------------
# State Machine
# ----------------------------
class State(Enum):
    PROBING = auto()
    DECIDE_NEXT = auto()
    MOVE_ONE_TILE = auto()
    RETURN_TO_START = auto()
    IDLE = auto()

# ----------------------------
# Helpers (sensors + motion)
# ----------------------------
def sector_values(ps_vals):
    front = max(ps_vals[0], ps_vals[7])
    left = max(ps_vals[5], ps_vals[6])
    right = max(ps_vals[1], ps_vals[2])
    return front, left, right

def drive_distance(robot, left_motor, right_motor, left_enc, right_enc, timestep, dist_m, speed):
    target = abs(dist_m)
    traveled = 0.0
    prev_l = left_enc.getValue()
    prev_r = right_enc.getValue()
    direction = 1.0 if dist_m >= 0.0 else -1.0
    left_motor.setVelocity(direction * speed)
    right_motor.setVelocity(direction * speed)
    while robot.step(timestep) != -1:
        l = left_enc.getValue()
        r = right_enc.getValue()
        dl = (l - prev_l) * WHEEL_RADIUS_M
        dr = (r - prev_r) * WHEEL_RADIUS_M
        prev_l, prev_r = l, r
        dc = 0.5 * (dl + dr)
        traveled += abs(dc)
        if traveled >= target:
            break
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

def turn_90(robot, left_motor, right_motor, left_enc, right_enc, timestep, direction, speed):
    target = (math.pi / 2.0) * 1.08
    turned = 0.0
    prev_l = left_enc.getValue()
    prev_r = right_enc.getValue()
    if direction == "left":
        left_motor.setVelocity(-speed)
        right_motor.setVelocity(speed)
    else:
        left_motor.setVelocity(speed)
        right_motor.setVelocity(-speed)
    while robot.step(timestep) != -1:
        l = left_enc.getValue()
        r = right_enc.getValue()
        dl = (l - prev_l) * WHEEL_RADIUS_M
        dr = (r - prev_r) * WHEEL_RADIUS_M
        prev_l, prev_r = l, r
        dtheta = (dr - dl) / AXLE_LENGTH_M
        turned += abs(dtheta)
        if turned >= target:
            break
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

def probe_is_wall(robot, ps, left_motor, right_motor, left_enc, right_enc, timestep, rel_dir):
    if rel_dir == "left":
        turn_90(robot, left_motor, right_motor, left_enc, right_enc, timestep, "left", TURN_SPEED)
    elif rel_dir == "right":
        turn_90(robot, left_motor, right_motor, left_enc, right_enc, timestep, "right", TURN_SPEED)
    drive_distance(robot, left_motor, right_motor, left_enc, right_enc, timestep, PROBE_DIST_M, FWD_SPEED)
    ps_vals = [s.getValue() for s in ps]
    front, _, _ = sector_values(ps_vals)
    wall = front >= WALL_SPIKE_THRESH
    drive_distance(robot, left_motor, right_motor, left_enc, right_enc, timestep, -PROBE_DIST_M, FWD_SPEED)
    if rel_dir == "left":
        turn_90(robot, left_motor, right_motor, left_enc, right_enc, timestep, "right", TURN_SPEED)
    elif rel_dir == "right":
        turn_90(robot, left_motor, right_motor, left_enc, right_enc, timestep, "left", TURN_SPEED)
    return wall, front, ps_vals

def turn_to_dir(robot, ctx, target_dir):
    heading = ctx["heading"]
    while heading != target_dir:
        cur_idx = DIRS.index(heading)
        tgt_idx = DIRS.index(target_dir)
        diff = (tgt_idx - cur_idx) % 4
        if diff == 1:
            turn_90(robot, ctx["left_motor"], ctx["right_motor"], ctx["left_enc"], ctx["right_enc"],
                    ctx["timestep"], "right", TURN_SPEED)
            heading = RIGHT_OF[heading]
        elif diff == 3:
            turn_90(robot, ctx["left_motor"], ctx["right_motor"], ctx["left_enc"], ctx["right_enc"],
                    ctx["timestep"], "left", TURN_SPEED)
            heading = LEFT_OF[heading]
        else:
            turn_90(robot, ctx["left_motor"], ctx["right_motor"], ctx["left_enc"], ctx["right_enc"],
                    ctx["timestep"], "right", TURN_SPEED)
            heading = RIGHT_OF[heading]
            turn_90(robot, ctx["left_motor"], ctx["right_motor"], ctx["left_enc"], ctx["right_enc"],
                    ctx["timestep"], "right", TURN_SPEED)
            heading = RIGHT_OF[heading]
    ctx["heading"] = heading

def is_fully_explored(tile, ctx):
    if tile not in ctx["opens_map"]:
        return False
    return all((tile, d) in ctx["visited_exits"] for d in ctx["opens_map"][tile])

def mark_explored_neighbour_exits(cur, ctx):
    """On arriving at cur, mark exits toward fully-explored neighbours as visited."""
    for d in DIRS:
        neighbour = (cur[0] + DX[d], cur[1] + DY[d])
        if is_fully_explored(neighbour, ctx):
            ctx["visited_exits"].add((cur, d))

# ----------------------------
# Direction maps
# ----------------------------
DIRS = ["N","E","S","W"]
DX = {"N":0, "E":1, "S":0, "W":-1}
DY = {"N":1, "E":0, "S":-1, "W":0}
LEFT_OF  = {"N":"W", "E":"N", "S":"E", "W":"S"}
RIGHT_OF = {"N":"E", "E":"S", "S":"W", "W":"N"}
BACK_OF  = {"N":"S", "E":"W", "S":"N", "W":"E"}

# ----------------------------
# Behavior Layer Implementations
# ----------------------------
def behavior_probing(ctx):
    robot = ctx["robot"]
    ps = ctx["ps"]
    left_motor, right_motor = ctx["left_motor"], ctx["right_motor"]
    left_enc, right_enc = ctx["left_enc"], ctx["right_enc"]
    timestep = ctx["timestep"]

    fw, f_read, _ = probe_is_wall(robot, ps, left_motor, right_motor, left_enc, right_enc, timestep, "front")
    lw, l_read, _ = probe_is_wall(robot, ps, left_motor, right_motor, left_enc, right_enc, timestep, "left")
    rw, r_read, _ = probe_is_wall(robot, ps, left_motor, right_motor, left_enc, right_enc, timestep, "right")

    ctx["probe_result"] = {
        "front_wall": fw, "left_wall": lw, "right_wall": rw,
        "front_val": float(f_read), "left_val": float(l_read), "right_val": float(r_read),
    }

    print("PROBING |", "F:", fw, "val", round(f_read, 1),
          "| L:", lw, "val", round(l_read, 1),
          "| R:", rw, "val", round(r_read, 1))

    log_event(ctx, "PROBING", extra={"probe_result": ctx.get("probe_result")})
    ctx["state"] = State.DECIDE_NEXT


def behavior_decide_next(ctx):
    start = ctx["start_tile"]
    x, y = ctx["tile"]
    cur = (x, y)

    print("DECIDE_NEXT | RAW STATE | cur", cur, 
      "| opens_map:", ctx["opens_map"].get(cur),
      "| visited_exits:", [(d) for (t,d) in ctx["visited_exits"] if t == cur])

    # Push to stack on first visit
    if cur not in ctx["visited_tiles"]:
        ctx["visited_tiles"].add(cur)
        ctx["stack"].append(cur)
        print("DECIDE_NEXT | Visiting new tile:", cur, "| stack now:", ctx["stack"])
    else: 
        if cur in ctx["stack"]:
            idx = ctx["stack"].index(cur)
            ctx["stack"] = ctx["stack"][:idx+1]  # trim stack back to current tile (handles loops)
            print("DECIDE_NEXT | Revisiting tile:", cur, "| stack trimmed to:", ctx["stack"])
        elif not is_fully_explored(cur, ctx):
            print("DECIDE_NEXT | Revisiting tile:", cur, "| but it's not fully explored and not on stack -> stack now:", ctx["stack"] + [cur])
            ctx["stack"].append(cur)

    # Mark exits toward fully-explored neighbours so they don't appear as untried
    mark_explored_neighbour_exits(cur, ctx)

    # Build open_dirs from probe result (write once, trust forever)
    if cur not in ctx["opens_map"]:
        # Fresh tile — build from probe result
        pr = ctx.get("probe_result", {})
        heading = ctx["heading"]
        open_dirs = set()
        if not pr.get("front_wall", False):
            open_dirs.add(heading)
        if not pr.get("left_wall", False):
            open_dirs.add(LEFT_OF[heading])
        if not pr.get("right_wall", False):
            open_dirs.add(RIGHT_OF[heading])
        if cur != start:
            open_dirs.add(BACK_OF[heading])
        ctx["opens_map"][cur] = open_dirs
        
    open_dirs = ctx["opens_map"][cur]
    
    # Debug info to understand decision making at each step, especially how visited_exits evolves and interacts with stack state
    print("DECIDE_NEXT | cur", cur, "| opens_map:", ctx["opens_map"].get(cur), "| visited_exits for cur:", [d for (t,d) in ctx["visited_exits"] if t == cur])

    # Priority: forward > left > right > back (back = implicit single-step backtrack)
    heading = ctx["heading"]
    PRIORITY = [heading, LEFT_OF[heading], RIGHT_OF[heading]]

    untried = [d for d in PRIORITY if d in open_dirs and (cur, d) not in ctx["visited_exits"]]

    if untried:
        chosen = untried[0]  # already in priority order
        ctx["chosen_dir"] = chosen
        ctx["visited_exits"].add((cur, chosen))
        ctx["state"] = State.MOVE_ONE_TILE
        print("DECIDE_NEXT | cur", cur, "| open", sorted(open_dirs), "| untried:", untried, "| chosen:", chosen)
        return

    # All exits exhausted — pop stack (tracks completion only) then use parent map for direction
    if ctx["stack"] and ctx["stack"][-1] == cur:
        ctx["stack"].pop()

    print("DECIDE_NEXT | cur", cur, "| open", sorted(open_dirs), "| untried:", untried, "| all exits exhausted, stack now:", ctx["stack"])

    if not ctx["stack"]:
        print("DECIDE_NEXT | stack empty -> RETURN_TO_START")
        ctx["state"] = State.RETURN_TO_START
        return

    # Parent map gives the correct backtrack direction regardless of stack state
    target = ctx["parent"].get(cur)
    print("DECIDE_NEXT | parent chain:", cur, "->", target, "-> ", ctx["parent"].get(target))

    if target is None:
        print("DECIDE_NEXT | no parent for", cur, "(at root) -> RETURN_TO_START")
        ctx["state"] = State.RETURN_TO_START
        return

    dx = target[0] - cur[0]
    dy = target[1] - cur[1]
    back_dir = next((d for d in DIRS if DX[d] == dx and DY[d] == dy), None)

    if back_dir is None:
        print("DECIDE_NEXT | ERROR: parent", target, "not adjacent to", cur, "-> RETURN_TO_START")
        ctx["state"] = State.RETURN_TO_START
        return

    ctx["chosen_dir"] = back_dir
    ctx["visited_exits"].add((cur, back_dir))
    ctx["state"] = State.MOVE_ONE_TILE
    print("DECIDE_NEXT | cur", cur, "| all exits exhausted, backtrack to parent", target, "via", back_dir)

    log_event(ctx, "DECIDE_NEXT", extra={
        "open": sorted(list(open_dirs)),
        "untried": untried,
        "chosen": ctx["chosen_dir"]
    })


def behavior_move_one_tile(ctx):
    chosen = ctx["chosen_dir"]
    if chosen is None:
        print("MOVE_ONE_TILE | ERROR: chosen_dir is None -> IDLE")
        ctx["state"] = State.IDLE
        return

    robot = ctx["robot"]
    cur = ctx["tile"]
    x, y = cur

    turn_to_dir(robot, ctx, chosen)
    drive_distance(robot, ctx["left_motor"], ctx["right_motor"], ctx["left_enc"], ctx["right_enc"],
                   ctx["timestep"], TILE_SIZE_M, FWD_SPEED)

    nxt = (x + DX[chosen], y + DY[chosen])
    ctx["tile"] = nxt
    ctx["heading"] = chosen

    print("MOVE_ONE_TILE | after confirm | cur opens_map:", ctx["opens_map"].get(cur),
      "| cur visited_exits:", [d for (t,d) in ctx["visited_exits"] if t == cur],
      "| nxt opens_map:", ctx["opens_map"].get(nxt),
      "| nxt visited_exits:", [d for (t,d) in ctx["visited_exits"] if t == nxt])

    # After updating tile and parent:
    # Confirm this connection exists in both directions in opens_map
    if cur in ctx["opens_map"]:
        ctx["opens_map"][cur].add(chosen)
        # Already traversed this, mark visited
        ctx["visited_exits"].add((cur, chosen))

    if nxt in ctx["opens_map"]:
        ctx["opens_map"][nxt].add(BACK_OF[chosen])
        # nxt -> cur is already traversed, mark visited
        ctx["visited_exits"].add((nxt, BACK_OF[chosen]))


    # Record parent on first arrival (source of truth for backtrack direction)
    if nxt not in ctx["parent"]:
        ctx["parent"][nxt] = cur

    # Mark reverse exit visited so we don't immediately turn around
    ctx["visited_exits"].add((nxt, BACK_OF[chosen]))

    print("MOVE_ONE_TILE |", cur, "->", nxt, "| heading now", ctx["heading"])
    log_event(ctx, "MOVE_ONE_TILE", extra={"from": list(cur), "to": list(nxt), "via": chosen})

    ctx["state"] = State.DECIDE_NEXT if nxt in ctx["opens_map"] else State.PROBING


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
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    left_motor = robot.getDevice(LEFT_MOTOR_NAME)
    right_motor = robot.getDevice(RIGHT_MOTOR_NAME)
    left_motor.setPosition(float("inf"))
    right_motor.setPosition(float("inf"))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    left_enc = robot.getDevice(LEFT_ENC_NAME)
    right_enc = robot.getDevice(RIGHT_ENC_NAME)
    left_enc.enable(timestep)
    right_enc.enable(timestep)

    ps = []
    for n in PS_NAMES:
        s = robot.getDevice(n)
        s.enable(timestep)
        ps.append(s)

    ctx = {
        "robot": robot,
        "timestep": timestep,
        "left_motor": left_motor,
        "right_motor": right_motor,
        "left_enc": left_enc,
        "right_enc": right_enc,
        "ps": ps,
        "start_tile": (0, 0),
        "tile": (0, 0),
        "heading": "E",
        "visited_tiles": set(),
        "visited_exits": set(),
        "stack": [],
        "opens_map": {},
        "parent": {(0, 0): None},
        "state": State.PROBING,
        "probe_result": None,
        "chosen_dir": None,
        "events": []
    }

    for _ in range(10):
        if robot.step(timestep) == -1:
            return

    print("=== Maze Navigator ===")
    print("State starts at:", ctx["state"])

    cycle = 0
    last_dump_t = robot.getTime()
    while robot.step(timestep) != -1:
        cycle += 1
        if cycle % 15 != 0:
            continue

        st = ctx["state"]
        if st == State.PROBING:
            behavior_probing(ctx)
        elif st == State.DECIDE_NEXT:
            behavior_decide_next(ctx)
        elif st == State.MOVE_ONE_TILE:
            behavior_move_one_tile(ctx)
        elif st == State.RETURN_TO_START:
            behavior_return_to_start(ctx)
        else:
            behavior_idle(ctx)

        t = robot.getTime()
        if t - last_dump_t >= 30.0:
            save_dump(ctx)
            last_dump_t = t

    save_dump(ctx)
    print("Saved", DUMP_FILE, "events:", len(ctx["events"]))

if __name__ == "__main__":
    main()

