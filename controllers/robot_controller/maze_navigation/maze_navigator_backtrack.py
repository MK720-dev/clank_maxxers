# -*- coding: utf-8 -*-
"""
EECS 690 - Maze Navigation (Layered Controller Skeleton)
-------------------------------------------------------
Goal: keep it simple and build one behavior at a time.

Layers / Behaviors (state machine):
- PROBING: uses active probing (your working TEST 1) to classify walls around current tile center
- DECIDE_NEXT: placeholder (later: DFS chooses an unvisited open exit)
- MOVE_ONE_TILE: placeholder (later: move exactly one tile in chosen direction)
- BACKTRACK: placeholder (later: DFS backtracking)
- RETURN_TO_START: placeholder (later: BFS or parent-walk back to start)
- IDLE: stop

Right now:
- We ONLY implement PROBING fully.
- Everything else is a placeholder (prints what it would do and idles).

You can progressively fill placeholders without breaking the probing behavior.
"""

from controller import Robot
import math
from enum import Enum, auto
import random
import json
import time

# ----------------------------
# Tunables
# ----------------------------
DUMP_FILE = "maze_dump.json"

TIME_STEP = 16

# e-puck physical params
WHEEL_RADIUS_M = 0.0205
AXLE_LENGTH_M = 0.052

# Probing parameters
PROBE_DIST_M = 0.08         # your current working value
WALL_SPIKE_THRESH = 95.0     # based on your observation (90-150 near wall)

# Motion speeds (rad/s)
FWD_SPEED = 3.0
TURN_SPEED = 3.0

# Tile size (placeholder for later)
TILE_SIZE_M = 0.25            # you said tile size is 0.5 (for later MOVE_ONE_TILE)

# Devices (standard e-puck)
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
    # Convert opens_map (cell->set) to JSONable (str->list)
    opens_map_json = {}
    for cell, opens in ctx.get("opens_map", {}).items():
        opens_map_json[cell_to_key(cell)] = set_to_sorted_list(opens)

    # Convert visited sets
    visited_tiles_json = [[c[0], c[1]] for c in sorted(ctx.get("visited_tiles", set()))]
    visited_exits_json = [[[c[0], c[1]], d] for (c, d) in sorted(ctx.get("visited_exits", set()), key=lambda x: (x[0][0], x[0][1], x[1]))]

    # Parent dict if you have it (optional)
    parent_json = {}
    if "parent" in ctx:
        for child, p in ctx["parent"].items():
            parent_json[cell_to_key(child)] = None if p is None else [p[0], p[1]]

    payload = {
        "meta": {
            "time": float(ctx["robot"].getTime()),
        },
        "start_tile": list(ctx.get("start_tile", (0, 0))),
        "start_heading": ctx.get("start_heading"),
        "tile": list(ctx.get("tile", (0, 0))),
        "heading": ctx.get("heading"),
        "visited_tiles": visited_tiles_json,
        "visited_exits": visited_exits_json,
        "opens_map": opens_map_json,
        "parent": parent_json,
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
    BACKTRACK = auto()
    RETURN_TO_START = auto()
    IDLE = auto()


# ----------------------------
# Helpers (sensors + motion)
# ----------------------------
def sector_values(ps_vals):
    # Simple sectors
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
    # Your current tweak factor kept as-is
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
    """
    rel_dir: "front", "left", "right"
    Returns (wall_bool, front_reading_after_probe, ps_vals_after_probe)
    """

    # Rotate to face probe direction
    if rel_dir == "left":
        turn_90(robot, left_motor, right_motor, left_enc, right_enc, timestep, "left", TURN_SPEED)
    elif rel_dir == "right":
        turn_90(robot, left_motor, right_motor, left_enc, right_enc, timestep, "right", TURN_SPEED)

    # Step forward to get closer to potential wall
    drive_distance(robot, left_motor, right_motor, left_enc, right_enc, timestep, PROBE_DIST_M, FWD_SPEED)

    # Read sensors
    ps_vals = [s.getValue() for s in ps]
    front, _, _ = sector_values(ps_vals)

    wall = front >= WALL_SPIKE_THRESH

    # Go back to center
    drive_distance(robot, left_motor, right_motor, left_enc, right_enc, timestep, -PROBE_DIST_M, FWD_SPEED)

    # Rotate back
    if rel_dir == "left":
        turn_90(robot, left_motor, right_motor, left_enc, right_enc, timestep, "right", TURN_SPEED)
    elif rel_dir == "right":
        turn_90(robot, left_motor, right_motor, left_enc, right_enc, timestep, "left", TURN_SPEED)

    return wall, front, ps_vals


def turn_to_dir(robot, ctx, target_dir):
    # Minimal: rotate using 90-degree steps until heading matches
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
            # 180
            turn_90(robot, ctx["left_motor"], ctx["right_motor"], ctx["left_enc"], ctx["right_enc"],
                    ctx["timestep"], "right", TURN_SPEED)
            heading = RIGHT_OF[heading]
            turn_90(robot, ctx["left_motor"], ctx["right_motor"], ctx["left_enc"], ctx["right_enc"],
                    ctx["timestep"], "right", TURN_SPEED)
            heading = RIGHT_OF[heading]

    ctx["heading"] = heading


# ----------------------------
# Behavior Layer Implementations
# ----------------------------
def behavior_probing(ctx):
    """
    Fully implemented.
    Produces:
      ctx["probe_result"] = {"front_wall":bool,"left_wall":bool,"right_wall":bool}
    Then transitions to DECIDE_NEXT (placeholder).
    """
    robot = ctx["robot"]
    ps = ctx["ps"]
    left_motor = ctx["left_motor"]
    right_motor = ctx["right_motor"]
    left_enc = ctx["left_enc"]
    right_enc = ctx["right_enc"]
    timestep = ctx["timestep"]

    fw, f_read, _ = probe_is_wall(robot, ps, left_motor, right_motor, left_enc, right_enc, timestep, "front")
    lw, l_read, _ = probe_is_wall(robot, ps, left_motor, right_motor, left_enc, right_enc, timestep, "left")
    rw, r_read, _ = probe_is_wall(robot, ps, left_motor, right_motor, left_enc, right_enc, timestep, "right")

    ctx["probe_result"] = {
        "front_wall": fw,
        "left_wall": lw,
        "right_wall": rw,
        "front_val": float(f_read),
        "left_val": float(l_read),
        "right_val": float(r_read),
    }

    print(
        "PROBING |",
        "F:", fw, "val", round(f_read, 1),
        "| L:", lw, "val", round(l_read, 1),
        "| R:", rw, "val", round(r_read, 1),
    )

    log_event(ctx, "PROBING", extra={"probe_result": ctx.get("probe_result")})

    # Next behavior (placeholder)
    ctx["state"] = State.DECIDE_NEXT


# Decide-next implementation (drop-in replacement for behavior_decide_next)
# Assumptions:
# - ctx["tile"] = (x,y) current tile center (integers)
# - ctx["heading"] = one of {"N","E","S","W"} current heading
# - ctx["probe_result"] has booleans: front_wall, left_wall, right_wall
# - ctx has: visited_tiles (set), visited_exits (set), stack (list), opens_map (dict)
# - We treat "back" as open (you can return where you came from)
# - We pick randomly among untried open exits

DIRS = ["N","E","S","W"]
DX = {"N":0, "E":1, "S":0, "W":-1}
DY = {"N":1, "E":0, "S":-1, "W":0}
LEFT_OF = {"N":"W", "E":"N", "S":"E", "W":"S"}
RIGHT_OF = {"N":"E", "E":"S", "S":"W", "W":"N"}
BACK_OF = {"N":"S", "E":"W", "S":"N", "W":"E"}

def behavior_decide_next(ctx):
    """
    Implements DFS decision logic (no movement here).
    Output:
      - ctx["chosen_dir"] set to a global direction {"N","E","S","W"} or None
      - ctx["state"] set to MOVE_ONE_TILE or BACKTRACK
    """

    # 1) Mark current tile visited + push to stack if this is first time here
    start = ctx["start_tile"]
    x, y = ctx["tile"]
    cur = (x, y)

    if cur not in ctx["visited_tiles"]:
        ctx["visited_tiles"].add(cur)
        ctx["stack"].append(cur)
        print("DECIDE_NEXT | Visiting new tile:", cur, "| stack now:", ctx["stack"])

    # 2) Get probe results and determine open exits (relative to heading) 
    # (relative) -> open_dirs (global)

    pr = ctx.get("probe_result", {})
    heading = ctx["heading"]

    open_dirs = set()
    if not pr.get("front_wall", False):
        open_dirs.add(heading)
    if not pr.get("left_wall", False):
        open_dirs.add(LEFT_OF[heading])
    if not pr.get("right_wall", False):
        open_dirs.add(RIGHT_OF[heading])

    # back is assumed open (lets DFS backtrack)
    if cur != start:  # no back if we're at start (otherwise we get stuck)
        open_dirs.add(BACK_OF[heading])

    # store opens in map 
    if cur not in ctx["opens_map"]:
        ctx["opens_map"][cur] = open_dirs # write once, trust forever (you can also update if you want)

    # 3) Compute untried exits from this tile 
    untried = [d for d in open_dirs if (cur, d) not in ctx["visited_exits"]]
    PRIORITY = [heading, LEFT_OF[heading], RIGHT_OF[heading], BACK_OF[heading]]  # optional: add some priority to direction choices (e.g. forward > left > right > back)]

    # 4) Decide: take an untried exit if possible else backtrack 
    if untried: 
        chosen = next((d for d in PRIORITY if d in untried), None)  # pick first from priority list that is untried, else random among untried (you can change this logic if you want)))
        ctx["chosen_dir"] = chosen

        # mark the directed exit as visited (so we don't pick it again from this tile)
        ctx["visited_exits"].add((cur, chosen))
        ctx["state"] = State.MOVE_ONE_TILE
        print("DECIDE_NEXT | cur", cur,"| open", sorted(open_dirs), "| untried exits:", untried, "| chosen:", chosen)
        return

    ctx["chosen_dir"] = None
    ctx["state"] = State.BACKTRACK
    print("DECIDE_NEXT | cur", cur,"| open", sorted(open_dirs), "| no untried exits, backtracking")

    log_event(ctx, "DECIDE_NEXT", extra={
        "open": sorted(list(open_dirs)),
        "untried": sorted(list(untried)),
        "chosen": ctx["chosen_dir"]
    })

def behavior_move_one_tile(ctx):
    """
    Execute the chosen direction:
      - turn to chosen_dir
      - drive forward TILE_SIZE_M
      - update ctx["tile"] and ctx["heading"]
      - mark reverse exit visited
    Then go back to PROBING.
    """
    chosen = ctx["chosen_dir"]
    if chosen == None:
        print("MOVE_ONE_TILE | ERROR: chosen_dir is None -> BACKTRACK")
        ctx["state"] = State.BACKTRACK
        return

    robot = ctx["robot"]
    cur = ctx["tile"]
    x, y = cur 

    # Turn to chosen direction (if not already facing it)
    turn_to_dir(robot, ctx, chosen)

    # Drive forward one tile
    drive_distance(robot, ctx["left_motor"], ctx["right_motor"], ctx["left_enc"], ctx["right_enc"], ctx["timestep"], TILE_SIZE_M, FWD_SPEED)

    # Update tile and heading in context
    nxt = (x + DX[chosen], y + DY[chosen])
    ctx["tile"] = nxt
    ctx["heading"] = chosen # after moving, assume we are now facing the direction we moved (you can change this if you want)
    if nxt not in ctx["parent"]:
        ctx["parent"][nxt] = cur  # set parent for backtracking (you can also just use the stack)

    # Mark reverse exit as visited (so we don't immediately backtrack)
    back_dir = BACK_OF[chosen]
    ctx["visited_exits"].add((nxt, back_dir))

    print("MOVE_ON_TILE |",cur,"->",nxt,"| heading now", ctx["heading"])
    log_event(ctx, "MOVE_ONE_TILE", extra={"from": list(cur), "to": list(nxt), "via": chosen})

    # Next behavior (back to probing)
    ctx["state"] = State.PROBING if ctx["tile"] not in ctx["opens_map"] else State.DECIDE_NEXT  # if we already have probe results for this tile (e.g. from backtracking), skip probing"]


def behavior_backtrack(ctx):
    """
    Backtrack one step along the DFS stack:
      - If stack has <= 1, we're done (return to start or idle)
      - Otherwise, move from current tile to parent tile (previous stack element)
    Then go to PROBING.
    """

    robot = ctx["robot"]
    cur = ctx["tile"]

    if not ctx["stack"]:
        print("BACKTRACK | ERROR: stack is empty, going IDLE")
        ctx["state"] = State.IDLE
        return

    # If at start (or only one item), then exploration is done, return to start or idle
    if len(ctx["stack"]) <= 1:
        print("BACKTRACK | at root -> RETURN_TO_START")
        ctx["state"] = State.RETURN_TO_START
        return

    # Pop current node if it's on top of stack 
    if ctx["stack"][-1] == cur:
        print("BACKTRACK | popping current tile from stack:", cur, "| stack:", ctx["stack"])
        ctx["stack"].pop()
        print("BACKTRACK | new stack after pop:", ctx["stack"])

    p = ctx["parent"].get(cur)  # parent map (tile -> parent tile) for backtracking (you can also just use the stack)

    if p is None:
        # We're at start and have no parent, so we can't backtrack. This means we're done exploring. Go idle or return to start.
        ctx["state"] = State.IDLE  # or State.RETURN_TO_START if you implement that later
        return

    # Compute direction to parent (should be in opens_map)
    dx = p[0] - cur[0]
    dy = p[1] - cur[1]
    back_dir = None 
    for d in DIRS:
        if DX[d] == dx and DY[d] == dy:
            back_dir = d
            break

    if back_dir is None:
        # Safest thing to do is stop; something is inconsistent in our map/stack, we don't know where parent is
        print("BACKTRACK | ERROR: parent not adjacent. cur:", cur, "parent:", p, "Cannot determine back direction -> IDLE")
        ctx["state"] = State.IDLE
        return

    # Mark exit to parent as visited (so we don't try it again from current tile)
    if (cur, back_dir) not in ctx["visited_exits"]:
        ctx["visited_exits"].add((cur, back_dir))
    
    # Turn to face parent direction and drive forward one tile to backtrack
    turn_to_dir(robot, ctx, back_dir)
    drive_distance(robot, ctx["left_motor"], ctx["right_motor"], ctx["left_enc"], ctx["right_enc"], ctx["timestep"], TILE_SIZE_M, FWD_SPEED)

    ctx["tile"] = p
    ctx["heading"] = back_dir # after moving, assume we are now facing the direction we moved (you can change this if you want)

    if (p, BACK_OF[back_dir]) not in ctx["visited_exits"]:
        ctx["visited_exits"].add((p, BACK_OF[back_dir]))

    print("BACKTRACK |",cur,"->",p,"| heading now", ctx["heading"])
    log_event(ctx, "BACKTRACK", extra={"from": list(cur), "to": list(p), "via": back_dir})

    ctx["state"] = State.PROBING

 

def behavior_return_to_start(ctx):
    """
    Placeholder.
    Later: return to start (BFS on map or parent-walk).
    """
    print("RETURN_TO_START (placeholder)")
    ctx["state"] = State.IDLE


def behavior_idle(ctx):
    """
    Stop motors. Stay idle.
    """
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

    # Context (shared state across behaviors)
    ctx = {
        "robot": robot,
        "timestep": timestep,
        "left_motor": left_motor,
        "right_motor": right_motor,
        "left_enc": left_enc,
        "right_enc": right_enc,
        "ps": ps,

        # Maze / DFS placeholders (later)
        "start_tile": (0, 0),  
        "tile": (0, 0),
        "heading": "E",
        "visited_tiles": set(),
        "visited_exits": set(),
        "stack": [],
        "opens_map": {},

        # Behavior state
        "state": State.PROBING,
        "probe_result": None,
        "chosen_dir": None,

        "events": []
    }

    ctx["parent"] = {ctx["start_tile"]: None} # parent map for backtracking (tile -> parent tile) (you can also just use the stack)

    # Warm up
    for _ in range(10):
        if robot.step(timestep) == -1:
            return

    print("=== Layered Controller Skeleton ===")
    print("State starts at:", ctx["state"])
    print("PROBE_DIST_M:", PROBE_DIST_M, "| WALL_SPIKE_THRESH:", WALL_SPIKE_THRESH)
    print("FWD_SPEED:", FWD_SPEED, "| TURN_SPEED:", TURN_SPEED)
    print("TILE_SIZE_M (placeholder):", TILE_SIZE_M)

    cycle = 0
    last_dump_t = robot.getTime()
    while robot.step(timestep) != -1:
        cycle += 1

        # Run state actions at a slower cadence to keep output readable
        if cycle % 15 != 0:
            continue

        st = ctx["state"]

        if st == State.PROBING:
            behavior_probing(ctx)
        elif st == State.DECIDE_NEXT:
            behavior_decide_next(ctx)
        elif st == State.MOVE_ONE_TILE:
            behavior_move_one_tile(ctx)
        elif st == State.BACKTRACK:
            behavior_backtrack(ctx)
        elif st == State.RETURN_TO_START:
            behavior_return_to_start(ctx)
        else:
            behavior_idle(ctx)

        t = robot.getTime()
        if t - last_dump_t >= 30.0:   # dump every 30 seconds
            save_dump(ctx)
            last_dump_t = t

    save_dump(ctx)
    print("Saved", DUMP_FILE, "events:", len(ctx["events"]))

if __name__ == "__main__":
    main()
