from controller import Robot
from collections import deque
from enum import Enum, auto
import json
import os

# ----------------------------
# Tunables
# ----------------------------
DUMP_FILE = "maze_dump.json"
FORCE_REMAP = False

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
TURN_90_STEPS = 140
CAMERA_NAME = "camera"

# ----------------------------
# Direction maps
# ----------------------------
DIRS = ["N", "E", "S", "W"]
DX = {"N": 0, "E": 1, "S": 0, "W": -1}
DY = {"N": 1, "E": 0, "S": -1, "W": 0}
LEFT_OF = {"N": "W", "E": "N", "S": "E", "W": "S"}
RIGHT_OF = {"N": "E", "E": "S", "S": "W", "W": "N"}
BACK_OF = {"N": "S", "E": "W", "S": "N", "W": "E"}


class State(Enum):
    PROBING = auto()
    DECIDE_NEXT = auto()
    MOVE_ONE_TILE = auto()
    BACKTRACK = auto()
    RETURN_TO_START = auto()
    IDLE = auto()


# ----------------------------
# JSON helpers
# ----------------------------
def tile_to_key(tile):
    return f"{tile[0]},{tile[1]}"


def key_to_tile(key):
    x, y = key.split(",")
    return (int(x), int(y))


def dump_map(ctx):
    data = {
        "start_tile": list(ctx["start_tile"]),
        "start_heading": ctx["start_heading"],
        "goal_tile": list(ctx["goal_tile"]) if ctx["goal_tile"] is not None else None,
        "goal_dir": ctx["goal_dir"],
        "opens_map": {
            tile_to_key(tile): sorted(list(exits))
            for tile, exits in ctx["graph_map"].items()
        }
    }

    with open(DUMP_FILE, "w") as f:
        json.dump(data, f, indent=2)

    print(f"Saved map to {DUMP_FILE}")


def load_map():
    if not os.path.exists(DUMP_FILE):
        return None

    try:
        with open(DUMP_FILE, "r") as f:
            data = json.load(f)

        opens_map = {
            key_to_tile(k): set(v)
            for k, v in data["opens_map"].items()
        }

        maze = {
            "start_tile": tuple(data["start_tile"]),
            "start_heading": data.get("start_heading", "E"),
            "goal_tile": tuple(data["goal_tile"]) if data.get("goal_tile") is not None else None,
            "goal_dir": data.get("goal_dir"),
            "opens_map": opens_map,
        }
        return maze

    except Exception as e:
        print(f"Failed to load {DUMP_FILE}: {e}")
        return None


# ----------------------------
# Shortest path helpers
# ----------------------------
def shortest_path_dirs(opens_map, start_tile, goal_tile):
    q = deque([start_tile])
    parent = {start_tile: None}
    move_used = {}

    while q:
        cur = q.popleft()

        if cur == goal_tile:
            break

        for d in opens_map.get(cur, []):
            nxt = (cur[0] + DX[d], cur[1] + DY[d])

            if nxt not in parent:
                parent[nxt] = cur
                move_used[nxt] = d
                q.append(nxt)

    if goal_tile not in parent:
        return None

    dirs = []
    cur = goal_tile

    while cur != start_tile:
        dirs.append(move_used[cur])
        cur = parent[cur]

    dirs.reverse()
    return dirs


# ----------------------------
# Color detection
# ----------------------------
def sees_color_wall(camera, color_name):
    image = camera.getImage()
    if image is None:
        return False

    width = camera.getWidth()
    height = camera.getHeight()

    match_count = 0
    total_count = 0

    for x in range(width // 4, 3 * width // 4):
        for y in range(height // 4, 3 * height // 4):
            r = camera.imageGetRed(image, width, x, y)
            g = camera.imageGetGreen(image, width, x, y)
            b = camera.imageGetBlue(image, width, x, y)

            total_count += 1

            if color_name == "red":
                if r > 90 and r > 1.35 * g and r > 1.35 * b:
                    match_count += 1

            elif color_name == "green":
                if g > 90 and g > 1.35 * r and g > 1.20 * b:
                    match_count += 1

    if total_count == 0:
        return False

    return (match_count / total_count) > 0.08


# ----------------------------
# Low level motion
# ----------------------------
def drive_distance(robot, left_motor, right_motor, left_enc, right_enc, timestep, dist_m, speed):
    target = abs(dist_m)
    traveled = 0.0
    prev_l = left_enc.getValue()
    prev_r = right_enc.getValue()
    direction = 1.0 if dist_m >= 0.0 else -1.0

    left_motor.setVelocity(direction * speed)
    right_motor.setVelocity(direction * speed)

    while robot.step(timestep) != -1:
        l, r = left_enc.getValue(), right_enc.getValue()
        dl = (l - prev_l) * WHEEL_RADIUS_M
        dr = (r - prev_r) * WHEEL_RADIUS_M
        prev_l, prev_r = l, r
        traveled += abs(0.5 * (dl + dr))
        if traveled >= target:
            break

    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)


def turn_90(robot, left_motor, right_motor, timestep, turn_dir, speed):
    lv, rv = (-speed, speed) if turn_dir == "left" else (speed, -speed)
    left_motor.setVelocity(lv)
    right_motor.setVelocity(rv)

    for _ in range(TURN_90_STEPS):
        robot.step(timestep)

    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)


def turn_to_dir(robot, ctx, target_dir):
    while ctx["heading"] != target_dir:
        cur_idx = DIRS.index(ctx["heading"])
        tgt_idx = DIRS.index(target_dir)
        diff = (tgt_idx - cur_idx) % 4

        if diff == 1:
            turn_90(robot, ctx["left_motor"], ctx["right_motor"], ctx["timestep"], "right", TURN_SPEED)
            ctx["heading"] = RIGHT_OF[ctx["heading"]]

        elif diff == 3:
            turn_90(robot, ctx["left_motor"], ctx["right_motor"], ctx["timestep"], "left", TURN_SPEED)
            ctx["heading"] = LEFT_OF[ctx["heading"]]

        else:
            for _ in range(2):
                turn_90(robot, ctx["left_motor"], ctx["right_motor"], ctx["timestep"], "right", TURN_SPEED)
            ctx["heading"] = BACK_OF[ctx["heading"]]


def probe_dir(robot, ctx, abs_dir):
    original_heading = ctx["heading"]
    turn_to_dir(robot, ctx, abs_dir)

    robot.step(ctx["timestep"])
    robot.step(ctx["timestep"])

    saw_red = sees_color_wall(ctx["camera"], "red")
    saw_green = sees_color_wall(ctx["camera"], "green")

    drive_distance(
        robot,
        ctx["left_motor"],
        ctx["right_motor"],
        ctx["left_enc"],
        ctx["right_enc"],
        ctx["timestep"],
        PROBE_DIST_M,
        FWD_SPEED
    )

    robot.step(ctx["timestep"])
    robot.step(ctx["timestep"])

    saw_red = saw_red or sees_color_wall(ctx["camera"], "red")
    saw_green = saw_green or sees_color_wall(ctx["camera"], "green")

    ps_vals = [s.getValue() for s in ctx["ps"]]
    reading = max(ps_vals[0], ps_vals[7])
    is_wall = reading >= WALL_SPIKE_THRESH

    drive_distance(
        robot,
        ctx["left_motor"],
        ctx["right_motor"],
        ctx["left_enc"],
        ctx["right_enc"],
        ctx["timestep"],
        -PROBE_DIST_M,
        FWD_SPEED
    )

    turn_to_dir(robot, ctx, original_heading)
    return is_wall, reading, saw_red, saw_green


# ----------------------------
# Mapping behavior
# ----------------------------
def behavior_probing(ctx):
    cur = ctx["tile"]

    if cur in ctx["opens_map"]:
        ctx["state"] = State.DECIDE_NEXT
        return

    traversal_open_exits = set()
    graph_open_exits = set()

    original_heading = ctx["heading"]
    probing_dirs = [RIGHT_OF[original_heading], LEFT_OF[original_heading], original_heading]

    if cur != ctx["start_tile"]:
        graph_open_exits.add(BACK_OF[original_heading])

    for d in probing_dirs:
        is_wall, _, saw_red, saw_green = probe_dir(ctx["robot"], ctx, d)

        if not is_wall:
            traversal_open_exits.add(d)
            graph_open_exits.add(d)

        if is_wall and saw_green and ctx["goal_tile"] is None:
            ctx["goal_tile"] = cur
            ctx["goal_dir"] = d
            print(f"Found green end wall at tile {cur}, dir {d}")

    ctx["opens_map"][cur] = traversal_open_exits
    ctx["graph_map"][cur] = graph_open_exits
    ctx["state"] = State.DECIDE_NEXT


def behavior_decide_next(ctx):
    cur = ctx["tile"]

    if cur not in ctx["visited_tiles"]:
        ctx["visited_tiles"].add(cur)

    open_dirs = ctx["opens_map"][cur]
    ctx["chosen_dir"] = None

    priority_dirs = [d for d in DIRS if d in open_dirs and d != BACK_OF[ctx["heading"]]]

    for d in priority_dirs:
        if (cur, d) not in ctx["visited_exits"]:
            ctx["chosen_dir"] = d
            ctx["visited_exits"].add((cur, d))
            break

    if ctx["chosen_dir"] is None:
        ctx["state"] = State.BACKTRACK
    else:
        ctx["state"] = State.MOVE_ONE_TILE


def behavior_move_one_tile(ctx):
    cur = ctx["tile"]
    chosen = ctx["chosen_dir"]

    turn_to_dir(ctx["robot"], ctx, chosen)
    nxt = (cur[0] + DX[chosen], cur[1] + DY[chosen])

    ctx["stack"].append(nxt)
    ctx["tile"] = nxt

    drive_distance(
        ctx["robot"],
        ctx["left_motor"],
        ctx["right_motor"],
        ctx["left_enc"],
        ctx["right_enc"],
        ctx["timestep"],
        TILE_SIZE_M,
        FWD_SPEED,
    )

    ctx["state"] = State.PROBING


def behavior_backtrack(ctx):
    if len(ctx["stack"]) <= 1:
        ctx["state"] = State.RETURN_TO_START
        return

    current_loc = ctx["stack"].pop()
    parent_loc = ctx["stack"][-1]

    dx = parent_loc[0] - current_loc[0]
    dy = parent_loc[1] - current_loc[1]
    back_dir = next(d for d in DIRS if DX[d] == dx and DY[d] == dy)

    print(f"BACKTRACKING: {current_loc} -> {parent_loc}")

    turn_to_dir(ctx["robot"], ctx, back_dir)

    drive_distance(
        ctx["robot"],
        ctx["left_motor"],
        ctx["right_motor"],
        ctx["left_enc"],
        ctx["right_enc"],
        ctx["timestep"],
        TILE_SIZE_M,
        FWD_SPEED,
    )

    ctx["tile"] = parent_loc
    ctx["state"] = State.DECIDE_NEXT


def behavior_return_to_start(ctx):
    if ctx["tile"] != ctx["start_tile"]:
        ctx["state"] = State.BACKTRACK
        return

    for target_dir in DIRS:
        turn_to_dir(ctx["robot"], ctx, target_dir)
        ctx["robot"].step(ctx["timestep"])
        ctx["robot"].step(ctx["timestep"])

        if sees_color_wall(ctx["camera"], "red"):
            dump_map(ctx)
            print("FINISHED MAPPING")
            ctx["led0"].set(0)
            ctx["state"] = State.IDLE
            return


# ----------------------------
# Path run mode
# ----------------------------
def run_shortest_path(ctx, maze):
    if maze["goal_tile"] is None:
        print("Map file has no goal_tile. Falling back to mapping.")
        return False

    path_dirs = shortest_path_dirs(
        maze["opens_map"],
        maze["start_tile"],
        maze["goal_tile"]
    )

    if path_dirs is None:
        print("No valid path found in JSON. Falling back to mapping.")
        return False

    print("Loaded existing map. Running shortest path:")
    print(path_dirs)

    ctx["tile"] = maze["start_tile"]
    ctx["start_tile"] = maze["start_tile"]
    ctx["heading"] = maze["start_heading"]
    ctx["start_heading"] = maze["start_heading"]

    ctx["robot"].step(ctx["timestep"])
    ctx["robot"].step(ctx["timestep"])

    for d in path_dirs:
        print(f"Move {d}")
        turn_to_dir(ctx["robot"], ctx, d)
        drive_distance(
            ctx["robot"],
            ctx["left_motor"],
            ctx["right_motor"],
            ctx["left_enc"],
            ctx["right_enc"],
            ctx["timestep"],
            TILE_SIZE_M,
            FWD_SPEED,
        )
        ctx["tile"] = (ctx["tile"][0] + DX[d], ctx["tile"][1] + DY[d])

    if maze["goal_dir"] is not None:
        turn_to_dir(ctx["robot"], ctx, maze["goal_dir"])

    ctx["left_motor"].setVelocity(0.0)
    ctx["right_motor"].setVelocity(0.0)
    ctx["led0"].set(0)
    print("DONE")
    return True


# ----------------------------
# Main
# ----------------------------
def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    l_motor = robot.getDevice(LEFT_MOTOR_NAME)
    r_motor = robot.getDevice(RIGHT_MOTOR_NAME)
    l_motor.setPosition(float("inf"))
    r_motor.setPosition(float("inf"))

    l_enc = robot.getDevice(LEFT_ENC_NAME)
    r_enc = robot.getDevice(RIGHT_ENC_NAME)
    l_enc.enable(timestep)
    r_enc.enable(timestep)

    led0 = robot.getDevice("led0")
    led0.set(1)

    camera = robot.getDevice(CAMERA_NAME)
    camera.enable(timestep)

    ps = [robot.getDevice(n) for n in PS_NAMES]
    for s in ps:
        s.enable(timestep)

    ctx = {
        "robot": robot,
        "timestep": timestep,
        "left_motor": l_motor,
        "right_motor": r_motor,
        "left_enc": l_enc,
        "right_enc": r_enc,
        "ps": ps,
        "camera": camera,
        "led0": led0,
        "tile": (0, 0),
        "start_tile": (0, 0),
        "start_heading": "E",
        "heading": "E",
        "opens_map": {},
        "graph_map": {},
        "stack": [(0, 0)],
        "visited_exits": set(),
        "visited_tiles": set(),
        "goal_tile": None,
        "goal_dir": None,
        "state": State.PROBING,
    }

    robot.step(timestep)
    robot.step(timestep)

    if not FORCE_REMAP:
        maze = load_map()
        if maze is not None:
            if run_shortest_path(ctx, maze):
                while robot.step(timestep) != -1:
                    pass
                return

    print("Running mapping mode")

    while robot.step(timestep) != -1:
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
        elif st == State.IDLE:
            break


if __name__ == "__main__":
    main()
