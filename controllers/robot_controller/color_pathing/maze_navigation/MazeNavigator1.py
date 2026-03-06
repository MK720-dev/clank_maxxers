# -*- coding: utf-8 -*-

"""
Maze Navigation - Grid Mapping + BFS (ASCII-only file)
-----------------------------------------------------
Two-phase approach:

Run 1 (map):
  - Explore the entire reachable maze on a grid using DFS + backtracking
  - Build a wall map per cell (N,E,S,W)
  - Detect goal using camera green threshold (simple)
  - Save to maze_map.json

Run 2 (solve):
  - Load maze_map.json
  - BFS to find shortest path from start to goal
  - Execute path as a sequence of N/E/S/W moves

IMPORTANT:
- This file is ASCII-only (no special punctuation). It will not contain 0x96.
- If your editor forces Windows-1252, this still works because it contains only ASCII.
- You MUST adjust device names and tune thresholds for your robot/world.

Usage:
  py MazeNavigator1_clean.py --phase map
  py MazeNavigator1_clean.py --phase solve
"""

from controller import Robot
import math
import json
import sys
from collections import deque

# ----------------------------
# Parameters (tune these)
# ----------------------------

TIME_STEP = 32

WHEEL_RADIUS_M = 0.0205
AXLE_LENGTH_M = 0.053

CELL_SIZE_M = 0.10

FWD_SPEED = 3.0
TURN_SPEED = 2.0

WALL_THRESHOLD = 80.0

GREEN_RATIO_THRESHOLD = 0.10
GREEN_MIN_G = 120
GREEN_MAX_R = 90
GREEN_MAX_B = 90

MAP_FILENAME = "maze_map.json"

# Device names (change if needed)
LEFT_MOTOR_NAME = "left wheel motor"
RIGHT_MOTOR_NAME = "right wheel motor"
LEFT_ENC_NAME = "left wheel sensor"
RIGHT_ENC_NAME = "right wheel sensor"
CAMERA_NAME = "camera"
PS_NAMES = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]

DIRS = ["N", "E", "S", "W"]
DX = {"N": 0, "E": 1, "S": 0, "W": -1}
DY = {"N": 1, "E": 0, "S": -1, "W": 0}
LEFT_OF = {"N": "W", "W": "S", "S": "E", "E": "N"}
RIGHT_OF = {"N": "E", "E": "S", "S": "W", "W": "N"}
BACK_OF = {"N": "S", "S": "N", "E": "W", "W": "E"}


def cell_key(x, y):
    return str(x) + "," + str(y)


def make_walls():
    return {"N": 0, "E": 0, "S": 0, "W": 0}


def clamp_angle_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class MazeRobot:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.left_motor = self.robot.getDevice(LEFT_MOTOR_NAME)
        self.right_motor = self.robot.getDevice(RIGHT_MOTOR_NAME)
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        self.left_enc = self.robot.getDevice(LEFT_ENC_NAME)
        self.right_enc = self.robot.getDevice(RIGHT_ENC_NAME)
        self.left_enc.enable(self.timestep)
        self.right_enc.enable(self.timestep)

        self.camera = self.robot.getDevice(CAMERA_NAME)
        self.camera.enable(self.timestep)

        self.ps = []
        for name in PS_NAMES:
            s = self.robot.getDevice(name)
            s.enable(self.timestep)
            self.ps.append(s)

        self.prev_l = None
        self.prev_r = None

        # Discrete grid pose
        self.grid_x = 0
        self.grid_y = 0
        self.grid_dir = "E"  # change if your start orientation differs

        # Continuous heading (optional for odom)
        self.theta = 0.0

    def step(self):
        return self.robot.step(self.timestep)

    def set_wheel_speeds(self, left_rad_s, right_rad_s):
        self.left_motor.setVelocity(left_rad_s)
        self.right_motor.setVelocity(right_rad_s)

    def stop(self):
        self.set_wheel_speeds(0.0, 0.0)

    def read_proximity(self):
        return [s.getValue() for s in self.ps]

    def detect_walls_relative(self):
        vals = self.read_proximity()

        # Typical e-puck mapping (tune if needed)
        front = max(vals[0], vals[7]) > WALL_THRESHOLD
        left = max(vals[5], vals[6]) > WALL_THRESHOLD
        right = max(vals[1], vals[2]) > WALL_THRESHOLD

        return {"front": front, "left": left, "right": right}

    def detect_goal_green(self):
        img = self.camera.getImage()
        w = self.camera.getWidth()
        h = self.camera.getHeight()

        stride = 2
        green_count = 0
        total = 0

        for y in range(0, h, stride):
            for x in range(0, w, stride):
                r = self.camera.imageGetRed(img, w, x, y)
                g = self.camera.imageGetGreen(img, w, x, y)
                b = self.camera.imageGetBlue(img, w, x, y)
                total += 1
                if g >= GREEN_MIN_G and r <= GREEN_MAX_R and b <= GREEN_MAX_B:
                    green_count += 1

        ratio = float(green_count) / float(max(1, total))
        return ratio >= GREEN_RATIO_THRESHOLD

    def _turn_in_place(self, direction, angle_rad):
        target = abs(angle_rad)
        turned = 0.0

        self.prev_l = self.left_enc.getValue()
        self.prev_r = self.right_enc.getValue()

        if direction == "left":
            self.set_wheel_speeds(-TURN_SPEED, TURN_SPEED)
        else:
            self.set_wheel_speeds(TURN_SPEED, -TURN_SPEED)

        while self.step() != -1:
            l = self.left_enc.getValue()
            r = self.right_enc.getValue()

            dl = (l - self.prev_l) * WHEEL_RADIUS_M
            dr = (r - self.prev_r) * WHEEL_RADIUS_M
            self.prev_l = l
            self.prev_r = r

            dtheta = (dr - dl) / AXLE_LENGTH_M
            self.theta = clamp_angle_pi(self.theta + dtheta)
            turned += abs(dtheta)

            if turned >= target:
                break

        self.stop()

    def turn_left_90(self):
        self._turn_in_place("left", math.pi / 2.0)
        self.grid_dir = LEFT_OF[self.grid_dir]

    def turn_right_90(self):
        self._turn_in_place("right", math.pi / 2.0)
        self.grid_dir = RIGHT_OF[self.grid_dir]

    def turn_to_dir(self, target_dir):
        while self.grid_dir != target_dir:
            cur_idx = DIRS.index(self.grid_dir)
            tgt_idx = DIRS.index(target_dir)
            diff = (tgt_idx - cur_idx) % 4
            if diff == 1:
                self.turn_right_90()
            elif diff == 3:
                self.turn_left_90()
            else:
                self.turn_right_90()
                self.turn_right_90()

    def move_forward_one_cell(self):
        target_dist = CELL_SIZE_M
        traveled = 0.0

        self.prev_l = self.left_enc.getValue()
        self.prev_r = self.right_enc.getValue()

        self.set_wheel_speeds(FWD_SPEED, FWD_SPEED)

        while self.step() != -1:
            l = self.left_enc.getValue()
            r = self.right_enc.getValue()

            dl = (l - self.prev_l) * WHEEL_RADIUS_M
            dr = (r - self.prev_r) * WHEEL_RADIUS_M
            self.prev_l = l
            self.prev_r = r

            dc = 0.5 * (dl + dr)
            traveled += abs(dc)

            if traveled >= target_dist:
                break

        self.stop()

        self.grid_x += DX[self.grid_dir]
        self.grid_y += DY[self.grid_dir]


def update_cell_walls_from_sensors(walls_map, x, y, grid_dir, rel_walls):
    k = cell_key(x, y)
    if k not in walls_map:
        walls_map[k] = make_walls()

    front_dir = grid_dir
    left_dir = LEFT_OF[grid_dir]
    right_dir = RIGHT_OF[grid_dir]

    items = [
        (front_dir, rel_walls["front"]),
        (left_dir, rel_walls["left"]),
        (right_dir, rel_walls["right"]),
    ]

    for d, blocked in items:
        walls_map[k][d] = 1 if blocked else 0

        nx = x + DX[d]
        ny = y + DY[d]
        nk = cell_key(nx, ny)
        if nk not in walls_map:
            walls_map[nk] = make_walls()
        if blocked:
            walls_map[nk][BACK_OF[d]] = 1


def get_unvisited_open_neighbors(walls_map, visited, x, y):
    k = cell_key(x, y)
    if k not in walls_map:
        walls_map[k] = make_walls()

    out = []
    for d in DIRS:
        if walls_map[k].get(d, 0) == 1:
            continue
        nx = x + DX[d]
        ny = y + DY[d]
        if (nx, ny) not in visited:
            out.append(d)
    return out


def dfs_full_mapping(mr):
    walls_map = {}
    visited = set()
    stack = [(mr.grid_x, mr.grid_y)]
    goal = None

    walls_map[cell_key(mr.grid_x, mr.grid_y)] = make_walls()

    while stack and mr.step() != -1:
        cx = mr.grid_x
        cy = mr.grid_y

        if (cx, cy) not in visited:
            visited.add((cx, cy))

        rel = mr.detect_walls_relative()
        update_cell_walls_from_sensors(walls_map, cx, cy, mr.grid_dir, rel)

        if goal is None and mr.detect_goal_green():
            goal = (cx, cy)

        candidates = get_unvisited_open_neighbors(walls_map, visited, cx, cy)

        if candidates:
            chosen = None
            for d in DIRS:
                if d in candidates:
                    chosen = d
                    break

            nx = cx + DX[chosen]
            ny = cy + DY[chosen]

            mr.turn_to_dir(chosen)
            mr.move_forward_one_cell()
            stack.append((nx, ny))
        else:
            stack.pop()
            if not stack:
                break

            tx, ty = stack[-1]
            ddx = tx - cx
            ddy = ty - cy

            back_dir = None
            for d in DIRS:
                if DX[d] == ddx and DY[d] == ddy:
                    back_dir = d
                    break

            if back_dir is None:
                continue

            mr.turn_to_dir(back_dir)
            mr.move_forward_one_cell()

    return walls_map, goal


def neighbors_from_map(walls_map, x, y):
    k = cell_key(x, y)
    if k not in walls_map:
        return []
    res = []
    for d in DIRS:
        if walls_map[k].get(d, 0) == 1:
            continue
        nx = x + DX[d]
        ny = y + DY[d]
        nk = cell_key(nx, ny)
        if nk in walls_map and walls_map[nk].get(BACK_OF[d], 0) == 1:
            continue
        res.append((nx, ny, d))
    return res


def bfs_shortest_path(walls_map, start, goal):
    q = deque([start])
    prev = {start: None}
    prev_dir = {start: None}

    while q:
        cur = q.popleft()
        if cur == goal:
            break
        x, y = cur
        for nx, ny, d in neighbors_from_map(walls_map, x, y):
            nxt = (nx, ny)
            if nxt in prev:
                continue
            prev[nxt] = cur
            prev_dir[nxt] = d
            q.append(nxt)

    if goal not in prev:
        return None, None

    path_cells = []
    cur = goal
    while cur is not None:
        path_cells.append(cur)
        cur = prev[cur]
    path_cells.reverse()

    actions = []
    for i in range(1, len(path_cells)):
        c = path_cells[i]
        actions.append(prev_dir[c])

    return path_cells, actions


def execute_actions(mr, actions):
    for d in actions:
        mr.turn_to_dir(d)
        mr.move_forward_one_cell()


def save_map(filename, walls_map, goal):
    payload = {
        "walls": walls_map,
        "goal": list(goal) if goal is not None else None,
        "cell_size_m": CELL_SIZE_M,
    }
    with open(filename, "w") as f:
        json.dump(payload, f, indent=2)


def load_map(filename):
    with open(filename, "r") as f:
        payload = json.load(f)
    walls_map = payload["walls"]
    goal = tuple(payload["goal"]) if payload["goal"] is not None else None
    return walls_map, goal


def parse_phase():
    phase = "map"
    for i, a in enumerate(sys.argv):
        if a == "--phase" and i + 1 < len(sys.argv):
            phase = sys.argv[i + 1].strip().lower()
    if phase not in ("map", "solve"):
        phase = "map"
    return phase


def main():
    phase = parse_phase()
    mr = MazeRobot()

    # warm-up steps for sensors
    for _ in range(5):
        if mr.step() == -1:
            return

    if phase == "map":
        print("[RUN 1] Mapping...")
        walls_map, goal = dfs_full_mapping(mr)
        if goal is None:
            print("WARNING: Goal not detected. Saving map anyway.")
        else:
            print("Goal detected at:", goal)
        save_map(MAP_FILENAME, walls_map, goal)
        print("Saved map to", MAP_FILENAME)
        mr.stop()
    else:
        print("[RUN 2] Solving...")
        walls_map, goal = load_map(MAP_FILENAME)
        if goal is None:
            print("ERROR: Goal is missing in map. Run mapping first.")
            mr.stop()
            return
        start = (0, 0)
        path_cells, actions = bfs_shortest_path(walls_map, start, goal)
        if actions is None:
            print("ERROR: BFS could not find a path to goal.")
            mr.stop()
            return
        print("Shortest steps:", len(actions))
        execute_actions(mr, actions)
        print("Executed path.")
        mr.stop()

    while mr.step() != -1:
        mr.stop()


if __name__ == "__main__":
    main()