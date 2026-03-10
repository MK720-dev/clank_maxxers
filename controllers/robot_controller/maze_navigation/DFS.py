"""
This code generates a perfect maze and then explores it using a depth-first search (DFS) algorithm. 
The maze is represented as a grid of cells, where each cell can have passages in the four cardinal directions (N, E, S, W). 
The DFS algorithm visits each cell and marks the exits it has taken to avoid revisiting them. 
Finally, it checks if all cells were visited and prints the results.
This code is a simplified version and the testbed of the maze exploration process for the second competition.
"""
import random

DIRS = ["N","E","S","W"]
DX = {"N":0,"E":1,"S":0,"W":-1}
DY = {"N":1,"E":0,"S":-1,"W":0}
BACK = {"N":"S","S":"N","E":"W","W":"E"}

def generate_connected_maze(W, H, seed=0, p_extra=0.15):
    rng = random.Random(seed)
    opens = {(x,y): set() for x in range(W) for y in range(H)}

    def in_bounds(x,y):
        return 0 <= x < W and 0 <= y < H

    def connect(a, b, d):
        # connect a -> b in direction d (and b back to a)
        opens[a].add(d)
        opens[b].add(BACK[d])

    # 1) Perfect maze (DFS carving) => connected
    start = (0,0)
    visited = {start}
    stack = [start]

    def neighbors(cell):
        x, y = cell
        out = []
        for d in DIRS:
            nx, ny = x + DX[d], y + DY[d]
            if in_bounds(nx, ny):
                out.append((d, (nx, ny)))
        return out

    while stack:
        cur = stack[-1]
        unvisited = [(d,nxt) for (d,nxt) in neighbors(cur) if nxt not in visited]
        if not unvisited:
            stack.pop()
            continue
        d, nxt = rng.choice(unvisited)
        connect(cur, nxt, d)
        visited.add(nxt)
        stack.append(nxt)

    # 2) Add extra openings with probability p_extra (keeps connected, adds loops)
    for x in range(W):
        for y in range(H):
            a = (x,y)
            for d in ["E","N"]:  # avoid double-adding (only right and up)
                nx, ny = x + DX[d], y + DY[d]
                if not in_bounds(nx, ny):
                    continue
                b = (nx, ny)
                # if wall exists between a and b, sometimes remove it (add connection)
                if d not in opens[a] and rng.random() < p_extra:
                    connect(a, b, d)

    def get_open_dirs(cell):
        return opens.get(cell, set())

    return start, (W-1, H-1), get_open_dirs, set(opens.keys())

def dfs_explore(start, get_open_dirs):
    visited_tiles = set()
    visited_exits = set()
    parent = {start: None}
    stack = [start]

    while stack:
        cur = stack[-1]
        visited_tiles.add(cur)

        open_dirs = get_open_dirs(cur)

        chosen = None
        for d in DIRS:
            if d in open_dirs and ((cur, d) not in visited_exits):
                chosen = d
                break

        if chosen is not None:
            visited_exits.add((cur, chosen))
            nxt = (cur[0] + DX[chosen], cur[1] + DY[chosen])
            visited_exits.add((nxt, BACK[chosen]))
            if nxt not in parent:
                parent[nxt] = cur
            stack.append(nxt)
        else:
            stack.pop()

    return visited_tiles, visited_exits, parent

if __name__ == "__main__":
    start, goal, get_open_dirs, ground_truth_cells = generate_connected_maze(10, 6, seed=42)
    visited_tiles, visited_exits, parent = dfs_explore((0,0), get_open_dirs)

    print("Ground truth cells:", sorted(ground_truth_cells))
    print("Visited tiles:", sorted(visited_tiles))
    print("Visited count:", len(visited_tiles), "expected:", len(ground_truth_cells))

    missing = ground_truth_cells - visited_tiles
    extra = visited_tiles - ground_truth_cells
    print("Missing:", missing)
    print("Extra:", extra)

    # Check exit revisits (basic sanity)
    # Each undirected corridor should produce 2 directed exits visited eventually.
    print("Visited exits count:", len(visited_exits))

