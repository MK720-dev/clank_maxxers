# soccer_controller_full.py
# Main variant: Kickoff rush + goal-lock + wall-arc escape + flank + shove + search + defend.
# Uses Webots Python API: Camera, Lidar, Motor (velocity mode).
# Includes debug prints for diagnostics (color counts, ball counts, lidar, triggers, etc).

from controller import Robot, Camera, Lidar, DistanceSensor
import math

# ===== Configurable device names =====
DEVICE = {
    "camera": "camera",
    "lidar":   "LDS-01",
    "ds_left":  None,    # optional if no lidar
    "ds_front": None,
    "ds_right": None,
    "wheels": ["front_left_wheel", "front_right_wheel", "back_left_wheel", "back_right_wheel"]
}

# ===== Tunable constants =====
MAX_SPEED = 10.0  # motor max velocity (UNSPECIFIED, set to your robot's max)
TIME_STEP = 64

# Vision
BALL_STRIDE_X = 4; BALL_STRIDE_Y = 4
GOAL_STRIDE_X = 6; GOAL_STRIDE_Y = 6
BALL_MIN_COUNT = 3       # low threshold (ball only yellow)
BALL_SEEN_ONCE_COUNT = 3
BALL_CLOSE_COUNT = 200
FORCE_ATTACK_GOAL_COLOR = None
DEFAULT_ATTACK_GOAL_COLOR = "cyan"

# Speeds and gains
RUSH_SPEED = 0.98 * MAX_SPEED
DRIBBLE_SPEED = 1.0 * MAX_SPEED
K_TURN_CHASE   = 1.25
K_TURN_DRIBBLE = 0.95
GOAL_BLEND_CLOSE = 0.55

# Wall avoidance
WALL_ENTER_FRONT = 0.14
WALL_ENTER_SIDE  = 0.12
WALL_EXIT_FRONT_MULT = 2.0
WALL_EXIT_SIDE_MULT  = 1.4
K_WALL_SIDE = 0.30
K_WALL_FRONT_SIDE = 0.15
CORRIDOR_DIST = 0.18

WALL_BACK_SPEED = 0.60 * MAX_SPEED
WALL_ARC_SPEED  = 0.55 * MAX_SPEED
WALL_BACK_STEER = 0.45 * MAX_SPEED
WALL_ARC_STEER  = 0.60 * MAX_SPEED
WALL_BACK_MIN = 4
WALL_BACK_MAX = 16
WALL_ARC_MIN  = 6
WALL_ARC_MAX  = 22

# Deadlock/flank/shove
OBSTACLE_FRONT_DIST = 0.24
STALL_DCX_PX = 6.0
STALL_DCOUNT = 40
STALL_DFRONT_M = 0.02
STALL_DECAY_FAST = 1.5
STALL_DECAY_SLOW = 0.5
FLANK_TRIGGER_BASE = 7.0
FLANK_TRIGGER_GAIN = 0.015
FLANK_BASE = 0.82 * MAX_SPEED
STEER_MAX_FLANK = 0.28 * MAX_SPEED
W_GOAL = 0.80
W_OPEN = 0.20
SHOVE_TRIGGER_DELTA = 6.0
SHOVE_CLEAR_FRONT = 0.32
SHOVE_REV_SPEED = -0.72 * MAX_SPEED
SHOVE_FWD_SPEED = 1.00 * MAX_SPEED
SHOVE_REV_BUDGET_MIN = 3
SHOVE_REV_BUDGET_MAX = 25
SHOVE_FWD_BUDGET_MIN = 3
SHOVE_FWD_BUDGET_MAX = 30
SHOVE_STOP_STEPS = 1

# Ball lost / search
BALL_LOST_ENTER_EVIDENCE = 10.0
BALL_LOST_DECAY = 2.0
BALL_LOST_GROW  = 1.0
SEARCH_STOP_STEPS = 1
SEARCH_SPIN_SPEED = 0.35 * MAX_SPEED
SEARCH_360_STEPS = 180    # tune for ~360°
SEARCH_ARC_BASE = 0.5 * MAX_SPEED
SEARCH_ARC_TURN = 0.5 * MAX_SPEED
REORIENT_MAX_STEPS = 35
REORIENT_TURN_SPEED = 0.5 * MAX_SPEED
REORIENT_CENTER_TOL = 0.15

# Defensive reset: (go to own goal) - for simplicity, we'll just stop
DEFEND_TIMEOUT = 50  # UNSPECIFIED count of steps to defend

DEBUG = True
PRINT_N = 10

# Color thresholds
Y_R_MIN = 70; Y_G_MIN = 70; Y_B_MAX = 190; Y_DELTA = 20

def clamp(x, lo, hi): return max(lo, min(hi, x))
def norm_err_x(cx, w):
    center = (w-1)*0.5
    return (cx - center) / center

def is_yellow(r,g,b):
    return (r>=Y_R_MIN and g>=Y_G_MIN and b<=Y_B_MAX and r>=b+Y_DELTA and g>=b+Y_DELTA)
def is_cyan_loose(r,g,b):
    return (g>80 and b>80 and g>r+10 and b>r+10)
def is_magenta_loose(r,g,b):
    return (r>80 and b>80 and r>g+10 and b>g+10)
def is_cyan_strict(r,g,b):
    return (g>140 and b>140 and r<120 and g>r+30 and b>r+30)
def is_magenta_strict(r,g,b):
    return (r>140 and b>140 and g<120 and r>g+30 and b>g+30)

def scan_blob(image, width, pred, sx, sy, y0, y1):
    cnt=0; sumx=0
    for y in range(y0, y1, sy):
        for x in range(0, width, sx):
            r=Camera.imageGetRed(image,width,x,y)
            g=Camera.imageGetGreen(image,width,x,y)
            b=Camera.imageGetBlue(image,width,x,y)
            if pred(r,g,b):
                cnt+=1; sumx+=x
    if cnt==0: return 0,None
    return cnt, sumx/float(cnt)

def lidar_min(distances, a,b):
    a=max(0,min(len(distances)-1,a))
    b=max(0,min(len(distances),b))
    if b<=a: return float('inf')
    m=float('inf')
    for d in distances[a:b]:
        if not math.isinf(d) and d<m: m=d
    return m

def lidar_5(dist):
    n=len(dist)
    left  = lidar_min(dist, 0, n//5)
    fl    = lidar_min(dist, n//5, 2*n//5)
    front = lidar_min(dist, 2*n//5, 3*n//5)
    fr    = lidar_min(dist, 3*n//5, 4*n//5)
    right = lidar_min(dist, 4*n//5, n)
    return left,fl,front,fr,right

robot = Robot()
ts = int(robot.getBasicTimeStep()) or TIME_STEP

cam = robot.getDevice(DEVICE["camera"]); cam.enable(ts)
W,H = cam.getWidth(), cam.getHeight()

lid = None; use_lid=False
try:
    lid = robot.getDevice(DEVICE["lidar"]); lid.enable(ts); use_lid=True
except: use_lid=False

# DS fallback (UNSPECIFIED mapping)
use_ds=False
if not use_lid:
    try:
        if DEVICE["ds_left"] and DEVICE["ds_front"] and DEVICE["ds_right"]:
            dsL=robot.getDevice(DEVICE["ds_left"]); dsF=robot.getDevice(DEVICE["ds_front"]); dsR=robot.getDevice(DEVICE["ds_right"])
            dsL.enable(ts); dsF.enable(ts); dsR.enable(ts)
            use_ds=True
    except: use_ds=False

wheels=[]
for n in DEVICE["wheels"]:
    m=robot.getDevice(n); m.setPosition(float('inf')); m.setVelocity(0.0); wheels.append(m)
def set_speed(l,r):
    l=clamp(l,-MAX_SPEED,MAX_SPEED); r=clamp(r,-MAX_SPEED,MAX_SPEED)
    wheels[0].setVelocity(l); wheels[2].setVelocity(l)
    wheels[1].setVelocity(r); wheels[3].setVelocity(r)

def read_prox():
    if use_lid: return lidar_5(lid.getRangeImage())
    if use_ds:
        eps=1e-6
        return (1.0/(dsL.getValue()+eps), 1.0/(dsF.getValue()+eps),
                1.0/(dsF.getValue()+eps), 1.0/(dsF.getValue()+eps), 1.0/(dsR.getValue()+eps))
    return float('inf'),float('inf'),float('inf'),float('inf'),float('inf')

def wall_rep(left,fl,fr,right):
    eps=1e-6
    ls = (1.0/(left+eps)) + K_WALL_FRONT_SIDE*(1.0/(fl+eps))
    rs = (1.0/(right+eps)) + K_WALL_FRONT_SIDE*(1.0/(fr+eps))
    return (ls - rs) * K_WALL_SIDE * MAX_SPEED

def choose_dir(left,fl,fr,right):
    eps=1e-6
    lb = (1.0/(left+eps))  + 0.7*(1.0/(fl+eps))
    rb = (1.0/(right+eps)) + 0.7*(1.0/(fr+eps))
    return 1.0 if lb>rb else -1.0

attack_goal = FORCE_ATTACK_GOAL_COLOR
goal_lock_pending = (attack_goal is None)
first_image=False
mode = "PLAY"
ball_seen_once=False
ball_lost_evidence=0.0
stall_evidence=0.0
wall_dir=1.0; back_steps=0; arc_steps=0
search_stop=0; search_spin=0; search_dir=1.0; reorient_steps=0
flank_steps=0
shove_rev=shove_stop=shove_fwd=0
prev_ball_cx=None; prev_ball_count=None; prev_front=None
last_goal_err=0.0
defend_steps = 0
step_i=0

while robot.step(ts) != -1:
    step_i+=1; t=robot.getTime()
    image = cam.getImage()
    if image and not first_image:
        first_image=True

    left_d, fl_d, front_d, fr_d, right_d = read_prox()
    rep = wall_rep(left_d, fl_d, fr_d, right_d)

    # First-frame goal lock
    if goal_lock_pending and first_image:
        cyan_n,_ = scan_blob(image, W, is_cyan_loose, GOAL_STRIDE_X, GOAL_STRIDE_Y, 0, int(0.75*H))
        mag_n,_  = scan_blob(image, W, is_magenta_loose, GOAL_STRIDE_X, GOAL_STRIDE_Y, 0, int(0.75*H))
        attack_goal = DEFAULT_ATTACK_GOAL_COLOR if (cyan_n==0 and mag_n==0) else ("cyan" if cyan_n>mag_n else "magenta")
        goal_lock_pending=False
        if DEBUG:
            print(f"[{t:7.3f}s] GOAL_LOCK first frame: cyan={cyan_n}, magenta={mag_n}, attack={attack_goal}")

    # Ball detection
    ball_count=0; ball_cx=None
    if first_image:
        ball_count, ball_cx = scan_blob(image, W, is_yellow, BALL_STRIDE_X, BALL_STRIDE_Y, int(0.3*H), H)
    ball_seen = (ball_count>=BALL_MIN_COUNT and ball_cx is not None)
    if ball_count >= BALL_SEEN_ONCE_COUNT:
        ball_seen_once = True
    ball_close = (ball_count >= BALL_CLOSE_COUNT)

    # Goal centroid
    goal_err=0.0; goal_visible=False
    if first_image and (attack_goal in ("cyan","magenta")):
        pred = is_cyan_strict if attack_goal=="cyan" else is_magenta_strict
        gcount,gcx = scan_blob(image, W, pred, GOAL_STRIDE_X, GOAL_STRIDE_Y, 0, H)
        if gcount>=35 and gcx is not None:
            goal_err = norm_err_x(gcx, W)
            goal_visible=True
            last_goal_err=goal_err

    # Defensive reset logic: if ball never seen or lost for long
    if defend_steps>0:
        # Defense mode: simply stop (or move to goal, if coordinates known)
        set_speed(0.0, 0.0)
        defend_steps -= 1
        if defend_steps<=0:
            mode="PLAY"
        continue

    # Wall entry check
    frontish=min(front_d,fl_d,fr_d); sideish=min(left_d,right_d)
    if (frontish < WALL_ENTER_FRONT or sideish < WALL_ENTER_SIDE) and mode not in ("WALL_BACKOFF","WALL_ARCOUT"):
        wall_dir = choose_dir(left_d,fl_d,fr_d,right_d)
        sev_front = clamp((WALL_ENTER_FRONT-frontish)/WALL_ENTER_FRONT, 0.0, 1.0)
        sev_side  = clamp((WALL_ENTER_SIDE - sideish)/WALL_ENTER_SIDE, 0.0, 1.0)
        sev = max(sev_front, sev_side)
        back_steps = int(WALL_BACK_MIN + sev*(WALL_BACK_MAX - WALL_BACK_MIN))
        arc_steps  = int(WALL_ARC_MIN + sev*(WALL_ARC_MAX - WALL_ARC_MIN))
        mode="WALL_BACKOFF"
        if DEBUG:
            print(f"[{t:7.3f}s] WALL_ESCAPE enter: sev={sev:.2f}, dir={wall_dir:+.0f}, back={back_steps}, arc={arc_steps}")

    # Ball-lost evidence (gated)
    if first_image and ball_seen_once:
        if ball_seen:
            ball_lost_evidence = max(0.0, ball_lost_evidence - BALL_LOST_DECAY)
        else:
            ball_lost_evidence += BALL_LOST_GROW

    # Enter search only if ball_seen_once
    if mode=="PLAY" and ball_seen_once and ball_lost_evidence > BALL_LOST_ENTER_EVIDENCE:
        mode="SEARCH_STOP"; search_stop=SEARCH_STOP_STEPS
        search_dir = 10 if left_d<right_d else -1.0
        if DEBUG:
            print(f"[{t:7.3f}s] SEARCH enter: lostEv={ball_lost_evidence:.1f}, spinDir={search_dir:+.0f}")

    # ----- State handling -----
    if mode=="WALL_BACKOFF":
        base = -WALL_BACK_SPEED
        steer = wall_dir*WALL_BACK_STEER + rep
        set_speed(base+steer, base-steer)
        back_steps -= 10
        if back_steps<=0: mode="WALL_ARCOUT"
        continue

    if mode=="WALL_ARCOUT":
        corridor = (left_d<CORRIDOR_DIST and right_d<CORRIDOR_DIST)
        base = WALL_ARC_SPEED * (0.6 if corridor else 1.0)
        steer = wall_dir*WALL_ARC_STEER + rep
        set_speed(base+steer, base-steer)
        arc_steps -= 10
        clear_front = (frontish > WALL_ENTER_FRONT*WALL_EXIT_FRONT_MULT)
        clear_side  = (sideish > WALL_ENTER_SIDE*WALL_EXIT_SIDE_MULT)
        if arc_steps<=0 or (clear_front and clear_side):
            mode="PLAY"
            if DEBUG:
                print(f"[{t:7.3f}s] WALL_ESCAPE exit: front={front_d:.3f} sideL={left_d:.3f} sideR={right_d:.3f}")
        continue

    if mode=="SEARCH_STOP":
        set_speed(0.0, 0.0)
        search_stop -= 10
        if DEBUG and search_stop==SEARCH_STOP_STEPS-1:
            print(f"[{t:7.3f}s] SEARCH_STOP")
        if search_stop<=0:
            mode="SEARCH_360"; search_spin=SEARCH_360_STEPS
        continue

    if mode=="SEARCH_360":
        if ball_seen:
            mode="PLAY"; ball_lost_evidence=0.0
            if DEBUG: print(f"[{t:7.3f}s] SEARCH found during spin")
            continue
        near_wall = (frontish < OBSTACLE_FRONT_DIST) or (sideish < WALL_ENTER_SIDE)
        if near_wall:
            mode="SEARCH_ARC"; continue
        turn = search_dir * SEARCH_SPIN_SPEED
        set_speed(+turn, -turn)
        search_spin -= 1
        if DEBUG and search_spin % 20 == 0:
            print(f"[{t:7.3f}s] SEARCH_360 remaining={search_spin}")
        if search_spin<=0:
            # Fail: ball still not found after full spin
            mode="DEFEND"; defend_steps = DEFEND_TIMEOUT
            if DEBUG: print(f"[{t:7.3f}s] SEARCH_FAILED: going to DEFEND")
        continue

    if mode=="SEARCH_ARC":
        if ball_seen:
            mode="PLAY"; ball_lost_evidence=0.0
            if DEBUG: print(f"[{t:7.3f}s] SEARCH found during arc")
            continue
        turn_dir = 1.0 if left_d<right_d else -1.0
        base = SEARCH_ARC_BASE
        steer = turn_dir*SEARCH_ARC_TURN + rep
        set_speed(base+steer, base-steer)
        if (frontish>OBSTACLE_FRONT_DIST*1.6) and (sideish>WALL_ENTER_SIDE*1.2):
            mode="SEARCH_360"
        continue

    if mode=="REORIENT_GOAL":
        if ball_seen:
            mode="PLAY"; ball_lost_evidence=0.0; continue
        if goal_visible and abs(goal_err)<REORIENT_CENTER_TOL:
            mode="PLAY"; ball_lost_evidence=0.0
            if DEBUG: print(f"[{t:7.3f}s] REORIENT done")
            continue
        dir = (1.0 if goal_err>0 else -1.0) if goal_visible else (1.0 if right_d>left_d else -1.0)
        turn = dir * REORIENT_TURN_SPEED
        set_speed(+turn, -turn)
        reorient_steps -= 1
        if reorient_steps<=0:
            mode="PLAY"; ball_lost_evidence=0.0
        continue

    # Deadlock evidence (PLAY)
    front_blocked = (frontish < OBSTACLE_FRONT_DIST)
    if ball_seen and ball_close and front_blocked:
        dcx    = abs(ball_cx - prev_ball_cx)   if prev_ball_cx is not None else 999.0
        dcount = abs(ball_count - prev_ball_count) if prev_ball_count is not None else 999.0
        dfront = abs(front_d - prev_front)      if prev_front is not None else 999.0
        lowchg = (dcx<STALL_DCX_PX) and (dcount<STALL_DCOUNT) and (dfront<STALL_DFRONT_M)
        stall_evidence = stall_evidence+1.0 if lowchg else max(0.0, stall_evidence-STALL_DECAY_SLOW)
    else:
        stall_evidence = max(0.0, stall_evidence-STALL_DECAY_FAST)

    flank_th = max(1.0, FLANK_TRIGGER_BASE - FLANK_TRIGGER_GAIN*(ball_count - BALL_CLOSE_COUNT))
    shove_th = flank_th + SHOVE_TRIGGER_DELTA

    if stall_evidence > shove_th:
        mode="SHOVE_REV"
        shove_rev = int(clamp(3+stall_evidence, SHOVE_REV_BUDGET_MIN, SHOVE_REV_BUDGET_MAX))
        shove_fwd = int(clamp(4+1.5*stall_evidence, SHOVE_FWD_BUDGET_MIN, SHOVE_FWD_BUDGET_MAX))
        shove_stop = SHOVE_STOP_STEPS
        if DEBUG: print(f"[{t:7.3f}s] SHOVE_TRIGGER ev={stall_evidence:.1f} revB={shove_rev} fwdB={shove_fwd}")

    if mode=="SHOVE_REV":
        open_bias = (fr_d - fl_d)/(fr_d+fl_d+1e-6)
        goal_bias = goal_err if goal_visible else last_goal_err
        bias = clamp(0.70*goal_bias + 0.30*open_bias, -1.0, 1.0)
        steer = bias*(0.18*MAX_SPEED) + rep
        set_speed(SHOVE_REV_SPEED+steer, SHOVE_REV_SPEED-steer)
        shove_rev -= 1
        if frontish>SHOVE_CLEAR_FRONT or shove_rev<=0:
            mode="SHOVE_STOP"
        continue

    if mode=="SHOVE_STOP":
        set_speed(0.0, 0.0)
        shove_stop -= 1
        if shove_stop<=0: mode="SHOVE_FWD"
        continue

    if mode=="SHOVE_FWD":
        open_bias = (fr_d - fl_d)/(fr_d+fl_d+1e-6)
        goal_bias = goal_err if goal_visible else last_goal_err
        bias = clamp(0.70*goal_bias + 0.30*open_bias, -1.0, 1.0)
        steer = bias*(0.22*MAX_SPEED) + rep
        set_speed(SHOVE_FWD_SPEED+steer, SHOVE_FWD_SPEED-steer)
        shove_fwd -= 1
        if (not ball_close) or (not front_blocked) or shove_fwd<=0:
            mode="PLAY"; stall_evidence=0.0
            if DEBUG: print(f"[{t:7.3f}s] SHOVE_EXIT")
        continue

    if mode=="FLANK":
        open_bias = (fr_d - fl_d)/(fr_d+fl_d+1e-6)
        goal_bias = goal_err if goal_visible else last_goal_err
        bias = clamp(W_GOAL*goal_bias + W_OPEN*open_bias, -1.0, 1.0)
        steer = clamp(bias*STEER_MAX_FLANK, -STEER_MAX_FLANK, STEER_MAX_FLANK) + rep
        set_speed(FLANK_BASE+steer, FLANK_BASE-steer)
        flank_steps -= 1
        if (not front_blocked) or (not ball_close) or flank_steps<=0:
            mode="PLAY"; stall_evidence=0.0
            if DEBUG: print(f"[{t:7.3f}s] FLANK_EXIT")
        continue

    if stall_evidence > flank_th and mode=="PLAY":
        mode="FLANK"
        flank_steps = int(clamp(6 + 2.0*stall_evidence, 6, 35))
        if DEBUG: print(f"[{t:7.3f}s] FLANK_TRIGGER ev={stall_evidence:.1f} goalErr={last_goal_err:+.2f} budget={flank_steps}")

    # PLAY (kickoff/driving)
    if ball_seen:
        err = norm_err_x(ball_cx, W)
        if ball_close and goal_visible:
            steer = (1.0-GOAL_BLEND_CLOSE)*(K_TURN_DRIBBLE*err*MAX_SPEED) + GOAL_BLEND_CLOSE*(goal_err*MAX_SPEED)
            base = DRIBBLE_SPEED
        elif ball_close:
            steer = K_TURN_DRIBBLE*err*MAX_SPEED; base = DRIBBLE_SPEED
        else:
            steer = K_TURN_CHASE*err*MAX_SPEED; base = RUSH_SPEED
    else:
        steer=0.0; base=RUSH_SPEED

    if left_d<CORRIDOR_DIST and right_d<CORRIDOR_DIST:
        base*=0.65
    steer += rep
    set_speed(base+steer, base-steer)

    if ball_seen: prev_ball_cx = ball_cx; prev_ball_count = ball_count
    prev_front = front_d

    if DEBUG and step_i % PRINT_N == 0:
        print(f"[{t:7.3f}s] TRACE mode={mode} ball={ball_count:4d} close={ball_close} front={front_d:.3f} "
              f"L={left_d:.3f} R={right_d:.3f} stallEv={stall_evidence:.1f} lostEv={ball_lost_evidence:.1f}")
