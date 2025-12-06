#!/usr/bin/env pybricks-micropython
# EV3 FINAL — 단순/견고 버전
# - 시작 분기: (0,0) 블록 있으면 1칸 진입→(0,0) '탐지 모드'→픽업→180°→즉시 분류(후퇴 금지)
# - 일반: (0,0) 진입 후 BFS, 모든 타깃 접근의 마지막 hop은 반드시 '탐지 모드'
# - 픽업 직후 1칸 복귀(방금 온 교차로) → 역경로로 (0,0) 복귀 → 분류
# - 분류 직전 (0,0) 헤딩을 W로 정규화
# - 교차로 감지: 좌/우 패턴 + 크리프 확정
# - 드롭은 straight(50) 후 release, straight(-50)로 복귀

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from collections import deque

# ============== HW ==============
ev3   = EV3Brick()
left  = Motor(Port.B)
right = Motor(Port.C)
arm   = Motor(Port.D)
ultra = UltrasonicSensor(Port.S1)
csL   = ColorSensor(Port.S3)
csR   = ColorSensor(Port.S4)
csC   = ColorSensor(Port.S2)
robot = DriveBase(left, right, 55.5, 104)

# ============ CONST ============
N,E,S,W = 1,2,3,4
START   = (-2,0)  # 논리상 시작 좌표(두 칸 E면 (0,0))

# 라인/P제어(미세 D 억제용 0) — 필요시 Kd만 0이거나 아주 작게
BLACK_TH       = 30
BASE_SPEED     = 120
Kp, Kd         = 1.20, 0.00
LPF_ALPHA_REF  = 0.35
LPF_ALPHA_ERR  = 0.50
TURN_MAX       = 160
ERR_DEADBAND   = 2.0
SPEED_MIN      = 80
SPEED_MAX      = 140
ERR_FOR_SLOW   = 15.0

# 교차로 감지(3cm 라인)
WINDOW_SAMPLES = 10
WIDE_MARGIN    = 8
DEBOUNCE_N     = 3
BOTH_BLACK_N   = 3
HOLD_BLACK_N   = 4
BLIP_MIN_N     = 1

# 크리프(후보 확정 모드)
CREEP_SPEED     = 70
CREEP_CONFIRM_N = 2
CREEP_MAX_MS    = 500

# 정렬/대기
SOFT_WAIT_MS        = 15
SENSOR_TO_WHEEL_MM  = 28
SETTLE_MS           = 80

# 탐지/접근(400mm 셀)
SEEK_TRIGGER_MM = 280
STOP_DIST_MM    = 40
BUMP_IN_MM      = 40

# 회전 보정
TURN_GAIN_90  = 1.00
TURN_GAIN_180 = 1.00

# 맵(#=block) — 필요시 교체
MAP = [
    ['.','.','.'],
    ['.','.','.'],
    ['.','.','#'],
]
H, W_ = len(MAP), len(MAP[0])

# ========== DBG ==========
def dbg(msg):     print(msg)
def action(msg):  print("[ACTION] " + str(msg))
def ok(msg="OK"): print("[OK] " + str(msg))
def warn(msg):
    print("[WARN] " + str(msg))
    try: ev3.speaker.beep()
    except: pass

def _announce_zero():
    try: ev3.speaker.say("Zero")
    except: pass

# ===== P-follow =====
_last_err_f = 0.0
_refL_f     = None
_refR_f     = None

def _lpf(prev, x, a):
    if prev is None: return float(x)
    return a*float(x) + (1.0-a)*float(prev)

def _clamp(x, lo, hi):
    return lo if x<lo else hi if x>hi else x

def _adaptive_speed(err_mag):
    if err_mag >= ERR_FOR_SLOW: return SPEED_MIN
    ratio = err_mag/ERR_FOR_SLOW
    return int(SPEED_MAX - (SPEED_MAX-SPEED_MIN)*ratio)

def p_follow(v_base=BASE_SPEED):
    global _last_err_f, _refL_f, _refR_f
    L = csL.reflection(); R = csR.reflection()
    _refL_f = _lpf(_refL_f, L, LPF_ALPHA_REF)
    _refR_f = _lpf(_refR_f, R, LPF_ALPHA_REF)
    err_raw = _refL_f - _refR_f
    err_f   = _lpf(_last_err_f, err_raw, LPF_ALPHA_ERR)
    derr    = err_f - _last_err_f
    _last_err_f = err_f
    if abs(err_f) < ERR_DEADBAND:
        err_f = 0.0; derr = 0.0
    turn = Kp*err_f + Kd*derr
    turn = _clamp(turn, -TURN_MAX, TURN_MAX)
    v_cmd = _adaptive_speed(abs(err_f))
    v_cmd = min(v_cmd, v_base)
    robot.drive(v_cmd, turn)
    wait(10)

# ===== 교차로 감지 =====
hist_L_black=[]; hist_R_black=[]; hist_L_val=[]; hist_R_val=[]; hist_both=[]
def hist_append(buf,val,maxlen):
    buf.append(val)
    if len(buf)>maxlen: del buf[0]

def update_hist():
    L = csL.reflection(); R = csR.reflection()
    lb = (L<BLACK_TH); rb = (R<BLACK_TH)
    hist_append(hist_L_val, L, WINDOW_SAMPLES)
    hist_append(hist_R_val, R, WINDOW_SAMPLES)
    hist_append(hist_L_black, lb, WINDOW_SAMPLES)
    hist_append(hist_R_black, rb, WINDOW_SAMPLES)
    hist_append(hist_both, (lb and rb), WINDOW_SAMPLES)

def _max_run_true(seq):
    run=0; best=0
    for v in seq:
        if v:
            run += 1
            if run>best: best=run
        else:
            run = 0
    return best

def _count_true(seq):
    c=0
    for v in seq:
        if v: c+=1
    return c

def intersection_flags():
    seq=0; both_ok=False
    for v in hist_both:
        if v:
            seq += 1
            if seq >= BOTH_BLACK_N:
                both_ok=True; break
        else:
            seq=0
    temporal_ok = (any(hist_L_black) and any(hist_R_black))
    if len(hist_L_val) < WINDOW_SAMPLES:
        wide_ok = False
    else:
        avgL = sum(hist_L_val)/len(hist_L_val)
        avgR = sum(hist_R_val)/len(hist_R_val)
        wide_ok = (avgL < BLACK_TH + WIDE_MARGIN and avgR < BLACK_TH + WIDE_MARGIN)
    holdL = (_max_run_true(hist_L_black) >= HOLD_BLACK_N)
    holdR = (_max_run_true(hist_R_black) >= HOLD_BLACK_N)
    blipL = (_count_true(hist_L_black)    >= BLIP_MIN_N)
    blipR = (_count_true(hist_R_black)    >= BLIP_MIN_N)
    asym_ok = (holdL and blipR) or (holdR and blipL)
    return both_ok, temporal_ok, wide_ok, asym_ok

def dir_name(d):
    if d == N: return "N"
    if d == E: return "E"
    if d == S: return "S"
    if d == W: return "W"
    return str(d)

# ===== 1칸 전진(soft)
def forward_one_cell_soft(heading=None, tag=None):
    if tag is not None:
        if heading is not None:
            action("one_cell_soft " + str(tag) + " dir=" + dir_name(heading))
        else:
            action("one_cell_soft " + str(tag))
    else:
        action("one_cell_soft")

    consec=0
    del hist_L_black[:]; del hist_R_black[:]
    del hist_L_val[:];   del hist_R_val[:]
    del hist_both[:]

    candidate=False; creep_cnt=0; creep_ms=0

    while True:
        if not candidate:
            p_follow(BASE_SPEED)
        else:
            p_follow(CREEP_SPEED)
            creep_ms += 10

        update_hist()
        both_ok, temporal_ok, wide_ok, asym_ok = intersection_flags()

        if not candidate and (both_ok or temporal_ok or wide_ok or asym_ok):
            candidate=True; creep_cnt=0; creep_ms=0
            ok("X-cand")

        if candidate:
            if both_ok or wide_ok or asym_ok or temporal_ok:
                creep_cnt += 1
            if creep_cnt >= CREEP_CONFIRM_N:
                try: ev3.speaker.beep()
                except: pass
                wait(SOFT_WAIT_MS)
                ok("1cell")
                return
            if creep_ms >= CREEP_MAX_MS:
                warn("X-cancel")
                candidate=False; consec=0
                del hist_L_black[:]; del hist_R_black[:]
                del hist_L_val[:];   del hist_R_val[:]
                del hist_both[:]
                continue

        if both_ok or wide_ok:
            consec += 1
        else:
            consec = 0
        if consec >= DEBOUNCE_N:
            try: ev3.speaker.beep()
            except: pass
            wait(SOFT_WAIT_MS)
            ok("1cell")
            return

# ===== 탐지모드(마지막 hop)
def detect_mode_until_grab():
    action("detect_mode")
    while True:
        p_follow(BASE_SPEED)
        d=ultra.distance()
        if d is not None and d<=SEEK_TRIGGER_MM:
            robot.stop(); ok("seek_hit"); break
    t=0
    while True:
        d=ultra.distance()
        if d is not None and d<=STOP_DIST_MM:
            robot.stop(); ok("approach_hit"); break
        p_follow(70); t+=10
        if t>6000:
            robot.stop(); warn("approach_timeout"); break
    robot.straight(BUMP_IN_MM)
    arm.run_until_stalled(200, then=Stop.COAST, duty_limit=50)
    wait(120); ok("grabbed")

# ===== 정렬/턴
def align_fwd():
    action("align_fwd")
    robot.straight(SENSOR_TO_WHEEL_MM)
    robot.stop(); wait(SETTLE_MS)
    ok("aligned_fwd")

def precise_turn(angle_deg):
    if angle_deg==90 or angle_deg==-90:
        robot.turn(angle_deg*TURN_GAIN_90)
    elif angle_deg==180 or angle_deg==-180:
        robot.turn(angle_deg*TURN_GAIN_180)
    else:
        robot.turn(angle_deg)
    robot.stop(); wait(50)

def precise_turn_180():
    action("turn180")
    precise_turn(220)
    ok("turned_180")

def turn_min(now_dir, target_dir):
    diff=(target_dir-now_dir)%4
    if diff==0:
        ok("no_turn"); return target_dir
    angle = 110 if diff==1 else -110 if diff==3 else 220
    action("turn_to_"+dir_name(target_dir))
    precise_turn(angle)
    ok("turned_"+str(angle))
    return target_dir

def dir_from_to(a,b):
    ax,ay=a; bx,by=b
    if bx==ax+1 and by==ay: return E
    if bx==ax-1 and by==ay: return W
    if by==ay+1 and bx==ax: return S
    if by==ay-1 and bx==ax: return N
    return None

# ===== 경로 실행(마지막 hop은 무조건 '탐지 모드')
def exec_path_soft_with_turns(path, now_dir, last_hop_detect=True):
    if not path or len(path)==1: return path[-1], now_dir, False
    dbg("[PATH] "+str(path)+" last_detect="+str(last_hop_detect))
    did_retreat=False
    for i in range(1,len(path)):
        a=path[i-1]; b=path[i]
        is_last = (i==len(path)-1) and last_hop_detect
        next_dir = dir_from_to(a,b)

        if next_dir!=now_dir:
            align_fwd()
            now_dir = turn_min(now_dir, next_dir)

        if is_last:
            detect_mode_until_grab()
            precise_turn_180()
            if   now_dir==N: now_dir=S
            elif now_dir==S: now_dir=N
            elif now_dir==E: now_dir=W
            else:            now_dir=E
            forward_one_cell_soft(now_dir, "retreat_after_pick")
            did_retreat=True
            ok("detect_hop_done dir="+dir_name(now_dir))
        else:
            forward_one_cell_soft(now_dir)
    return path[-1], now_dir, did_retreat

# ===== BFS
def bfs(start,goal,pass_goal=True):
    sx,sy=start; gx,gy=goal
    dist=[[None]*W_ for _ in range(H)]
    prev=[[None]*W_ for _ in range(H)]
    q=deque([(sx,sy)]); dist[sy][sx]=0
    while q:
        x,y=q.popleft()
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
            nx=x+dx; ny=y+dy
            if not(0<=nx<W_ and 0<=ny<H): continue
            if (nx,ny)==(gx,gy):
                if dist[ny][nx] is None and pass_goal:
                    dist[ny][nx]=dist[y][x]+1; prev[ny][nx]=(x,y); q.append((nx,ny))
                continue
            if MAP[ny][nx]=='.' and dist[ny][nx] is None:
                dist[ny][nx]=dist[y][x]+1; prev[ny][nx]=(x,y); q.append((nx,ny))
    if dist[gy][gx] is None: return []
    path=[]; p=(gx,gy)
    while p: path.append(p); p=prev[p[1]][p[0]]
    path.reverse(); return path

# ===== back_path 실행 보정
def run_back_path_ensure_origin(back_path, now_dir):
    if not back_path:
        return (0,0), now_dir
    if len(back_path) >= 2:
        now, now_dir, _ = exec_path_soft_with_turns(back_path, now_dir, last_hop_detect=False)
        return now, now_dir
    forward_one_cell_soft(now_dir, "finish_return_to_00")
    return back_path[0], now_dir

# ===== (0,0) 진입/특수
def enter_field_to_cell00():
    action("enter_field")
    forward_one_cell_soft(E, "to_entrance")  # (−2,0)→(−1,0)
    forward_one_cell_soft(E, "to_cell00")    # (−1,0)→(0,0)
    ok("(0,0)"); _announce_zero()
    return (0,0), E

def start_special_try_cell00(timeout_ms=2500):
    action("special_cell00")
    forward_one_cell_soft(E, "special_enter")  # (−2,0)→(−1,0)
    t=0
    while True:
        p_follow(BASE_SPEED); update_hist()
        d=ultra.distance()
        if d is not None and d<=SEEK_TRIGGER_MM:
            robot.stop(); ok("special_seek"); break
        t+=10
        if t>=timeout_ms:
            robot.stop(); warn("special_timeout"); return False
    detect_mode_until_grab()
    precise_turn_180()          # 헤딩 W
    ok("special_done")
    _announce_zero()
    return True                 # 후퇴 없이 즉시 분류

# ===== 색상/존 스크립트
def detect_color_RB():
    r,g,b=csC.rgb()
    col='R' if r>b else 'B'
    dbg("[COLOR] "+str((r,g,b))+" -> "+col)
    return col

def zone_script_red(now_dir):
    dbg("[ZONE] RED (start W @ (0,0))")
    forward_one_cell_soft(W, "Z_RED W1")                 # follow(R) 상황
    align_fwd(); now_dir = turn_min(W, S)
    forward_one_cell_soft(S, "Z_RED S1")                 # follow(L)
    align_fwd(); now_dir = turn_min(S, W)
    forward_one_cell_soft(W, "Z_RED W1-2")               # follow(L)
    action("drop_red")
    robot.straight(50); arm.run_until_stalled(-200, then=Stop.COAST, duty_limit=50); robot.straight(-50)
    robot.stop(); precise_turn_180(); now_dir=E
    forward_one_cell_soft(E, "Z_RED E1")                 # follow(R)
    align_fwd(); now_dir = turn_min(E, N)
    forward_one_cell_soft(N, "Z_RED N1")                 # follow(L)
    align_fwd(); now_dir = turn_min(N, E)
    forward_one_cell_soft(E, "Z_RED E1-2")               # follow(L)
    ok("zone_red_done"); return E

def zone_script_blue(now_dir):
    dbg("[ZONE] BLUE (start W @ (0,0))")
    forward_one_cell_soft(W, "Z_BLUE W1")                # follow(R)
    align_fwd(); now_dir = turn_min(W, S)
    forward_one_cell_soft(S, "Z_BLUE S1")                # follow(L)
    forward_one_cell_soft(S, "Z_BLUE S2")                # follow(L)
    align_fwd(); now_dir = turn_min(S, W)
    forward_one_cell_soft(W, "Z_BLUE W1-2")              # follow(L)
    action("drop_blue")
    robot.straight(50); arm.run_until_stalled(-200, then=Stop.COAST, duty_limit=50); robot.straight(-50)
    robot.stop(); precise_turn_180(); now_dir=E
    forward_one_cell_soft(E, "Z_BLUE E1")                # follow(R)
    align_fwd(); now_dir = turn_min(E, N)
    forward_one_cell_soft(N, "Z_BLUE N1")                # follow(L)
    align_fwd(); now_dir = turn_min(N, E)
    forward_one_cell_soft(E, "Z_BLUE E1-2")              # follow(L)
    ok("zone_blue_done"); return E

# ============ MAIN ============
def run():
    dbg("RUN FINAL")
    try: ev3.speaker.beep(); ev3.speaker.beep(); ev3.speaker.beep()
    except: pass

    action("release")
    arm.run_until_stalled(-200, then=Stop.COAST, duty_limit=50)

    now=START; now_dir=E

    # 시작 분기
    if MAP[0][0] == '#':
        handled = start_special_try_cell00(timeout_ms=2500)
        if not handled:
            now, now_dir = enter_field_to_cell00()
        else:
            color = detect_color_RB()
            if now_dir != W:
                align_fwd(); now_dir = turn_min(now_dir, W)
            if color=='R': now_dir = zone_script_red(W)
            else:          now_dir = zone_script_blue(W)
            MAP[0][0] = '.'
            now, now_dir = (0,0), E
    else:
        now, now_dir = enter_field_to_cell00()

    # 일반 루프
    while True:
        targets=[(x,y) for y in range(H) for x in range(W_) if MAP[y][x]=='#']
        dbg("targets="+str(targets))
        if not targets: break

        best_path=None; best_goal=None
        for t in targets:
            p=bfs(now,t)
            if p and (best_path is None or len(p)<len(best_path)):
                best_path=p; best_goal=t
        dbg("[BFS] path="+str(best_path)+" goal="+str(best_goal))

        now, now_dir, did_retreat = exec_path_soft_with_turns(best_path, now_dir, last_hop_detect=True)

        # 픽업 후 색 판별
        color = detect_color_RB()

        # 역순 복귀(픽업 직후 1칸 복귀를 했으면 첫 hop 스킵)
        back_path = list(reversed(best_path))
        if did_retreat and len(back_path) >= 2:
            back_path = back_path[1:]
        dbg("[BACK] path="+str(back_path))
        now, now_dir = run_back_path_ensure_origin(back_path, now_dir)
        _announce_zero()

        # 분류 시작 전에 W로 표준화
        if now_dir != W:
            align_fwd(); now_dir = turn_min(now_dir, W)

        if color=='R': now_dir = zone_script_red(W)
        else:          now_dir = zone_script_blue(W)

        now=(0,0); now_dir=E
        gx,gy=best_goal; MAP[gy][gx]='.'
        ok("cleared "+str(best_goal))

    dbg("FINISH")

if __name__=="__main__":
    run()
