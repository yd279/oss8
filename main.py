#!/usr/bin/env pybricks-micropython
# EV3 FINAL — P-only, Every-Intersection Wheel-On-Top (straight(20))
# - 모든 교차로: stop → beep → straight(CROSS_ON_MM)로 바퀴 올리기
# - 마지막 hop만 초음파 탐지/픽업→180°→1칸 복귀
# - (0,0) 특수: START→E 1칸→탐지→180°→즉시 색상영역(복귀 없음)
# - 좌표×방향→follow(L/R) 매핑: 사용자 최종표 반영

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait
from pybricks.robotics import DriveBase 
from collections import deque

# ================= HW =================
ev3   = EV3Brick()
left  = Motor(Port.B)
right = Motor(Port.C)
arm   = Motor(Port.D)
ultra = UltrasonicSensor(Port.S1)
csL   = ColorSensor(Port.S3)   # 왼쪽
csR   = ColorSensor(Port.S4)   # 오른쪽
csC   = ColorSensor(Port.S2)   # 중앙(색)
robot = DriveBase(left, right, 55.5, 104)

# =============== CONST ===============
N,E,S,W = 1,2,3,4
START   = (-2,0)

# P 제어 (심플)
P_SPEED       = 140
KP            = 1.2
TURN_CLAMP    = 140
DEAD_BAND     = 2.0
WHITE_LATCH_N = 10
RECOVER_TURN  = 0     # 요청대로 꺼둠(단순화)

# 교차로 확정(심플)
HOLD_N    = 6      # 상대 센서 연속 흑 프레임 수
BOTH_N    = 6      # 양쪽 흑 연속 프레임 수(+)에서 보조
TICK_MS   = 10
CROSS_ON_MM = 20   # << 모든 교차로에서 바퀴 올리기 거리

# 회전 보정
TURN_GAIN_90       = 1.00
TURN_GAIN_180      = 1.00

# 탐지/접근
SEEK_TRIGGER_MM = 120
STOP_DIST_MM    = 40
BUMP_IN_MM      = 40

# ====== MAP 예시 ======
MAP = [
    ['#','.','#'],
    ['.','#','.'],
    ['.','.','#'],
]
H, W_ = len(MAP), len(MAP[0])

# ============== LOG ==============
def dbg(msg):     print(msg)
def action(msg):  print("[ACTION] " + str(msg))
def ok(msg="OK"): print("[OK] " + str(msg))
def warn(msg):    print("[WARN] " + str(msg))

def dir_name(d):  return "NESW"[d-1] if d in (N,E,S,W) else str(d)
def say_zero():
    try: ev3.speaker.say("Zero")
    except: pass

# ===== 좌표×방향 → follow(L/R) (최종표) =====
_FMAP = {
 (0,0):{N:"L", E:"L", S:"R", W:"R"},
 (1,0):{N:"R", E:"L", S:"L", W:"R"},
 (2,0):{N:"R", E:"L", S:"L", W:"R"},

 (0,1):{N:"L", E:"R", S:"R", W:"L"},
 (1,1):{N:"R", E:"R", S:"L", W:"L"},
 (2,1):{N:"R", E:"R", S:"L", W:"L"},

 (0,2):{N:"L", E:"R", S:"R", W:"L"},
 (1,2):{N:"R", E:"R", S:"L", W:"L"},
 (2,2):{N:"R", E:"R", S:"L", W:"L"},
}
def follow_side(x,y,heading):
    return _FMAP.get((x,y), {N:"L",E:"L",S:"R",W:"R"}).get(heading, "L")

# ===== 임계값 자동 보정(1회) =====
_thresh_L = None
_thresh_R = None
def auto_threshold_once():
    global _thresh_L, _thresh_R
    if _thresh_L is not None: return
    samplesL, samplesR = [], []
    for _ in range(20):
        samplesL.append(csL.reflection()); samplesR.append(csR.reflection()); wait(TICK_MS)
    samplesL.sort(); samplesR.sort()
    _thresh_L = samplesL[len(samplesL)//2]
    _thresh_R = samplesR[len(samplesR)//2]
    dbg("[THR] L="+str(_thresh_L)+" R="+str(_thresh_R))

# ===== P-제어 1칸: 교차로 확정 → stop+beep+straight(CROSS_ON_MM) =====
def follow_one_cell_P(side="L"):
    auto_threshold_once()
    white_run_primary=0
    hold_other=0; both_hold=0

    while True:
        L = csL.reflection(); R = csR.reflection()
        Lb = (L < _thresh_L); Rb = (R < _thresh_R)

        # P 제어(기준 센서만 사용)
        err = (L - _thresh_L) if side=="L" else (R - _thresh_R)
        if abs(err) < DEAD_BAND: err = 0.0
        turn = KP * err
        if turn >  TURN_CLAMP: turn =  TURN_CLAMP
        if turn < -TURN_CLAMP: turn = -TURN_CLAMP

        # (옵션) 기준센서가 흰색 오래 → 재획득 편향(OFF)
        if RECOVER_TURN != 0:
            primary_black = Lb if side=="L" else Rb
            if primary_black:
                white_run_primary = 0
            else:
                white_run_primary += 1
                if white_run_primary >= WHITE_LATCH_N:
                    turn += (-RECOVER_TURN if side=="L" else RECOVER_TURN)
                    white_run_primary = WHITE_LATCH_N

        robot.drive(P_SPEED, turn)
        wait(TICK_MS)

        # 교차로 확정 (상대센서 또는 양쪽동시)
        other_black = Rb if side=="L" else Lb
        hold_other = hold_other+1 if other_black else 0
        both_hold  = both_hold+1  if (Lb and Rb) else 0

        if hold_other >= HOLD_N or both_hold >= BOTH_N:
            robot.stop()
            try: ev3.speaker.beep()
            except: pass
            # 모든 교차로에서 바퀴 올리기
            robot.straight(CROSS_ON_MM)
            ok("1cell")
            return

# ===== 1칸 전진 래퍼 =====
def forward_one_cell_soft(heading, pos, side_override=None, tag=None):
    side = side_override if side_override else follow_side(pos[0], pos[1], heading)
    if tag: action("one_cell_soft "+str(tag)+" dir="+dir_name(heading)+" side="+side)
    else:   action("one_cell_soft dir="+dir_name(heading)+" side="+side)
    follow_one_cell_P(side)

# ===== 회전 (정렬 없이 그 자리에서 회전) =====
def precise_turn(angle_deg):
    if angle_deg in (90,-90):
        robot.turn(angle_deg*TURN_GAIN_90)
    elif angle_deg in (180,-180):
        robot.turn(angle_deg*TURN_GAIN_180)
    else:
        robot.turn(angle_deg)
    robot.stop(); wait(50)

def turn_to(now_dir, target_dir):
    if now_dir==target_dir: return now_dir
    diff=(target_dir-now_dir)%4
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

# ===== 마지막 hop 탐지/픽업 =====
def detect_mode_until_grab():
    action("detect_mode")
    # 트리거까지 천천히
    while True:
        robot.drive(90, 0)
        d=ultra.distance()
        if d is not None and d<=SEEK_TRIGGER_MM:
            robot.stop(); ok("seek_hit"); break
        wait(10)
    # 근접 정지
    t=0
    while True:
        d=ultra.distance()
        if d is not None and d<=STOP_DIST_MM:
            robot.stop(); ok("approach_hit"); break
        robot.drive(60, 0); wait(10); t+=10
        if t>5000:
            robot.stop(); warn("approach_timeout"); break
    # bump + grab
    robot.straight(BUMP_IN_MM)
    arm.run_until_stalled(200, then=Stop.COAST, duty_limit=50)
    wait(120); ok("grabbed")

def precise_turn_180():
    action("turn180")
    precise_turn(220)
    ok("turned_180")

# ===== BFS =====
def bfs(start,goal,pass_goal=True):
    sx,sy=start; gx,gy=goal
    dist=[[None]*W_ for _ in range(H)]
    prev=[[None]*W_ for _ in range(H)]
    q=deque([(sx,sy)]); dist[sy][sx]=0
    while q:
        x,y=q.popleft()
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
            nx,ny=x+dx,y+dy
            if not(0<=nx<W_ and 0<=ny<H): continue
            if (nx,ny)==(gx,gy):
                if dist[ny][nx] is None and pass_goal:
                    dist[ny][nx]=dist[y][x]+1; prev[ny][nx]=(x,y); q.append((nx,ny))
                continue
            if MAP[ny][nx]=='.' and dist[ny][nx] is None:
                dist[ny][nx]=dist[y][x]+1; prev[ny][nx]=(x,y); q.append((nx,ny))
    if dist[gy][gx] is None: return []
    path=[]; p=(gx,gy)
    while p:
        path.append(p); p=prev[p[1]][p[0]]
    path.reverse(); return path

# ===== 경로 실행(마지막 hop만 탐지) — 모든 교차로 바퀴 올리기 적용됨
def exec_path_soft_with_turns(path, now_pos, now_dir, last_hop_detect=False):
    if not path or len(path)==1: return now_pos, now_dir, False
    dbg("[PATH] "+str(path)+" last_detect="+str(last_hop_detect))
    did_retreat=False

    for i in range(1,len(path)):
        a=path[i-1]; b=path[i]
        need_detect = (last_hop_detect and i==len(path)-1)
        next_dir = dir_from_to(a,b)

        if next_dir!=now_dir:
            now_dir = turn_to(now_dir, next_dir)

        if need_detect:
            # b 셀로 접근/픽업
            detect_mode_until_grab()
            precise_turn_180()
            # 1칸 복귀: a로 되돌아옴
            back_dir = N if now_dir==S else S if now_dir==N else W if now_dir==E else E
            forward_one_cell_soft(back_dir, pos=b, tag="retreat_after_pick")
            now_pos = a
            now_dir = back_dir
            did_retreat=True
            ok("detect_hop_done dir="+dir_name(now_dir))
        else:
            forward_one_cell_soft(now_dir, pos=a)
            now_pos = b

    return now_pos, now_dir, did_retreat

# ===== (0,0) 진입 =====
def enter_to_00_from_start():
    pos=START; heading=E
    forward_one_cell_soft(heading, pos=pos, tag="to_-1,0"); pos=(-1,0)
    forward_one_cell_soft(heading, pos=pos, tag="to_0,0");  pos=(0,0)
    ok("(0,0)"); say_zero()
    return pos, heading

# ===== 색판별 =====
def detect_color_RB():
    r,g,b = csC.rgb()
    col='R' if r>b else 'B'
    dbg("[COLOR] "+str((r,g,b))+" -> "+col)
    return col

# ===== 색상영역 스크립트 =====
def zone_script_red():
    # 시작: (0,0), 헤딩 W
    action("Z_RED")
    forward_one_cell_soft(W, (0,0), side_override="R", tag="Z_RED W1")
    now_dir = turn_to(W,S)
    forward_one_cell_soft(S, (0,0), side_override="L", tag="Z_RED S1")
    now_dir = turn_to(S,W)
    forward_one_cell_soft(W, (0,1), side_override="L", tag="Z_RED W1-2")
    robot.straight(50); arm.run_until_stalled(-200, then=Stop.COAST, duty_limit=50); robot.straight(-50)
    precise_turn_180()   # → E
    forward_one_cell_soft(E, (-1,1), side_override="R", tag="Z_RED E1")
    now_dir = turn_to(E,N)
    forward_one_cell_soft(N, (0,1), side_override="L", tag="Z_RED N1")
    now_dir = turn_to(N,E)
    forward_one_cell_soft(E, (0,0), side_override="L", tag="Z_RED E1-2")
    ok("zone_red_done"); return E

def zone_script_blue():
    # 시작: (0,0), 헤딩 W
    action("Z_BLUE")
    forward_one_cell_soft(W, (0,0), side_override="R", tag="Z_BLUE W1")
    now_dir = turn_to(W,S)
    forward_one_cell_soft(S, (0,0), side_override="L", tag="Z_BLUE S1")
    forward_one_cell_soft(S, (0,1), side_override="L", tag="Z_BLUE S2")
    now_dir = turn_to(S,W)
    forward_one_cell_soft(W, (0,2), side_override="L", tag="Z_BLUE W1-2")
    robot.straight(50); arm.run_until_stalled(-200, then=Stop.COAST, duty_limit=50); robot.straight(-50)
    precise_turn_180()   # → E
    forward_one_cell_soft(E, (-1,2), side_override="R", tag="Z_BLUE E1")
    now_dir = turn_to(E,N)
    forward_one_cell_soft(N, (0,2), side_override="L", tag="Z_BLUE N1")
    forward_one_cell_soft(N, (0,1), side_override="L", tag="Z_BLUE N2")
    now_dir = turn_to(N,E)
    forward_one_cell_soft(E, (0,0), side_override="L", tag="Z_BLUE E1-2")
    ok("zone_blue_done"); return E

# ============== MAIN ==============
def run():
    dbg("RUN FINAL")
    try:
        ev3.speaker.beep(); ev3.speaker.beep(); ev3.speaker.beep()
    except: pass

    # 집게 열기
    action("release")
    arm.run_until_stalled(-200, then=Stop.COAST, duty_limit=50)

    now_pos=START; now_dir=E

    # 시작: (0,0) 특수
    if MAP[0][0]=='#':
        forward_one_cell_soft(E, now_pos, tag="to_-1,0"); now_pos=(-1,0)
        detect_mode_until_grab()
        precise_turn_180()  # → W
        color = detect_color_RB()
        if color=='R': now_dir = zone_script_red()
        else:          now_dir = zone_script_blue()
        say_zero()
        MAP[0][0]='.'
        now_pos=(0,0); now_dir=E
    else:
        now_pos, now_dir = enter_to_00_from_start()

    # 메인 루프
    while True:
        targets=[(x,y) for y in range(H) for x in range(W_) if MAP[y][x]=='#']
        dbg("targets="+str(targets))
        if not targets: break

        # 최단 경로 선택
        best_path=None; best_goal=None
        for t in targets:
            p=bfs(now_pos,t)
            if p and (best_path is None or len(p)<len(best_path)):
                best_path=p; best_goal=t
        dbg("[BFS] path="+str(best_path)+" goal="+str(best_goal))

        # 접근: 마지막 hop만 탐지
        now_pos, now_dir, _ = exec_path_soft_with_turns(best_path, now_pos, now_dir, last_hop_detect=True)

        # 색 판별
        color = detect_color_RB()

        # 역복귀: goal 제거(이미 1칸 복귀해서 a에 있음)
        back_rev = list(reversed(best_path))      # [goal, ..., (0,0)]
        back_path = back_rev[1:] if back_rev else []
        dbg("[BACK] path="+str(back_path))

        for i in range(1, len(back_path)):
            a=back_path[i-1]; b=back_path[i]
            nd = dir_from_to(a,b)
            if nd!=now_dir:
                now_dir = turn_to(now_dir, nd)
            forward_one_cell_soft(now_dir, pos=a)
            now_pos=b

        # (0,0) 복귀
        say_zero()

        # (0,0)에서 색상영역 시작: 이미 W면 턴 생략
        if now_dir != W:
            now_dir = turn_to(now_dir, W)

        # 색상 영역 수행
        if color=='R': now_dir = zone_script_red()
        else:          now_dir = zone_script_blue()

        # 상태 갱신
        now_pos=(0,0); now_dir=E
        gx,gy=best_goal; MAP[gy][gx]='.'
        ok("cleared "+str(best_goal))

    # 종료: START로 복귀(W 2칸)
    action("return_to_START")
    forward_one_cell_soft(W, (0,0))
    forward_one_cell_soft(W, (-1,0))
    robot.stop()
    dbg("FINISH")

if __name__=="__main__":
    run()
