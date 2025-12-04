#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Color
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from collections import deque

# ========================= 장치 설정 =========================
ev3 = EV3Brick()
left_motor  = Motor(Port.B)
right_motor = Motor(Port.C)
arm_motor   = Motor(Port.D)
ultra_sensor   = UltrasonicSensor(Port.S1)
left_sensor    = ColorSensor(Port.S3)
right_sensor   = ColorSensor(Port.S4)
color_detector = ColorSensor(Port.S2)
robot = DriveBase(left_motor, right_motor, 55.5, 104)

# ========================= 상수/파라미터 ======================
N, E, S, W = 1,2,3,4

# 2센서 PID 파라미터
BLACK_TH   = 30
DEBOUNCE_N = 6
PID_SPEED  = 120
Kp         = 1.4
Kd         = 0.20
CELL_OVERSHOOT_MS     = 230
POST_TURN_STRAIGHT_MS = 260

# (0,0) 근접 탐지용
STOP_DIST_MM   = 60
APPROACH_SPEED = 70

# ========================= MAP (예시) ========================
# '#' = 물체, '.' = 빈칸  (실행 전 세팅해서 사용)
MAP = [
    [".", "#", "."],
    [".", ".", "."],
    [".", ".", "."]
]
H, W = len(MAP), len(MAP[0])

# ========================= 유틸 =============================
def log(msg):
    print(msg)

def print_map():
    print("=== MAP ===")
    for r in MAP:
        print(" ".join(r))
    print("")

# ========================= BFS ==============================
def bfs(start, goal):
    dist = [[None]*W for _ in range(H)]
    prev = [[None]*W for _ in range(H)]
    q = deque([start])
    dist[start[1]][start[0]] = 0

    while q:
        x, y = q.popleft()
        for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
            nx, ny = x+dx, y+dy
            if 0 <= nx < W and 0 <= ny < H and MAP[ny][nx]=='.' and dist[ny][nx] is None:
                dist[ny][nx] = dist[y][x]+1
                prev[ny][nx] = (x,y)
                q.append((nx,ny))

    if dist[goal[1]][goal[0]] is None:
        return [], dist

    path = []
    p = goal
    while p:
        path.append(p)
        p = prev[p[1]][p[0]]
    return path[::-1], dist

# ========================= 2-센서 PID ========================
_last_err = 0
def pid_follow(speed):
    global _last_err
    err  = left_sensor.reflection() - right_sensor.reflection()
    turn = Kp*err + Kd*(err - _last_err)
    _last_err = err
    robot.drive(speed, turn)
    wait(10)

def on_black_both():
    return left_sensor.reflection() < BLACK_TH and right_sensor.reflection() < BLACK_TH

def follow_line_one_cell():
    log("[CELL] 1칸 이동 (PID)")
    consec = 0
    while True:
        pid_follow(PID_SPEED)
        if on_black_both():
            consec += 1
        else:
            consec = 0
        if consec >= DEBOUNCE_N:
            break
    robot.stop()
    robot.drive(100,0); wait(CELL_OVERSHOOT_MS)
    robot.stop()

# ========================= 회전/이동 =========================
def precise_turn_180():
    robot.stop(); wait(150)
    robot.turn(92);  robot.stop(); wait(80)
    robot.turn(88);  robot.stop(); wait(80)

def turn_min(now_dir, target_dir):
    diff  = (target_dir - now_dir) % 4
    angle = [0,90,180,-90][diff]
    robot.stop(); wait(150)
    if angle == 180:
        precise_turn_180()
    else:
        robot.turn(angle); robot.stop(); wait(150)
    return target_dir

def move_manhatten(start, goal, now_dir):
    log("[MOVE] {} -> {}".format(start, goal))
    x,y = start
    gx,gy = goal
    if gx != x:
        target = E if gx>x else W
        now_dir = turn_min(now_dir, target)
        for _ in range(abs(gx-x)):
            follow_line_one_cell()
            x += 1 if target==E else -1
    if gy != y:
        target = S if gy>y else N
        now_dir = turn_min(now_dir, target)
        for _ in range(abs(gy-y)):
            follow_line_one_cell()
            y += 1 if target==S else -1
    return (x,y), now_dir

# ========================= 1-센서 추종 세트 ===================
# (팀원 하드코딩과 동일 계열)  ← (0,0) 전용 처리에서 사용
def left_line_following(speed, kp):
    threshold = 40
    err = left_sensor.reflection() - threshold
    robot.drive(speed, kp*err)

def right_line_following(speed, kp):
    threshold = 40
    err = right_sensor.reflection() - threshold
    robot.drive(speed, kp*err)

def n_move(n, direction="right"):
    for _ in range(n):
        if direction == "right":
            while right_sensor.reflection() > 40:
                left_line_following(100, 1.2)
            while right_sensor.reflection() <= 40:
                right_line_following(100, 1.2)
        else:
            while left_sensor.reflection() > 40:
                right_line_following(100, 1.2)
            while left_sensor.reflection() <= 40:
                left_line_following(100, 0.8)
    robot.stop()

# ========================= 집게/색상 =========================
def grab():
    log("[GRAB] 집기")
    arm_motor.run_until_stalled(200, then=Stop.COAST, duty_limit=50)

def release():
    log("[RELEASE] 놓기")
    arm_motor.run_until_stalled(-200, then=Stop.COAST, duty_limit=50)

def detect_color_simple_rb():
    r,g,b = color_detector.rgb()
    log("[COLOR] RGB = {},{},{}".format(r,g,b))
    if r > b: return 'R'
    if b > r: return 'B'
    return 'U'

# ========================= (0,0) 전용 분류 동선 =================
# (너가 명시한 정확한 시나리오대로)  — (0,0)에서 서쪽(W)을 보게 한 뒤 시작
def classify_from_cell00(color, now_dir):
    # 시작을 W 로 강제
    now_dir = turn_min(now_dir, W)

    def one():  n_move(1, direction="left")    # 좌측 라인 기준 1칸
    def two():  one(); one()

    # 앞으로 한칸 → 좌회전(S) → 전진 → 우회전(W) → 전진 → 투입
    one()
    now_dir = turn_min(now_dir, S)

    if color == 'R':
        one()
    elif color == 'B':
        two()
    else:
        # 알 수 없으면 안전하게 즉시 리턴
        log("[WARN] Unknown color at (0,0). Abort classify.")
        return now_dir

    now_dir = turn_min(now_dir, W)
    one()

    # straight(50) 넣고 release 이후 straight(-50)
    robot.straight(50);  release(); robot.straight(-50)

    # 180도 턴
    now_dir = turn_min(now_dir, E)

    # 복귀(역순)
    one()
    now_dir = turn_min(now_dir, S)
    if color == 'R':
        one()
    else:
        two()
    now_dir = turn_min(now_dir, E)
    one()

    # 최종 (0,0) 도달 가정
    return now_dir

# ========================= (0,0) 전용 처리 ====================
# Start→straight(300) 후, (0,0) 물체가 있으면 여기로!
def handle_cell00_once(now_dir):
    log("[C00] (0,0) 전용 처리 시작")
    # (0,0) 앞에서 물체 탐색 (1센서 스타일 접근)
    # 필요시 살짝 더 전진해서 실제 물체까지 붙기
    timer = StopWatch(); timer.reset()
    while ultra_sensor.distance() > STOP_DIST_MM and timer.time() < 3000:
        left_line_following(70, 1.7)
    robot.stop()
    robot.straight(40)
    grab(); wait(200)
    # 색상 판정 후, (0,0)에서 서쪽 기준 분류 동선 실행
    color = detect_color_simple_rb()
    now_dir = classify_from_cell00(color, now_dir)
    log("[C00] 분류 완료 및 (0,0) 복귀")
    return now_dir

# ========================= 일반 목표 접근(거리기준) ============
def approach_object_pid(stop_mm=50, speed=70, timeout_ms=6000):
    log("[APPROACH] PID 접근")
    global _last_err
    t=0
    while True:
        d = ultra_sensor.distance()
        if d is not None and d < stop_mm: break
        err  = left_sensor.reflection() - right_sensor.reflection()
        turn = Kp*err + Kd*(err - _last_err)
        _last_err = err
        robot.drive(speed, turn); wait(10)
        t+=10
        if t >= timeout_ms: break
    robot.stop()
    robot.straight(20)
    grab(); wait(150)

# ========================= 메인 ===============================
def run():
    # 집게 열고 시작
    release()

    # Start → (0,0) 전방 300
    log("[START] straight(300)")
    robot.straight(300)

    # (0,0) 물체 있으면: 전용 하드코딩 처리
    d = ultra_sensor.distance()
    if d is not None and d < STOP_DIST_MM:
        now = (0,0)
        now_dir = E  # 시작 기본 헤딩을 E로 가정
        now_dir = handle_cell00_once(now_dir)
        MAP[0][0] = "."
        print_map()
    else:
        # 없으면 (0,0)로 진입 후 일반 루프
        now = (0,0)
        now_dir = E
        log("[INFO] (0,0) 비어있음")

    # ===== 메인 루프: BFS로 모든 물체 수거 =====
    while True:
        targets = [(x,y) for y in range(H) for x in range(W) if MAP[y][x] == "#"]
        if not targets:
            break

        # 가장 짧은 경로의 목표 선택
        best_path = None
        best_goal = None
        for t in targets:
            path,_ = bfs(now, t)
            if path and (best_path is None or len(path) < len(best_path)):
                best_path = path
                best_goal = t

        log("[BFS] 경로 = {}".format(best_path))

        # 경로 따라 이동
        for i in range(1, len(best_path)):
            now, now_dir = move_manhatten(now, best_path[i], now_dir)

        # 도착 → 물체 접근/집기
        approach_object_pid(stop_mm=50, speed=70)
        # 색상 판정
        color = detect_color_simple_rb()

        # (0,0) 복귀
        path,_ = bfs(now, (0,0))
        for i in range(1, len(path)):
            now, now_dir = move_manhatten(now, path[i], now_dir)

        # (0,0) 기준 색상 분류 동선 실행
        now_dir = classify_from_cell00(color, now_dir)

        # (0,0)로 복귀 완료 가정
        now = (0,0)

        # 방금 처리한 목표 제거
        if best_goal:
            gx,gy = best_goal
            MAP[gy][gx] = "."
        print_map()

    # 종료: START로 후진 복귀(필요 시 길이 조정)
    log("[FINISH] START 복귀")
    robot.straight(-250)
    log("[END] 완료")

# =============================================================
run()
