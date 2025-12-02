#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from collections import deque
import time

# --- 1. 하드웨어 설정 ---
ev3 = EV3Brick()

# 모터 및 센서
arm_motor = Motor(Port.D)
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

object_detector = ColorSensor(Port.S2)
ultra_sensor = UltrasonicSensor(Port.S1)
left_sensor = ColorSensor(Port.S3)
right_sensor = ColorSensor(Port.S4)

# 로봇 물리 엔진
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
# [튜닝] 속도 20 하향 조정 (안정성 확보)
robot.settings(straight_speed=150, straight_acceleration=120, turn_rate=120, turn_acceleration=80)

# --- 2. 맵 및 네비게이션 설정 ---

# 방향 상수 (0:북, 1:동, 2:남, 3:서)
N, E, S, W = 0, 1, 2, 3
DIR_VEC = {N: (0, -1), E: (1, 0), S: (0, 1), W: (-1, 0)}

# 전역 상태
current_x, current_y = 0, 0
current_dir = E  # 초기 방향 (1번 지역 진입 시 동쪽을 바라본다고 가정)

# [수정됨] 3x3 실제 환경에 맞춘 맵 정의
# 1  2  3  
# 5  6  7  
# 9 10 11 
# 7번 위치는 (Row 1, Col 2) -> MAP[1][2]
MAP = [
    [".", ".", "."],
    [".", "#", "."],
    [".", ".", "."]
]
H, W_GRID = len(MAP), len(MAP[0])

# --- 3. 주행 및 제어 함수 (Driver Layer) ---

def turn_to_direction(target_dir):
    """최소 회전 각도로 방향 전환"""
    global current_dir
    diff = (target_dir - current_dir) % 4
    
    if diff == 0: return

    # [유지] 회전축 정렬 (Pivot Alignment)
    # 턴하기 전에 바퀴를 교차로 중심에 맞추기 위해 30mm 더 전진
    robot.straight(30)
    wait(100)

    # 1: 90도 우회전, 2: 180도, 3: 90도 좌회전(-90)
    angle = [0, 90, 180, -90][diff]
    robot.turn(angle)
    wait(100) # 관성 제거
    current_dir = target_dir

def left_line_following(speed, kp):
    threshold = 30 
    error = left_sensor.reflection() - threshold
    robot.drive(speed, kp * error)

def right_line_following(speed, kp):
    threshold = 30
    error = right_sensor.reflection() - threshold
    robot.drive(speed, kp * error)

def follow_line_one_cell(speed=180, kp=1.3):
    """
    한 칸 이동 (센서 + 거리 퓨전)
    [튜닝] 기본 속도 200 -> 180으로 하향
    """
    both_dark_count = 0
    DARK_THR = 30 
    
    robot.reset() # 거리 측정 시작

    while True:
        # 기본: 라인 트레이싱
        left_line_following(speed, kp)

        # 센서 감지
        if left_sensor.reflection() < DARK_THR and right_sensor.reflection() < DARK_THR:
            both_dark_count += 1
        else:
            if both_dark_count > 0: both_dark_count = 0 

        # 종료 조건 1: 교차로 인식
        if both_dark_count >= 3:
            break
            
        wait(10)

    robot.stop(Stop.BRAKE)
    wait(50)
    # 교차로 중앙 정렬을 위한 1차 미세 전진 (기본값)
    robot.straight(40)

def execute_path(path):
    """BFS로 생성된 좌표 리스트를 따라 이동"""
    global current_x, current_y
    
    if not path or len(path) < 2: return

    print("Executing Path:", path)
    # path[0]은 현재 위치, path[1]부터 이동
    for next_pos in path[1:]:
        nx, ny = next_pos
        
        # 다음 이동 방향 계산
        dx = nx - current_x
        dy = ny - current_y
        
        target_dir = current_dir
        if dx == 1: target_dir = E
        elif dx == -1: target_dir = W
        elif dy == 1: target_dir = S
        elif dy == -1: target_dir = N
        
        # 회전 및 이동
        turn_to_direction(target_dir)
        follow_line_one_cell()
        
        # 좌표 업데이트
        current_x, current_y = nx, ny

# --- 4. 알고리즘 (Brain Layer) ---

def bfs(start, goal):
    """시작점에서 목표점까지 최단 경로(좌표 리스트) 반환"""
    q = deque([start])
    visited = {start}
    parent = {start: None}
    
    while q:
        cx, cy = q.popleft()
        if (cx, cy) == goal:
            break
        
        # [중요] BFS 탐색 순서: N, E, S, W
        # (0,0)에서 출발 시 E(1,0)가 S(0,1)보다 먼저 큐에 들어감
        # 따라서 1->2->3->7 경로가 1->5->6->7보다 우선됨
        for d in [N, E, S, W]:
            dx, dy = DIR_VEC[d]
            nx, ny = cx + dx, cy + dy
            
            # 맵 범위 내 이동 가능 여부 확인
            if 0 <= nx < W_GRID and 0 <= ny < H:
                if (nx, ny) not in visited:
                    visited.add((nx, ny))
                    parent[(nx, ny)] = (cx, cy)
                    q.append((nx, ny))
    
    # 경로 재구성 (Backtracking)
    path = []
    curr = goal
    if curr not in parent: return None # 경로 없음
    
    while curr is not None:
        path.append(curr)
        curr = parent[curr]
    path.reverse()
    return path

def find_target_on_map():
    """맵에서 물체(#)의 좌표 탐색"""
    for y in range(H):
        for x in range(W_GRID):
            if MAP[y][x] == "#":
                return (x, y)
    return None

# --- 5. 액션 및 배달 (Application Layer) ---

def approach_and_grab():
    """물체 정밀 접근 및 그랩"""
    print("Approaching...")
    # 초음파로 정밀 접근 (최대 3초)
    start_t = time.time()
    # [수정] 40 -> 30 (더 가까이 붙을 때까지 라인트레이싱 유지)
    while ultra_sensor.distance() > 30:
        left_line_following(100, 1.2)
        if time.time() - start_t > 3: break
    
    robot.stop()
    
    # [수정] 물체와의 거리 확보를 위해 과감하게 전진 (60 -> 100)
    # 그래야 그랩이 허공을 가르지 않습니다.
    robot.straight(100) 
    wait(300) # 그랩 전 안정화
    
    # 그랩 (파워 최대)
    arm_motor.run_until_stalled(200, then=Stop.HOLD, duty_limit=100)
    wait(300)

def release_object():
    arm_motor.run_until_stalled(-200, then=Stop.COAST, duty_limit=80)
    wait(500)

def deliver_logic(color):
    """(0,0) 복귀 후 색상별 배달 로직"""
    print("Delivering Color:", color)
    
    # 배달 시작 전, 기준 방향(Start Zone 방향 = 서쪽) 정렬
    turn_to_direction(W)
    
    if color == Color.RED:
        # 예시: 빨간색 배달 경로
        robot.straight(120)
        robot.turn(220) # 유턴 비슷하게
        robot.straight(30)
        # 하드코딩된 동작 수행...
        robot.straight(100)
        release_object()
        robot.straight(-100) # 복귀
        
    elif color == Color.GREEN:
        robot.straight(120)
        robot.turn(220)
        robot.straight(30)
        # ... 사용자 로직 ...
        robot.straight(100)
        release_object()
        robot.straight(-100)

    elif color == Color.BLUE:
        robot.straight(80)
        robot.turn(180)
        # ... 사용자 로직 ...
        robot.straight(100)
        release_object()
        robot.straight(-100)
    
    else:
        # 색상 인식 실패 시 기본 동작
        print("Unknown Color")
        robot.straight(50)
        release_object()
        robot.straight(-50)

    # 배달 후 다시 1번 지역(0,0) 위치 및 방향(동쪽)으로 재정렬
    print("Returning to Grid Origin (0,0)")
    # 물리적으로 (0,0) 위치로 돌아와서 E 방향을 보도록 초기화
    robot.turn(180) # 다시 그리드 쪽 보기
    robot.straight(150) # 그리드 진입
    
    global current_dir, current_x, current_y
    current_x, current_y = 0, 0
    current_dir = E

# --- 6. 메인 미션 루프 ---

def run_mission():
    global current_x, current_y, current_dir
    
    print("Mission System Start")
    release_object()
    ev3.speaker.beep()

    # 1. 1번 지역(0,0)으로 이동 (Start Zone에서 진입)
    robot.straight(150) 
    # 첫 진입 시 (0,0)에 있고 동쪽을 본다고 가정
    current_x, current_y = 0, 0
    current_dir = E

    while True:
        # 2. 맵에서 물체(#) 탐색
        target_pos = find_target_on_map()
        
        # 10. 없으면 Finish
        if target_pos is None:
            print("All Targets Cleared!")
            ev3.speaker.say("Mission Complete")
            break
        
        print("Target Found:", target_pos)

        # 3. BFS로 최단 경로 파악
        path_to_target = bfs((current_x, current_y), target_pos)
        
        if not path_to_target:
            print("Error: No path to target!")
            break

        # 4 & 5. 이동 (Move Manhatten Logic via BFS Path)
        execute_path(path_to_target)

        # 물체 도착 후 액션
        approach_and_grab()
        detected_color = object_detector.color()
        if detected_color is None: detected_color = Color.RED # Fallback

        # 8. 맵 업데이트 (물체 제거) -> 로직상 잡고 나서 바로 업데이트
        MAP[target_pos[1]][target_pos[0]] = "."

        # 6. (0,0)으로 복귀 (BFS 계산)
        print("Returning Home (0,0)...")
        path_home = bfs((current_x, current_y), (0, 0))
        execute_path(path_home)

        # 7. 배달 및 7-2. 1번 지역 복귀
        deliver_to_zone_logic = deliver_logic(detected_color)
        
        # 9. 맵 순회 반복 (Loop)

    ev3.speaker.beep()

if __name__ == "__main__":
    run_mission()