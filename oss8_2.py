#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from collections import deque


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
ev3.speaker.beep()


ev3 = EV3Brick()

arm_motor = Motor(Port.D)
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

object_detector = ColorSensor(Port.S2)
ultra_sensor = UltrasonicSensor(Port.S1)
left_sensor = ColorSensor(Port.S3)
right_sensor = ColorSensor(Port.S4)

robot = DriveBase(left_motor, right_motor, 55.5, 104)

ev3.speaker.beep()
# # Write your program here.
N, E, S, W = 1,2,3,4

def turn_min(now_dir, target_dir):
    diff = (target_dir - now_dir) % 4
    angle = [0,90,180,-90][diff]
    robot.turn(angle)
    return target_dir

def left_line_following(speed, kp):
    threshold = 40
    left_reflection = left_sensor.reflection()
    error = left_reflection - threshold
    turn_rate = kp*error
    robot.drive(speed, turn_rate)
    
def right_line_following(speed, kp):
    threshold = 40
    right_reflection = right_sensor.reflection()
    error = right_reflection - threshold
    turn_rate = kp*error
    robot.drive(speed, turn_rate)    
    
def n_move(n, direction="right"):
    for _ in range(n):
        if direction == "right":
            while right_sensor.reflection() > 40:
                left_line_following(100, 1.2)
            while right_sensor.reflection() <= 40:
                right_line_following(100, 1.2) 
        elif direction == "left":
            while left_sensor.reflection() > 40:
                right_line_following(100, 1.2)
            while left_sensor.reflection() <= 40:
                left_line_following(100, 1.2)    
    robot.stop()
    
def grab_object():
    arm_motor.run_until_stalled(200,then = Stop.COAST, duty_limit=50)
    
def release_object():
    arm_motor.run_until_stalled(-200,then = Stop.COAST, duty_limit=50)                                     
    
def move_manhatten(start_xy, goal_xy, now_dir):
    x,y = start_xy
    gx,gy = goal_xy
    dx = gx -x
    dy = gy - y
    
    if dx != 0:
        target_dir = E if dx > 0 else W
        now_dir = turn_min(now_dir, target_dir)
        steps = abs(dx)
        for _ in range(steps):
            follow_line_one_cell()
            x += 1 if target_dir == E else -1

    if dy != 0:
        target_dir = N if dy > 0 else S
        now_dir = turn_min(now_dir, target_dir)
        steps = abs(dy)
        for _ in range(steps):
            follow_line_one_cell()
            y += 1 if target_dir == N else -1
    
    return (x,y), now_dir

MAP = [
    ".....",
    "..#.#",
    "..#.#",
    "..#.#",
    "....."
]

G = [[1 if c== "#" else 0 for c in r] for r in MAP]
H,W = len(G), len(G[0])
S,E = (0,0), (4,4)

def bfs(s, g):
        dist = [[None]*W for _ in range(H)]
        prev = [[None]*W for _ in range(H)]
        q = deque([s])
        dist[s[1]][s[0]] = 0
        while q:
            x, y = q.popleft()
            if (x,y) == g:
                break
            for dx, dy in ((1,0), (-1,0), (0,1), (0,-1)):
                nx, ny = x+dx, y+dy
                if 0 <= nx < W and 0<=ny<H and not G[ny][nx] and dist[ny][nx] is None:
                    dist[ny][nx] = dist[y][x]+1
                    prev[ny][nx] = (x,y)
                    q.append((nx,ny))
        
        path = []
        if dist[g[1]][g[0]] is not None:
            p = g
            while p:
                path.append(p)
                p = prev[p[1]][p[0]]
            path.reverse()
        
        return path, dist
 
# def show(path, dist):
#     grid = [['#' if G[y][x] else '.' for x in range(W)] for y in range(H)]
#     for x,y in path:
#         if (x,y) not in (S,E): grid[y][x] = '*'
#     grid[S[1]][S[0]], grid[E[1]][E[0]] = 'S', 'G'
#     print("=== 경로 맵 ===")
#     for r in grid: print(''.join(r))
#     print("\n=== 거리 히트맵 ===")                
#     for r in dist: print(' '.join(' .' if d is None else f"{d:2d}" for d in r))
#     print("\n경로:",path)
#     print("길이:", len(path)-1 if path else "없음")

release_object()
robot.straight(100)
n_move(1,direction="left")
while ultra_sensor.distance() > 50:
    left_line_following(70, 1.2)
robot.stop()
robot.straight(20)

grab_object()
wait(500)

object_color= object_detector.color()
print("Detected object color:", object_color)

if object_color == Color.RED:
    robot.straight(120)
    robot.turn(220)
    robot.straight(30)
    n_move(1, direction="left")

    robot.straight(100)
    ev3.speaker.beep()
    robot.turn(-90)
    n_move(1, direction="right")
    ev3.speaker.beep()
    robot.turn(90)
    n_move(1, direction="left")
    robot.straight(50)
    ev3.speaker.beep()
    release_object()

elif object_color ==Color.GREEN:
    robot.straight(80)
    robot.turn(180)
    wait(100)
    n_move(1, direction="left")
    wait(100)
    robot.turn(-90)
    wait(100)
    n_move(2, direction="left")
    wait(100)
    robot.turn(90)
    wait(100)
    n_move(1, direction="left")
    wait(100)
    release_object()
    ev3.speaker.beep()

# if __name__ == "__main__":
    
#     path,dist = bfs(S,E)
    # print("=== 현재 MAP 출력 ===")
    # for r in MAP:
    #     print(r)
    # print()
    # show(path,dist)        
