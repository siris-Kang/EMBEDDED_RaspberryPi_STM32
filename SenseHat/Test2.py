from sense_hat import SenseHat
from time import sleep
import copy
from sense_hat import SenseHat, ACTION_PRESSED, ACTION_HELD, ACTION_RELEASED
from signal import pause

sense = SenseHat()

BLACK = (0,0,0)
RED = (255, 0, 0)
GREEN = (0,255,0)
WHITE = (255,255,255)

maps = [
    [0,1,0,0,0,0,0,0],
    [0,1,1,1,1,1,1,0],
    [0,0,0,0,0,0,1,0],
    [0,1,1,1,1,0,1,0],
    [0,0,1,0,0,0,1,0],
    [0,0,1,0,1,1,1,0],
    [0,1,1,0,0,0,0,0],
    [0,0,0,0,1,1,0,0]
]

dx = [0, 0, -1, 1]
dy = [-1, 1, 0, 0]

start = (0, 0)
fin = (7, 7)

visited = [[0] * 8 for _ in range(8)]

dist = []

def oob(x, y):
    if x < 0 or x >= 8 or y < 0 or y >= 8:
        return True
    else:
        return False

def dfs(now):
    x = now[0]
    y = now[1]

    if x == fin[0] and y == fin[1]:
        dist.append(copy.deepcopy(visited))
        return

    for dir in range(4):
        nx = x + dx[dir]
        ny = y + dy[dir]

        if oob(nx, ny):
            continue
        if visited[nx][ny] != 0:
            continue
        if maps[nx][ny] != 0:
            continue

        visited[nx][ny] = 1
        dfs((nx, ny))
        visited[nx][ny] = 0

def draw_map(num):
    for x in range(8):
        for y in range(8):
            if maps[x][y] == 1:
                sense.set_pixel(y, x, WHITE)
            elif dist[num][x][y] == 1:
                sense.set_pixel(y, x, GREEN)
            else:
                sense.set_pixel(y, x, BLACK)

def plus(event):
    if event.action != ACTION_RELEASED:
        return
    global num
    sense.clear()
    draw_map(num)
    num = (num + 1) % len(dist)


## start
visited[start[0]][start[1]] = 1

dfs(start)

num = 0
sense.stick.direction_any = plus

pause()
