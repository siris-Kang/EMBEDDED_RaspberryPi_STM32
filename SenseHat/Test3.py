from collections import deque
from sense_hat import SenseHat
from time import sleep
from sense_hat import SenseHat, ACTION_PRESSED, ACTION_HELD, ACTION_RELEASED
from signal import pause

sense = SenseHat()

maps = [
    [0,1,0,0,0,0,0,0],
    [0,1,1,1,1,1,1,0],
    [0,0,0,0,0,0,1,0],
    [0,1,1,1,1,0,1,0],
    [0,0,1,0,0,0,1,0],
    [0,0,1,0,1,1,1,0],
    [0,1,1,0,0,0,0,0],
    [0,0,0,0,1,1,1,0]
]

RED = (255, 0, 0)
BLACK = (0,0,0)
GREEN = (0,255,0)
WHITE = (255,255,255)

dx = [0, 0, -1, 1]
dy = [-1, 1, 0, 0]

move_x, move_y = 0, 0
fin = (7, 7)

dist = [[-1 for _ in range(8)] for _ in range(8)]
prev = [[(0, 0) for _ in range(8)] for _ in range(8)]
show_dist = [[0 for _ in range(8)] for _ in range(8)]

def oob(x, y):
    return x < 0 or x >= 8 or y < 0 or y >= 8

def bfs(move_x, move_y):
    global dist, prev
    dist = [[-1 for _ in range(8)] for _ in range(8)]
    prev = [[(0, 0) for _ in range(8)] for _ in range(8)]

    que = deque()
    que.append((move_x, move_y))
    dist[move_x][move_y] = 0

    while que:
        x, y = que.popleft()
        for dir in range(4):
            nx = x + dx[dir]
            ny = y + dy[dir]

            if oob(nx, ny):
                continue
            if dist[nx][ny] != -1:
                continue
            if maps[nx][ny] != 0:
                continue

            dist[nx][ny] = dist[x][y] + 1
            prev[nx][ny] = (x, y)
            que.append((nx, ny))

def get_way(move_x, move_y, fin):
    for i in range(8):
        for j in range(8):
            show_dist[i][j] = 0

    now_x = fin[0]
    now_y = fin[1]

    while (now_x == move_x and now_y == move_y) == False:
        show_dist[now_x][now_y] = 1
        now_x, now_y = prev[now_x][now_y][0], prev[now_x][now_y][1]
    show_dist[now_x][now_y] = 1

def draw_map():
    global move_x
    global move_y
    sense.clear()
    for x in range(8):
        for y in range(8):
            if maps[x][y] == 1:
                sense.set_pixel(y, x, WHITE)
            elif show_dist[x][y] == 1:
                sense.set_pixel(y, x, RED)
            else:
                sense.set_pixel(y, x, BLACK)
    sense.set_pixel(move_y, move_x, GREEN)


def call():
    bfs(move_x, move_y)
    get_way(move_x, move_y, fin)
    draw_map()

# joystick
def pushed_down(event):
    global move_x
    global move_y
    if event.action != ACTION_PRESSED:
        return
    if move_x + 1 >= 8:
        return
    if maps[move_x + 1][move_y] == 1:
        return
    move_x += 1
    call()
    

def pushed_up(event):
    global move_x
    global move_y
    if event.action != ACTION_PRESSED:
        return
    if move_x - 1 < 0:
        return
    if maps[move_x - 1][move_y] == 1:
        return
    move_x -= 1
    call()

def pushed_right(event):
    global move_x
    global move_y
    if event.action != ACTION_PRESSED:
        return
    if move_y + 1 >= 8:
        return
    if maps[move_x][move_y + 1] == 1:
        return
    move_y += 1
    call()

def pushed_left(event):
    global move_x
    global move_y
    if event.action != ACTION_PRESSED:
        return
    if move_y - 1 < 0:
        return
    if maps[move_x][move_y - 1] == 1:
        return
    move_y -= 1
    call()
 

sense.stick.direction_up = pushed_up
sense.stick.direction_down = pushed_down
sense.stick.direction_left = pushed_left
sense.stick.direction_right = pushed_right

pause()