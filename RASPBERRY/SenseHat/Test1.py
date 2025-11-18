from sense_hat import SenseHat
from time import sleep

sense = SenseHat()

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

BLACK = (0,0,0)
GREEN = (0,255,0)
WHITE = (255,255,255)

def draw_map(px, py):
    for x in range(8):
        for y in range(8):
            if maps[x][y] == 1:
                sense.set_pixel(y, x, WHITE)
            else:
                sense.set_pixel(y, x, BLACK)
    sense.set_pixel(py, px, GREEN)

x, y = 4, 4
draw_map(x, y)

while True:
    ori = sense.get_orientation_degrees()
    pitch = ori['pitch']
    roll = ori['roll']

    nx = x
    ny = y

    if pitch - 180 < 0:
        npitch = pitch
    elif pitch - 180 > 0:
        npitch = 360 - pitch

    if roll - 180 < 0:
        nroll = roll
    elif roll - 180 > 0:
        nroll = 360 - roll

    if nroll > 20 or npitch > 20:
        if npitch > nroll: # to picth dir
            if pitch - 180 < 0:# left
                ny = y - 1
            else: # right
                ny = y + 1
        else: # to roll dir
            if roll - 180 < 0: # up
                nx = x + 1
            else: # down
                nx = x - 1

    nx = max(0, min(7, nx))
    ny = max(0, min(7, ny))

    if maps[nx][ny] == 0:
        x, y = nx, ny

    draw_map(x, y)
    sleep(0.2)
