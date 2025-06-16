import math

def upstraight(t):
    if t < 5:
        return 0, 0, -t, 1.77
    elif t < 50:
        return 0, 0, -t, 0
    else:
        return 0.3*(t-5), 0.2*(t-5), -t, 0

def circle(t):
    xd = 2*math.sin(0.5*t)
    yd = 2*math.cos(0.5*t)
    zd = -0.2*t
    psid = 1.77
    return xd, yd, zd, psid

def test1(t):
    return 0, 0, -t, 1.77

def test2(t):
    up_time = 100
    if t < up_time:
        return 0, 0, -t, 1.77
    else:
        return 0.3*(t-up_time), 0.2*(t-up_time), -t, 1.77
    
def test3(t):
    return 0.1*t, 0.2*t, -t, 1.77

def test4(t):
    return 0, 0, -t, 1.77+0.1*math.sin(0.5*t)
