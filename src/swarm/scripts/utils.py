import math

def angle_of_vector(x, y):
    angle = math.atan(abs(y)/abs(x))

    if x >= 0 and y >= 0:
        return angle
    elif x < 0 and y >= 0:
        return math.pi - angle
    elif x < 0 and y < 0:
        return math.pi + angle
    else:
        return 2*math.pi - angle