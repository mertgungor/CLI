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

# Input is distance between agents and output is the distance to the center of the formation. It uses cos theorem.
def distance_to_radius(distance, number_of_edge):
    return abs(distance**2/(2*(1 - math.cos(2*math.pi/number_of_edge))))**(0.5)

    