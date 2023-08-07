import math
import pyray as rl

# deg to radiant conversion
def deg2rad(deg):
    return deg * (math.pi/180)

# radiant to deg conversion
def rad2deg(rad):
    return rad / (math.pi/180)

def min(a, b):
    return a if a<b else b

def max(a, b):
    return a if a>b else b

def clamp(val, minimum=-1, maximum=1):
    return max(min(val, maximum), minimum)

def vector3_find_max_value(v):
    max_abs = max(max(abs(v.x), abs(v.y)), abs(v.z))

    if vector3_contains_value(v, max_abs):
        return max_abs
    else:
        return -max_abs
    
def vector3_contains_value(v, value):
    list = vector3_to_list(v)
    if value in list:
        return True

def vector3_to_list(v):
    return [v.x, v.y, v.z]

def calc_angle_between_vectors(vector1, vector2):
    return rad2deg(math.acos(rl.vector_3dot_product(rl.vector3_normalize(vector1), rl.vector3_normalize(vector2)) / (rl.vector3_length(rl.vector3_normalize(vector1))* rl.vector3_length(rl.vector3_normalize(vector2)))))

def print_vector3(v):
    print(f"x:{v.x}, y:{v.y}, z:{v.z}")

# function to rotate a vector3 around (https://blog.molecular-matters.com/2013/05/24/a-faster-quaternion-vector-multiplication/)
def vector3_rotate_by_quaternion(vector3, quaternion):
    t = rl.vector3_scale(rl.vector3_cross_product(rl.Vector3(quaternion.x, quaternion.y, quaternion.z), vector3), 2)
    return rl.vector3_add(rl.vector3_add(vector3, rl.vector3_scale(t, quaternion.w)), rl.vector3_cross_product(rl.Vector3(quaternion.x, quaternion.y, quaternion.z), t))

def min_around_zero(a, b):
    if (abs(a) < abs(b)):
        return a
    else:
        return b