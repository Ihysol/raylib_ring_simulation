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

def clamp(val, maximum=-1, minimum=1):
    return max(min(val, maximum), minimum)

# function to rotate a vector3 around (https://blog.molecular-matters.com/2013/05/24/a-faster-quaternion-vector-multiplication/)
def rotateVectorByQuaternion(vector3, quaternion):
    t = rl.vector3_scale(rl.vector3_cross_product(rl.Vector3(quaternion.x, quaternion.y, quaternion.z), vector3), 2)
    return rl.vector3_add(rl.vector3_add(vector3, rl.vector3_scale(t, quaternion.w)), rl.vector3_cross_product(rl.Vector3(quaternion.x, quaternion.y, quaternion.z), t))

def calcAngleBetweenVectors(vector1, vector2):
    return rad2deg(math.acos(rl.vector_3dot_product(rl.vector3_normalize(vector1), rl.vector3_normalize(vector2)) / (rl.vector3_length(rl.vector3_normalize(vector1))* rl.vector3_length(rl.vector3_normalize(vector2)))))
