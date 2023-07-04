import math
import pyray as rl

# deg to radiant conversion
def deg2rad(deg):
    return deg * (math.pi/180)

# function to rotate a vector3 around
def rotateVectorByQuaternion(vector3, quaternion):
    t = rl.vector3_scale(rl.vector3_cross_product(rl.Vector3(quaternion.x, quaternion.y, quaternion.z), vector3), 2)
    return rl.vector3_add(rl.vector3_add(vector3, rl.vector3_scale(t, quaternion.w)), rl.vector3_cross_product(rl.Vector3(quaternion.x, quaternion.y, quaternion.z), t))
