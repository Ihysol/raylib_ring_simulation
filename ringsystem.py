import pyray as rl
import math
from utils import *
import copy

class RingSystem:
    def __init__(self, s_circle_radius = 42.5, offset_s_m=3):
        # geometry
        self.s_circle_radius = s_circle_radius
        self.offset_s_m = offset_s_m
        self.m_circle_radius = s_circle_radius+offset_s_m

        self.m_pos = [] # current position values
        self.intersections = [rl.vector3_zero() for i in range(2)] # intersection points
        self.v_normal = rl.vector3_zero()
        self.dir_vectors = [rl.vector3_zero() for i in range(2)]
        self.rotation = [0, 0, 0]
        self.rotation_offsets = [rl.vector3_zero() for i in range(2)] # container for rotation changes
        
        # permanent snapshot values
        self.s_pos = []                                                     # sensor position 
        self.m_pos_snapshot = [rl.vector3_zero() for i in range(4)]         # first magnet position for calibration
        self.v_normal_snapshot = rl.vector3_zero()                           # snapshot for normal vector
        self.dir_vectors_snapshot = [rl.vector3_zero() for i in range(2)]   # snapshot for dir vectors
        self.rotation_snapshot = rl.vector3_zero()                          # snapshot for rotation

        # detected movement
        self.right_tilt_detected = 0
        self.fwd_tilt_detected = 0
        self.cw_rot_detected = 0

        # calc initial sensor/ magnet positions positions
        for i in range(4):
            angle = i*deg2rad(-90)
            x = math.cos(angle)
            z = math.sin(angle)
            self.s_pos.append(rl.vector3_scale(rl.vector3_normalize(rl.Vector3(x, 0, z)), self.s_circle_radius))
            self.m_pos.append(rl.vector3_scale(rl.vector3_normalize(rl.Vector3(x, 0, z)), self.m_circle_radius))

        # snapshot initial values
        self.calc_all()
        self.snapshot()

        print(self.dir_vectors_snapshot[0].x)

    def calc_dir_vectors(self, positions):
        # calc direction vectors
        dir_vectors = [rl.vector3_zero() for i in range(2)]
        dir_vectors[0] = rl.vector3_subtract(positions[2], positions[0])
        dir_vectors[1] = rl.vector3_subtract(positions[3], positions[1])
        return dir_vectors

    def calc_normal_vector(self, dir_vectors):      
        # calc normal vector
        normal = rl.vector3_cross_product(dir_vectors[0], dir_vectors[1])
        return normal

    def calc_intersection(self, positions, normal, dir_vectors):  
        # find points 
        t1 = rl.vector_3dot_product(rl.vector3_cross_product(dir_vectors[1], normal), rl.vector3_subtract(positions[1], positions[0])) / rl.vector_3dot_product(normal, normal)
        t2 = rl.vector_3dot_product(rl.vector3_cross_product(dir_vectors[0], normal), rl.vector3_subtract(positions[1], positions[0])) / rl.vector_3dot_product(normal, normal)

        # calc intersections
        intersections = [rl.vector3_zero() for i in range(2)]
        intersections[0] = rl.vector3_add(positions[0], rl.vector3_scale(dir_vectors[0], t1))
        intersections[1] = rl.vector3_add(positions[1], rl.vector3_scale(dir_vectors[1], t1))
        return intersections

    def calc_rotation(self, vector, origin_axis_vector):

        target_axis_vector = rl.vector3_cross_product(rl.Vector3(0, 1, 0), origin_axis_vector) # get target component which is perpendicular to rotation axis y and the target axis
        v1 = rl.vector3_length(rl.vector3_multiply(rl.vector3_normalize(vector), target_axis_vector)) # select only target element
        print(rad2deg(math.asin(v1)))


    def calc_rotation_from_quaternion_between_vectors(self, vector1, vector2):
        # Normalize vectors
        v1 = rl.vector3_normalize(vector1)
        v2 = rl.vector3_normalize(vector2)

        # Calculate normal vector
        n = rl.vector3_cross_product(v1, v2)

        # Calculate dot product
        dotProduct = rl.vector_3dot_product(v1, v2)

        # x, y, z is equal to crossproduct x/y/z and w is |v1||v2|+dotProduct(v1,v2)
        q = rl.quaternion_normalize(rl.Vector4(n.x, n.y, n.z, rl.vector3_length(v1)*rl.vector3_length(v2)+dotProduct))

        roll = math.atan2(2*(q.w*q.x + q.y*q.z), 1 - 2 *(math.pow(q.x, 2) + math.pow(q.y, 2)))
        pitch = math.asin(2*(q.w*q.y - q.z*q.x))
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2  *(math.pow(q.y, 2) + math.pow(q.z, 2)))

        vector_roll_pitch_yaw = [rad2deg(roll), rad2deg(pitch), rad2deg(yaw)]

        # print(f"roll: {rad2deg(roll)}, pitch:{rad2deg(pitch)}, yaw:{rad2deg(yaw)}")

        return vector_roll_pitch_yaw


    def calc_all(self):
        self.dir_vectors = self.calc_dir_vectors(positions=self.m_pos)
        self.v_normal = self.calc_normal_vector(dir_vectors=self.dir_vectors)
        self.intersections = self.calc_intersection(positions=self.m_pos, dir_vectors=self.dir_vectors, normal=self.v_normal)

        for idx, dir_vector in enumerate(self.dir_vectors):
            self.rotation_offsets[idx] = self.calc_rotation_from_quaternion_between_vectors(dir_vector, self.dir_vectors_snapshot[idx])

        # print(self.rotation_offsets[0])
        # print(f"rot:[{self.rotation_offsets[0][1]}, {self.rotation_offsets[1][1]}] \nlrtilt:[{self.rotation_offsets[0][2]}] \nfbtilt:[{self.rotation_offsets[1][0]}]")

        if self.rotation_offsets[0][1] >= 5.0:
            self.cw_rot_detected = 1
        elif self.rotation_offsets[0][1] <= -5.0:
            self.cw_rot_detected = -1
        else: 
            self.cw_rot_detected = 0


        if self.rotation_offsets[1][0] >= 5.0:
            self.fwd_tilt_detected = 1
        elif self.rotation_offsets[1][0] <= -5.0:
            self.fwd_tilt_detected = -1
        else:
            self.fwd_tilt_detected = 0

        if self.rotation_offsets[0][2] >= 5.0:
            self.right_tilt_detected = 1
        elif self.rotation_offsets[0][2] <= -5.0:
            self.right_tilt_detected = -1
        else:
            self.right_tilt_detected = 0

        print(f"cw_rot:{self.cw_rot_detected}, fwd_tilt:{self.fwd_tilt_detected}, right_tilt:{self.right_tilt_detected}")
        
    def snapshot(self):
        self.m_pos_snapshot = self.m_pos[:]
        self.v_normal_snapshot = self.v_normal
        self.dir_vectors_snapshot = self.dir_vectors[:]
        self.rotation_snapshot = self.rotation[:]
