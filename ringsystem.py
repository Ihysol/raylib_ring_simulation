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
        
        # permanent snapshot values
        self.s_pos = []                                                     # sensor position 
        self.m_pos_snapshot = [rl.vector3_zero() for i in range(4)]         # first magnet position for calibration
        self.v_normal_snapshot = rl.vector3_zero()                           # snapshot for normal vector
        self.dir_vectors_snapshot = [rl.vector3_zero() for i in range(2)]   # snapshot for dir vectors
        self.rotation_snapshot = rl.vector3_zero()                          # snapshot for rotation

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

    def calc_rotation(self, vector1, vector2):
        v1 = rl.vector3_normalize(vector1)
        v2 = rl.vector3_normalize(vector2)

        dotProduct = rl.vector_3dot_product(v1, v2)
        print(f"({v1.x},{v1.y},{v1.z}) * ({v2.x},{v2.y},{v2.z}) = {dotProduct}")

        x_angle = math.acos(dotProduct/(rl.vector3_length(v1)*rl.vector3_length(v2)))

        print(rad2deg(x_angle)) # goes only from 0-180°, is non-negative (->bug)

        # TODO: get accurate readings against all 3 coordinate axis
        # TODO: then get the angles between the 2nd vector also split in all 3 axis
    
    def calc_all(self):
        self.dir_vectors = self.calc_dir_vectors(positions=self.m_pos)
        self.v_normal = self.calc_normal_vector(dir_vectors=self.dir_vectors)
        self.intersections = self.calc_intersection(positions=self.m_pos, dir_vectors=self.dir_vectors, normal=self.v_normal)
        # self.calc_rotation(self.dir_vectors[0], self.dir_vectors_snapshot[0])

    def snapshot(self):
        self.m_pos_snapshot = self.m_pos[:]
        self.v_normal_snapshot = self.v_normal
        self.dir_vectors_snapshot = self.dir_vectors[:]
        self.rotation_snapshot = self.rotation[:]
