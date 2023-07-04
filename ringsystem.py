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
        
        # self.m_pos_snapshot = self.m_pos

    def calc_dir_vectors(self):
        # calc direction vectors
        self.dir_vectors[0] = rl.vector3_subtract(self.m_pos[2], self.m_pos[0])
        self.dir_vectors[1] = rl.vector3_subtract(self.m_pos[3], self.m_pos[1])

        return self.dir_vectors

    def calc_normal_vector(self):      
        # calc normal vector
        self.v_normal = rl.vector3_cross_product(self.dir_vectors[0], self.dir_vectors[1])
        
        return self.v_normal

    def calc_intersection(self):  
        # find points 
        t1 = rl.vector_3dot_product(rl.vector3_cross_product(self.dir_vectors[1], self.v_normal), rl.vector3_subtract(self.m_pos[1], self.m_pos[0])) /  rl.vector_3dot_product(self.v_normal, self.v_normal)
        t2 = rl.vector_3dot_product(rl.vector3_cross_product(self.dir_vectors[0], self.v_normal), rl.vector3_subtract(self.m_pos[1], self.m_pos[0])) / rl.vector_3dot_product(self.v_normal, self.v_normal)

        # calc intersections
        self.intersections[0] = rl.vector3_add(self.m_pos[0], rl.vector3_scale(self.dir_vectors[0], t1))
        self.intersections[1] = rl.vector3_add(self.m_pos[1], rl.vector3_scale(self.dir_vectors[1], t1))
        
        return self.intersections

    
    def calc_rotation(self):
        test = rl.vector_3dot_product(self.dir_vectors[0], self.dir_vectors[1])
        # print(math.sin(test))
        return 0
    
    def calc_all(self):
        self.calc_dir_vectors()
        self.calc_normal_vector()
        self.calc_intersection()
        self.calc_rotation()

    def test(self, value):
        return value


    def snapshot(self):
        self.m_pos_snapshot = self.m_pos[:]
        self.v_normal_snapshot = self.v_normal
        self.dir_vectors_snapshot = self.dir_vectors[:]
        self.rotation_snapshot = self.rotation[:]
