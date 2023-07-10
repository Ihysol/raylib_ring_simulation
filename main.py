import pyray as rl
import math

import serial
import threading
import time

from ringsystem import RingSystem
from utils import *

# init window
WIDTH, HEIGHT = 800, 600
rl.init_window(WIDTH, HEIGHT, "green wie ein philipp - 3d vector demo for a commercial product that nobody needs, but yet it is still in worked on.")
rl.set_target_fps(60)

# camera
camera = rl.Camera3D()
camera.position = rl.Vector3(0, 50, 100)
camera.target = rl.Vector3(0, 0, 0)
camera.up = rl.Vector3(0, 1, 0)
camera.fovy = 45.0

ring = RingSystem()

delta_scale = 0.5
rotation_scale = 1E-2

def processUserInputs():
    _q = [rl.quaternion_identity() for _ in range(3)]

    if rl.is_key_down(rl.KeyboardKey.KEY_W):
        forward = rl.vector3_subtract(camera.target, camera.position)
        camera.position = rl.vector3_add(camera.position, rl.vector3_scale(rl.vector3_normalize(forward), camera_scale))
    if rl.is_key_down(rl.KeyboardKey.KEY_S):
        backward = rl.vector3_subtract(camera.position, camera.target)
        camera.position = rl.vector3_add(camera.position, rl.vector3_scale(rl.vector3_normalize(backward), camera_scale))
    if rl.is_key_down(rl.KeyboardKey.KEY_D) or rl.is_key_down(rl.KeyboardKey.KEY_A):
        # get vector perpendicular to forward vector and camera.up vector = vector right
        right = rl.vector3_cross_product(camera.up, rl.vector3_normalize(rl.vector3_subtract(camera.target, camera.position)))
        if rl.is_key_down(rl.KeyboardKey.KEY_D):
            right = rl.vector3_scale(right, -1)
        camera.position = rl.vector3_add(camera.position, rl.vector3_scale(right, camera_scale))     

    if rl.is_key_down(rl.KeyboardKey.KEY_UP) or rl.is_key_down(rl.KeyboardKey.KEY_DOWN):
        q_angle = rotation_scale if rl.is_key_down(rl.KeyboardKey.KEY_DOWN) else -rotation_scale
        _q[0] = rl.Vector4(math.sin(q_angle/2)*1, 0, 0, math.cos(q_angle/2))

    if rl.is_key_down(rl.KeyboardKey.KEY_PAGE_UP) or rl.is_key_down(rl.KeyboardKey.KEY_PAGE_DOWN):
        q_angle = rotation_scale if rl.is_key_down(rl.KeyboardKey.KEY_PAGE_UP) else -rotation_scale       
        _q[1] = rl.Vector4(0, math.sin(q_angle/2)*1, 0, math.cos(q_angle/2))
        
    if rl.is_key_down(rl.KeyboardKey.KEY_LEFT) or rl.is_key_down(rl.KeyboardKey.KEY_RIGHT):
        q_angle = rotation_scale if rl.is_key_down(rl.KeyboardKey.KEY_LEFT) else -rotation_scale
        _q[2] = rl.Vector4(0, 0, math.sin(q_angle/2)*1, math.cos(q_angle/2))

    q = rl.quaternion_multiply(rl.quaternion_multiply(_q[0], _q[1]), _q[2])
    for idx,pos in enumerate(ring.m_pos):
        ring.m_pos[idx] = rotateVectorByQuaternion(pos, q)
       
    if rl.is_key_down(rl.KeyboardKey.KEY_ENTER):
        for idx, pos in enumerate(ring.m_pos):
            angle = idx*deg2rad(-90)
            ring.m_pos[idx] = rl.vector3_scale(rl.vector3_normalize(rl.Vector3(math.cos(angle), 0, math.sin(angle))), ring.m_circle_radius)

    rl.draw_text("WASD:cam control, Arrow Keys:Tilt, Enter: reset tilt, PG_DOWN/PG_UP: rotate, TAB: snapshot, LCTRL:calcAngle", 10, 10, 12, rl.BEIGE)

    if rl.is_key_down(rl.KeyboardKey.KEY_J) or rl.is_key_down(rl.KeyboardKey.KEY_L):
        delta_x = delta_scale if rl.is_key_down(rl.KeyboardKey.KEY_L) else -delta_scale
        translation_matrix = rl.matrix_translate(delta_x, 0, 0)
        for idx, pos in enumerate(ring.m_pos):
            ring.m_pos[idx] = rl.vector3_transform(pos, translation_matrix)
    elif rl.is_key_down(rl.KeyboardKey.KEY_I) or rl.is_key_down(rl.KeyboardKey.KEY_K):
        delta_z = delta_scale if rl.is_key_down(rl.KeyboardKey.KEY_K) else -delta_scale
        translation_matrix = rl.matrix_translate(0, 0, delta_z)
        for idx, pos in enumerate(ring.m_pos):
            ring.m_pos[idx] = rl.vector3_transform(pos, translation_matrix)

    if rl.is_key_down(rl.KeyboardKey.KEY_TAB):
        # print(f"{ring.v_normal_snapshot.x}, {ring.v_normal_snapshot.y}, {ring.v_normal_snapshot.z}")
        # print(f"{ring.calc_rotation(ring.dir_vectors[0], ring.dir_vectors_snapshot[0])}")
        ring.snapshot()

    if rl.is_key_down(rl.KeyboardKey.KEY_LEFT_CONTROL):
        ring.calc_all()
        ring.calc_rotation(ring.dir_vectors[0], ring.dir_vectors_snapshot[0])

def draw_orientation_marker():
    orientation_marker_pos = [ring.m_pos[3], rl.vector3_subtract(ring.s_pos[3], rl.Vector3(3,0,0)), rl.vector3_add(ring.s_pos[3], rl.Vector3(3,0,0))]
    rl.draw_triangle_3d(orientation_marker_pos[2], orientation_marker_pos[1], orientation_marker_pos[1], rl.WHITE)

def draw_snapshot_data():
    rl.draw_line_3d(ring.m_pos_snapshot[0], ring.m_pos_snapshot[2], rl.GRAY)     
    rl.draw_line_3d(ring.m_pos_snapshot[1], ring.m_pos_snapshot[3], rl.GRAY) 
    rl.draw_line_3d(ring.intersections[0], ring.v_normal_snapshot, rl.DARKPURPLE)    
    for pos in ring.m_pos_snapshot:
        rl.draw_sphere(pos, 0.75, rl.DARKGRAY) 

def getInputs():
    inputs = []
    while True:
        if ser.in_waiting > 0:
            try:
                sensor_data = ser.readline().decode().rstrip().split(";")
                sensor_data.pop()

                sensor_data = [float(value) for value in sensor_data]
                print(sensor_data)

                ring.m_pos[0] = rl.vector3_add(ring.s_pos[0], rl.Vector3(sensor_data[0], sensor_data[2], sensor_data[1])) # swap nothing
                ring.m_pos[1] = rl.vector3_add(ring.s_pos[1], rl.Vector3(sensor_data[4], sensor_data[5], sensor_data[3])) # swap x and z
                ring.m_pos[2] = rl.vector3_add(ring.s_pos[2], rl.Vector3(-sensor_data[6], sensor_data[8], -sensor_data[7])) # negate x and z
                ring.m_pos[3] = rl.vector3_add(ring.s_pos[3], rl.Vector3(-sensor_data[10], sensor_data[11], -sensor_data[9])) # flip and negate x and z
            except:
                pass

# ser = serial.Serial('COM7', 115200)
thread = threading.Thread(target=getInputs, daemon=True)
thread.start()

while not rl.window_should_close():
    """
        INPUT HANDLING AND MATH FOR LOOP
    """
    # make it interactive (demo simulation)
    processUserInputs()

    ring.calc_all() # calc all values inside the ring system

    """
        ONLY DRAW CALLS FROM HERE ON
    """
    rl.begin_drawing()
    rl.clear_background(rl.BLACK)
    rl.begin_mode_3d(camera)
   
    # draw base circles
    rl.draw_circle_3d(rl.Vector3(0,0,0), ring.s_circle_radius, rl.Vector3(1,0,0), 90, rl.WHITE)
    rl.draw_circle_3d(rl.Vector3(0,0,0), ring.m_circle_radius, rl.Vector3(1,0,0), 90, rl.WHITE)

    # orientation marker
    draw_orientation_marker()   # buggy atm 

    # draw fixed sensor positions                      
    for pos in ring.s_pos:                  
        rl.draw_sphere(pos, 0.75, rl.RED)     
    
    # draw magnet positions
    for pos in ring.m_pos:
        rl.draw_sphere(pos, 0.75, rl.GREEN)     

    rl.draw_sphere(rl.vector3_scale(rl.vector3_normalize(ring.dir_vectors[0]), 5), 0.5, rl.YELLOW)
    rl.draw_line_3d(rl.Vector3(0,0,0), rl.vector3_scale(rl.vector3_normalize(ring.dir_vectors[0]), 5), rl.YELLOW)
    rl.draw_sphere(rl.vector3_scale(rl.vector3_normalize(ring.dir_vectors_snapshot[0]), 5), 0.5, rl.ORANGE)
    rl.draw_line_3d(rl.Vector3(0,0,0), rl.vector3_scale(rl.vector3_normalize(ring.dir_vectors_snapshot[0]), 5), rl.ORANGE)


    # draw magnet vectors
    rl.draw_line_3d(ring.m_pos[0], ring.m_pos[2], rl.BLUE)
    rl.draw_line_3d(ring.m_pos[1], ring.m_pos[3], rl.BLUE) 

    # draw intersection point
    rl.draw_sphere(ring.intersections[0], 0.5, rl.RED)
    # draw normal vector from intersection
    rl.draw_line_3d(ring.intersections[0], ring.v_normal, rl.RED)

    # draw snapshot vector data
    draw_snapshot_data() # use TAB to snapshot -> see processUserInputs()

    rl.end_mode_3d()
    rl.end_drawing()

    camera_scale = 1

    # ring.calc_rotation(ring.dir_vectors[0], ring.dir_vectors_snapshot[0])


# thread.join()

rl.close_window()


