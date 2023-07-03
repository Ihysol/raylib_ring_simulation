import pyray as rl
import math
import numpy as np

import serial
import threading
import time

def deg2rad(deg):
    return deg * (math.pi/180)

def find_intersection_4points(a1, a2, b1, b2):
    # calc direction vectors
    a_dir = rl.vector3_subtract(a2, a1)
    b_dir = rl.vector3_subtract(b2, b1)
    
    # calc cross product of both direction vectors
    n = rl.vector3_cross_product(a_dir, b_dir)

    # find points 
    t1 = rl.vector_3dot_product(rl.vector3_cross_product(b_dir, n), rl.vector3_subtract(b1, a1)) /  rl.vector_3dot_product(n, n)
    t2 = rl.vector_3dot_product(rl.vector3_cross_product(a_dir, n), rl.vector3_subtract(b1, a1)) / rl.vector_3dot_product(n, n)

    # calc intersections
    points = []
    points.append(rl.vector3_add(a1, rl.vector3_scale(a_dir, t1)))
    points.append(rl.vector3_add(b1, rl.vector3_scale(b_dir, t1)))
    return points

def find_intersection(points):
    return find_intersection_4points(points[0], points[2], points[1], points[3])

def calc_normal_vector_4points(a1, a2, b1, b2):
    # direction vectors
    a_dir = rl.vector3_subtract(a2, a1)
    b_dir = rl.vector3_subtract(b2, b1)

    # calc normal vector
    normal_vector = rl.vector3_scale(rl.vector3_cross_product(b_dir, a_dir), -1)
    return normal_vector

def calc_normal_vector(points):
    return calc_normal_vector_4points(points[0], points[2], points[1], points[3])

# init window
WIDTH, HEIGHT = 800, 600
rl.init_window(WIDTH, HEIGHT, "green wie ein philipp - 3d vector demo for a commercial product that nobody needs, but yet it is still in worked on.")
rl.set_target_fps(60)

# camera
camera = rl.Camera3D()
camera.position = rl.Vector3(0, 20, 100)
camera.target = rl.Vector3(0, 0, 0)
camera.up = rl.Vector3(0, 1, 0)
camera.fovy = 45.0

# geometry
offset_s_m = 3
s_circle_radius = 42.5
m_circle_radius = s_circle_radius + 2*offset_s_m

# sensor positions
s_positions = []
m_positions = []
for i in range(4):
    angle = i*deg2rad(-90)
    x = math.cos(angle)
    z = math.sin(angle)
    s_positions.append(rl.vector3_scale(rl.vector3_normalize(rl.Vector3(x, 0, z)), s_circle_radius))
    m_positions.append(rl.vector3_scale(rl.vector3_normalize(rl.Vector3(x, 0, z)), m_circle_radius))



delta_scale = 0.5

def processUserInputs():
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
        

    if rl.is_key_down(rl.KeyboardKey.KEY_LEFT) or rl.is_key_down(rl.KeyboardKey.KEY_RIGHT):
        delta = delta_scale if rl.is_key_down(rl.KeyboardKey.KEY_LEFT) else -delta_scale
        rotation_matrix = rl.matrix_rotate_z(deg2rad(delta))
        for idx, pos in enumerate(m_positions):
            m_positions[idx] = rl.vector3_transform(pos, rotation_matrix)

    if rl.is_key_down(rl.KeyboardKey.KEY_UP) or rl.is_key_down(rl.KeyboardKey.KEY_DOWN):
        delta = delta_scale if rl.is_key_down(rl.KeyboardKey.KEY_DOWN) else -delta_scale
        rotation_matrix = rl.matrix_rotate_x(deg2rad(delta))

        for idx, pos in enumerate(m_positions):
            m_positions[idx] = rl.vector3_transform(pos, rotation_matrix)

    if rl.is_key_down(rl.KeyboardKey.KEY_PAGE_UP) or rl.is_key_down(rl.KeyboardKey.KEY_PAGE_DOWN):
        delta = delta_scale if rl.is_key_down(rl.KeyboardKey.KEY_PAGE_DOWN) else -delta_scale
        rotation_matrix = rl.matrix_rotate_y(deg2rad(delta))

        for idx, pos in enumerate(m_positions):
            m_positions[idx] = rl.vector3_transform(pos, rotation_matrix)

        for i in range(4):
            m_positions[i] = rl.vector3_transform(m_positions[i], rotation_matrix)

    if rl.is_key_down(rl.KeyboardKey.KEY_ENTER):
        for idx, pos in enumerate(m_positions):
            angle = idx*deg2rad(90)
            m_positions[idx] = rl.vector3_scale(rl.vector3_normalize(rl.Vector3(math.cos(angle), 0, math.sin(angle))), m_circle_radius)

    rl.draw_text("WASD:cam control, Arrow Keys:Tilt, Enter: reset tilt", 10, 10, 10, rl.BEIGE)

    if rl.is_key_down(rl.KeyboardKey.KEY_J) or rl.is_key_down(rl.KeyboardKey.KEY_L):
        delta_x = delta_scale if rl.is_key_down(rl.KeyboardKey.KEY_L) else -delta_scale
        translation_matrix = rl.matrix_translate(delta_x, 0, 0)
        for idx, pos in enumerate(m_positions):
            m_positions[idx] = rl.vector3_transform(pos, translation_matrix)
    elif rl.is_key_down(rl.KeyboardKey.KEY_I) or rl.is_key_down(rl.KeyboardKey.KEY_K):
        delta_z = delta_scale if rl.is_key_down(rl.KeyboardKey.KEY_K) else -delta_scale
        translation_matrix = rl.matrix_translate(0, 0, delta_z)
        for idx, pos in enumerate(m_positions):
            m_positions[idx] = rl.vector3_transform(pos, translation_matrix)


orientation_marker_pos = []
orientation_marker_pos.append(m_positions[3])
orientation_marker_pos.append(rl.vector3_subtract(s_positions[3], rl.Vector3(3,0,0)))
orientation_marker_pos.append(rl.vector3_add(s_positions[3], rl.Vector3(3,0,0)))

# quaternion debug
# dummy = rl.Vector3(0, 10, 0)
# rotation_axis = [1, 0, 0]
# q_angle = 22.5
# qx = rl.Vector4(math.sin(deg2rad(q_angle/2))*rotation_axis[0], math.sin(deg2rad(q_angle/2))*rotation_axis[1], math.sin(deg2rad(q_angle/2))*rotation_axis[2], math.cos(deg2rad(q_angle/2)))

# rotation_axis = [0, 1, 0]
# q_angle = 45
# qy = rl.Vector4(math.sin(deg2rad(q_angle/2))*rotation_axis[0], math.sin(deg2rad(q_angle/2))*rotation_axis[1], math.sin(deg2rad(q_angle/2))*rotation_axis[2], math.cos(deg2rad(q_angle/2)))

# q_combined = rl.quaternion_multiply(qy, qx)
# dummy = rl.vector3_rotate_by_quaternion(dummy, q_combined)


def getInputs():
    inputs = []
    while True:
        if ser.in_waiting > 0:
            try:
                sensor_data = ser.readline().decode().rstrip().split(";")
                sensor_data.pop()

                sensor_data = [float(value) for value in sensor_data]
                print(sensor_data)

                # for idx, pos in enumerate(m_positions):
                #     m_positions[idx] = rl.vector3_add(s_positions[idx], rl.Vector3(sensor_data[idx], sensor_data[idx+1], sensor_data[idx+2]))
                #     print(m_positions[idx])

                m_positions[0] = rl.vector3_add(s_positions[0], rl.Vector3(sensor_data[0], sensor_data[2], sensor_data[1])) # swap nothing
                m_positions[1] = rl.vector3_add(s_positions[1], rl.Vector3(sensor_data[4], sensor_data[5], sensor_data[3])) # swap x and z
                m_positions[2] = rl.vector3_add(s_positions[2], rl.Vector3(-sensor_data[6], sensor_data[8], -sensor_data[7])) # negate x and z
                m_positions[3] = rl.vector3_add(s_positions[3], rl.Vector3(-sensor_data[10], sensor_data[11], -sensor_data[9])) # flip and negate x and z
            except:
                pass
            # m_positions[1] = rl.vector3_add(s_positions[1], )
          
            # for idx, pos in enumerate(m_positions):
            #     m_positions[idx] = rl.vector3_add(pos, rl.Vector3(sensor_data[idx+0], sensor_data[idx+1], sensor_data[idx+2]))
            #     print(f"m{idx}({m_positions[idx].x}, {m_positions[idx].y}, {m_positions[idx].z})")
            #     pass

        # time.sleep(0.1)

        

ser = serial.Serial('COM7', 115200)
thread = threading.Thread(target=getInputs, daemon=True)
thread.start()

while not rl.window_should_close():
    """
        INPUT HANDLING AND MATH FOR LOOP
    """
    # make it interactive (demo simulation)
    processUserInputs()
    # getInputs()

    # find intersection point
    points = find_intersection(m_positions)


    """
        ONLY DRAW CALLS FROM HERE ON
    """
    rl.begin_drawing()
    rl.clear_background(rl.BLACK)
    rl.begin_mode_3d(camera)


    # rl.draw_line_3d(rl.Vector3(0,0,0), dummy, rl.PURPLE)

    
    # draw geometry
    rl.draw_circle_3d(rl.Vector3(0,0,0), s_circle_radius, rl.Vector3(1,0,0), 90, rl.WHITE)
    rl.draw_circle_3d(rl.Vector3(0,0,0), m_circle_radius, rl.Vector3(1,0,0), 90, rl.WHITE)
    rl.draw_triangle_3d(orientation_marker_pos[0], orientation_marker_pos[1], orientation_marker_pos[2], rl.WHITE)

    # draw sensor positions
    for pos in s_positions:
        rl.draw_sphere(pos, 0.75, rl.RED)

    # rl.draw_circle_3d(m_positions[2], 3, rl.Vector3(0,0,0), 0, rl.PURPLE)

    # draw magnet positions and vectors
    for pos in m_positions:
        rl.draw_sphere(pos, 0.75, rl.GREEN)
    rl.draw_line_3d(m_positions[0], m_positions[2], rl.BLUE)
    rl.draw_line_3d(m_positions[1], m_positions[3], rl.BLUE)

    # draw intersection point
    rl.draw_sphere(points[0], 0.5, rl.RED)
    
    # draw normal vector
    rl.draw_line_3d(points[0], calc_normal_vector(m_positions), rl.RED)

    rl.end_mode_3d()
    rl.end_drawing()

    camera_scale = 1

# thread.join()

rl.close_window()


